#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import numpy as np
import time
import threading
import signal
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class StreamPublisher(Node):
    def __init__(self):
        super().__init__('stream_publisher')
        
        # 声明参数
        self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8554/ghadron')
        # self.declare_parameter('width', 1280)
        # self.declare_parameter('height', 720)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 360)
        self.declare_parameter('retry_max', 10)
        self.declare_parameter('retry_delay', 2.0)
        self.declare_parameter('fps', 5)  # Added FPS parameter
        
        # 获取参数
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.WIDTH = self.get_parameter('width').value
        self.HEIGHT = self.get_parameter('height').value
        self.retry_max = self.get_parameter('retry_max').value
        self.retry_delay = self.get_parameter('retry_delay').value
        self.fps = self.get_parameter('fps').value
        
        # 运行状态控制
        self.is_running = True
        self.retry_count = 0
        self.process = None
        self.process_lock = threading.Lock()
        
        # Performance monitoring
        self.frame_count = 0
        self.last_log_time = time.time()
        self.last_frame_time = 0
        
        # 创建QoS配置 - Modified for real-time performance
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Using BEST_EFFORT for lower latency video streaming
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        
        # 创建发布者
        self.publisher_ = self.create_publisher(Image, 'image_raw', qos)
        self.bridge = CvBridge()
        
        # 启动RTSP流处理线程
        self.stream_thread = threading.Thread(target=self.process_stream)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
        # 启动监控线程
        self.monitor_thread = threading.Thread(target=self.monitor_process)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def get_ffmpeg_cmd(self):
        """获取FFMPEG命令行配置"""
        return [
            "ffmpeg",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-rtsp_transport", "tcp",
            "-stimeout", "5000000",    # 5秒连接超时
            "-use_wallclock_as_timestamps", "1",
            "-avioflags", "direct",
            "-flush_packets", "1",
            "-probesize", "32",        # Reduce initial buffering
            "-analyzeduration", "0",   # Skip detailed stream analysis
            "-thread_queue_size", "512", # Increase thread queue size
            "-hwaccel", "auto",        # Use hardware acceleration if available
            "-i", self.rtsp_url,
            "-vsync", "0",
            "-copyts",
            "-vf", f"fps={self.fps},scale={self.WIDTH}:{self.HEIGHT}",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "-"
        ]
    
    def start_ffmpeg(self):
        """启动FFMPEG进程"""
        try:
            cmd = self.get_ffmpeg_cmd()
            self.get_logger().info(f'启动FFMPEG命令: {" ".join(cmd)}')
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            
            if self.process.poll() is None:
                self.get_logger().info(f'成功启动FFMPEG，URL: {self.rtsp_url}')
                return True
            else:
                self.get_logger().error('FFMPEG进程启动后立即退出')
                return False
                
        except Exception as e:
            self.get_logger().error(f'启动FFMPEG失败: {str(e)}')
            return False
    
    def monitor_process(self):
        """监控FFMPEG进程状态"""
        while self.is_running:
            # 检查进程是否存在且运行
            if self.process and self.process.poll() is not None:
                self.get_logger().warn('FFMPEG进程已终止，将重新启动...')
                if self.retry_count < self.retry_max:
                    # 通知流处理线程重启FFMPEG
                    with self.process_lock:
                        self.process = None
                else:
                    self.get_logger().error(f'达到最大重试次数 ({self.retry_max})，停止重试')
            time.sleep(1.0)
    
    def process_stream(self):
        """处理RTSP流并发布图像"""
        frame_size = self.WIDTH * self.HEIGHT * 3
        
        # Pre-allocate the buffer for better performance
        frame_buffer = bytearray(frame_size)
        view = memoryview(frame_buffer)
        
        # Flag to print resolution only once
        resolution_printed = False
        
        while self.is_running:
            # 如果进程不存在或已终止，尝试启动
            if not self.process or self.process.poll() is not None:
                # 计算重试延迟（使用指数退避）
                retry_wait = self.retry_delay * (1.5 ** min(self.retry_count, 10))
                self.retry_count += 1
                
                self.get_logger().info(f'尝试启动FFMPEG (尝试 {self.retry_count}/{self.retry_max})...')
                if self.retry_count > 1:
                    self.get_logger().info(f'等待 {retry_wait:.1f} 秒后重试...')
                    time.sleep(retry_wait)
                
                if not self.start_ffmpeg():
                    if self.retry_count >= self.retry_max:
                        self.get_logger().error(f'达到最大重试次数 ({self.retry_max})，停止尝试')
                        break
                    continue
                else:
                    # 成功启动，重置重试计数
                    self.retry_count = 0
                    # Reset resolution printed flag when reconnecting
                    resolution_printed = False
            
            # 处理视频帧
            try:
                # More efficient reading using pre-allocated buffer
                bytes_read = 0
                while bytes_read < frame_size and self.is_running:
                    chunk = self.process.stdout.read(frame_size - bytes_read)
                    if not chunk:
                        break
                    view[bytes_read:bytes_read+len(chunk)] = chunk
                    bytes_read += len(chunk)
                
                if bytes_read != frame_size:
                    self.get_logger().warn('读取到不完整帧，跳过...')
                    continue
                
                # Skip frame if we're falling behind
                current_time = time.time()
                if self.last_frame_time > 0:
                    frame_interval = 1.0/self.fps
                    if current_time - self.last_frame_time < frame_interval * 0.5:
                        # Processing too fast, might want to add a small sleep
                        pass
                    elif current_time - self.last_frame_time > frame_interval * 2:
                        # We're falling behind, skip this frame
                        self.get_logger().debug('处理速度慢，跳过当前帧')
                        self.last_frame_time = current_time
                        continue
                
                self.last_frame_time = current_time
                    
                # 转换并发布帧
                frame = np.frombuffer(frame_buffer, np.uint8).reshape((self.HEIGHT, self.WIDTH, 3))
                
                # Print the original frame resolution 
                if not resolution_printed:
                    frame_height, frame_width = frame.shape[:2]
                    self.get_logger().info(f'RTSP Stream Resolution: {frame_width}x{frame_height}')
                    resolution_printed = True
                
                frame = cv2.resize(frame, (320, 180), interpolation=cv2.INTER_AREA)
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                
                self.publisher_.publish(msg)
                
                # Performance monitoring
                self.frame_count += 1
                if current_time - self.last_log_time > 5.0:  # Log every 5 seconds
                    fps = self.frame_count / (current_time - self.last_log_time)
                    self.get_logger().info(f'当前帧率: {fps:.2f} FPS')
                    self.frame_count = 0
                    self.last_log_time = current_time
                
            except Exception as e:
                self.get_logger().error(f'处理帧时出错: {str(e)}')
                time.sleep(0.1)
    
    def shutdown(self):
        """干净地关闭节点"""
        self.get_logger().info('关闭stream_publisher节点...')
        self.is_running = False
        
        # 等待线程结束
        if hasattr(self, 'stream_thread') and self.stream_thread.is_alive():
            self.stream_thread.join(timeout=2.0)
            
        if hasattr(self, 'monitor_thread') and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        
        # 终止FFMPEG进程
        if self.process and self.process.poll() is None:
            self.get_logger().info('终止FFMPEG进程...')
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('FFMPEG进程未响应，强制终止')
                self.process.kill()
        
        self.get_logger().info('stream_publisher节点已安全关闭')

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = StreamPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"错误: {str(e)}")
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
