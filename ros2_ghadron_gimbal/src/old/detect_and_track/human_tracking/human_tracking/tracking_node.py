#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Bool
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class HumanTrackingNode(Node):
    def __init__(self):
        super().__init__('human_tracking_node')
        
        # Create best effort QoS profile
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅检测框和云台姿态
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'detection_box',
            self.detection_callback,
            best_effort_qos  # Use best effort QoS
        )
        
        self.attitude_sub = self.create_subscription(
            Point,  # 注意：gimbal_attitude使用Point消息类型
            'gimbal_attitude',
            self.attitude_callback,
            best_effort_qos  # Use best effort QoS
        )
        
        # 订阅图像以获取分辨率
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            best_effort_qos  # Use best effort QoS
        )
        
        # 订阅航点等待状态
        self.waypoint_sub = self.create_subscription(
            Bool,
            'spin_survey',
            self.waypoint_callback,
            best_effort_qos  # Use best effort QoS
        )
        
        # 发布云台控制命令
        self.gimbal_pub = self.create_publisher(
            Vector3,
            'gimbal_angles',
            10
        )
        
        # 图像参数（将在收到第一帧图像时更新）
        self.image_width = None
        self.image_height = None
        self.last_image_timestamp = None
        
        # 云台控制参数
        self.angle_step = 15.0  # 进一步增加步长以加快响应
        self.error_scale = 2.0  # 进一步减小偏移阈值
        
        # PID控制器参数
        self.kp = 0.8  # 降低比例系数 (从1.5降低到0.8)
        self.ki = 0.1  # 降低积分系数 (从0.2降低到0.1)
        self.kd = 0.2  # 降低微分系数 (从0.3降低到0.2)
        
        # PID控制器状态
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        
        # 积分项限制
        self.integral_limit = 30.0  # 适当降低积分限制 (从40.0降低到30.0)
        
        # 平滑过滤参数
        self.smoothing_factor = 0.6  # 降低新值权重以减少抖动 (从0.85降低到0.6)
        self.prev_target_pitch = None
        self.prev_target_yaw = None
        
        # 记录上次检测时间用于计算dt
        self.last_detection_time = None
        
        # 当前云台角度（从attitude_callback更新）
        self.pitch = None  # 从x获取
        self.roll = None   # 从y获取
        self.yaw = None    # 从z获取
        
        # 扫描相关参数
        self.spin_survey = False
        self.scanning = True
        self.scan_interval = 0.3  # 减少到0.3秒以加快扫描速度
        self.last_scan_time = 0.0
        self.scan_seq_index = 0
        
        # 创建定时器，更频繁地检查
        self.scan_timer = self.create_timer(0.02, self.scan_for_people)  # 0.02秒检查一次
        self.get_logger().info('Human tracking node initialized with scanning capability')

    def image_callback(self, msg):
        """更新图像分辨率和时间戳"""
        self.image_width = msg.width
        self.image_height = msg.height
        self.last_image_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def attitude_callback(self, msg):
        """更新当前云台姿态"""
        self.pitch = msg.x  # pitch from x
        self.roll = msg.y   # roll from y
        self.yaw = msg.z    # yaw from z
        self.get_logger().debug(f'Current attitude - Pitch: {self.pitch:.2f}, Roll: {self.roll:.2f}, Yaw: {self.yaw:.2f}')

    def waypoint_callback(self, msg):
        """更新航点等待状态"""
        self.spin_survey = msg.data
        if self.spin_survey:
            self.get_logger().info('开始等待航点，启动人员扫描模式')
            # 重新启动扫描
            self.restart_scan()
        else:
            self.get_logger().info('航点等待结束，停止扫描')
            self.scanning = False

    def scan_for_people(self):
        """在无人时扫描区域寻找人员，每个动作之间有2秒间隔"""
        # 如果不在扫描模式，则返回
        if not self.scanning or not self.spin_survey:
            return
        
        # 检查是否已经过了足够的间隔时间
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_scan_time < self.scan_interval:
            return  # 间隔不足2秒，不执行动作
        
        # 检查是否已完成整个序列
        scan_sequence = [
            (-45.0, 0.0, -120.0), # 向左扫描
            (-45.0, 0.0, 0.0),    # 起始位置
            (-45.0, 0.0, 120.0),  # 向右扫描
            (-45.0, 0.0, 0.0),    # 向下看
        ]
        
        # 如果已经完成整个序列，则停止扫描
        if self.scan_seq_index >= len(scan_sequence):
            self.get_logger().info('已完成一次完整扫描序列，停止扫描')
            self.scanning = False
            return
        
        # 获取目标位置
        target_pitch, target_roll, target_yaw = scan_sequence[self.scan_seq_index]
        
        # 发送云台命令
        gimbal_cmd = Vector3()
        gimbal_cmd.x = float(target_pitch)  
        gimbal_cmd.y = float(target_roll)   
        gimbal_cmd.z = float(target_yaw)    
        
        self.gimbal_pub.publish(gimbal_cmd)
        self.get_logger().info(f'扫描位置 {self.scan_seq_index+1}/{len(scan_sequence)}: '
                             f'pitch={target_pitch:.1f}°, roll={target_roll:.1f}°, yaw={target_yaw:.1f}°')
        
        # 更新索引到下一个扫描点
        self.scan_seq_index += 1
        
        # 更新最后扫描时间
        self.last_scan_time = current_time

    def restart_scan(self):
        """重新启动扫描序列"""
        self.scan_seq_index = 0
        self.scanning = True
        # 如果定时器已取消，重新创建
        if not self.scan_timer.is_alive():
            self.scan_timer = self.create_timer(self.scan_interval, self.scan_for_people)
        self.get_logger().info('重新启动扫描序列')

    def detection_callback(self, msg):
        try:
            # 确保我们已经获得图像分辨率和云台姿态
            if self.image_width is None or self.image_height is None or self.pitch is None:
                self.get_logger().warn('Waiting for image resolution or gimbal attitude...')
                return
            
            # 检查是否有检测结果
            if not msg.detections:
                self.get_logger().debug('No detections received')
                
                # 如果在等待航点且没有检测到人，则开始扫描
                if self.spin_survey:
                    self.scanning = True
                return
            
            # 如果检测到人，停止扫描
            self.scanning = False
            
            # 获取当前时间和检测消息时间戳
            current_time = self.get_clock().now().nanoseconds / 1e9
            detection_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 检查检测消息是否在0.5秒内 (从2.0秒减少到0.5秒)
            if current_time - detection_timestamp > 0.5:
                self.get_logger().warn(f'Detection too old: {current_time - detection_timestamp:.2f}s')
                return
                
            # 检查图像和检测消息时间戳是否匹配（相差不超过0.5秒）
            if self.last_image_timestamp is None or abs(detection_timestamp - self.last_image_timestamp) > 0.5:
                self.get_logger().warn(f'Image and detection timestamps mismatch: {abs(detection_timestamp - self.last_image_timestamp):.2f}s')
                return
                
            # 获取第一个检测框的中心点位置
            detection = msg.detections[0]
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            
            # 如果中心点是默认值(0,0)，不进行处理
            if center_x == 0.0 and center_y == 0.0:
                return
                
            # 计算偏差百分比（相对于整个图像尺寸）
            error_x_percent = (center_x / self.image_width) * 100 - 50  # 中心是50%
            error_y_percent = (center_y / self.image_height) * 100 - 50
            
            # 计算时间差
            if self.last_detection_time is None:
                dt = 0.1  # 默认值
            else:
                dt = max(0.01, min(0.5, current_time - self.last_detection_time))  # 限制在合理范围内
            self.last_detection_time = current_time
            
            # PID控制 - Yaw (水平方向)
            p_term_x = self.kp * error_x_percent
            
            # 更新积分项
            self.integral_x += error_x_percent * dt
            self.integral_x = max(-self.integral_limit, min(self.integral_limit, self.integral_x))
            i_term_x = self.ki * self.integral_x
            
            # 计算微分项
            d_term_x = self.kd * (error_x_percent - self.prev_error_x) / dt if dt > 0 else 0
            self.prev_error_x = error_x_percent
            
            # PID控制 - Pitch (垂直方向)
            p_term_y = self.kp * error_y_percent
            
            # 更新积分项
            self.integral_y += error_y_percent * dt
            self.integral_y = max(-self.integral_limit, min(self.integral_limit, self.integral_y))
            i_term_y = self.ki * self.integral_y
            
            # 计算微分项
            d_term_y = self.kd * (error_y_percent - self.prev_error_y) / dt if dt > 0 else 0
            self.prev_error_y = error_y_percent
            
            # 计算PID输出
            pid_output_x = (p_term_x + i_term_x + d_term_x)  # 移除负号以修正方向
            pid_output_y = (p_term_y + i_term_y + d_term_y)  # 移除负号以修正方向
            
            # 将PID输出转换为角度变化
            angle_change_yaw = pid_output_x * 0.3  # 降低系数 (从0.5降低到0.3)
            angle_change_pitch = pid_output_y * 0.3  # 降低系数 (从0.5降低到0.3)
            
            # 计算目标角度
            target_pitch = self.pitch + angle_change_pitch
            target_yaw = self.yaw + angle_change_yaw
            
            # 平滑过滤
            if self.prev_target_pitch is not None and self.prev_target_yaw is not None:
                target_pitch = self.smoothing_factor * target_pitch + (1 - self.smoothing_factor) * self.prev_target_pitch
                target_yaw = self.smoothing_factor * target_yaw + (1 - self.smoothing_factor) * self.prev_target_yaw
            
            self.prev_target_pitch = target_pitch
            self.prev_target_yaw = target_yaw
            
            self.get_logger().debug(f"PID - P_x: {p_term_x:.2f}, I_x: {i_term_x:.2f}, D_x: {d_term_x:.2f}")
            self.get_logger().debug(f"PID - P_y: {p_term_y:.2f}, I_y: {i_term_y:.2f}, D_y: {d_term_y:.2f}")
            self.get_logger().debug(f"target_pitch: {target_pitch}, target_yaw: {target_yaw}")
            
            # 限制云台角度范围之前记录超出限制的情况
            if target_pitch > 90.0 or target_pitch < -90.0:
                self.get_logger().warn(f'目标Pitch角度 {target_pitch:.1f}° 超出限制范围 [-90°, 90°]')
                    
            if target_yaw > 120.0 or target_yaw < -120.0:
                self.get_logger().warn(f'目标Yaw角度 {target_yaw:.1f}° 超出限制范围 [-120°, 120°]')

            # 限制云台角度范围
            target_pitch = max(min(target_pitch, 90.0), -90.0)
            target_yaw = max(min(target_yaw, 120.0), -120.0)
            
            # 发送云台控制命令，不需要额外的角度变化阈值
            gimbal_cmd = Vector3()
            gimbal_cmd.x = float(target_pitch)
            gimbal_cmd.y = float(self.roll)
            gimbal_cmd.z = float(target_yaw)
            
            self.gimbal_pub.publish(gimbal_cmd)
            
            # 减少日志输出频率，只在调试模式下输出详细信息
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.INFO:
                self.get_logger().info(
                    f'跟踪目标: error_x={error_x_percent:.1f}%, error_y={error_y_percent:.1f}%, '
                    f'Angles: pitch={target_pitch:.1f}°, yaw={target_yaw:.1f}°'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in tracking: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackingNode()
    try:
        rclpy.spin(node)  # 这会让节点持续运行并处理回调
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()