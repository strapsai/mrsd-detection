#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


class DetectAndTrackNode(Node):
    def __init__(self):
        super().__init__('detect_and_track_node')
        
        # 创建 QoS 配置
        sensor_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # 发布者
        self.gimbal_pub = self.create_publisher(Vector3, 'gimbal_angles', 10)
        self.detection_pub = self.create_publisher(Detection2DArray, 'detection_box', 10)
        self.image_pub = self.create_publisher(Image, 'image_with_box', 10)
        
        # 订阅者 - 为图像话题使用sensor_qos
        # self.image_sub = self.create_subscription(
        #     Image, 
        #     'image_raw', 
        #     self.image_callback, 
        #     qos_profile=sensor_qos  # 使用传感器数据的QoS配置
        # )
        
        self.attitude_sub = self.create_subscription(Point, 'gimbal_attitude', self.attitude_callback, 10)
        self.lockon_sub = self.create_subscription(Bool, 'lockon_start', self.lockon_callback, 10)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        self.get_logger().info('正在加载 YOLOv11x-pose 模型...')
        model_path = "models/yolo11x-pose.pt"
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"模型文件 {model_path} 不存在！")
            self.get_logger().info("请先下载模型: wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11x-pose.pt -O models/yolo11x-pose.pt")
            raise FileNotFoundError(f"模型文件 {model_path} 不存在")
        
        self.model = YOLO(model_path)
        self.get_logger().info('YOLOv11x-pose 模型加载成功')
        
        # 图像参数
        self.image_width = None
        self.image_height = None
        self.last_image_timestamp = None
        
        # 云台参数
        self.pitch = None
        self.roll = None
        self.yaw = None
        self.angle_step = 3.0
        self.error_scale = 10
        
        # 声明参数，设置初始值为 True
        self.declare_parameter('lockon_enabled', True)
        self.lockon_enabled = self.get_parameter('lockon_enabled').value
        
        # 添加参数变更回调
        self.add_on_set_parameters_callback(self.parameters_callback)
        
    def lockon_callback(self, msg):
        """处理锁定命令"""
        self.lockon_enabled = msg.data
        if not self.lockon_enabled:
            self.get_logger().info('锁定模式已关闭，云台回到默认位置')
        else:
            self.get_logger().info('锁定模式已启动')
    
    def image_callback(self, msg):
        try:
            # 更新图像参数
            self.image_width = msg.width
            self.image_height = msg.height
            self.last_image_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 转换图像并进行检测
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            display_image = cv_image.copy()  # 创建副本用于绘制
            results = self.model(cv_image)
            
            # 创建检测消息数组
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            center_x = center_y = None
            has_detection = False
            
            # 处理检测结果
            if len(results) > 0:
                result = results[0]
                boxes = result.boxes
                
                # 筛选人类检测框
                person_boxes = []
                for box in boxes:
                    if hasattr(box, 'cls') and int(box.cls[0]) == 0:
                        confidence = float(box.conf[0].cpu().numpy())
                        if confidence > 0.4:
                            person_boxes.append(box)
                
                if person_boxes:
                    has_detection = True
                    # 选择置信度最高的检测框
                    confidences = [float(box.conf[0].cpu().numpy()) for box in person_boxes]
                    highest_conf_idx = np.argmax(confidences)
                    box = person_boxes[highest_conf_idx]
                    
                    # 获取边界框信息
                    x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    confidence = float(box.conf[0].cpu().numpy())
                    
                    # 在图像上绘制检测框
                    cv2.rectangle(display_image, 
                                (int(x1), int(y1)), 
                                (int(x2), int(y2)), 
                                (0, 255, 0), 2)
                    
                    # 添加置信度文本
                    text = f"Confidence: {confidence:.2f}"
                    cv2.putText(display_image, text, 
                              (int(x1), int(y1) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 
                              0.5, (0, 255, 0), 2)
                    
                    # 创建Detection2D消息
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # 设置边界框中心和大小
                    detection.bbox.center.position.x = center_x
                    detection.bbox.center.position.y = center_y
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    detection.bbox.center.theta = confidence
                    
                    # 添加类别ID
                    if hasattr(box, 'cls'):
                        detection.id = str(int(box.cls[0]))
                    
                    detection_array.detections.append(detection)
                    self.get_logger().info(f'检测到人：置信度={confidence:.4f}, 位置=({x1:.1f},{y1:.1f})-({x2:.1f},{y2:.1f})')
            
            # 发布检测框
            self.get_logger().info(f'发布检测框: {detection_array}')
            self.detection_pub.publish(detection_array)
            
            # 如果没有检测到人，添加文字提示
            if not detection_array.detections:
                text = "No People Detected"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
                text_x = (display_image.shape[1] - text_size[0]) // 2
                text_y = (display_image.shape[0] + text_size[1]) // 2
                cv2.putText(display_image, text,
                           (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX,
                           1, (0, 0, 255), 2)
            
            # 发布带有检测框的图像
            image_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
            image_msg.header = msg.header
            self.image_pub.publish(image_msg)
            
            # 如果有检测结果且锁定模式开启，则进行跟踪
            if detection_array.detections and self.lockon_enabled:
                self.track_target(center_x, center_y)
            elif self.lockon_enabled and not detection_array.detections:
                self.get_logger().debug('锁定模式开启但未检测到目标')
            
        except Exception as e:
            self.get_logger().error(f'检测错误: {str(e)}')
    
    def attitude_callback(self, msg):
        """更新当前云台姿态"""
        self.pitch = msg.x
        self.roll = msg.y
        self.yaw = msg.z
    
    def track_target(self, center_x, center_y):
        """跟踪目标 - 直接移动到目标位置"""
        if self.image_width is None or self.pitch is None:
            return
            
        # 计算目标在图像中的相对位置（0-1范围）
        rel_x = center_x - (self.image_width / 2.0)
        rel_y = center_y - (self.image_height / 2.0)
        
        # 计算目标角度
        # 将图像坐标映射到角度范围
        # yaw: -120 到 120度的范围
        # pitch: -90 到 90度的范围
        norm_error_x = rel_x / (self.image_width / 2.0)
        norm_error_y = rel_y / (self.image_height / 2.0)
	print("Norm error: " + str(norm_error_x))
	print("Norm error y: " + str(norm_error_y))
	
	gain_yaw = 0.1
	gain_pitch = 0.1        
       	target_yaw = norm_error_x * max_yaw * gain_yaw
	target_pitch = -norm_error_y * max_pitch * gain_pitch
	

        target_pitch = max(min(target_pitch, 90.0), -90.0)
        target_yaw = max(min(target_yaw, 120.0), -120.0)
        
        # 发送云台控制命令
        # 只在角度差异足够大时才发送命令
        if abs(target_pitch - self.pitch) > 0.1 or abs(target_yaw - self.yaw) > 0.1:
            gimbal_cmd = Vector3()
            gimbal_cmd.x = float(target_pitch)
            gimbal_cmd.y = float(self.roll)  # 保持当前roll角度
            gimbal_cmd.z = float(target_yaw)
            self.gimbal_pub.publish(gimbal_cmd)
            self.get_logger().info(f'发送云台命令: pitch={target_pitch:.1f}, yaw={target_yaw:.1f}')

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'lockon_enabled':
                self.lockon_enabled = param.value
                self.get_logger().info(f'Lock-on 状态已更改为: {self.lockon_enabled}')
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DetectAndTrackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 
