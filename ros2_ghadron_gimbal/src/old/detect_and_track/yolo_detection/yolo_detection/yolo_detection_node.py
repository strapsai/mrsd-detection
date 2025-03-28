#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch
import time  # Added import for timing
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Remove these debug prints once PyTorch is correctly installed
# print(torch.__version__)
# print(torch.version.cuda)
# print(torch.cuda.is_available())

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # 只保留检测框发布者
        self.detection_pub = self.create_publisher(Detection2DArray, 'detection_box', 10)
        
        # 创建QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            qos
        )
        
        self.bridge = CvBridge()
        
        # 添加参数控制
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('use_preprocessing', True)
        self.declare_parameter('frame_skip', 0)  # 处理每一帧
        
        self.frame_count = 0
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.use_preprocessing = self.get_parameter('use_preprocessing').value
        self.frame_skip = self.get_parameter('frame_skip').value
        
        # 加载 YOLOv11x-pose 模型
        self.get_logger().info('正在加载 YOLOv11x-pose 模型...')
        model_path = "models/yolo11x-pose.pt"
        
        # 加载模型并优化
        self.model = YOLO(model_path)
        
        # 使用GPU（如果可用）
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # If using Jetson, you can try forcing CUDA device even if torch.cuda.is_available() returns False
        # Uncomment the line below if you've installed the Jetson-specific PyTorch wheel
        # self.device = torch.device('cuda:0')  
        self.model.to(self.device)
        
        # 配置模型以优化推理
        self.model.fuse()  # 融合模型层以提高速度
        
        # 创建CLAHE对象 - 只创建一次以提高效率
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        
        self.get_logger().info(f'YOLOv11x-pose 模型加载成功，运行于 {self.device}')
    
    def preprocess_image(self, image):
        """优化的图像预处理函数"""
        if not self.use_preprocessing:
            return image
            
        # 转换为灰度图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 应用自适应直方图均衡化
        equalized = self.clahe.apply(gray)
        
        # 转换回BGR格式
        equalized_bgr = cv2.cvtColor(equalized, cv2.COLOR_GRAY2BGR)
        
        return equalized_bgr
        
    def image_callback(self, msg):
        try:
            # 帧跳过逻辑 - 跳过一些帧以提高处理速度
            self.frame_count += 1
            if self.frame_skip > 0 and (self.frame_count - 1) % (self.frame_skip + 1) != 0:
                return
                
            # 转换ROS图像消息到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 应用图像预处理（如果启用）
            if self.use_preprocessing:
                processed_image = self.preprocess_image(cv_image)
            else:
                processed_image = cv_image
     
            # 测量推理时间
            start_time = time.time()
            
            # 运行YOLO检测，限制只检测类别0（人）
            # 配置更多参数以提高速度
            results = self.model(processed_image, 
                                classes=[0],
                                conf=self.confidence_threshold,  # 直接在模型中过滤置信度
                                verbose=False)  # 关闭冗余输出
            
            # 计算并记录推理时间
            inference_time = time.time() - start_time
            self.get_logger().info(f'inference time: {inference_time:.4f}秒')
            
            # 创建检测消息数组
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # 处理检测结果
            if len(results) > 0:
                result = results[0]
                boxes = result.boxes
                
                # 处理所有检测到的人
                if len(boxes) > 0:
                    for box in boxes:
                        # 获取边界框坐标
                        xyxy = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = map(float, xyxy)
                        
                        # 获取置信度
                        confidence = float(box.conf)
                        
                        # 创建Detection2D消息
                        detection = Detection2D()
                        detection.header = msg.header
                        
                        # 设置边界框中心和大小
                        detection.bbox.center.position.x = float((x1 + x2) / 2)
                        detection.bbox.center.position.y = float((y1 + y2) / 2)
                        detection.bbox.size_x = float(x2 - x1)
                        detection.bbox.size_y = float(y2 - y1)
                        detection.bbox.center.theta = confidence
                        
                        # 添加类别ID
                        detection.id = "0"  # 人类的ID是0
                        
                        # 只在调试模式下记录详细信息
                        if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                            self.get_logger().debug(f'检测到人：置信度={confidence:.4f}, 位置=({x1:.1f},{y1:.1f})-({x2:.1f},{y2:.1f})')
                        
                        detection_array.detections.append(detection)
            
            # 发布检测结果
            self.detection_pub.publish(detection_array)
            
        except Exception as e:
            self.get_logger().error(f'Error in detection: {str(e)}')
            
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YoloDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 