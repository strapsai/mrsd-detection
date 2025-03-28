#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from datetime import datetime

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        
        # 创建保存图像的文件夹
        self.save_dir = os.path.expanduser('/home/dtc/humanflow/ros2_ghadron_gimbal/src/image_viewer/assets/')
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f'图像将保存到: {self.save_dir}')
        
        # 设置最后一帧图像的固定文件名
        self.last_frame_path = os.path.join(self.save_dir, 'last_frame.jpg')
        
        # 添加互斥锁，防止多线程同时写入图像
        self.image_lock = threading.Lock()
        
        # QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        
        # 订阅带检测框的图像话题
        self.image_sub = self.create_subscription(
            Image,
            'image_with_box',
            self.image_callback,
            qos
        )
        
        self.bridge = CvBridge()
        
    def save_last_frame(self, cv_image):
        """保存最后一帧图像，覆盖之前的文件"""
        with self.image_lock:
            cv2.imwrite(self.last_frame_path, cv_image)
            self.get_logger().debug(f'已更新最后一帧图像: {self.last_frame_path}')
    
    def image_callback(self, image_msg):
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 获取当前时间并格式化
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # 添加时间戳
            cv2.putText(cv_image, current_time, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 保存最后一帧图像
            self.save_last_frame(cv_image)
                
        except Exception as e:
            self.get_logger().error(f'图像处理出错: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = ImageViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from vision_msgs.msg import Detection2D, Detection2DArray
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import message_filters
# import os
# import datetime
# import threading

# class ImageViewer(Node):
#     def __init__(self):
#         super().__init__('image_viewer')
        
#         # 创建保存图像的文件夹
#         self.save_dir = os.path.expanduser('/home/dtc/humanflow/ros2_ghadron_gimbal/src/image_viewer/assets/')
#         os.makedirs(self.save_dir, exist_ok=True)
#         self.get_logger().info(f'图像将保存到: {self.save_dir}')
        
#         # 设置最后一帧图像的固定文件名
#         self.last_frame_path = os.path.join(self.save_dir, 'last_frame.jpg')
        
#         # 添加互斥锁，防止多线程同时写入图像
#         self.image_lock = threading.Lock()
        
#         # 最新的检测信息
#         self.latest_detections = None
#         self.latest_detection_time = 0
        
#         # QoS配置
#         qos = QoSProfile(
#             depth=1,
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#         )
        
#         # 创建订阅者
#         self.image_sub = message_filters.Subscriber(
#             self, Image, 'image_raw', qos_profile=qos)
            
#         self.detection_sub = message_filters.Subscriber(
#             self, Detection2DArray, 'detection_box', qos_profile=qos)
        
#         # 时间同步器
#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.image_sub, self.detection_sub],
#             queue_size=10,
#             slop=0.1
#         )
#         self.ts.registerCallback(self.combined_callback)
        
#         # 单独的检测回调 - 添加检测数据的接收
#         self.detection_only_sub = self.create_subscription(
#             Detection2DArray,
#             'detection_box',
#             self.detection_only_callback,
#             qos
#         )
        
#         # 单独的图像回调
#         self.image_only_sub = self.create_subscription(
#             Image,
#             'image_raw',
#             self.image_only_callback,
#             qos
#         )
        
#         self.bridge = CvBridge()
        
#     def save_last_frame(self, cv_image):
#         """保存最后一帧图像，覆盖之前的文件"""
#         with self.image_lock:
#             cv2.imwrite(self.last_frame_path, cv_image)
#             self.get_logger().debug(f'已更新最后一帧图像: {self.last_frame_path}')
    
#     def detection_only_callback(self, detection_msg):
#         """单独接收检测数据，保存最新检测结果"""
#         current_time = self.get_clock().now().nanoseconds / 1e9
#         self.latest_detections = detection_msg
#         self.latest_detection_time = current_time
#         self.get_logger().debug(f'接收到新的检测数据，包含 {len(detection_msg.detections)} 个检测框')
        
#     def add_timestamps(self, cv_image, msg_time, current_time):
#         """添加时间戳到图像上"""
#         # 消息时间戳
#         msg_timestamp = f"Image Time: {msg_time:.3f}"
#         cv2.putText(cv_image, msg_timestamp, (10, 30),
#                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
#         # 当前时间戳
#         now_timestamp = f"Current Time: {current_time:.3f}"
#         cv2.putText(cv_image, now_timestamp, (10, 70),
#                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
#         # 计算延迟
#         delay = current_time - msg_time
#         delay_text = f"Delay: {delay:.3f}s"
#         cv2.putText(cv_image, delay_text, (10, 110),
#                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
#     def draw_detections(self, cv_image, detection_msg):
#         """将检测结果绘制到图像上"""
#         if detection_msg.detections:
#             for i, detection in enumerate(detection_msg.detections):
#                 # 从检测框中获取中心点和尺寸
#                 center_x = int(detection.bbox.center.position.x)
#                 center_y = int(detection.bbox.center.position.y)
#                 width = int(detection.bbox.size_x)
#                 height = int(detection.bbox.size_y)
                
#                 # 获取置信度（从theta字段）
#                 confidence = detection.bbox.center.theta
                
#                 # 计算边界框的左上角和右下角坐标
#                 x1 = int(center_x - width / 2)
#                 y1 = int(center_y - height / 2)
#                 x2 = int(center_x + width / 2)
#                 y2 = int(center_y + height / 2)
                
#                 # 确保坐标在图像范围内
#                 h, w = cv_image.shape[:2]
#                 x1 = max(0, x1)
#                 y1 = max(0, y1)
#                 x2 = min(w-1, x2)
#                 y2 = min(h-1, y2)
                
#                 # 根据置信度调整颜色（置信度越高，颜色越绿）
#                 # 从红(0,0,255)到绿(0,255,0)
#                 green = int(255 * confidence)
#                 red = int(255 * (1 - confidence))
#                 color = (0, green, red)
                
#                 # 绘制边界框
#                 cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                
#                 # 显示置信度
#                 conf_text = f"Person: {confidence:.2f}"
#                 cv2.putText(cv_image, conf_text, (x1, y1 - 10),
#                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
#                 # 显示中心点坐标
#                 center_text = f"({center_x}, {center_y})"
#                 cv2.putText(cv_image, center_text, (center_x + 10, center_y),
#                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
#                 # 记录检测信息到日志
#                 self.get_logger().debug(f'检测框 {i}: 置信度={confidence:.2f}, 位置=({x1},{y1})-({x2},{y2})')
#         else:
#             # 当没有检测到物体时，显示"No people detected"文字
#             h, w = cv_image.shape[:2]
#             # 计算文本位置 - 在图像中央
#             text = "No people detected"
#             font = cv2.FONT_HERSHEY_SIMPLEX
#             font_scale = 1.5
#             thickness = 3
#             text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
#             text_x = (w - text_size[0]) // 2
#             text_y = (h + text_size[1]) // 2
            
#             # 添加半透明背景使文字更清晰
#             # 先绘制一个黑色矩形作为背景
#             padding = 20  # 文字周围的填充
#             cv2.rectangle(cv_image, 
#                         (text_x - padding, text_y - text_size[1] - padding),
#                         (text_x + text_size[0] + padding, text_y + padding),
#                         (0, 0, 0), -1)  # -1表示填充矩形
            
#             # 绘制文字 (红色)
#             cv2.putText(cv_image, text, (text_x, text_y),
#                       font, font_scale, (0, 0, 255), thickness)
            
#             self.get_logger().debug('没有检测到物体，显示"No people detected"')
            
#     def image_only_callback(self, image_msg):
#         try:
#             # 获取消息时间戳
#             msg_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
#             # 获取当前时间
#             current_time = self.get_clock().now().nanoseconds / 1e9
            
#             cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
#             # 添加时间戳
#             self.add_timestamps(cv_image, msg_time, current_time)
            
#             # 检查是否有最新的检测数据可用
#             if self.latest_detections is not None:
#                 # 检测数据时间与图像时间的差异不超过1秒
#                 if abs(current_time - self.latest_detection_time) < 1.0:
#                     # 在图像上绘制最新的检测框
#                     self.draw_detections(cv_image, self.latest_detections)
#                     self.get_logger().debug(f'使用最新的检测数据绘制边界框，差异时间: {current_time - self.latest_detection_time:.3f}s')
#                 else:
#                     self.get_logger().debug('最新检测数据过旧，不绘制边界框')
            
#             # 保存最后一帧图像
#             self.save_last_frame(cv_image)
                
#         except Exception as e:
#             self.get_logger().error(f'图像处理出错: {str(e)}')
#             import traceback
#             self.get_logger().error(traceback.format_exc())
            
#     def combined_callback(self, image_msg, detection_msg):
#         try:
#             # 获取消息时间戳
#             msg_time = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
#             # 获取当前时间
#             current_time = self.get_clock().now().nanoseconds / 1e9
            
#             cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
#             # 添加时间戳
#             self.add_timestamps(cv_image, msg_time, current_time)
            
#             # 添加日志显示检测框数量
#             self.get_logger().info(f'接收到 {len(detection_msg.detections)} 个检测框')
            
#             # 绘制检测框
#             self.draw_detections(cv_image, detection_msg)
            
#             # 保存最后一帧图像
#             self.save_last_frame(cv_image)
            
#         except Exception as e:
#             self.get_logger().error(f'绘制检测框时出错: {str(e)}')
#             import traceback
#             self.get_logger().error(traceback.format_exc())

# def main(args=None):
#     rclpy.init(args=args)
    
#     try:
#         viewer = ImageViewer()
#         rclpy.spin(viewer)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()