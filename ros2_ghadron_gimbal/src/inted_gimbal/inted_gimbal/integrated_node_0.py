# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from vision_msgs.msg import Detection2D, Detection2DArray
# from geometry_msgs.msg import Point, Vector3
# from std_msgs.msg import Bool
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from rcl_interfaces.msg import ParameterDescriptor, ParameterType
# import cv2
# import threading
# from cv_bridge import CvBridge
# from .streaming import Streaming
# from .yolo import Detection
# from .track import SimpleTracker
# import time

# class IntegratedNode(Node):
#     def __init__(self):
#         super().__init__('integrated_node')

#         self.gimbal_pub = self.create_publisher(Vector3, 'gimbal_angles', 10)
#         self.detection_pub = self.create_publisher(Detection2DArray, 'detection_box', 10)
#         self.image_pub = self.create_publisher(Image, 'image_raw', qos_profile=QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST))

#         self.spin_sub = self.create_subscription(Bool, 'gimbal/spin', self.spin_loop, 10)
#         self.lockon_sub = self.create_subscription(Bool, 'gimbal/lockon', self.lockon_loop, 10)
        
#         self.streaming_module = Streaming()
#         self.detection_model = Detection()
#         self.bridge = CvBridge()
#         self.direction = -1.0

#         self.start()

#     def start(self):
#         self.streaming_module.start_ffmpeg()
        
#         self.detection_model.load_model()

#         self.stream_thread = threading.Thread(target=self.process_stream)
#         self.stream_thread.daemon = True
#         self.stream_thread.start()

#     def lockon_loop(self, msg):
#         # Commented out tracking functionality
#         """
#         while rclpy.ok() and self.lockon_sub.get_last_msg().data == True:
#             frame = self.streaming_module.retrieve_image()

#             cv_image = self.detection_model.yolo_detect(frame)
#             frame = cv2.resize(cv_image, (640, 360))
#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

#             if frame is None:
#                 self.get_logger().error("No current image")
#                 break
#             bounding_boxes = Detection.yolo_detect(frame)
#             if bounding_boxes is None:
#                 self.get_logger().error("No people detected in image")
#                 break
            
#             #Now, perform the tracking
#             target_yaw, target_pitch = SimpleTracker.track(bounding_boxes[0], frame.shape[1], frame.shape[0])
#             self.gimbal_pub.publish(Vector3(x=target_yaw, y=0.0, z=target_pitch))
#         """
#         pass

#     def spin_loop(self, msg):
#         while rclpy.ok() and msg.data == True:
#             frame = self.streaming_module.retrieve_image()

#             cv_image = self.detection_model.yolo_detect(frame)
#             frame = cv2.resize(cv_image, (640, 512))
#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

#             if frame is None:
#                 self.get_logger().error("No current image")
#                 continue

#             bounding_boxes = Detection.yolo_detect(frame)
#             # Commented out tracking switch
#             """
#             if bounding_boxes is not None and len(bounding_boxes) > 0:
#                 self.get_logger().error("Detected a person, switching to lockon mode")
#                 return self.lockon_loop(msg)
#             """
            
#             current_yaw = self.gimbal_pub.get_last_msg().x
            
#             if(abs(current_yaw) > 120):
#                 self.direction *= -1.0

#     def process_stream(self):
#         fps_limit = 10  # Limit to 10 frames per second
#         frame_time = 1.0 / fps_limit
        
#         while rclpy.ok():
#             start_time = time.time()
            
#             frame = self.streaming_module.retrieve_image()
#             if frame is not None:
#                 print("Sending out image data")
#                 self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            
#             # Calculate sleep time to maintain desired FPS
#             elapsed = time.time() - start_time
#             sleep_time = max(0, frame_time - elapsed)
#             time.sleep(sleep_time)


# def main(args=None):
#     rclpy.init(args=args)
#     try:
#         node = IntegratedNode()
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main() 