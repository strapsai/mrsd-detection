#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import cv2
import threading
from cv_bridge import CvBridge
from .streaming import Streaming
from .yolo import Detection
from .track import SimpleTracker
import time
import os
from datetime import datetime

class IntegratedNode(Node):
    def __init__(self):
        super().__init__('integrated_node')

        # Use compressed image transport for more efficient transmission
        self.detection_pub = self.create_publisher(Detection2DArray, 'detection_box', 10)
        self.image_pub = self.create_publisher(Image, 'image_raw', qos_profile=QoSProfile(
            depth=1, 
            reliability=QoSReliabilityPolicy.BEST_EFFORT, 
            history=QoSHistoryPolicy.KEEP_LAST
        ))
        self.compressed_image_pub = self.create_publisher(
            Image, 'image_raw/compressed', 
            qos_profile=QoSProfile(
                depth=1, 
                reliability=QoSReliabilityPolicy.BEST_EFFORT, 
                history=QoSHistoryPolicy.KEEP_LAST
            )
        )

        # Comment out the subscriptions for gimbal control
        # self.spin_sub = self.create_subscription(Bool, 'gimbal/spin', self.spin_loop, 10)
        # self.lockon_sub = self.create_subscription(Bool, 'gimbal/lockon', self.lockon_loop, 10)
        
        self.streaming_module = Streaming()
        self.detection_model = Detection()
        self.bridge = CvBridge()
        # self.direction = -1.0  # Not needed anymore

        # Add a separate thread for detection to not block image publishing
        self.detection_thread = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # Add parameters for recording the stream
        self.declare_parameter('record_stream', False, 
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, 
                               description='Whether to record the stream locally'))
        self.record_stream = self.get_parameter('record_stream').value
        
        self.declare_parameter('record_path', '/home/dtc/humanflow/ros2_ghadron_gimbal', 
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, 
                               description='Directory to save recorded streams'))
        self.record_path = self.get_parameter('record_path').value
        
        # Initialize video recording only after streaming has started
        # (Move this to start() method instead of here)
        self.video_writer = None
        
        # Add frame skipping parameter
        self.declare_parameter('frame_skip', 3, 
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, 
                               description='Number of frames to skip between transmissions'))
        self.frame_skip = self.get_parameter('frame_skip').value
        self.frame_counter = 0

        self.start()

    def start(self):
        self.streaming_module.start_ffmpeg()
        
        self.detection_model.load_model()
        
        # Give streaming some time to initialize before trying to record
        time.sleep(2.0)  # Add a 2-second delay
        
        # Initialize video recording if enabled
        if self.record_stream:
            self.init_video_recording()

        # Separate threads for streaming and detection
        self.stream_thread = threading.Thread(target=self.process_stream)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
        self.detection_thread = threading.Thread(target=self.process_detection)
        self.detection_thread.daemon = True
        self.detection_thread.start()

    # Comment out the gimbal control methods
    """
    def lockon_loop(self, msg):
        # Commented out tracking functionality
        pass

    def spin_loop(self, msg):
        while rclpy.ok() and msg.data == True:
            frame = self.streaming_module.retrieve_image()

            cv_image = self.detection_model.yolo_detect(frame)
            frame = cv2.resize(cv_image, (640, 512))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            if frame is None:
                self.get_logger().error("No current image")
                continue

            bounding_boxes = Detection.yolo_detect(frame)
            
            # Publish detection centroids
            if bounding_boxes is not None and len(bounding_boxes) > 0:
                detection_array = Detection2DArray()
                for box in bounding_boxes:
                    # box is expected to be in format [x1, y1, x2, y2, confidence, class_id]
                    x1, y1, x2, y2 = box[0:4]
                    # Calculate centroid
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    detection = Detection2D()
                    detection.bbox.center.position.x = float(center_x)
                    detection.bbox.center.position.y = float(center_y)
                    detection.bbox.center.position.z = 0.0
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    
                    detection_array.detections.append(detection)
                
                self.detection_pub.publish(detection_array)
                self.get_logger().info(f"Published {len(detection_array.detections)} centroids")
            
            current_yaw = self.gimbal_pub.get_last_msg().x
            
            if(abs(current_yaw) > 120):
                self.direction *= -1.0
    """

    def init_video_recording(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"stream_{timestamp}.mkv"
        filepath = os.path.join(self.record_path, filename)
        
        # Ensure directory exists
        os.makedirs(self.record_path, exist_ok=True)
        
        # Try multiple times to get the first frame
        max_attempts = 5
        for attempt in range(max_attempts):
            self.get_logger().info(f"Trying to get first frame for recording (attempt {attempt+1}/{max_attempts})")
            frame = self.streaming_module.retrieve_image()
            if frame is not None:
                height, width = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Use XVID codec
                self.video_writer = cv2.VideoWriter(filepath, fourcc, 30.0, (width, height))
                self.get_logger().info(f"Recording stream to {filepath}")
                # Store this frame as latest_frame so we don't lose it
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                return
            self.get_logger().warn("Failed to get frame, retrying...")
            time.sleep(1.0)  # Wait a second before trying again
        
        self.get_logger().error("Could not initialize video recording - no frame received after multiple attempts")

    def process_stream(self):
        # Increase FPS limit for lower latency
        fps_limit = 30  # Increased from 10 to 30
        frame_time = 1.0 / fps_limit
        
        # Add timer for logging messages
        last_log_time = 0
        log_interval = 1.0  # Log message every 1 second
        
        while rclpy.ok():
            start_time = time.time()
            
            frame = self.streaming_module.retrieve_image()
            if frame is not None:
                # Store the frame for detection thread to use
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                
                # Record frame if enabled
                if self.record_stream and self.video_writer is not None:
                    self.video_writer.write(frame)
                
                # Only publish every Nth frame based on frame_skip parameter
                self.frame_counter += 1
                if self.frame_counter >= self.frame_skip:
                    self.frame_counter = 0
                    
                    # Reduce resolution further for faster transmission
                    resized_frame = cv2.resize(frame, (256, 256))  # Reduced from 256x256
                    
                    # Publish image without waiting for detection
                    try:
                        img_msg = self.bridge.cv2_to_imgmsg(resized_frame, "bgr8")
                        # Add timestamp to help with synchronization
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        self.image_pub.publish(img_msg)
                        
                        # Log message about publishing images (once per second)
                        current_time = time.time()
                        if current_time - last_log_time >= log_interval:
                            self.get_logger().info("Publishing images")
                            last_log_time = current_time
                            
                    except Exception as e:
                        self.get_logger().error(f"Error publishing image: {str(e)}")
            
            # Maintain target FPS
            elapsed = time.time() - start_time
            sleep_time = max(0, frame_time - elapsed)
            time.sleep(sleep_time)

    def process_detection(self):
        # Run detection at a lower rate to avoid overloading the system
        detection_fps = 5  # Run detection at 5 FPS
        detection_time = 1.0 / detection_fps
        
        while rclpy.ok():
            start_time = time.time()
            
            # Get the latest frame for detection
            frame_for_detection = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame_for_detection = self.latest_frame.copy()
            
            if frame_for_detection is not None:
                # Run detection on a separate thread
                detection_array = self.detection_model.yolo_detect(frame_for_detection)


                # TODO:
                # 
                
                if detection_array and len(detection_array.detections) > 0:
                    # Add timestamp for synchronization
                    detection_array.header.stamp = self.get_clock().now().to_msg()
                    self.detection_pub.publish(detection_array)
            
            # Sleep to maintain detection FPS
            elapsed = time.time() - start_time
            sleep_time = max(0, detection_time - elapsed)
            time.sleep(sleep_time)


    # def gps_estimator(self, XXX):
    #     pass

    # TODO: Publish Detection2DArray
    # 

def main(args=None):
    rclpy.init(args=args)
    try:
        node = IntegratedNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure video writer is released properly
        if hasattr(node, 'video_writer') and node.video_writer is not None:
            node.video_writer.release()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 