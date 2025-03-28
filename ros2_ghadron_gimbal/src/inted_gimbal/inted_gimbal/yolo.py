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

class Detection:
    model = None

    def load_model(self):
        print('Loading YOLOv11x-pose model...')
        model_path = "models/yolo11x-pose.pt"
        if not os.path.exists(model_path):
            print("Model not found!")
            return False
        self.model = YOLO(model_path)
        return True

    def yolo_detect(self, image):
        # Process at 1280x720 resolution
        cv_image = cv2.resize(image, (1280, 720)) if image.shape[0] != 720 or image.shape[1] != 1280 else image
        
        # Run the model with appropriate parameters for 1280x720, only detect person class (0)
        result = self.model(cv_image, 
                          conf=0.5,        # Confidence threshold
                          iou=0.45,        # NMS IOU threshold
                          max_det=10,      # Maximum detections
                          classes=[0],     # Only detect person class
                          verbose=False)   # Suppress verbose output

        # Detection array
        detection_array = Detection2DArray()

        if len(result) > 0:
            boxes = result[0].boxes
            
            # No need to filter by class since we specified classes=[0] above
            person_boxes = []
            for box in boxes:
                confidence = float(box.conf[0].cpu().numpy())
                if confidence > 0.4:
                    person_boxes.append(box)
            
            if person_boxes:
                # Sort person boxes by confidence (highest first)
                person_boxes.sort(key=lambda box: float(box.conf[0].cpu().numpy()), reverse=True)
                
                # Take top 5 persons (or fewer if less than 5 detected)
                top_persons = person_boxes[:min(5, len(person_boxes))]
                
                # Process each of the top persons
                for box in top_persons:
                    x1, y1, x2, y2 = map(float, box.xyxy[0].cpu().numpy())
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    confidence = float(box.conf[0].cpu().numpy())
                    
                    detection = Detection2D()
                    detection.bbox.center.position.x = center_x
                    detection.bbox.center.position.y = center_y
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    detection.bbox.center.theta = confidence
                    
                    # Always set id to "0" for person class
                    detection.id = "0"
                    
                    detection_array.detections.append(detection)
                    
        return detection_array
    

    # def Reid(self, detection_array: Detection2DArray):
    # subscribe to GPS topic
    #     pass

