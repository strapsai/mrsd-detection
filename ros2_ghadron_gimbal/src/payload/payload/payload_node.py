import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from enum import Enum
import math
import time
from payload.track import track

class GimbalState(Enum):
    MAPPING = 0
    MANUAL_CONTROL = 1
    LOCK_ON = 2
    SPIN = 3

class PayloadNode(Node):
    def __init__(self):
        super().__init__("payload_node")
        self.get_logger().info("Payload node started")

        # Declare parameters
        self.declare_parameter("payload_id", "default_payload")
        self.declare_parameter("payload_type", "default_type")

        # Get parameters
        self.payload_id = self.get_parameter("payload_id").get_parameter_value().string_value
        self.payload_type = self.get_parameter("payload_type").get_parameter_value().string_value

        # State machine variables
        self.current_state = GimbalState.MANUAL_CONTROL
        self.current_angle = Vector3()
        self.target_angle = Vector3()
        self.bounding_box_position = None
        self.detection_found = False
        self.spin_increment = 5.0  # Degrees
        self.spin_current_angle = 0.0

        # Publishers and subscribers
        self.gimbal_angle_pub = self.create_publisher(Vector3, '/gimbal_angles', 10)
        self.gimbal_angle_sub = self.create_subscription(
            Vector3, 
            '/gimbal_angles', 
            self.gimbal_angle_callback, 
            10)
        self.command_sub = self.create_subscription(
            String,
            '/gimbal_command',
            self.command_callback,
            10)
        self.bbox_sub = self.create_subscription(
            Vector3,  # Assuming bounding box center is published as Vector3 (x, y, 0)
            '/detection/bbox_center',
            self.bbox_callback,
            10)
        self.detection_sub = self.create_subscription(
            String,
            '/detection/status',
            self.detection_callback,
            10)
        # Timers for different states
        self.state_timer = self.create_timer(0.1, self.state_machine_callback)

        # Camera frame dimensions (example values, adjust based on your camera)
        self.frame_width = 640  
        self.frame_height = 480

    def state_machine_callback(self):
        """Main state machine logic"""
        if self.current_state == GimbalState.MAPPING:
            self.mapping_mode()
        elif self.current_state == GimbalState.MANUAL_CONTROL:
            self.manual_control_mode()
        elif self.current_state == GimbalState.LOCK_ON:
            self.lock_on_mode()
        elif self.current_state == GimbalState.SPIN:
            self.spin_mode()
        
        self.get_logger().debug(f"Current state: {self.current_state.name}")
        self.publish_gimbal_angles()

    def mapping_mode(self):
        """Define mapping mode behavior"""
        # Set gimbal to point straight down
        self.target_angle.x = 90.0  # Pitch down 90 degrees
        self.target_angle.y = 0.0
        self.target_angle.z = 0.0
        self.get_logger().info("Mapping mode active: Pointing straight down")

    def manual_control_mode(self):
        """Pass-through mode for manual gimbal control"""
        # In this mode, the gimbal simply listens to /gimbal_command
        self.get_logger().info("Manual control mode active")
        pass

    def lock_on_mode(self):
        """Lock on to detected object using bounding box"""
        if self.detection_found and self.bounding_box_position:
            # Use track function to get yaw and pitch offsets
            center_x = self.bounding_box_position.x
            center_y = self.bounding_box_position.y
            yaw_offset, pitch_offset = track((center_x, center_y), self.frame_width, self.frame_height)

            # Adjust target angles based on the offsets
            self.target_angle.z += yaw_offset
            self.target_angle.x += pitch_offset

            self.get_logger().info(f"Lock-on mode active - Yaw offset: {yaw_offset}, Pitch offset: {pitch_offset}")
        else:
            self.get_logger().warn("No detection found or bounding box position is None")

    def spin_mode(self):
        """Spin the gimbal continuously, panning yaw from -120 to 120 degrees with pitch at -70"""
        # Define the range for yaw and the fixed pitch
        min_yaw = -120.0
        max_yaw = 120.0
        fixed_pitch = -70.0

        # Update the yaw angle
        self.spin_current_angle += self.spin_increment

        # Reverse direction if we reach the min or max yaw
        if self.spin_current_angle > max_yaw:
            self.spin_increment *= -1
            self.spin_current_angle = max_yaw
        elif self.spin_current_angle < min_yaw:
            self.spin_increment *= -1
            self.spin_current_angle = min_yaw

        # Set the target angles
        self.target_angle.x = fixed_pitch  # Pitch
        self.target_angle.z = self.spin_current_angle  # Yaw

        self.get_logger().info(f"Spin mode active - Current yaw: {self.spin_current_angle}, Pitch: {fixed_pitch}")

    def publish_gimbal_angles(self):
        """Publishes the target gimbal angles"""
        self.gimbal_angle_pub.publish(self.target_angle)
        self.get_logger().debug(f"Publishing gimbal angles: pitch={self.target_angle.x}, roll={self.target_angle.y}, yaw={self.target_angle.z}")

    def gimbal_angle_callback(self, msg):
        """Updates current gimbal angle from subscribed topic"""
        """Updates current gimbal angle from subscribed topic"""
        self.current_angle = msg
        self.get_logger().debug(f"Current gimbal angle: pitch={msg.x}, roll={msg.y}, yaw={msg.z}")

    def command_callback(self, msg):
        """Processes commands from ground station"""
        command = msg.data.strip().upper()
        self.get_logger().info(f"Received command: {command}")
        
        if command == "MAPPING":
            self.transition_to_state(GimbalState.MAPPING)
        elif command == "MANUAL":
            self.transition_to_state(GimbalState.MANUAL_CONTROL)
        elif command == "LOCK_ON":
            self.transition_to_state(GimbalState.LOCK_ON)
        elif command == "SPIN":
            self.transition_to_state(GimbalState.SPIN)
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    def bbox_callback(self, msg):
        """Updates bounding box position"""
        self.bounding_box_position = msg
        self.get_logger().debug(f"Received bounding box position: x={msg.x}, y={msg.y}")

    def detection_callback(self, msg):
        """Updates detection status"""
        self.detection_found = msg.data.lower() == "detected"
        self.get_logger().debug(f"Detection status: {self.detection_found}")

    def transition_to_state(self, new_state):
        """Handle state transitions"""
        if new_state == self.current_state:
            return
            
        self.get_logger().info(f"Transitioning from {self.current_state.name} to {new_state.name}")
        
        # Exit actions for current state
        if self.current_state == GimbalState.SPIN:
            # Reset spin variables
            self.spin_current_angle = 0.0
        
        # Entry actions for new state
        if new_state == GimbalState.MAPPING:
            # Set gimbal to point straight down
            self.target_angle.x = 90.0  # Pitch down 90 degrees
            self.target_angle.y = 0.0  