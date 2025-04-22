import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Header
import numpy as np
from staticmap import StaticMap, CircleMarker # Use staticmap for map generation and plotting
from PIL import Image as PILImage
from cv_bridge import CvBridge # To convert between OpenCV/PIL images and ROS Image messages
import cv2 # OpenCV for image conversion
import math
from geopy.distance import geodesic # For accurate ground distance calculation
import sys
import time

# Constants
EARTH_RADIUS_METERS = 6371000.0
IMAGE_WIDTH = 1024 # Pixels
IMAGE_HEIGHT = 1024 # Pixels
TARGET_GROUND_SIDE_METERS = 40.0 # Desired map coverage
# Google Maps Satellite Tile URL (Ensure compliance with terms of service)
SAT_TILE_URL = 'https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}'
# Alternative: OpenStreetMap (No satellite)
# SAT_TILE_URL = 'https://a.tile.openstreetmap.org/{z}/{x}/{y}.png'

class AtakSimulatorNode(Node):
    """
    ROS2 Node to simulate an ATAK-like display.
    - Takes initial GPS points.
    - Displays them on a satellite map centered at the centroid.
    - Subscribes to target GPS points and displays them.
    - Calculates distance between new targets and closest initial point.
    - Publishes the map image.
    """
    def __init__(self):
        super().__init__('atak_simulator_node')

        # --- State Variables ---
        self.blue_points = [] # List of (lat, lon) tuples for initial points
        self.target_points = {} # Dict: {frame_id: (lat, lon)} for subscribed points
        self.map_object = None # Holds the StaticMap instance
        self.current_pil_image = None # Holds the rendered PIL Image
        self.bridge = CvBridge() # For ROS Image message conversion
        self.centroid = None # (lat, lon) of initial points
        self.zoom_level = 18 # Default zoom, recalculated later
        self.last_publish_time = 0
        self.publish_rate_limit = 1.0 # Seconds - limit publish rate

        # --- Get Initial Coordinates ---
        self.get_initial_coordinates()

        if not self.blue_points:
            self.get_logger().error("No initial GPS coordinates provided. Map generation disabled.")
            # Node will still run to potentially subscribe, but won't publish maps initially.
        else:
            # --- Generate Initial Map ---
            self.generate_initial_map()

        # --- ROS Publishers/Subscribers ---
        self.image_publisher = self.create_publisher(Image, '/ground/log/atak_image', 10)
        self.target_subscriber = self.create_subscription(
            NavSatFix,
            '/target_gps_list',
            self.target_callback,
            10) # QoS profile depth 10

        # Publish initial map if generated
        if self.current_pil_image:
            self.publish_map("Initial map generated")

        self.get_logger().info("ATAK Simulator Node initialized.")
        if self.blue_points:
             self.get_logger().info(f"Centroid: {self.centroid}, Zoom: {self.zoom_level}")


    def get_initial_coordinates(self):
        """ Prompts user for initial GPS coordinates via command line. """
        self.get_logger().info("--- Enter Initial GPS Coordinates ---")
        print("Enter latitude and longitude (decimal degrees), separated by space.")
        print("Example: 40.443 -79.945")
        while True:
            try:
                line = input("Lat Lon (or press Enter to finish): ").strip()
                if not line:
                    if not self.blue_points:
                        print("No coordinates entered.")
                    else:
                        print(f"Finished collecting {len(self.blue_points)} points.")
                    break

                parts = line.split()
                if len(parts) != 2:
                    print("Invalid input. Please enter latitude and longitude separated by space.")
                    continue

                lat = float(parts[0])
                lon = float(parts[1])

                # Basic validation
                if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                     print("Invalid latitude (-90 to 90) or longitude (-180 to 180) values.")
                     continue

                self.blue_points.append((lat, lon))
                print(f"Added: ({lat}, {lon}). Points so far: {len(self.blue_points)}")

            except ValueError:
                print("Invalid input. Please enter numeric values for latitude and longitude.")
            except EOFError: # Handle Ctrl+D
                 print("\nInput terminated.")
                 break

        self.get_logger().info(f"Collected {len(self.blue_points)} initial coordinates.")

    def calculate_zoom_level(self, center_lat):
        """ Calculates the zoom level needed for ~TARGET_GROUND_SIDE_METERS map width. """
        if not (-90 < center_lat < 90):
             self.get_logger().warn("Invalid latitude for zoom calculation, using default.")
             return 18 # Default zoom

        try:
            # Formula derived from map tile scaling properties
            # Resolution (m/px) = (2 * pi * R * cos(lat)) / (tile_size * 2^zoom)
            # We want: IMAGE_WIDTH * Resolution = TARGET_GROUND_SIDE_METERS
            # IMAGE_WIDTH * (2 * pi * R * cos(lat)) / (256 * 2^zoom) = TARGET_GROUND_SIDE_METERS
            # 2^zoom = IMAGE_WIDTH * (2 * pi * R * cos(lat)) / (256 * TARGET_GROUND_SIDE_METERS)

            cos_lat = math.cos(math.radians(center_lat))
            # Avoid issues near poles or with zero cos_lat
            if cos_lat < 1e-9:
                cos_lat = 1e-9 # Use a small value instead of zero

            numerator = IMAGE_WIDTH * 2 * math.pi * EARTH_RADIUS_METERS * cos_lat
            denominator = 256.0 * TARGET_GROUND_SIDE_METERS
            if denominator <= 0:
                 self.get_logger().warn("Invalid target side length, using default zoom.")
                 return 18

            zoom_val = numerator / denominator
            if zoom_val <= 0:
                 self.get_logger().warn("Non-positive value in zoom calculation, using default zoom.")
                 return 18

            zoom_float = math.log2(zoom_val)
            # staticmap uses integer zoom levels
            zoom_int = int(round(zoom_float))
            # Clamp zoom level to a reasonable range (e.g., 1 to 22, typical for web maps)
            zoom_int = max(1, min(22, zoom_int))
            self.get_logger().info(f"Calculated zoom level {zoom_int} for {TARGET_GROUND_SIDE_METERS}m view at lat {center_lat:.4f}")
            return zoom_int

        except Exception as e:
            self.get_logger().error(f"Error calculating zoom level: {e}. Using default 18.")
            return 18

    def generate_initial_map(self):
        """ Calculates centroid, determines zoom, fetches map, and plots blue points. """
        if not self.blue_points:
            self.get_logger().error("Cannot generate map: No initial coordinates.")
            return

        # Calculate centroid
        lats = [p[0] for p in self.blue_points]
        lons = [p[1] for p in self.blue_points]
        self.centroid = (np.mean(lats), np.mean(lons))

        # Calculate zoom level based on centroid latitude
        self.zoom_level = self.calculate_zoom_level(self.centroid[0])

        # Create map object using staticmap
        self.map_object = StaticMap(IMAGE_WIDTH, IMAGE_HEIGHT, url_template=SAT_TILE_URL)

        # Add blue markers for initial points
        for lat, lon in self.blue_points:
            # staticmap uses (longitude, latitude) for coordinates
            marker = CircleMarker((lon, lat), 'blue', 8) # 8 pixel radius blue circle
            self.map_object.add_marker(marker)

        # Render the map image centered on the centroid
        try:
            # staticmap uses (longitude, latitude) for center
            self.current_pil_image = self.map_object.render(zoom=self.zoom_level, center=(self.centroid[1], self.centroid[0]))
            self.get_logger().info(f"Initial map rendered with {len(self.blue_points)} blue points.")
        except Exception as e:
            self.get_logger().error(f"Failed to render initial map: {e}")
            self.current_pil_image = None
            self.map_object = None # Invalidate map object if render fails

    def target_callback(self, msg: NavSatFix):
        """ Handles incoming target GPS coordinates. """
        if self.map_object is None or self.centroid is None:
             # self.get_logger().warn("Received target GPS, but map is not initialized. Skipping.")
             return # Don't process if map isn't ready

        lat = msg.latitude
        lon = msg.longitude
        frame_id = msg.header.frame_id
        if not frame_id:
            frame_id = "unknown_target" # Assign a default if empty

        # --- Input Validation ---
        if msg.status.status == msg.status.STATUS_NO_FIX:
             # self.get_logger().debug(f"Received NavSatFix with no fix for frame_id '{frame_id}'. Skipping.")
             return
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
             self.get_logger().warn(f"Received invalid lat/lon ({lat}, {lon}) for frame_id '{frame_id}'. Skipping.")
             return

        new_coord = (lat, lon)
        current_coord = self.target_points.get(frame_id)

        # --- Check if Update Needed ---
        # Update if frame_id is new or coordinates changed significantly
        TOLERANCE_DEG = 1e-7 # Small tolerance for floating point comparison
        needs_update = False
        if frame_id not in self.target_points or \
           abs(current_coord[0] - new_coord[0]) > TOLERANCE_DEG or \
           abs(current_coord[1] - new_coord[1]) > TOLERANCE_DEG:
            needs_update = True

        if needs_update:
            # self.get_logger().info(f"Updating target: ID='{frame_id}', Pos=({lat:.6f}, {lon:.6f})")
            self.target_points[frame_id] = new_coord

            # --- Recalculate Closest Distance ---
            closest_blue_dist_m = float('inf')
            closest_blue_point = None
            if self.blue_points:
                for b_point in self.blue_points:
                    try:
                        dist_m = geodesic(new_coord, b_point).m
                        if dist_m < closest_blue_dist_m:
                            closest_blue_dist_m = dist_m
                            closest_blue_point = b_point
                    except ValueError as e:
                        self.get_logger().warn(f"Could not calculate distance for point {new_coord} and {b_point}: {e}")


                if closest_blue_point:
                    self.get_logger().info(f"Target '{frame_id}' ({lat:.5f}, {lon:.5f}): Closest blue point ({closest_blue_point[0]:.5f}, {closest_blue_point[1]:.5f}), Distance: {closest_blue_dist_m:.2f} m")
                else:
                     self.get_logger().warn(f"Target '{frame_id}' ({lat:.5f}, {lon:.5f}): Could not find closest blue point.")

            # --- Re-render the Map ---
            # Create a new map object instance to redraw all points cleanly
            self.map_object = StaticMap(IMAGE_WIDTH, IMAGE_HEIGHT, url_template=SAT_TILE_URL)

            # Add blue markers
            for b_lat, b_lon in self.blue_points:
                marker = CircleMarker((b_lon, b_lat), 'blue', 8)
                self.map_object.add_marker(marker)

            # Add red markers (all current ones)
            for r_lat, r_lon in self.target_points.values():
                marker = CircleMarker((r_lon, r_lat), 'red', 8)
                self.map_object.add_marker(marker)

            try:
                 # Render map with updated points, centered on original centroid
                 self.current_pil_image = self.map_object.render(zoom=self.zoom_level, center=(self.centroid[1], self.centroid[0]))
                 # self.get_logger().info(f"Map re-rendered with {len(self.target_points)} red points.")

                 # --- Publish Updated Map (Rate Limited) ---
                 now = time.time()
                 if now - self.last_publish_time > self.publish_rate_limit:
                     self.publish_map(f"Map updated with target {frame_id}")
                     self.last_publish_time = now
                 # else:
                 #     self.get_logger().debug("Map update publish skipped due to rate limit.")


            except Exception as e:
                 self.get_logger().error(f"Failed to re-render map after target update: {e}")
                 self.current_pil_image = None
                 # Invalidate map object? Or keep old one? Let's reset to be safe.
                 self.map_object = StaticMap(IMAGE_WIDTH, IMAGE_HEIGHT, url_template=SAT_TILE_URL)


    def publish_map(self, reason=""):
        """ Converts the current PIL image to a ROS Image message and publishes it. """
        if self.current_pil_image is None:
            # self.get_logger().warn(f"Attempted to publish map ({reason}), but no valid image exists.")
            return

        try:
            # Convert PIL Image (RGB) to NumPy array
            numpy_image = np.array(self.current_pil_image)

            # Convert RGB to BGR for OpenCV compatibility if needed (cv_bridge expects BGR by default for 'bgr8')
            cv_image_bgr = cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)

            # Create ROS Image message using CvBridge
            ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image_bgr, encoding="bgr8")

            # Set header timestamp and frame_id
            ros_image_msg.header = Header()
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "atak_map_image" # Descriptive frame ID

            self.image_publisher.publish(ros_image_msg)
            self.get_logger().info(f"Published map image. Reason: {reason}")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error converting image: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to convert or publish map image: {e}")


def main(args=None):
    rclpy.init(args=args)
    atak_simulator_node = None # Initialize to None
    try:
        atak_simulator_node = AtakSimulatorNode()
        # Check again if points were provided after initialization attempt
        if not atak_simulator_node.blue_points and rclpy.ok():
             atak_simulator_node.get_logger().warn("Node running but map features disabled due to no initial points.")
        # Keep spinning regardless to allow subscription
        if rclpy.ok():
            rclpy.spin(atak_simulator_node)
    except KeyboardInterrupt:
        print(" Keyboard interrupt detected, shutting down.")
    except Exception as e:
         # Use basic print if logger failed during init
         if atak_simulator_node:
             atak_simulator_node.get_logger().fatal(f"Critical error during node execution: {e}", exc_info=True)
         else:
             print(f"Critical error creating node or during spin: {e}")
    finally:
        # Cleanup
        if atak_simulator_node:
            atak_simulator_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ATAK Simulator node shutdown complete.")

if __name__ == '__main__':
    main()