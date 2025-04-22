import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Header
import numpy as np
from staticmap import StaticMap, CircleMarker, Line # Use staticmap for map generation and plotting
from PIL import Image as PILImage, ImageDraw, ImageFont # Pillow for drawing text
from cv_bridge import CvBridge, CvBridgeError # To convert between OpenCV/PIL images and ROS Image messages
import cv2 # OpenCV for image conversion
import math
from geopy.distance import geodesic # For accurate ground distance calculation
import sys
import time
import os # Needed for font path finding if necessary

# Constants
EARTH_RADIUS_METERS = 6371000.0
IMAGE_WIDTH = 1024 # Pixels
IMAGE_HEIGHT = 1024 # Pixels
TARGET_GROUND_SIDE_METERS = 50.0 # Desired map coverage
# Google Maps Satellite Tile URL (Ensure compliance with terms of service)
SAT_TILE_URL = 'https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}'
# Alternative: OpenStreetMap (No satellite)
# SAT_TILE_URL = 'https://a.tile.openstreetmap.org/{z}/{x}/{y}.png'

# --- Visual Style Constants ---
BLUE_MARKER_COLOR = 'blue'
RED_MARKER_COLOR = 'red'
MARKER_RADIUS = 12 # Increased marker size
ANNOTATION_COLOR = 'yellow'
ANNOTATION_LINE_COLOR = 'yellow'
ANNOTATION_LINE_WIDTH = 3 # Increased line thickness
GEOFENCE_COLOR = 'cyan'
GEOFENCE_LINE_WIDTH = 2

# Attempt to load a default font, fallback possible
try:
    ANNOTATION_FONT_SIZE = 16 # Increased font size
    ANNOTATION_FONT = ImageFont.truetype("arial.ttf", ANNOTATION_FONT_SIZE)
except IOError:
    # Fallback for Windows if arial.ttf isn't directly found
    try:
        font_path = os.path.join(os.environ.get('WINDIR', 'C:\\Windows'), 'Fonts', 'arial.ttf')
        ANNOTATION_FONT = ImageFont.truetype(font_path, ANNOTATION_FONT_SIZE)
        print("INFO: Using Arial font found at:", font_path)
    except IOError:
        print("WARN: Arial font not found in common locations. Using default PIL font.")
        ANNOTATION_FONT = ImageFont.load_default() # Default font size might be small


def geo_to_pixel(lat, lon, center_lat, center_lon, zoom, width, height):
    """Converts geographic coordinates to pixel coordinates within the image."""
    # Calculate pixel coordinates of the center point on the world map (Mercator projection)
    center_x_world = (center_lon + 180.0) / 360.0 * (256 * 2**zoom)
    sin_center_lat = math.sin(math.radians(center_lat))
    # Clamp sin_center_lat to avoid math domain errors near poles
    sin_center_lat = max(min(sin_center_lat, 0.9999), -0.9999)
    center_y_world = (0.5 - math.log((1 + sin_center_lat) / (1 - sin_center_lat)) / (4 * math.pi)) * (256 * 2**zoom)

    # Calculate pixel coordinates of the target point on the world map
    target_x_world = (lon + 180.0) / 360.0 * (256 * 2**zoom)
    sin_target_lat = math.sin(math.radians(lat))
    # Clamp sin_target_lat
    sin_target_lat = max(min(sin_target_lat, 0.9999), -0.9999)
    target_y_world = (0.5 - math.log((1 + sin_target_lat) / (1 - sin_target_lat)) / (4 * math.pi)) * (256 * 2**zoom)

    # Calculate pixel coordinates relative to the center of the image
    pixel_x = (target_x_world - center_x_world) + width / 2.0
    pixel_y = (target_y_world - center_y_world) + height / 2.0

    # Return integers, clamping to image bounds just in case
    return (
        max(0, min(width - 1, int(round(pixel_x)))),
        max(0, min(height - 1, int(round(pixel_y))))
    )


class AtakSimulatorNode(Node):
    """
    ROS2 Node to simulate an ATAK-like display.
    - Takes geofence points (cyan polygon).
    - Takes initial GPS points (blue circles).
    - Displays them on a satellite map centered at the centroid of blue points.
    - Subscribes to target GPS points (red circles).
    - Calculates distance between new targets and closest initial point.
    - Draws a line and distance annotation on the map for the latest target.
    - Publishes the annotated map image.
    """
    def __init__(self):
        super().__init__('atak_simulator_node')

        # --- State Variables ---
        self.geofence_points = [] # List of (lat, lon) for geofence polygon
        self.blue_points = [] # List of (lat, lon) tuples for initial points
        self.target_points = {} # Dict: {frame_id: (lat, lon)} for subscribed points
        self.map_object = None # Holds the StaticMap instance
        self.current_pil_image = None # Holds the rendered PIL Image (potentially with annotations)
        self.bridge = CvBridge() # For ROS Image message conversion
        self.centroid = None # (lat, lon) of initial points
        self.zoom_level = 18 # Default zoom, recalculated later
        self.last_publish_time = 0
        self.publish_rate_limit = 0.5 # Seconds - allow faster updates for annotation
        self.last_annotation_details = None # Store details for drawing text post-render

        # --- Get Geofence Coordinates ---
        self.get_geofence_coordinates()

        # --- Get Initial Coordinates ---
        self.get_initial_coordinates()

        if not self.blue_points:
            self.get_logger().error("No initial GPS coordinates provided. Map generation disabled.")
        else:
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
        if self.geofence_points:
             self.get_logger().info(f"Geofence defined with {len(self.geofence_points)} points.")

    def _get_coordinates_from_user(self, prompt_title, point_type_name):
        """ Helper function to get a list of coordinates from user input. """
        points = []
        self.get_logger().info(f"--- {prompt_title} ---")
        print("Enter latitude and longitude (decimal degrees), separated by space.")
        print("Example: 40.443 -79.945")
        while True:
            try:
                line = input(f"{point_type_name} Lat Lon (or press Enter to finish): ").strip()
                if not line:
                    if not points:
                        print(f"No {point_type_name.lower()} coordinates entered.")
                    else:
                        print(f"Finished collecting {len(points)} {point_type_name.lower()} points.")
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

                points.append((lat, lon))
                print(f"Added {point_type_name} Point: ({lat}, {lon}). Points so far: {len(points)}")

            except ValueError:
                print("Invalid input. Please enter numeric values for latitude and longitude.")
            except EOFError: # Handle Ctrl+D
                 print("\nInput terminated.")
                 break
        self.get_logger().info(f"Collected {len(points)} {point_type_name.lower()} coordinates.")
        return points

    def get_geofence_coordinates(self):
        """ Prompts user for geofence GPS coordinates via command line. """
        self.geofence_points = self._get_coordinates_from_user("Enter Geofence Coordinates (in order)", "Geofence")

    def get_initial_coordinates(self):
        """ Prompts user for initial (blue) GPS coordinates via command line. """
        self.blue_points = self._get_coordinates_from_user("Enter Initial (Blue) GPS Coordinates", "Blue")

    def calculate_zoom_level(self, center_lat):
        """ Calculates the zoom level needed for ~TARGET_GROUND_SIDE_METERS map width. """
        if not (-90 < center_lat < 90):
             self.get_logger().warn("Invalid latitude for zoom calculation, using default.")
             return 18 # Default zoom

        try:
            cos_lat = math.cos(math.radians(center_lat))
            if cos_lat < 1e-9: cos_lat = 1e-9

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
            zoom_int = int(round(zoom_float))
            zoom_int = max(1, min(22, zoom_int))
            self.get_logger().info(f"Calculated zoom level {zoom_int} for {TARGET_GROUND_SIDE_METERS}m view at lat {center_lat:.4f}")
            return zoom_int

        except Exception as e:
            self.get_logger().error(f"Error calculating zoom level: {e}. Using default 18.")
            return 18

    def _add_map_elements(self):
        """ Helper to add geofence, blue points, and red points to the map object. """
        if self.map_object is None: return

        # Add Geofence Lines
        if len(self.geofence_points) >= 2:
            self.get_logger().debug(f"Drawing geofence with {len(self.geofence_points)} points.")
            # Convert to (lon, lat) for staticmap
            fence_coords_lonlat = [(p[1], p[0]) for p in self.geofence_points]
            # Draw lines between consecutive points
            for i in range(len(fence_coords_lonlat) - 1):
                line = Line([fence_coords_lonlat[i], fence_coords_lonlat[i+1]], GEOFENCE_COLOR, GEOFENCE_LINE_WIDTH)
                self.map_object.add_line(line)
            # Draw line connecting last to first if it's a polygon (3+ points)
            if len(fence_coords_lonlat) >= 3:
                line = Line([fence_coords_lonlat[-1], fence_coords_lonlat[0]], GEOFENCE_COLOR, GEOFENCE_LINE_WIDTH)
                self.map_object.add_line(line)

        # Add blue circle markers
        for lat, lon in self.blue_points:
            marker = CircleMarker((lon, lat), BLUE_MARKER_COLOR, MARKER_RADIUS)
            self.map_object.add_marker(marker)

        # Add red circle markers
        for r_lat, r_lon in self.target_points.values():
            marker = CircleMarker((r_lon, r_lat), RED_MARKER_COLOR, MARKER_RADIUS)
            self.map_object.add_marker(marker)

    def generate_initial_map(self):
        """ Calculates centroid, determines zoom, fetches map, adds elements. """
        if not self.blue_points:
            self.get_logger().error("Cannot generate map: No initial coordinates.")
            return

        lats = [p[0] for p in self.blue_points]
        lons = [p[1] for p in self.blue_points]
        self.centroid = (np.mean(lats), np.mean(lons))
        self.zoom_level = self.calculate_zoom_level(self.centroid[0])

        self.map_object = StaticMap(IMAGE_WIDTH, IMAGE_HEIGHT, url_template=SAT_TILE_URL)

        # Add geofence and blue points
        self._add_map_elements()

        try:
            self.current_pil_image = self.map_object.render(zoom=self.zoom_level, center=(self.centroid[1], self.centroid[0]))
            self.get_logger().info(f"Initial map rendered with {len(self.blue_points)} blue points.")
        except Exception as e:
            self.get_logger().error(f"Failed to render initial map: {e}")
            self.current_pil_image = None
            self.map_object = None

    def target_callback(self, msg: NavSatFix):
        """ Handles incoming target GPS coordinates, calculates distance, draws line/text, publishes map. """
        if self.centroid is None: # Need centroid even if map object failed initially
             # self.get_logger().warn("Received target GPS, but centroid is not calculated. Skipping.")
             return

        lat = msg.latitude
        lon = msg.longitude
        frame_id = msg.header.frame_id
        if not frame_id:
            frame_id = f"unknown_target_{int(time.time())}" # Assign a unique default if empty

        # --- Input Validation ---
        if msg.status.status == msg.status.STATUS_NO_FIX:
             # self.get_logger().debug(f"Received NavSatFix with no fix for frame_id '{frame_id}'. Skipping.")
             return
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
             self.get_logger().warn(f"Received invalid lat/lon ({lat}, {lon}) for frame_id '{frame_id}'. Skipping.")
             return

        new_coord = (lat, lon)
        current_coord = self.target_points.get(frame_id)
        self.last_annotation_details = None # Reset annotation for this update

        # --- Check if Update Needed ---
        TOLERANCE_DEG = 1e-7
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
                    dist_str = f"{closest_blue_dist_m:.1f} m"
                    self.get_logger().info(f"Target '{frame_id}' ({lat:.5f}, {lon:.5f}): Closest blue ({closest_blue_point[0]:.5f}, {closest_blue_point[1]:.5f}), Dist: {dist_str}")
                    # Store details for drawing line and text AFTER rendering
                    self.last_annotation_details = {
                        "red_coord": new_coord,
                        "blue_coord": closest_blue_point,
                        "dist_str": dist_str
                    }
                else:
                     self.get_logger().warn(f"Target '{frame_id}' ({lat:.5f}, {lon:.5f}): Could not find closest blue point.")

            # --- Re-render the Map ---
            self.map_object = StaticMap(IMAGE_WIDTH, IMAGE_HEIGHT, url_template=SAT_TILE_URL)

            # Add geofence, blue points, and ALL red points
            self._add_map_elements()

            # Add annotation line if closest blue point was found for the *current* target update
            if self.last_annotation_details:
                line_coords = [
                    (self.last_annotation_details["red_coord"][1], self.last_annotation_details["red_coord"][0]), # lon, lat
                    (self.last_annotation_details["blue_coord"][1], self.last_annotation_details["blue_coord"][0]) # lon, lat
                ]
                line = Line(line_coords, ANNOTATION_LINE_COLOR, ANNOTATION_LINE_WIDTH)
                self.map_object.add_line(line)

            try:
                 # Render map with updated points and potentially a line
                 self.current_pil_image = self.map_object.render(zoom=self.zoom_level, center=(self.centroid[1], self.centroid[0]))
                 # self.get_logger().info(f"Map re-rendered with {len(self.target_points)} red points.")

                 # --- Add Text Annotation using PIL ---
                 if self.last_annotation_details and self.current_pil_image:
                     try:
                         px_red, py_red = geo_to_pixel(
                             self.last_annotation_details["red_coord"][0], self.last_annotation_details["red_coord"][1],
                             self.centroid[0], self.centroid[1], self.zoom_level,
                             IMAGE_WIDTH, IMAGE_HEIGHT
                         )
                         px_blue, py_blue = geo_to_pixel(
                             self.last_annotation_details["blue_coord"][0], self.last_annotation_details["blue_coord"][1],
                             self.centroid[0], self.centroid[1], self.zoom_level,
                             IMAGE_WIDTH, IMAGE_HEIGHT
                         )

                         text_x = (px_red + px_blue) / 2 + 5
                         text_y = (py_red + py_blue) / 2 - 10

                         draw = ImageDraw.Draw(self.current_pil_image)
                         draw.text((text_x, text_y), self.last_annotation_details["dist_str"],
                                   fill=ANNOTATION_COLOR, font=ANNOTATION_FONT)
                         # self.get_logger().info(f"Added distance annotation '{self.last_annotation_details['dist_str']}' to image.")
                     except Exception as pil_e:
                         self.get_logger().error(f"Failed to draw text annotation: {pil_e}")


                 # --- Publish Updated Map (Rate Limited) ---
                 now = time.time()
                 # Publish immediately if annotation was added, otherwise respect rate limit
                 if self.last_annotation_details or (now - self.last_publish_time > self.publish_rate_limit):
                     self.publish_map(f"Map updated with target {frame_id}")
                     self.last_publish_time = now
                 # else:
                 #     self.get_logger().debug("Map update publish skipped due to rate limit.")


            except Exception as e:
                 self.get_logger().error(f"Failed to re-render map or add annotation after target update: {e}")
                 self.current_pil_image = None
                 # Reset map object? Maybe not, keep state but clear image.
                 # self.map_object = StaticMap(IMAGE_WIDTH, IMAGE_HEIGHT, url_template=SAT_TILE_URL)


    def publish_map(self, reason=""):
        """ Converts the current PIL image to a ROS Image message and publishes it. """
        if self.current_pil_image is None:
            # self.get_logger().warn(f"Attempted to publish map ({reason}), but no valid image exists.")
            return

        try:
            # Convert PIL Image (RGB) to NumPy array
            numpy_image = np.array(self.current_pil_image.convert('RGB')) # Ensure RGB

            # Convert RGB to BGR for OpenCV compatibility (cv_bridge expects BGR for 'bgr8')
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
         if atak_simulator_node and atak_simulator_node.get_logger().is_enabled_for(rclpy.logging.LoggingSeverity.FATAL):
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