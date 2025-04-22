import numpy as np
from staticmap import StaticMap, CircleMarker, Line # Use staticmap for map generation and plotting
from PIL import Image as PILImage, ImageDraw, ImageFont # Pillow for drawing text
import math
from geopy.distance import geodesic # For accurate ground distance calculation
import os
import time

# Constants
EARTH_RADIUS_METERS = 6371000.0
IMAGE_WIDTH = 1024 # Pixels
IMAGE_HEIGHT = 1024 # Pixels
TARGET_GROUND_SIDE_METERS = 30.0 # Desired map coverage
# Google Maps Satellite Tile URL (Ensure compliance with terms of service)
SAT_TILE_URL = 'https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}'
# Alternative: OpenStreetMap (No satellite)
# SAT_TILE_URL = 'https://a.tile.openstreetmap.org/{z}/{x}/{y}.png'
OUTPUT_DIR = "atak_sim_output" # Directory to save map images
# Text Annotation Settings
ANNOTATION_COLOR = 'yellow'
LINE_COLOR = 'yellow'
LINE_WIDTH = 2
# Attempt to load a default font, fallback possible
try:
    ANNOTATION_FONT = ImageFont.truetype("arial.ttf", 14)
except IOError:
    print("WARN: Arial font not found. Using default PIL font.")
    ANNOTATION_FONT = ImageFont.load_default()


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


class AtakSimulatorLocal:
    """
    Non-ROS simulator for ATAK-like map display.
    - Takes initial GPS points (blue circles).
    - Displays them on a satellite map centered at the centroid.
    - Allows adding target points (red circles) interactively.
    - Calculates distance between new targets and closest initial point.
    - Draws a line and distance annotation on the map.
    - Saves the map image locally.
    """
    def __init__(self):
        # --- State Variables ---
        self.blue_points = [] # List of (lat, lon) tuples for initial points
        self.target_points = {} # Dict: {target_id: (lat, lon)} for added points
        self.map_object = None # Holds the StaticMap instance
        self.current_pil_image = None # Holds the rendered PIL Image (potentially with annotations)
        self.centroid = None # (lat, lon) of initial points
        self.zoom_level = 18 # Default zoom, recalculated later
        self.target_counter = 0 # Simple counter for target IDs
        self.last_annotation_details = None # Store details for drawing text post-render

        # --- Setup Output Directory ---
        if not os.path.exists(OUTPUT_DIR):
            os.makedirs(OUTPUT_DIR)
            print(f"Created output directory: {OUTPUT_DIR}")

        # --- Get Initial Coordinates ---
        self.get_initial_coordinates()

        if not self.blue_points:
            print("ERROR: No initial GPS coordinates provided. Cannot generate map.")
            return # Exit if no blue points

        # --- Generate Initial Map ---
        self.generate_initial_map()
        if self.current_pil_image:
            self.save_map("initial_map")
            print(f"Initial map saved as initial_map.png in {OUTPUT_DIR}/")

        print("\n--- ATAK Simulator Local Initialized ---")
        if self.blue_points:
             print(f"Centroid: {self.centroid}, Zoom: {self.zoom_level}")

    def get_initial_coordinates(self):
        """ Prompts user for initial GPS coordinates via command line. """
        print("\n--- Enter Initial (Blue) GPS Coordinates ---")
        print("Enter latitude and longitude (decimal degrees), separated by space.")
        print("Example: 40.443 -79.945")
        while True:
            try:
                line = input("Lat Lon (or press Enter to finish): ").strip()
                if not line:
                    if not self.blue_points:
                        print("No coordinates entered.")
                    else:
                        print(f"Finished collecting {len(self.blue_points)} blue points.")
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
                print(f"Added Blue Point: ({lat}, {lon}). Points so far: {len(self.blue_points)}")

            except ValueError:
                print("Invalid input. Please enter numeric values for latitude and longitude.")
            except EOFError: # Handle Ctrl+D
                 print("\nInput terminated.")
                 break

        print(f"Collected {len(self.blue_points)} initial coordinates.")

    def calculate_zoom_level(self, center_lat):
        """ Calculates the zoom level needed for ~TARGET_GROUND_SIDE_METERS map width. """
        if not (-90 < center_lat < 90):
             print("WARN: Invalid latitude for zoom calculation, using default.")
             return 18 # Default zoom

        try:
            cos_lat = math.cos(math.radians(center_lat))
            if cos_lat < 1e-9: cos_lat = 1e-9 # Avoid issues near poles

            numerator = IMAGE_WIDTH * 2 * math.pi * EARTH_RADIUS_METERS * cos_lat
            denominator = 256.0 * TARGET_GROUND_SIDE_METERS
            if denominator <= 0:
                 print("WARN: Invalid target side length, using default zoom.")
                 return 18

            zoom_val = numerator / denominator
            if zoom_val <= 0:
                 print("WARN: Non-positive value in zoom calculation, using default zoom.")
                 return 18

            zoom_float = math.log2(zoom_val)
            zoom_int = int(round(zoom_float))
            zoom_int = max(1, min(22, zoom_int)) # Clamp zoom level
            print(f"INFO: Calculated zoom level {zoom_int} for {TARGET_GROUND_SIDE_METERS}m view at lat {center_lat:.4f}")
            return zoom_int

        except Exception as e:
            print(f"ERROR: Error calculating zoom level: {e}. Using default 18.")
            return 18

    def generate_initial_map(self):
        """ Calculates centroid, determines zoom, fetches map, and plots blue points. """
        if not self.blue_points:
            print("ERROR: Cannot generate map: No initial coordinates.")
            return

        lats = [p[0] for p in self.blue_points]
        lons = [p[1] for p in self.blue_points]
        self.centroid = (np.mean(lats), np.mean(lons))
        self.zoom_level = self.calculate_zoom_level(self.centroid[0])

        self.map_object = StaticMap(IMAGE_WIDTH, IMAGE_HEIGHT, url_template=SAT_TILE_URL)

        # Add blue circle markers
        for lat, lon in self.blue_points:
            # staticmap uses (longitude, latitude)
            marker = CircleMarker((lon, lat), 'blue', 8) # 8 pixel radius blue circle
            self.map_object.add_marker(marker)

        try:
            # Render the map image centered on the centroid
            # staticmap uses (longitude, latitude) for center
            self.current_pil_image = self.map_object.render(zoom=self.zoom_level, center=(self.centroid[1], self.centroid[0]))
            print(f"INFO: Initial map rendered with {len(self.blue_points)} blue points.")
        except Exception as e:
            print(f"ERROR: Failed to render initial map: {e}")
            self.current_pil_image = None
            self.map_object = None

    def add_target_point(self, lat, lon):
        """ Adds a red target point, calculates distance, draws line/text, and re-renders the map. """
        if self.map_object is None or self.centroid is None:
             print("WARN: Cannot add target point, map is not initialized.")
             return

        # --- Input Validation ---
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
             print(f"WARN: Received invalid lat/lon ({lat}, {lon}). Skipping.")
             return

        new_coord = (lat, lon)
        target_id = f"target_{self.target_counter}"
        self.target_counter += 1
        self.target_points[target_id] = new_coord
        print(f"\nINFO: Adding Target Point: ID='{target_id}', Pos=({lat:.6f}, {lon:.6f})")

        # --- Calculate Closest Distance ---
        closest_blue_dist_m = float('inf')
        closest_blue_point = None
        annotation_details = None # Reset annotation details

        if self.blue_points:
            for b_point in self.blue_points:
                try:
                    # Use geodesic for accurate ground distance
                    dist_m = geodesic(new_coord, b_point).m
                    if dist_m < closest_blue_dist_m:
                        closest_blue_dist_m = dist_m
                        closest_blue_point = b_point
                except ValueError as e:
                    print(f"WARN: Could not calculate distance for point {new_coord} and {b_point}: {e}")

            if closest_blue_point:
                dist_str = f"{closest_blue_dist_m:.1f} m"
                print(f"  -> Closest Blue Point: ({closest_blue_point[0]:.5f}, {closest_blue_point[1]:.5f})")
                print(f"  -> Ground Distance: {dist_str}")
                # Store details needed for drawing line and text AFTER rendering
                annotation_details = {
                    "red_coord": new_coord,
                    "blue_coord": closest_blue_point,
                    "dist_str": dist_str
                }
            else:
                 print(f"WARN: Could not find closest blue point for target '{target_id}'.")

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

        # Add line if closest blue point was found
        if annotation_details:
            line_coords = [
                (annotation_details["red_coord"][1], annotation_details["red_coord"][0]), # lon, lat
                (annotation_details["blue_coord"][1], annotation_details["blue_coord"][0]) # lon, lat
            ]
            line = Line(line_coords, LINE_COLOR, LINE_WIDTH)
            self.map_object.add_line(line)

        try:
             # Render map with updated points and line, centered on original centroid
             self.current_pil_image = self.map_object.render(zoom=self.zoom_level, center=(self.centroid[1], self.centroid[0]))
             print(f"INFO: Map re-rendered with {len(self.target_points)} red points.")

             # --- Add Text Annotation using PIL ---
             if annotation_details and self.current_pil_image:
                 # Calculate pixel coordinates for text placement
                 px_red, py_red = geo_to_pixel(
                     annotation_details["red_coord"][0], annotation_details["red_coord"][1],
                     self.centroid[0], self.centroid[1], self.zoom_level,
                     IMAGE_WIDTH, IMAGE_HEIGHT
                 )
                 px_blue, py_blue = geo_to_pixel(
                     annotation_details["blue_coord"][0], annotation_details["blue_coord"][1],
                     self.centroid[0], self.centroid[1], self.zoom_level,
                     IMAGE_WIDTH, IMAGE_HEIGHT
                 )

                 # Calculate midpoint for text anchor (slightly offset)
                 text_x = (px_red + px_blue) / 2 + 5
                 text_y = (py_red + py_blue) / 2 - 10 # Offset slightly above the line midpoint

                 # Draw the text onto the PIL image
                 draw = ImageDraw.Draw(self.current_pil_image)
                 draw.text((text_x, text_y), annotation_details["dist_str"],
                           fill=ANNOTATION_COLOR, font=ANNOTATION_FONT)
                 print(f"INFO: Added distance annotation '{annotation_details['dist_str']}' to image.")


             # --- Save Updated Map ---
             self.save_map(f"map_update_{target_id}")
             print(f"INFO: Updated map saved as map_update_{target_id}.png in {OUTPUT_DIR}/")

        except Exception as e:
             print(f"ERROR: Failed to re-render map or add annotation after target update: {e}")
             self.current_pil_image = None
             # Reset map object to avoid inconsistent state on next add
             self.map_object = StaticMap(IMAGE_WIDTH, IMAGE_HEIGHT, url_template=SAT_TILE_URL)


    def save_map(self, filename_base):
        """ Saves the current PIL image to the output directory. """
        if self.current_pil_image is None:
            print("WARN: Attempted to save map, but no valid image exists.")
            return

        try:
            # Ensure filename is safe
            safe_filename = "".join(c for c in filename_base if c.isalnum() or c in ('_', '-')).rstrip()
            filepath = os.path.join(OUTPUT_DIR, f"{safe_filename}.png")
            self.current_pil_image.save(filepath)
            # print(f"Map image saved to: {filepath}") # Redundant with print in calling function
        except Exception as e:
            print(f"ERROR: Failed to save map image to {filepath}: {e}")

    def run_interactive_loop(self):
        """ Allows the user to interactively add target points. """
        if not self.blue_points:
            print("Exiting: Cannot run interactive loop without initial blue points.")
            return

        print("\n--- Enter Target (Red) GPS Coordinates ---")
        print("Enter latitude and longitude (decimal degrees), separated by space.")
        print("Example: 40.444 -79.946")
        print("Press Enter with no input to quit.")
        while True:
            try:
                line = input("Add Red Point Lat Lon (or Enter to quit): ").strip()
                if not line:
                    print("Exiting interactive mode.")
                    break

                parts = line.split()
                if len(parts) != 2:
                    print("Invalid input. Please enter latitude and longitude separated by space.")
                    continue

                lat = float(parts[0])
                lon = float(parts[1])

                self.add_target_point(lat, lon)

            except ValueError:
                print("Invalid input. Please enter numeric values for latitude and longitude.")
            except EOFError:
                 print("\nInput terminated. Exiting.")
                 break


def main():
    simulator = AtakSimulatorLocal()
    # Only run the interactive part if initialization was successful (had blue points)
    if simulator.blue_points:
        simulator.run_interactive_loop()
    print("\nATAK Simulator Local finished.")

if __name__ == '__main__':
    main()