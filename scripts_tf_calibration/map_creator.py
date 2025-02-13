import numpy as np
import cv2
import yaml

def create_map(image_name, resolution, origin, free_space_coords, obstacles, map_size):
    """
    Creates a map with obstacles and free space, then generates a YAML description.
    
    :param image_name: Name for the map image (without extension)
    :param resolution: Map resolution in meters per pixel (e.g., 0.05)
    :param origin: List of [x, y, theta] for the map's origin
    :param free_space_coords: Tuple (x_min, y_min, x_max, y_max) defining the free space area
    :param obstacles: List of tuples representing obstacles [(x1, y1, width, height), ...]
    :param map_size: Size of the map in meters (width, height)
    """
    # Convert the map size to pixel size based on resolution
    map_width_pixels = int(map_size[0] / resolution)
    map_height_pixels = int(map_size[1] / resolution)

    # Create a blank map with gray background
    map_image = np.ones((map_height_pixels, map_width_pixels), dtype=np.uint8) * 127  # Gray image (127 is gray)

    # Define free space (white) and add black contour
    x_min, y_min, x_max, y_max = free_space_coords
    free_space_pixels = [
        (int((x - origin[0]) / resolution), int((y - origin[1]) / resolution)) 
        for x, y in [(x_min, y_min), (x_max, y_max)]
    ]
    
    # Draw white rectangle for free space
    cv2.rectangle(map_image, free_space_pixels[0], free_space_pixels[1], (255), -1)  # white inside
    
    # Add black contour around the free space
    cv2.rectangle(map_image, free_space_pixels[0], free_space_pixels[1], (0), 2)  # black contour

    # Draw obstacles (gray fill with black contour)
    for (x, y, width, height) in obstacles:
        # Convert coordinates to pixels based on resolution
        x1 = int((x - origin[0]) / resolution)
        y1 = int((y - origin[1]) / resolution)
        x2 = int((x + width - origin[0]) / resolution)
        y2 = int((y + height - origin[1]) / resolution)
        
        # Draw obstacle: inside gray, contour black
        cv2.rectangle(map_image, (x1, y1), (x2, y2), (0), 2)  # black contour
        cv2.rectangle(map_image, (x1, y1), (x2, y2), (127), -1)  # gray inside

    # Save the map image as PGM
    pgm_file = f"{image_name}.pgm"
    cv2.imwrite(pgm_file, map_image)

    # Create the YAML file
    yaml_data = {
        'image': pgm_file,
        'resolution': resolution,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }

    yaml_file = f"{image_name}.yaml"
    with open(yaml_file, 'w') as file:
        yaml.dump(yaml_data, file)

    print(f"Map and YAML file generated: {pgm_file}, {yaml_file}")

# Example usage:
image_name = 'custom_map'
resolution = 0.05  # 5 cm per pixel
origin = [0.0, 0.0, 0.0]  # Origin in meters [x, y, theta]
free_space_coords = (0.0, 0.0, 17.8, 14.3)  # Free space in meters (x_min, y_min, x_max, y_max)
obstacles = [(0.0, 5.2, 1, 9.1),(1,13.3,2.5,1),(3.5,9.9,1,4.4),(4.5,9.9,2.5,1),(9.5,9.9,5.7,1),(14.2,10.9,1,3.4),(5.7,6.6,9.4,1.9),(14.1,5.3,1,1.3),(2.2,0,2,4.4),(4.22,3.4,1.3,1),(9.0,3.4,7.6,1),(15.6,0,1,3.4)]  # List of rectangles (x, y, width, height)
map_size = (17.8, 14.3)  # Map size in meters (width, height)

create_map(image_name, resolution, origin, free_space_coords, obstacles, map_size)

