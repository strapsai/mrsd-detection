import numpy as np

def track(bbox, frame_width, frame_height):
    """
    Calculates the gimbal offset to center the bounding box.

    Args:
        bbox (tuple): (center_x, center_y) of the bounding box.
        frame_width (int): Width of the frame.
        frame_height (int): Height of the frame.

    Returns:
        tuple: (yaw_offset, pitch_offset) in degrees.
    """
    center_x, center_y = bbox
    
    # Normalize the bounding box center to the range [-0.5, 0.5]
    norm_x = (center_x / frame_width) - 0.5
    norm_y = (center_y / frame_height) - 0.5
    
    # Define gains for yaw and pitch.  These can be tuned.
    yaw_gain = 1.0   
    pitch_gain = 1.0 
    
    # Calculate the offsets
    yaw_offset = norm_x * frame_width * yaw_gain
    pitch_offset = norm_y * frame_height * pitch_gain
    
    return yaw_offset, pitch_offset