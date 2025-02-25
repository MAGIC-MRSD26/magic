import numpy as np
import math

def quat2axisangle(quat):
    """
    Convert quaternion to axis-angle representation.
    """
    # Extract quaternion components
    w, x, y, z = quat
    
    # Calculate the angle
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    
    # Handle the case where angle is close to zero (identity rotation)
    norm = np.sqrt(x*x + y*y + z*z)
    
    if norm < 1e-10:
        # If we're very close to zero, return a default axis
        return np.array([1.0, 0.0, 0.0]), 0.0
    
    # Calculate the normalized axis
    axis = np.array([x, y, z]) / norm
    
    return axis, angle