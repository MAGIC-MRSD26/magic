import numpy as np
import math

def quat2mat(quat: np.ndarray):
    
    """Convert quaternion to rotation matrix"""
    w, x, y, z = quat
    
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
        [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])

def quat2euler(quat: np.ndarray):
    
    """Convert quaternion to euler angles (xyz convention)"""
    # Extract quaternion components
    w, x, y, z = quat
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([roll, pitch, yaw])

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