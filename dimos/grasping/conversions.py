"""
Data type conversion utilities for GraspGen integration.

This module provides helper functions for converting between different
data formats used in the grasp generation pipeline.
"""

import numpy as np
from typing import Tuple

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    
    Convention: XYZ extrinsic (roll-pitch-yaw)
    Rotation order: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    
    Args:
        q: Quaternion orientation
        
    Returns:
        (roll, pitch, yaw) in radians, range [-π, π]
    
    Example:
        >>> from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        >>> q = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        >>> roll, pitch, yaw = quaternion_to_euler(q)
        >>> print(f"RPY: {roll:.2f}, {pitch:.2f}, {yaw:.2f}")
    """
    # Extract components
    x, y, z, w = q.x, q.y, q.z, q.w
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return (roll, pitch, yaw)


def grasp_to_ik_input(grasp: Pose) -> list[float]:
    """
    Convert a grasp Pose to IK solver input format.
    
    The IK solver (XArm, Drake, etc.) typically expects:
        [x, y, z, roll, pitch, yaw]
    where position is in meters and angles are in radians.
    
    Args:
        grasp: Pose from PoseArray (6-DOF grasp)
        
    Returns:
        [x, y, z, roll, pitch, yaw] suitable for IK solver
    
    Example:
        >>> grasp = grasp_array.poses[0]  # Best grasp
        >>> ik_input = grasp_to_ik_input(grasp)
        >>> code, joints = arm.get_inverse_kinematics(ik_input)
    """
    # Extract position (already in meters)
    x = grasp.position.x
    y = grasp.position.y
    z = grasp.position.z
    
    # Convert orientation quaternion to Euler angles
    roll, pitch, yaw = quaternion_to_euler(grasp.orientation)
    
    return [x, y, z, roll, pitch, yaw]


def validate_quaternion(q: Quaternion, epsilon: float = 0.01) -> bool:
    """
    Validate that a quaternion is normalized.
    
    A valid quaternion must have unit length: ||q|| = 1
    
    Args:
        q: Quaternion to validate
        epsilon: Tolerance for normalization (default 0.01)
        
    Returns:
        True if quaternion is valid, False otherwise
    
    Example:
        >>> q = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        >>> is_valid = validate_quaternion(q)
    """
    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    return abs(norm - 1.0) < epsilon


def normalize_quaternion(q: Quaternion) -> Quaternion:
    """
    Normalize a quaternion to unit length.
    
    Args:
        q: Quaternion (possibly unnormalized)
        
    Returns:
        Normalized quaternion with ||q|| = 1
    """
    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if norm == 0:
        # Return identity rotation if degenerate
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    
    return Quaternion(
        x=q.x / norm,
        y=q.y / norm,
        z=q.z / norm,
        w=q.w / norm
    )
