"""
GraspGen Grasping Module

This package provides GraspGen grasp generation integrated into the Dimos architecture.
The module manages a Docker container running the GraspGen service and provides
standard LCM interfaces for seamless integration with other Dimos modules.
"""

__version__ = "0.1.0"

from dimos.grasping.graspgen_module import GraspGenModule, GraspGenConfig
from dimos.grasping.conversions import (
    quaternion_to_euler,
    grasp_to_ik_input,
    validate_quaternion,
    normalize_quaternion,
)
from dimos.grasping.gripper_adapter import (
    GripperAdapter,
    GripperGeometry,
    get_recommended_adapter,
    GRIPPER_GEOMETRIES,
)

__all__ = [
    "GraspGenModule",
    "GraspGenConfig",
    "quaternion_to_euler",
    "grasp_to_ik_input",
    "validate_quaternion",
    "normalize_quaternion",
    "GripperAdapter",
    "GripperGeometry",
    "get_recommended_adapter",
    "GRIPPER_GEOMETRIES",
]
