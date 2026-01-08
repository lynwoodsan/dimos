# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Gripper Geometric Adaptation

This module provides geometric transformations to adapt grasps generated for one
gripper (e.g., Robotiq 2F-140) to work with a different gripper (e.g., UFactory xArm).

The adaptation accounts for differences in:
- Finger width (affects grasp center offset)
- Maximum opening/stroke (affects grasp feasibility)
- TCP offset (Tool Center Point position)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.geometry_msgs.Quaternion import Quaternion


@dataclass
class GripperGeometry:
    """Physical geometry parameters for a gripper."""

    name: str
    finger_width: float  # Width of gripper fingers in meters
    max_stroke: float    # Maximum opening distance in meters
    tcp_offset: float    # Tool Center Point offset from base in meters
    min_grasp_width: float = 0.01  # Minimum graspable object width in meters


# Pre-defined gripper geometries
GRIPPER_GEOMETRIES: dict[str, GripperGeometry] = {
    "robotiq_2f_140": GripperGeometry(
        name="robotiq_2f_140",
        finger_width=0.015,  # 15mm finger width
        max_stroke=0.140,    # 140mm max opening
        tcp_offset=0.175,    # ~175mm TCP offset (typical)
        min_grasp_width=0.01,
    ),
    "franka_panda": GripperGeometry(
        name="franka_panda",
        finger_width=0.010,  # 10mm finger width (thinner)
        max_stroke=0.080,    # 80mm max opening
        tcp_offset=0.103,    # ~103mm TCP offset
        min_grasp_width=0.005,
    ),
    "ufactory_xarm": GripperGeometry(
        name="ufactory_xarm",
        finger_width=0.015,  # 15mm finger width (similar to Robotiq)
        max_stroke=0.086,    # 86mm max opening
        tcp_offset=0.175,    # ~175mm TCP offset (estimated, adjust based on URDF)
        min_grasp_width=0.01,
    ),
}


class GripperAdapter:
    """
    Adapts grasps from a source gripper to a target gripper.

    The adaptation applies geometric transformations to account for differences
    in gripper geometry while preserving grasp quality and orientation.

    Transformation Pipeline:
        1. Adjust grasp approach distance based on finger width difference
        2. Filter grasps that exceed target gripper's max stroke
        3. Adjust TCP offset if grippers have different mount points
        4. Optionally scale grasp scores based on compatibility

    Example:
        >>> adapter = GripperAdapter("robotiq_2f_140", "ufactory_xarm")
        >>> adapted_poses = adapter.adapt_grasps(robotiq_poses)
    """

    def __init__(
        self,
        source_gripper: str | GripperGeometry,
        target_gripper: str | GripperGeometry,
        filter_infeasible: bool = True,
    ) -> None:
        """Initialize gripper adapter.

        Args:
            source_gripper: Gripper used to generate grasps (name or geometry)
            target_gripper: Gripper to adapt grasps for (name or geometry)
            filter_infeasible: Whether to filter grasps exceeding target's max stroke
        """
        # Resolve gripper geometries
        if isinstance(source_gripper, str):
            if source_gripper not in GRIPPER_GEOMETRIES:
                raise ValueError(
                    f"Unknown source gripper: {source_gripper}. "
                    f"Available: {list(GRIPPER_GEOMETRIES.keys())}"
                )
            self.source = GRIPPER_GEOMETRIES[source_gripper]
        else:
            self.source = source_gripper

        if isinstance(target_gripper, str):
            if target_gripper not in GRIPPER_GEOMETRIES:
                raise ValueError(
                    f"Unknown target gripper: {target_gripper}. "
                    f"Available: {list(GRIPPER_GEOMETRIES.keys())}"
                )
            self.target = GRIPPER_GEOMETRIES[target_gripper]
        else:
            self.target = target_gripper

        self.filter_infeasible = filter_infeasible

        # Compute transformation parameters
        self._compute_transform_params()

    def _compute_transform_params(self) -> None:
        """Compute transformation parameters based on gripper geometries."""
        # Finger width difference affects grasp center
        # If target has wider fingers, grasp needs to be adjusted inward
        self.finger_width_offset = (
            self.target.finger_width - self.source.finger_width
        ) / 2.0

        # TCP offset difference
        self.tcp_offset_delta = self.target.tcp_offset - self.source.tcp_offset

        # Maximum stroke compatibility ratio
        self.stroke_ratio = self.target.max_stroke / self.source.max_stroke

    def adapt_grasp(self, pose: Pose, grasp_width: float | None = None) -> Pose | None:
        """Adapt a single grasp pose from source to target gripper.

        The transformation adjusts the grasp position along the approach direction
        to account for finger width differences. This ensures the grasp center
        remains at the optimal position for the target gripper.

        Transformation:
            1. Extract approach direction (Z-axis of gripper orientation)
            2. Offset grasp position by finger_width_offset along approach
            3. Adjust TCP offset if needed
            4. Check feasibility against target gripper constraints

        Args:
            pose: Original grasp pose from source gripper
            grasp_width: Object width at grasp point (meters). If provided,
                        used to check if grasp is feasible for target gripper.

        Returns:
            Adapted pose for target gripper, or None if infeasible
        """
        # Check grasp width feasibility
        if grasp_width is not None and self.filter_infeasible:
            if grasp_width > self.target.max_stroke:
                return None  # Object too wide for target gripper
            if grasp_width < self.target.min_grasp_width:
                return None  # Object too thin to grasp reliably

        # Convert quaternion to rotation matrix
        rot_matrix = self._quaternion_to_rotation_matrix(pose.orientation)

        # Extract approach direction (Z-axis of gripper frame)
        # In standard grasp convention: Z = approach, X = opening direction
        approach_dir = rot_matrix[:, 2]  # Third column

        # Compute position offset
        # If target has wider fingers, move grasp slightly inward along approach
        position_offset = approach_dir * self.finger_width_offset

        # Also adjust for TCP offset difference
        tcp_offset_vector = approach_dir * self.tcp_offset_delta

        # Apply transformation
        adapted_position = Vector3(
            x=pose.position.x + position_offset[0] + tcp_offset_vector[0],
            y=pose.position.y + position_offset[1] + tcp_offset_vector[1],
            z=pose.position.z + position_offset[2] + tcp_offset_vector[2],
        )

        # Orientation remains the same (grippers are parallel-jaw)
        return Pose(position=adapted_position, orientation=pose.orientation)

    def adapt_grasps(
        self,
        poses: list[Pose],
        grasp_widths: list[float] | None = None,
    ) -> list[Pose]:
        """Adapt a list of grasp poses from source to target gripper.

        Args:
            poses: List of grasp poses from source gripper
            grasp_widths: Optional list of object widths at each grasp point

        Returns:
            List of adapted poses (may be shorter if infeasible grasps filtered)
        """
        adapted = []

        for i, pose in enumerate(poses):
            width = grasp_widths[i] if grasp_widths else None
            adapted_pose = self.adapt_grasp(pose, width)

            if adapted_pose is not None:
                adapted.append(adapted_pose)

        return adapted

    def get_compatibility_score(self) -> float:
        """Compute a compatibility score between source and target grippers.

        This score (0-1) indicates how well grasps from the source gripper
        will transfer to the target gripper. Higher is better.

        Factors:
            - Finger width similarity (most important)
            - Max stroke ratio
            - TCP offset similarity

        Returns:
            Compatibility score between 0 (incompatible) and 1 (identical)
        """
        # Finger width similarity (most critical for grasp quality)
        finger_width_ratio = min(
            self.source.finger_width, self.target.finger_width
        ) / max(self.source.finger_width, self.target.finger_width)

        # Stroke ratio (affects feasible workspace)
        stroke_score = min(self.stroke_ratio, 1.0 / self.stroke_ratio)

        # TCP offset similarity (less critical)
        tcp_offset_diff = abs(self.tcp_offset_delta)
        tcp_score = np.exp(-tcp_offset_diff / 0.05)  # Decay with 5cm scale

        # Weighted combination
        compatibility = (
            0.6 * finger_width_ratio +
            0.3 * stroke_score +
            0.1 * tcp_score
        )

        return float(compatibility)

    def get_info(self) -> dict:
        """Get information about the gripper adaptation.

        Returns:
            Dictionary with adaptation parameters and compatibility info
        """
        return {
            "source_gripper": self.source.name,
            "target_gripper": self.target.name,
            "finger_width_offset": self.finger_width_offset,
            "tcp_offset_delta": self.tcp_offset_delta,
            "stroke_ratio": self.stroke_ratio,
            "compatibility_score": self.get_compatibility_score(),
            "filter_infeasible": self.filter_infeasible,
        }

    @staticmethod
    def _quaternion_to_rotation_matrix(q: Quaternion) -> np.ndarray:
        """Convert quaternion to 3x3 rotation matrix.

        Args:
            q: Quaternion (x, y, z, w)

        Returns:
            3x3 rotation matrix
        """
        x, y, z, w = q.x, q.y, q.z, q.w

        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
            [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
            [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
        ])


def get_recommended_adapter(target_gripper: str) -> tuple[str, GripperAdapter] | None:
    """Get recommended source gripper and adapter for a target gripper.

    This function finds the best pre-trained gripper model to use as a source
    for adapting grasps to the target gripper, based on geometric compatibility.

    Args:
        target_gripper: Name of the target gripper

    Returns:
        Tuple of (source_gripper_name, adapter) or None if no good match
    """
    if target_gripper not in GRIPPER_GEOMETRIES:
        return None

    # Available pre-trained grippers
    available_grippers = ["robotiq_2f_140", "franka_panda"]

    best_source = None
    best_score = 0.0

    for source in available_grippers:
        if source == target_gripper:
            continue  # Skip if target has native model

        adapter = GripperAdapter(source, target_gripper)
        score = adapter.get_compatibility_score()

        if score > best_score:
            best_score = score
            best_source = source

    if best_source is None:
        return None

    return (best_source, GripperAdapter(best_source, target_gripper))
