# Copyright 2025 Dimensional Inc.
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

"""Grasp Execution Module for DimOS.

This module takes top-K grasps from the ROI grasp pipeline and executes them
in order, trying each grasp until one succeeds.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.core.rpc_client import RpcCall
from dimos.manipulation.utils import xyzrpy_from_pose
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class GraspExecutionModuleConfig(ModuleConfig):
    """Configuration for Grasp Execution Module."""

    # Manipulation module RPC (set via set_manipulation_module_rpc)
    manipulation_module_rpc: RpcCall | None = None

    # Execution parameters
    max_grasp_attempts: int = 5  # Try top-K grasps
    approach_offset: float = 0.05  # Approach distance in meters (move back before grasp)
    execution_timeout: float = 30.0  # Timeout per grasp attempt in seconds

    # Optional callback for custom grasp validation
    grasp_validator: Callable[[dict], bool] | None = None


class GraspExecutionModule(Module):
    """Grasp Execution Module.

    This module:
    1. Receives top-K grasps from ROI grasp pipeline
    2. Tries each grasp in order (highest score first)
    3. Converts grasp pose to xyzrpy format
    4. Executes via ManipulationModule RPC
    5. Stops on first successful execution

    Inputs:
        - grasps: List of grasp dictionaries from ROI grasp pipeline

    Outputs:
        - execution_result: Result of grasp execution attempt
    """

    # Inputs
    grasps: In[list[dict]] = None  # type: ignore[type-arg]

    # Outputs
    execution_result: Out[dict] = None  # type: ignore[type-arg]

    default_config = GraspExecutionModuleConfig

    _subscriptions: list[Disposable] = []
    _manipulation_rpc: RpcCall | None = None

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._subscriptions = []
        self._manipulation_rpc = self.config.manipulation_module_rpc

    @rpc
    def set_manipulation_module_rpc(self, rpc_call: RpcCall) -> str:
        """Set the ManipulationModule RPC call for executing grasps.

        Args:
            rpc_call: RPC call to ManipulationModule's move_to_pose method

        Returns:
            Status message
        """
        self._manipulation_rpc = rpc_call
        return "ManipulationModule RPC set"

    @rpc
    def execute_grasp(self, grasp: dict) -> dict:  # type: ignore[type-arg]
        """Execute a single grasp via RPC.

        Args:
            grasp: Grasp dictionary with 'translation' and 'rotation_matrix'

        Returns:
            Result dictionary with 'success' and 'message'
        """
        if self._manipulation_rpc is None:
            return {
                "success": False,
                "message": "ManipulationModule RPC not set. Call set_manipulation_module_rpc() first.",
            }

        try:
            # Extract pose from grasp
            translation = np.array(grasp.get("translation", [0, 0, 0]))
            rotation_matrix = np.array(grasp.get("rotation_matrix", np.eye(3)))

            # Build 4x4 pose matrix
            pose = np.eye(4)
            pose[:3, :3] = rotation_matrix
            pose[:3, 3] = translation

            # Convert to xyzrpy
            x, y, z, roll, pitch, yaw = xyzrpy_from_pose(pose)

            # Apply approach offset (move back along approach direction)
            # Approach direction is typically the Z-axis of the rotation matrix
            approach_dir = rotation_matrix[:, 2]  # Z-axis
            approach_pos = translation - approach_dir * self.config.approach_offset

            logger.info(
                f"Executing grasp at ({x:.3f}, {y:.3f}, {z:.3f}) "
                f"with approach offset {self.config.approach_offset:.3f}m"
            )

            # Call ManipulationModule's move_to_pose
            result = self._manipulation_rpc(
                x=float(approach_pos[0]),
                y=float(approach_pos[1]),
                z=float(approach_pos[2]),
                roll=float(roll),
                pitch=float(pitch),
                yaw=float(yaw),
            )

            if result:
                logger.info("Grasp execution successful")
                return {"success": True, "message": "Grasp executed successfully", "grasp": grasp}
            else:
                logger.warning("Grasp execution failed (IK or planning failed)")
                return {"success": False, "message": "IK or planning failed", "grasp": grasp}

        except Exception as e:
            logger.error(f"Error executing grasp: {e}", exc_info=True)
            return {"success": False, "message": f"Error: {str(e)}", "grasp": grasp}

    @rpc
    def execute_top_grasps(self, grasps: list[dict] | None = None) -> dict:  # type: ignore[type-arg]
        """Try executing top-K grasps in order until one succeeds.

        Args:
            grasps: List of grasps to try (if None, uses latest from stream)

        Returns:
            Result dictionary with 'success', 'attempted', and 'message'
        """
        if grasps is None:
            return {"success": False, "message": "No grasps provided"}

        if self._manipulation_rpc is None:
            return {
                "success": False,
                "message": "ManipulationModule RPC not set",
            }

        # Limit to max attempts
        grasps_to_try = grasps[: self.config.max_grasp_attempts]

        logger.info(f"Trying {len(grasps_to_try)} grasps (top-{self.config.max_grasp_attempts})")

        for i, grasp in enumerate(grasps_to_try):
            # Optional validation
            if self.config.grasp_validator is not None:
                if not self.config.grasp_validator(grasp):
                    logger.info(f"Grasp {i+1} failed validation, skipping")
                    continue

            logger.info(f"Attempting grasp {i+1}/{len(grasps_to_try)} (score: {grasp.get('score', 0.0):.3f})")

            result = self.execute_grasp(grasp)

            if result.get("success", False):
                logger.info(f"Successfully executed grasp {i+1}")
                return {
                    "success": True,
                    "attempted": i + 1,
                    "total": len(grasps_to_try),
                    "message": f"Grasp {i+1} executed successfully",
                    "grasp": grasp,
                }

            logger.warning(f"Grasp {i+1} failed: {result.get('message', 'Unknown error')}")

        # All grasps failed
        logger.error(f"All {len(grasps_to_try)} grasp attempts failed")
        return {
            "success": False,
            "attempted": len(grasps_to_try),
            "total": len(grasps_to_try),
            "message": f"All {len(grasps_to_try)} grasp attempts failed",
        }

    @rpc
    def start(self) -> str:
        """Start the module and begin processing grasp streams."""
        if self._subscriptions:
            return "already started"

        # Subscribe to grasps stream
        if self.grasps is not None:
            # Process grasps automatically when received
            sub = (
                self.grasps.observable()
                .pipe(
                    ops.filter(lambda gs: gs is not None and len(gs) > 0),
                    ops.map(lambda grasps: self._handle_grasps(grasps)),
                )
                .subscribe(lambda _: None)
            )
            self._subscriptions.append(sub)

        return "started"

    def _handle_grasps(self, grasps: list[dict]) -> None:  # type: ignore[type-arg]
        """Handle incoming grasps from stream."""
        if not grasps:
            return

        logger.info(f"Received {len(grasps)} grasps, attempting execution")

        result = self.execute_top_grasps(grasps)

        # Publish result
        if self.execution_result is not None:
            self.execution_result.publish(result)

    @rpc
    def stop(self) -> None:
        """Stop the module."""
        for sub in self._subscriptions:
            sub.dispose()
        self._subscriptions = []
        super().stop()


grasp_execution_module = GraspExecutionModule.blueprint

__all__ = ["GraspExecutionModule", "GraspExecutionModuleConfig", "grasp_execution_module"]

