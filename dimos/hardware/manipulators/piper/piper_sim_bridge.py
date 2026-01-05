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

"""
Piper Simulation Bridge - MuJoCo backend that mimics the Piper SDK interface.

This bridge allows the PiperSDKWrapper to use the same code path for both
hardware and simulation by implementing the subset of C_PiperInterface_V2
methods used by the wrapper.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Optional

import mujoco
import mujoco.viewer as viewer

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

logger = setup_logger()

# Unit conversion constants (matching piper_wrapper.py)
RAD_TO_PIPER = 57295.7795  # radians to Piper units (0.001 degrees)
PIPER_TO_RAD = 1.0 / RAD_TO_PIPER  # Piper units to radians


@dataclass
class JointState:
    """Mimics Piper SDK joint state message."""

    joint_1: float = 0.0
    joint_2: float = 0.0
    joint_3: float = 0.0
    joint_4: float = 0.0
    joint_5: float = 0.0
    joint_6: float = 0.0


@dataclass
class JointMsgs:
    """Mimics Piper SDK joint messages."""

    joint_state: JointState = None  # type: ignore

    def __post_init__(self):
        if self.joint_state is None:
            self.joint_state = JointState()


@dataclass
class ArmStatus:
    """Mimics Piper SDK arm status."""

    err_code: int = 0
    motion_status: int = 0


@dataclass
class ArmStatusMsg:
    """Mimics Piper SDK arm status message."""

    arm_status: ArmStatus = None  # type: ignore

    def __post_init__(self):
        if self.arm_status is None:
            self.arm_status = ArmStatus()


class PiperSimBridge:
    """
    Lightweight, in-process backend that mimics the subset of the Piper SDK
    (C_PiperInterface_V2) used by PiperSDKWrapper.

    The bridge keeps an internal joint state, runs MuJoCo simulation, and
    provides implementations for the SDK methods so the driver stack can
    operate without modification.
    """

    def __init__(
        self,
        control_frequency: float = 100.0,  # control frequency in Hz
    ):
        # Path: dimos/hardware/manipulators/piper/ -> dimos/simulation/data/piper/
        self._model_path = (
            Path(__file__).parent.parent.parent.parent
            / "simulation"
            / "data"
            / "piper"
            / "scene.xml"
        )

        self._num_joints = 6  # Piper is always 6-DOF
        self._control_frequency = control_frequency if control_frequency > 0 else 100.0

        # --- mujoco model & data --- #
        self._model = mujoco.MjModel.from_xml_path(str(self._model_path))
        self._data = mujoco.MjData(self._model)

        # --- state variables --- #
        self._connected: bool = False
        self._enabled: bool = False
        self._err_code: int = 0

        # --- joint targets and measured states --- #
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sim_thread: threading.Thread | None = None

        # Joint state (in Piper units: 0.001 degrees)
        self._joint_positions_piper = [0.0] * self._num_joints
        # Joint targets (in radians, for MuJoCo control)
        self._joint_position_targets = [0.0] * self._num_joints

        # Initialize position targets to current joint positions at startup
        for i in range(min(self._num_joints, self._model.nq)):
            self._joint_position_targets[i] = self._data.qpos[i]
            # Convert to Piper units for state
            self._joint_positions_piper[i] = self._data.qpos[i] * RAD_TO_PIPER

    # ============= Connection Management =============

    def ConnectPort(self, piper_init: bool = True, start_thread: bool = True) -> None:
        """Connect to simulation (mimics C_PiperInterface_V2.ConnectPort)."""
        logger.info("PiperSimBridge: ConnectPort()")
        with self._lock:
            self._connected = True
            self._stop_event.clear()

            # Start simulation thread
            if start_thread and self._sim_thread is None:
                self._sim_thread = threading.Thread(
                    target=self._sim_loop, name="PiperSimBridgeSim", daemon=True
                )
                self._sim_thread.start()

    def DisconnectPort(self) -> None:
        """Disconnect from simulation (mimics C_PiperInterface_V2.DisconnectPort)."""
        logger.info("PiperSimBridge: DisconnectPort()")
        with self._lock:
            self._connected = False
            self._enabled = False

        self._stop_event.set()
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)
        self._sim_thread = None

    # ============= Enable/Disable =============

    def EnablePiper(self) -> bool:
        """Enable the arm (mimics C_PiperInterface_V2.EnablePiper)."""
        with self._lock:
            self._enabled = True
            # Lock current positions when enabling
            for i in range(min(self._num_joints, self._model.nq)):
                self._joint_position_targets[i] = self._data.qpos[i]
        logger.info("PiperSimBridge: Arm enabled")
        return True

    def DisablePiper(self) -> None:
        """Disable the arm (mimics C_PiperInterface_V2.DisablePiper)."""
        with self._lock:
            self._enabled = False
        logger.info("PiperSimBridge: Arm disabled")

    # ============= Motion Control =============

    def MotionCtrl_2(
        self,
        ctrl_mode: int = 0x01,
        move_mode: int = 0x01,
        move_spd_rate_ctrl: int = 30,
        is_mit_mode: int = 0x00,
    ) -> None:
        """Set motion control parameters (mimics C_PiperInterface_V2.MotionCtrl_2)."""
        # In simulation, we just acknowledge the mode settings
        logger.debug(f"PiperSimBridge: MotionCtrl_2(ctrl_mode={ctrl_mode}, move_mode={move_mode})")

    def JointCtrl(
        self,
        joint_1: float,
        joint_2: float,
        joint_3: float,
        joint_4: float,
        joint_5: float,
        joint_6: float,
    ) -> None:
        """Set joint positions (mimics C_PiperInterface_V2.JointCtrl).

        Args:
            joint_1-6: Target positions in Piper units (0.001 degrees)
        """
        with self._lock:
            # Convert from Piper units to radians for MuJoCo
            self._joint_position_targets[0] = joint_1 * PIPER_TO_RAD
            self._joint_position_targets[1] = joint_2 * PIPER_TO_RAD
            self._joint_position_targets[2] = joint_3 * PIPER_TO_RAD
            self._joint_position_targets[3] = joint_4 * PIPER_TO_RAD
            self._joint_position_targets[4] = joint_5 * PIPER_TO_RAD
            self._joint_position_targets[5] = joint_6 * PIPER_TO_RAD

        logger.debug(f"PiperSimBridge: JointCtrl targets (rad): {self._joint_position_targets}")

    def EmergencyStop(self) -> None:
        """Emergency stop (mimics C_PiperInterface_V2.EmergencyStop)."""
        with self._lock:
            # Lock current positions
            for i in range(min(self._num_joints, self._model.nq)):
                self._joint_position_targets[i] = self._data.qpos[i]
        logger.info("PiperSimBridge: Emergency stop")

    # ============= State Query =============

    def GetArmJointMsgs(self) -> JointMsgs:
        """Get joint state messages (mimics C_PiperInterface_V2.GetArmJointMsgs).

        Returns:
            JointMsgs with joint_state containing positions in Piper units
        """
        with self._lock:
            joint_state = JointState(
                joint_1=self._joint_positions_piper[0],
                joint_2=self._joint_positions_piper[1],
                joint_3=self._joint_positions_piper[2],
                joint_4=self._joint_positions_piper[3],
                joint_5=self._joint_positions_piper[4],
                joint_6=self._joint_positions_piper[5],
            )
        return JointMsgs(joint_state=joint_state)

    def GetArmStatus(self) -> ArmStatusMsg:
        """Get arm status (mimics C_PiperInterface_V2.GetArmStatus).

        Returns:
            ArmStatusMsg with arm_status
        """
        with self._lock:
            arm_status = ArmStatus(
                err_code=self._err_code,
                motion_status=1 if self._enabled else 0,
            )
        return ArmStatusMsg(arm_status=arm_status)

    def GetPiperFirmwareVersion(self) -> str:
        """Get firmware version (mimics C_PiperInterface_V2.GetPiperFirmwareVersion).

        Returns:
            Firmware version string
        """
        return "PiperSimBridge v1.0.0"

    def ClearError(self) -> None:
        """Clear errors (mimics C_PiperInterface_V2.ClearError if it exists)."""
        with self._lock:
            self._err_code = 0
        logger.info("PiperSimBridge: Errors cleared")

    # ============= Optional: Gripper =============

    def GripperCtrl(self, percentage: int) -> None:
        """Control gripper (mimics C_PiperInterface_V2.GripperCtrl).

        Args:
            percentage: Gripper opening 0-100
        """
        # TODO: Implement gripper control if model has gripper
        logger.debug(f"PiperSimBridge: GripperCtrl({percentage})")

    def GetGripperState(self) -> int:
        """Get gripper state.

        Returns:
            Gripper position 0-100
        """
        # TODO: Return actual gripper state if model has gripper
        return 0

    # ============= Simulation Loop =============

    def _sim_loop(self) -> None:
        """Main simulation loop running MuJoCo."""
        logger.info("PiperSimBridge: sim loop started")
        dt = 1.0 / self._control_frequency

        with viewer.launch_passive(
            self._model, self._data, show_left_ui=False, show_right_ui=False
        ) as m_viewer:
            while m_viewer.is_running() and not self._stop_event.is_set():
                # Get current targets
                with self._lock:
                    pos_targets = list(self._joint_position_targets)
                    enabled = self._enabled

                # Apply control if enabled
                if enabled:
                    for i in range(self._num_joints):
                        if i < self._model.nu:
                            self._data.ctrl[i] = pos_targets[i]

                # Step simulation
                mujoco.mj_step(self._model, self._data)
                m_viewer.sync()

                # Update joint state from simulation
                with self._lock:
                    for i in range(min(self._num_joints, self._model.nq)):
                        # Store in Piper units (0.001 degrees)
                        self._joint_positions_piper[i] = self._data.qpos[i] * RAD_TO_PIPER

                # Control timing
                time.sleep(dt)

        logger.info("PiperSimBridge: sim loop stopped")
