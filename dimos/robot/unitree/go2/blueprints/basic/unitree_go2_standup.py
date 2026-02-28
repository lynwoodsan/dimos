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

"""Unitree Go2 low-level standup blueprint.

Composes the ControlCoordinator (500 Hz, 12-DOF quadruped adapter) with the
Go2LowLevelControl module that executes a 4-phase standup sequence.

Architecture:
    Go2LowLevelControl  ──joint_command──▸  ControlCoordinator
                         ◂──joint_state───      ↕ DDS (500 Hz)
                                           UnitreeGo2LowLevelAdapter

Usage:
    dimos run unitree-go2-standup
"""

from __future__ import annotations

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_quadruped_joints,
)
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.control.examples.go2_standup import go2_low_level_control
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import JointState

_go2_joints = make_quadruped_joints("go2")

_coordinator = control_coordinator(
    tick_rate=500.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="go2",
            hardware_type=HardwareType.QUADRUPED,
            joints=_go2_joints,
            adapter_type="unitree_go2",
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="servo_go2",
            type="servo",
            joint_names=_go2_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/go2/joint_command", JointState),
    }
)

_control = go2_low_level_control().transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/go2/joint_command", JointState),
    }
)

unitree_go2_standup = autoconnect(_coordinator, _control)

__all__ = ["unitree_go2_standup"]
