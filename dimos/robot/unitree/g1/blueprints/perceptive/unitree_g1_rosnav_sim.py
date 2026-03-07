#!/usr/bin/env python3
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

"""G1 with ROSNav in simulation mode (Unity)."""

from dimos.core.blueprints import autoconnect
from dimos.navigation.rosnav.rosnav_module import ROSNav
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_onboard import unitree_g1_onboard

unitree_g1_rosnav_sim = autoconnect(
    unitree_g1_onboard,
    ROSNav.blueprint(mode="simulation"),
).global_config(n_dask_workers=6, robot_model="unitree_g1")

__all__ = ["unitree_g1_rosnav_sim"]
