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

"""Visualization sub-blueprint: Rerun viewer with G1-specific visual overrides."""

from dimos.core.global_config import global_config
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.vis_module import vis_module

_vis = vis_module(
    viewer_backend=global_config.viewer_backend,
    rerun_config={
        "pubsubs": [LCM(autoconf=True)],
        "visual_override": {
            "world/camera_info": lambda camera_info: camera_info.to_rerun(
                image_topic="/world/color_image",
                optical_frame="camera_optical",
            ),
            "world/global_map": lambda grid: grid.to_rerun(voxel_size=0.1, mode="boxes"),
            "world/navigation_costmap": lambda grid: grid.to_rerun(
                colormap="Accent",
                z_offset=0.015,
                opacity=0.2,
                background="#484981",
            ),
        },
        "static": {
            "world/tf/base_link": lambda rr: [
                rr.Boxes3D(
                    half_sizes=[0.2, 0.15, 0.75],
                    colors=[(0, 255, 127)],
                    fill_mode="MajorWireframe",
                ),
                rr.Transform3D(parent_frame="tf#/base_link"),
            ]
        },
    },
)
