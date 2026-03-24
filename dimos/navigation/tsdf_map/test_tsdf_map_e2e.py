# Copyright 2026 Dimensional Inc.
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

"""E2E integration test for TSDFMap.

Exercises the full data flow through real LCM transports:

    MockScanSource ──registered_scan──▶ TSDFMap ──global_map──▶ (subscriber)
    MockOdomSource ──raw_odom────────▶ TSDFMap ──odom────────▶ (subscriber)

Phase 1: Publish a wall at x=3  → verify occupied voxels appear near x=3.
Phase 2: Move wall to x=5       → verify old wall clears, new wall appears.

Marked ``slow`` — excluded from fast CI.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.tsdf_map.module import TSDFMap

# ── Mock modules ──


class MapCollector(Module):
    """Subscribes to global_map and collects received clouds."""

    global_map: In[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self._received: list[bytes] = []
        self._disposables.add(self.global_map.subscribe(self._on_map))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_map(self, msg: PointCloud2) -> None:
        self._received.append(msg.lcm_encode())

    @rpc
    def get_count(self) -> int:
        return len(self._received)

    @rpc
    def get_last_map_bytes(self) -> bytes:
        if not self._received:
            return b""
        return self._received[-1]


class MockOdomSource(Module):
    """Publishes PoseStamped via RPC."""

    raw_odom: Out[PoseStamped]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def publish_pose(self, x: float, y: float, z: float) -> None:
        pose = PoseStamped()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        self.raw_odom.publish(pose)


class MockScanSource(Module):
    """Publishes PointCloud2 scans via RPC."""

    registered_scan: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def publish_wall(
        self, x: float, y_min: float, y_max: float, z: float = 0.0, n_points: int = 20
    ) -> None:
        """Publish a wall of points at the given x position."""
        ys = np.linspace(y_min, y_max, n_points)
        pts = np.column_stack([np.full(n_points, x), ys, np.full(n_points, z)]).astype(np.float32)
        cloud = PointCloud2.from_numpy(pts, frame_id="world", timestamp=time.time())
        self.registered_scan.publish(cloud)


# ── E2E Test ──


@pytest.mark.slow
@pytest.mark.timeout(20)
def test_tsdf_map_e2e_dynamic_clearing() -> None:
    """Full pipeline: publish scans + odom via LCM transports → TSDFMap → verify global_map output."""
    # Build the blueprint: MockSources → TSDFMap → MapCollector
    blueprint = autoconnect(
        MockOdomSource.blueprint(),
        MockScanSource.blueprint(),
        TSDFMap.blueprint(
            voxel_size=0.15,
            sdf_trunc=3.0,  # large trunc for clearing test
            max_range=15.0,
            map_publish_rate=4.0,  # fast publish for test
            max_weight=20.0,
            key_trans=0.3,  # low threshold so small movements trigger integration
        ),
        MapCollector.blueprint(),
    )

    coordinator: ModuleCoordinator = blueprint.build()
    try:
        coordinator.start()

        # Get module proxies (RPC-callable)
        odom_src = coordinator.get_instance(MockOdomSource)
        scan_src = coordinator.get_instance(MockScanSource)
        collector = coordinator.get_instance(MapCollector)

        assert odom_src is not None, "MockOdomSource not deployed"
        assert scan_src is not None, "MockScanSource not deployed"
        assert collector is not None, "MapCollector not deployed"

        # ── Phase 1: Wall at x=3 ──
        for i in range(10):
            odom_src.publish_pose(0.0, float(i) * 0.4, 0.0)
            time.sleep(0.02)
            scan_src.publish_wall(3.0, -1.0, 1.0, z=0.0)
            time.sleep(0.05)

        # Wait for at least one map publication
        deadline = time.time() + 5.0
        while time.time() < deadline:
            time.sleep(0.25)
            if collector.get_count() > 0:
                break

        count = collector.get_count()
        assert count > 0, "Phase 1: No global_map messages received"

        # Decode last map and check for points near x=3
        last_bytes = collector.get_last_map_bytes()
        assert len(last_bytes) > 0, "Phase 1: empty map bytes"
        last_map = PointCloud2.lcm_decode(last_bytes)
        raw = last_map.as_numpy()
        pts = raw[0] if isinstance(raw, tuple) else raw

        assert pts is not None and len(pts) > 0, "Phase 1: global_map is empty"
        near_x3 = pts[np.abs(pts[:, 0] - 3.0) < 0.5]
        assert len(near_x3) > 0, (
            f"Phase 1: Expected occupied voxels near x=3, "
            f"got {len(pts)} points with x range [{pts[:, 0].min():.2f}, {pts[:, 0].max():.2f}]"
        )

        # ── Phase 2: Wall moved to x=5 ──
        for i in range(15):
            odom_src.publish_pose(0.0, float(i + 10) * 0.4, 0.0)
            time.sleep(0.02)
            scan_src.publish_wall(5.0, -1.0, 1.0, z=0.0)
            time.sleep(0.05)

        # Wait for new maps
        deadline = time.time() + 5.0
        prev_count = count
        while time.time() < deadline:
            time.sleep(0.25)
            if collector.get_count() > prev_count:
                break

        assert collector.get_count() > prev_count, "Phase 2: No new global_map messages"

        last_bytes = collector.get_last_map_bytes()
        last_map = PointCloud2.lcm_decode(last_bytes)
        raw = last_map.as_numpy()
        pts = raw[0] if isinstance(raw, tuple) else raw

        assert pts is not None and len(pts) > 0, "Phase 2: global_map is empty"
        near_x5 = pts[np.abs(pts[:, 0] - 5.0) < 0.5]
        assert len(near_x5) > 0, (
            f"Phase 2: Expected occupied voxels near x=5, "
            f"got {len(pts)} points with x range [{pts[:, 0].min():.2f}, {pts[:, 0].max():.2f}]"
        )

    finally:
        coordinator.stop()
