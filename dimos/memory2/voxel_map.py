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

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.memory2.transform import Transformer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.mapping.voxels import VoxelGrid
    from dimos.memory2.type.observation import Observation


class VoxelMap(Transformer[PointCloud2, PointCloud2]):
    """Accumulate PointCloud2 observations into a global voxel map.

    Assumes input clouds are already in world frame (same as VoxelGridMapper).
    All keyword arguments except ``emit_every`` are forwarded to
    :class:`~dimos.mapping.voxels.VoxelGrid`.

    Args:
        emit_every: Yield the current accumulated map every *n* frames.
            ``1`` (default) = yield after every frame (live-compatible).
            ``0`` = yield only when upstream exhausts (batch mode).
        **grid_kwargs: Forwarded to ``VoxelGrid()``.
    """

    def __init__(self, *, emit_every: int = 1, **grid_kwargs: Any) -> None:
        self.emit_every = emit_every
        self._grid_kwargs = grid_kwargs

    def _make_obs(
        self, grid: VoxelGrid, last_obs: Observation[PointCloud2], count: int
    ) -> Observation[PointCloud2]:
        # pose=None: the global map is in world frame, per-observation pose is meaningless
        return last_obs.derive(
            data=grid.get_global_pointcloud2(),
            pose=None,
            tags={**last_obs.tags, "frame_count": count},
        )

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[PointCloud2]]:
        from dimos.mapping.voxels import VoxelGrid

        grid = VoxelGrid(**self._grid_kwargs)
        try:
            last_obs: Observation[PointCloud2] | None = None
            count = 0

            for obs in upstream:
                grid.add_frame(obs.data)
                last_obs = obs
                count += 1

                if self.emit_every > 0 and count % self.emit_every == 0:
                    yield self._make_obs(grid, last_obs, count)

            # Yield on exhaustion: always in batch mode, or if there are un-emitted frames
            if last_obs is not None and (self.emit_every == 0 or count % self.emit_every != 0):
                yield self._make_obs(grid, last_obs, count)
        finally:
            grid.dispose()
