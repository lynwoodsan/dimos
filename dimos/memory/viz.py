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

"""Visualization helpers for Memory2 search results.

Produces LCM-publishable messages (OccupancyGrid, PoseStamped) and
Rerun time-series plots from embedding search observations.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from dimos.memory.types import Observation
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


def similarity_heatmap(
    observations: list[Observation] | Any,
    *,
    resolution: float = 0.1,
    padding: float = 1.0,
    spread: float = 0.2,
    frame_id: str = "world",
) -> OccupancyGrid:
    """Build an OccupancyGrid heatmap from observations with similarity scores.

    Similarity values are normalized relative to the result set's min/max
    (so the full 0-100 color range is used even when CLIP similarities
    cluster in a narrow band).  Each dot's value spreads outward using
    ``distance_transform_edt`` — the same technique as
    :func:`dimos.mapping.occupancy.gradient.gradient` — fading to 0 at
    *spread* metres.

    Args:
        observations: Iterable of Observation (must have .pose and .similarity).
        resolution: Grid resolution in metres/cell.
        padding: Extra metres around the bounding box.
        spread: How far each dot's similarity radiates (metres).
        frame_id: Coordinate frame for the grid.

    Returns:
        OccupancyGrid publishable via LCMTransport.
    """
    from scipy.ndimage import distance_transform_edt

    from dimos.memory.types import EmbeddingObservation
    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid as OG

    posed: list[tuple[float, float, float]] = []
    for obs in observations:
        if obs.pose is None:
            continue
        sim = (
            obs.similarity
            if isinstance(obs, EmbeddingObservation) and obs.similarity is not None
            else 0.0
        )
        p = obs.pose.position
        posed.append((p.x, p.y, sim))

    if not posed:
        return OG(width=1, height=1, resolution=resolution, frame_id=frame_id)

    xs = [p[0] for p in posed]
    ys = [p[1] for p in posed]

    min_x = min(xs) - padding
    min_y = min(ys) - padding
    max_x = max(xs) + padding
    max_y = max(ys) + padding

    width = max(1, int((max_x - min_x) / resolution) + 1)
    height = max(1, int((max_y - min_y) / resolution) + 1)

    # Normalize similarities to 0-1 (CLIP similarities cluster in a narrow band)
    sims = np.array([s for _, _, s in posed])
    sim_min, sim_max = float(sims.min()), float(sims.max())
    sim_range = sim_max - sim_min
    sims_norm = (sims - sim_min) / sim_range if sim_range > 0 else np.full_like(sims, 0.5)

    # Stamp normalized values onto a float grid (0 = no observation)
    value_grid = np.zeros((height, width), dtype=np.float32)
    has_obs = np.zeros((height, width), dtype=bool)

    for (px, py, _), snorm in zip(posed, sims_norm, strict=False):
        gx = min(int((px - min_x) / resolution), width - 1)
        gy = min(int((py - min_y) / resolution), height - 1)
        if snorm > value_grid[gy, gx]:
            value_grid[gy, gx] = snorm
        has_obs[gy, gx] = True

    # Distance transform: distance (in cells) from each empty cell to nearest dot
    dist_cells: np.ndarray[Any, Any] = distance_transform_edt(~has_obs)  # type: ignore[assignment]
    dist_metres = dist_cells * resolution

    # Fade factor: 1.0 at the dot, 0.0 at `spread` metres away
    fade = np.clip(1.0 - dist_metres / spread, 0.0, 1.0)

    # For each cell, find the value of its nearest dot (via index output)
    _, nearest_idx = distance_transform_edt(~has_obs, return_indices=True)  # type: ignore[misc]
    nearest_value = value_grid[nearest_idx[0], nearest_idx[1]]

    # Final heatmap = nearest dot's value * distance fade
    heatmap = nearest_value * fade

    # Convert to int8 grid: observed region is 0-100, rest is -1
    grid = np.full((height, width), -1, dtype=np.int8)
    active = heatmap > 0
    grid[active] = (heatmap[active] * 100).clip(0, 100).astype(np.int8)
    # Ensure dot cells themselves are always visible
    grid[has_obs] = (value_grid[has_obs] * 100).clip(1, 100).astype(np.int8)

    origin = Pose(
        position=[min_x, min_y, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
    )

    return OG(grid=grid, resolution=resolution, origin=origin, frame_id=frame_id)


def similarity_poses(observations: list[Observation] | Any) -> list[PoseStamped]:
    """Extract PoseStamped from observations for spatial arrow rendering.

    Args:
        observations: Iterable of Observation with .pose.

    Returns:
        List of PoseStamped suitable for LCMTransport publishing.
    """
    result: list[PoseStamped] = []
    for obs in observations:
        if obs.pose is not None:
            result.append(obs.pose)
    return result


def log_similarity_timeline(
    observations: list[Observation] | Any,
    entity_path: str = "memory/similarity",
) -> None:
    """Log similarity scores as a Rerun time-series plot.

    Each observation is logged at its timestamp with its similarity score.
    Rerun auto-generates an interactive time-series graph in the timeline panel.

    Args:
        observations: Iterable of EmbeddingObservation with .similarity and .ts.
        entity_path: Rerun entity path for the scalar series.
    """
    import rerun as rr

    from dimos.memory.types import EmbeddingObservation

    for obs in observations:
        if not isinstance(obs, EmbeddingObservation):
            continue
        if obs.similarity is None or obs.ts is None:
            continue
        rr.set_time("memory_time", timestamp=obs.ts)
        rr.log(entity_path, rr.Scalars(obs.similarity))
