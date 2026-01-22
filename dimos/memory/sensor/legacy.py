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
"""Legacy pickle directory backend for SensorStore.

Compatible with TimedSensorReplay/TimedSensorStorage file format.
"""

from collections.abc import Callable, Iterator
import glob
import os
from pathlib import Path
import pickle
import re
from typing import Any, cast

from dimos.memory.sensor.base import SensorStore, T
from dimos.utils.data import get_data, get_data_dir


class LegacyPickleStore(SensorStore[T]):
    """Legacy pickle backend compatible with TimedSensorReplay/TimedSensorStorage.

    File format:
        {name}/
            000.pickle  # contains (timestamp, data) tuple
            001.pickle
            ...

    Files are assumed to be in chronological order (timestamps increase with file number).
    No index is built - iteration is lazy and memory-efficient for large datasets.

    Usage:
        # Load existing recording (auto-downloads from LFS if needed)
        store = LegacyPickleStore("unitree_go2_bigoffice/lidar")
        data = store.find_closest_seek(10.0)

        # Create new recording (directory created on first save)
        store = LegacyPickleStore("my_recording/images")
        store.save(image)  # uses image.ts for timestamp
    """

    def __init__(self, name: str | Path, autocast: Callable[[Any], T] | None = None) -> None:
        """
        Args:
            name: Data directory name (e.g. "unitree_go2_bigoffice/lidar") or absolute path.
            autocast: Optional function to transform data after loading (for replay) or
                      before saving (for storage). E.g., `Odometry.from_msg`.
        """
        self._name = str(name)
        self._root_dir: Path | None = None
        self._counter: int = 0
        self._autocast = autocast

    def _get_root_dir(self, for_write: bool = False) -> Path:
        """Get root directory, creating on first write if needed."""
        if self._root_dir is not None:
            # Ensure directory exists if writing
            if for_write:
                self._root_dir.mkdir(parents=True, exist_ok=True)
            return self._root_dir

        # If absolute path, use directly
        if Path(self._name).is_absolute():
            self._root_dir = Path(self._name)
            if for_write:
                self._root_dir.mkdir(parents=True, exist_ok=True)
        elif for_write:
            # For writing: use get_data_dir and create if needed
            self._root_dir = get_data_dir(self._name)
            self._root_dir.mkdir(parents=True, exist_ok=True)
        else:
            # For reading: use get_data (handles LFS download)
            self._root_dir = get_data(self._name)

        return self._root_dir

    def _iter_files(self) -> Iterator[Path]:
        """Iterate pickle files in sorted order (by number in filename)."""

        def extract_number(filepath: str) -> int:
            basename = os.path.basename(filepath)
            match = re.search(r"(\d+)\.pickle$", basename)
            return int(match.group(1)) if match else 0

        root_dir = self._get_root_dir()
        files = sorted(
            glob.glob(os.path.join(root_dir, "*.pickle")),
            key=extract_number,
        )
        for f in files:
            yield Path(f)

    def _save(self, timestamp: float, data: T) -> None:
        root_dir = self._get_root_dir(for_write=True)

        # Initialize counter from existing files if needed
        if self._counter == 0:
            existing = list(root_dir.glob("*.pickle"))
            if existing:
                # Find highest existing counter
                max_num = 0
                for filepath in existing:
                    match = re.search(r"(\d+)\.pickle$", filepath.name)
                    if match:
                        max_num = max(max_num, int(match.group(1)))
                self._counter = max_num + 1

        full_path = root_dir / f"{self._counter:03d}.pickle"

        if full_path.exists():
            raise RuntimeError(f"File {full_path} already exists")

        # Save as (timestamp, data) tuple for legacy compatibility
        with open(full_path, "wb") as f:
            pickle.dump((timestamp, data), f)

        self._counter += 1

    def _load(self, timestamp: float) -> T | None:
        """Load data at exact timestamp (linear scan)."""
        for ts, data in self._iter_items():
            if ts == timestamp:
                return data
        return None

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        """Lazy iteration - loads one file at a time."""
        for filepath in self._iter_files():
            try:
                with open(filepath, "rb") as f:
                    ts, data = pickle.load(f)
                    ts = float(ts)
            except Exception:
                continue

            if start is not None and ts < start:
                continue
            if end is not None and ts >= end:
                break

            if self._autocast is not None:
                data = self._autocast(data)
            yield (ts, cast("T", data))

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        """Linear scan with early exit (assumes timestamps are monotonically increasing)."""
        closest_ts: float | None = None
        closest_diff = float("inf")

        for ts, _ in self._iter_items():
            diff = abs(ts - timestamp)

            if diff < closest_diff:
                closest_diff = diff
                closest_ts = ts
            elif diff > closest_diff:
                # Moving away from target, can stop
                break

        if closest_ts is None:
            return None

        if tolerance is not None and closest_diff > tolerance:
            return None

        return closest_ts
