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

import threading
from typing import TYPE_CHECKING, Any, TypeVar

from dimos.memory2.observationstore.base import ObservationStore, ObservationStoreConfig

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.filter import StreamQuery
    from dimos.memory2.type.observation import Observation

T = TypeVar("T")


class NullObservationStoreConfig(ObservationStoreConfig):
    name: str = "<null>"


class NullObservationStore(ObservationStore[T]):
    """O(1) observation store that assigns IDs but discards data.

    Use for passthrough / live-only pipelines where replay is not needed.
    IDs are still monotonically increasing (required for live dedup).
    """

    default_config = NullObservationStoreConfig
    config: NullObservationStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._name = self.config.name
        self._next_id = 0
        self._lock = threading.Lock()

    @property
    def name(self) -> str:
        return self._name

    def insert(self, obs: Observation[T]) -> int:
        with self._lock:
            obs.id = self._next_id
            row_id = self._next_id
            self._next_id += 1
        return row_id

    def query(self, q: StreamQuery) -> Iterator[Observation[T]]:
        return iter([])

    def count(self, q: StreamQuery) -> int:
        return 0

    def fetch_by_ids(self, ids: list[int]) -> list[Observation[T]]:
        return []
