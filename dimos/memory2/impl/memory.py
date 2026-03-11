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
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from reactivex.disposable import Disposable

from dimos.memory2.store import Session, Store

if TYPE_CHECKING:
    from collections.abc import Iterator

    from reactivex.abc import DisposableBase

    from dimos.memory2.backend import Backend
    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.filter import StreamQuery
    from dimos.memory2.type import Observation

T = TypeVar("T")


class ListBackend(Generic[T]):
    """In-memory backend for experimentation. Thread-safe."""

    def __init__(self, name: str = "<memory>") -> None:
        self._name = name
        self._observations: list[Observation[T]] = []
        self._next_id = 0
        self._lock = threading.Lock()
        self._subscribers: list[BackpressureBuffer[Observation[T]]] = []

    @property
    def name(self) -> str:
        return self._name

    def append(self, obs: Observation[T]) -> Observation[T]:
        with self._lock:
            obs.id = self._next_id
            self._next_id += 1
            self._observations.append(obs)
            subs = list(self._subscribers)

        # Notify outside lock to avoid deadlocks
        for buf in subs:
            buf.put(obs)

        return obs

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        """Snapshot + apply all filters/ordering/offset/limit in Python.

        If query.live_buffer is set, subscribes before backfill, then
        switches to a live tail that blocks for new observations.
        """
        if query.search_vec is not None and query.live_buffer is not None:
            raise TypeError("Cannot combine .search() with .live() — search is a batch operation.")
        buf = query.live_buffer
        if buf is not None:
            # Subscribe BEFORE backfill to avoid missing items
            sub = self.subscribe(buf)
            return self._iterate_live(query, buf, sub)
        return self._iterate_snapshot(query)

    def _iterate_snapshot(self, query: StreamQuery) -> Iterator[Observation[T]]:
        with self._lock:
            snapshot = list(self._observations)

        # Apply filters
        for f in query.filters:
            snapshot = [obs for obs in snapshot if f.matches(obs)]

        # Text search — substring match (SqliteBackend will use FTS5)
        if query.search_text is not None:
            needle = query.search_text.lower()
            snapshot = [obs for obs in snapshot if needle in str(obs.data).lower()]

        # Vector search — brute-force cosine via Embedding.__matmul__
        if query.search_vec is not None:
            query_emb = query.search_vec
            scored: list[Observation[T]] = []
            for obs in snapshot:
                emb = getattr(obs, "embedding", None)
                if emb is not None:
                    sim = float(emb @ query_emb)
                    scored.append(obs.derive(data=obs.data, similarity=sim))
            scored.sort(key=lambda o: getattr(o, "similarity", 0.0) or 0.0, reverse=True)
            if query.search_k is not None:
                scored = scored[: query.search_k]
            snapshot = scored

        # Ordering
        if query.order_field:
            key = query.order_field
            snapshot.sort(
                key=lambda obs: getattr(obs, key) if getattr(obs, key, None) is not None else 0,
                reverse=query.order_desc,
            )

        # Offset
        if query.offset_val:
            snapshot = snapshot[query.offset_val :]

        # Limit
        if query.limit_val is not None:
            snapshot = snapshot[: query.limit_val]

        yield from snapshot

    def _iterate_live(
        self,
        query: StreamQuery,
        buf: BackpressureBuffer[Observation[T]],
        sub: DisposableBase,
    ) -> Iterator[Observation[T]]:
        from dimos.memory2.buffer import ClosedError

        # Backfill phase — use snapshot query (without live) for the backfill
        last_id = -1
        for obs in self._iterate_snapshot(query):
            last_id = max(last_id, obs.id)
            yield obs

        # Live tail
        filters = query.filters
        try:
            while True:
                obs = buf.take()
                if obs.id <= last_id:
                    continue
                last_id = obs.id
                if filters and not all(f.matches(obs) for f in filters):
                    continue
                yield obs
        except (ClosedError, StopIteration):
            sub.dispose()

    def count(self, query: StreamQuery) -> int:
        return sum(1 for _ in self.iterate(query))

    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase:
        with self._lock:
            self._subscribers.append(buf)

        def _unsubscribe() -> None:
            with self._lock:
                try:
                    self._subscribers.remove(buf)
                except ValueError:
                    pass

        return Disposable(action=_unsubscribe)


class MemorySession(Session):
    """In-memory session. Each stream is backed by a ListBackend."""

    def _create_backend(self, name: str, payload_type: type[Any] | None = None) -> Backend[Any]:
        return ListBackend(name)


class MemoryStore(Store):
    """In-memory store for experimentation."""

    def session(self) -> MemorySession:
        return MemorySession()
