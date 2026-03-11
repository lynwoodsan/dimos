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

from itertools import islice
import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.memory2.backend import Backend, LiveBackend
from dimos.memory2.buffer import BackpressureBuffer, KeepLast
from dimos.memory2.filter import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    Filter,
    NearFilter,
    PredicateFilter,
    StreamQuery,
    TagsFilter,
    TimeRangeFilter,
)
from dimos.memory2.transform import FnTransformer, Transformer
from dimos.memory2.type import EmbeddedObservation, Observation

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos.models.embedding.base import Embedding

T = TypeVar("T")
R = TypeVar("R")


class Stream(Generic[T]):
    """Lazy, pull-based stream over observations.

    Every filter/transform method returns a new Stream — no computation
    happens until iteration. Backends handle query application for stored
    data; transform sources apply filters as Python predicates.
    """

    def __init__(
        self,
        source: Backend[T] | Stream[Any],
        *,
        xf: Transformer[Any, T] | None = None,
        query: StreamQuery = StreamQuery(),
    ) -> None:
        self._source = source
        self._xf = xf
        self._query = query

    def is_live(self) -> bool:
        """True if this stream (or any ancestor in the chain) is in live mode."""
        if self._query.live_buffer is not None:
            return True
        if isinstance(self._source, Stream):
            return self._source.is_live()
        return False

    # ── Iteration ───────────────────────────────────────────────────

    def __iter__(self) -> Iterator[Observation[T]]:
        return self._build_iter()

    def _build_iter(self) -> Iterator[Observation[T]]:
        if isinstance(self._source, Stream):
            return self._iter_transform()
        # Backend handles all query application (including live if requested)
        return self._source.iterate(self._query)

    def _iter_transform(self) -> Iterator[Observation[T]]:
        """Iterate a transform source, applying query filters in Python."""
        assert isinstance(self._source, Stream) and self._xf is not None
        it: Iterator[Observation[T]] = self._xf(iter(self._source))

        # Apply filters as Python predicates
        filters = self._query.filters
        if filters:
            it = (obs for obs in it if all(f.matches(obs) for f in filters))

        # Text search — substring match
        if self._query.search_text is not None:
            needle = self._query.search_text.lower()
            it = (obs for obs in it if needle in str(obs.data).lower())

        # Vector search — brute-force cosine (materializes — rejects live)
        if self._query.search_vec is not None:
            if self.is_live():
                raise TypeError(
                    ".search() requires finite data — cannot rank an infinite live stream."
                )
            query_emb = self._query.search_vec
            scored = []
            for obs in it:
                emb = getattr(obs, "embedding", None)
                if emb is not None:
                    sim = float(emb @ query_emb)
                    scored.append(obs.derive(data=obs.data, similarity=sim))
            scored.sort(key=lambda o: getattr(o, "similarity", 0.0) or 0.0, reverse=True)
            k = self._query.search_k
            if k is not None:
                scored = scored[:k]
            it = iter(scored)

        # Sort if needed (materializes — rejects live)
        if self._query.order_field:
            if self.is_live():
                raise TypeError(
                    ".order_by() requires finite data — cannot sort an infinite live stream."
                )
            key = self._query.order_field
            desc = self._query.order_desc
            items = sorted(
                list(it),
                key=lambda obs: getattr(obs, key) if getattr(obs, key, None) is not None else 0,
                reverse=desc,
            )
            it = iter(items)

        # Offset + limit
        if self._query.offset_val:
            it = islice(it, self._query.offset_val, None)
        if self._query.limit_val is not None:
            it = islice(it, self._query.limit_val)

        return it

    # ── Query builders ──────────────────────────────────────────────

    def _replace_query(self, **overrides: Any) -> Stream[T]:
        q = self._query
        new_q = StreamQuery(
            filters=overrides.get("filters", q.filters),
            order_field=overrides.get("order_field", q.order_field),
            order_desc=overrides.get("order_desc", q.order_desc),
            limit_val=overrides.get("limit_val", q.limit_val),
            offset_val=overrides.get("offset_val", q.offset_val),
            live_buffer=overrides.get("live_buffer", q.live_buffer),
            search_vec=overrides.get("search_vec", q.search_vec),
            search_k=overrides.get("search_k", q.search_k),
            search_text=overrides.get("search_text", q.search_text),
        )
        return Stream(self._source, xf=self._xf, query=new_q)

    def _with_filter(self, f: Filter) -> Stream[T]:
        return self._replace_query(filters=(*self._query.filters, f))

    def after(self, t: float) -> Stream[T]:
        return self._with_filter(AfterFilter(t))

    def before(self, t: float) -> Stream[T]:
        return self._with_filter(BeforeFilter(t))

    def time_range(self, t1: float, t2: float) -> Stream[T]:
        return self._with_filter(TimeRangeFilter(t1, t2))

    def at(self, t: float, tolerance: float = 1.0) -> Stream[T]:
        return self._with_filter(AtFilter(t, tolerance))

    def near(self, pose: Any, radius: float) -> Stream[T]:
        return self._with_filter(NearFilter(pose, radius))

    def filter_tags(self, **tags: Any) -> Stream[T]:
        return self._with_filter(TagsFilter(tags))

    def order_by(self, field: str, desc: bool = False) -> Stream[T]:
        return self._replace_query(order_field=field, order_desc=desc)

    def limit(self, k: int) -> Stream[T]:
        return self._replace_query(limit_val=k)

    def offset(self, n: int) -> Stream[T]:
        return self._replace_query(offset_val=n)

    def search(self, query: Embedding, k: int) -> Stream[T]:
        """Return top-k observations by cosine similarity to *query*.

        The backend handles the actual computation. ListBackend does
        brute-force cosine; SqliteBackend (future) pushes down to vec0.
        """
        return self._replace_query(search_vec=query, search_k=k)

    def search_text(self, text: str) -> Stream[T]:
        """Filter observations whose data contains *text*.

        ListBackend does case-insensitive substring match;
        SqliteBackend (future) pushes down to FTS5.
        """
        return self._replace_query(search_text=text)

    # ── Functional API ──────────────────────────────────────────────

    def filter(self, pred: Callable[[Observation[T]], bool]) -> Stream[T]:
        """Filter by arbitrary predicate on the full Observation."""
        return self._with_filter(PredicateFilter(pred))

    def map(self, fn: Callable[[Observation[T]], Observation[R]]) -> Stream[Any]:
        """Transform each observation's data via callable."""
        return self.transform(FnTransformer(lambda obs: fn(obs)))

    # ── Transform ───────────────────────────────────────────────────

    def transform(self, xf: Transformer[T, R]) -> Stream[R]:
        """Wrap this stream with a transformer. Returns a new lazy Stream.

        When iterated, calls xf(iter(self)) — pulls lazily through the chain.
        """
        return Stream(source=self, xf=xf, query=StreamQuery())

    # ── Live mode ───────────────────────────────────────────────────

    def live(self, buffer: BackpressureBuffer[Observation[Any]] | None = None) -> Stream[T]:
        """Return a stream whose iteration never ends — backfill then live tail.

        Only valid on backend-backed streams whose backend implements
        LiveBackend. Call .live() before .transform(), not after.

        Default buffer: KeepLast(). The backend handles subscription, dedup,
        and backpressure — how it does so is its business.
        """
        if isinstance(self._source, Stream):
            raise TypeError(
                "Cannot call .live() on a transform stream. "
                "Call .live() on the source stream, then .transform()."
            )
        if not isinstance(self._source, LiveBackend):
            raise TypeError(f"Backend {self._source.name!r} does not support live mode.")
        buf = buffer if buffer is not None else KeepLast()
        return self._replace_query(live_buffer=buf)

    # ── Save ─────────────────────────────────────────────────────────

    def save(self, target: Stream[T]) -> Stream[T]:
        """Sync terminal: iterate self, append each obs to target's backend.

        Returns the target stream for continued querying.
        """
        if isinstance(target._source, Stream):
            raise TypeError("Cannot save to a transform stream. Target must be backend-backed.")
        backend = target._source
        for obs in self:
            backend.append(obs)
        return target

    # ── Terminals ───────────────────────────────────────────────────

    def fetch(self) -> list[Observation[T]]:
        """Materialize all observations into a list."""
        if self.is_live() and self._query.limit_val is None:
            raise TypeError(
                ".fetch() on a live stream without .limit() would collect forever. "
                "Use .limit(n).fetch(), .drain(), or .save(target) instead."
            )
        return list(self)

    def first(self) -> Observation[T]:
        """Return the first matching observation."""
        it = iter(self.limit(1))
        try:
            return next(it)
        except StopIteration:
            raise LookupError("No matching observation") from None

    def last(self) -> Observation[T]:
        """Return the last matching observation (by timestamp)."""
        return self.order_by("ts", desc=True).first()

    def count(self) -> int:
        """Count matching observations."""
        if isinstance(self._source, Backend):
            return self._source.count(self._query)
        if self.is_live():
            raise TypeError(".count() on a live transform stream would block forever.")
        return sum(1 for _ in self)

    def exists(self) -> bool:
        """Check if any matching observation exists."""
        return next(iter(self.limit(1)), None) is not None

    def drain(self) -> int:
        """Consume all observations, discarding results. Returns count consumed.

        Use for side-effect pipelines (e.g. live embed-and-store) where you
        don't need to collect results in memory.
        """
        n = 0
        for _ in self:
            n += 1
        return n

    # ── Write ───────────────────────────────────────────────────────

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: Any | None = None,
        tags: dict[str, Any] | None = None,
        embedding: Embedding | None = None,
    ) -> Observation[T]:
        """Append to the backing store. Only works if source is a Backend."""
        if isinstance(self._source, Stream):
            raise TypeError("Cannot append to a transform stream. Append to the source stream.")
        _ts = ts if ts is not None else time.time()
        _tags = tags or {}
        if embedding is not None:
            obs: Observation[T] = EmbeddedObservation(
                id=-1,
                ts=_ts,
                pose=pose,
                tags=_tags,
                _data=payload,
                embedding=embedding,
            )
        else:
            obs = Observation(id=-1, ts=_ts, pose=pose, tags=_tags, _data=payload)
        return self._source.append(obs)
