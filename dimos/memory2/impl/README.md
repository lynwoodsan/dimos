# impl — Backend implementations

Storage backends for memory2. Each backend implements the `Backend` protocol (and optionally `LiveBackend`) to provide observation storage with query support.

## Existing backends

| Backend         | File        | Status   | Storage                             |
|-----------------|-------------|----------|-------------------------------------|
| `ListBackend`   | `memory.py` | Complete | In-memory lists, brute-force search |
| `SqliteBackend` | `sqlite.py` | Stub     | SQLite (WAL, FTS5, vec0)            |

## Writing a new backend

### 1. Implement the Backend protocol

```python
from dimos.memory2.backend import Backend
from dimos.memory2.filter import StreamQuery
from dimos.memory2.type import Observation

class MyBackend(Generic[T]):
    @property
    def name(self) -> str:
        return self._name

    def append(self, obs: Observation[T]) -> Observation[T]:
        """Assign an id and store. Return the stored observation."""
        obs.id = self._next_id
        self._next_id += 1
        # ... persist obs ...
        return obs

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        """Yield observations matching the query."""
        # The backend is responsible for applying ALL query fields:
        #   query.filters      — list of Filter objects (each has .matches(obs))
        #   query.order_field   — sort field name (e.g. "ts")
        #   query.order_desc    — sort direction
        #   query.limit_val     — max results
        #   query.offset_val    — skip first N
        #   query.search_vec    — Embedding for vector search
        #   query.search_k      — top-k for vector search
        #   query.search_text   — substring text search
        #   query.live_buffer   — if set, switch to live mode (see LiveBackend)
        ...

    def count(self, query: StreamQuery) -> int:
        """Count matching observations."""
        ...
```

`Backend` is a `@runtime_checkable` Protocol — no base class needed, just implement the methods.

### 2. Add LiveBackend support (optional)

If your backend supports live subscriptions (push notifications on new observations):

```python
from dimos.memory2.backend import LiveBackend

class MyBackend(Generic[T]):
    # ... Backend methods ...

    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase:
        """Register a buffer for push notifications. Return a disposable to unsubscribe."""
        ...
```

The `iterate()` method should check `query.live_buffer`:
- If `None`: return a snapshot iterator
- If set: subscribe before backfill, then yield a live tail that deduplicates by `obs.id`

See `ListBackend._iterate_live()` for the reference implementation.

### 3. Add Store and Session

```python
from dimos.memory2.store import Session, Store

class MySession(Session):
    def _create_backend(self, name: str, payload_type: type | None = None) -> Backend:
        return MyBackend(self._conn, name)

class MyStore(Store):
    def session(self) -> MySession:
        return MySession(...)
```

### 4. Add to the grid test

In `test_impl.py`, add your store to the fixture so all standard tests run against it:

```python
@pytest.fixture(params=["memory", "sqlite", "mybackend"])
def store(request, tmp_path):
    if request.param == "mybackend":
        return MyStore(...)
    ...
```

Use `pytest.mark.xfail` for features not yet implemented — the grid test covers: append, fetch, iterate, count, first/last, exists, all filters, ordering, limit/offset, embeddings, text search.

### Query contract

The backend must handle the full `StreamQuery`. The Stream layer does NOT apply filters to backend results — it trusts the backend to do so. This allows backends to push queries down to their native query engine (SQL WHERE, FTS5 MATCH, vec0 knn).

For filters, each `Filter` object has a `.matches(obs) -> bool` method that backends can use directly if they don't have a native equivalent.
