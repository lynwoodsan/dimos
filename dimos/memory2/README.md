# memory2

Observation storage and streaming layer for DimOS. Pull-based, lazy, composable.

## Architecture

```
             Live Sensor Data
                    ↓
Store → Session → Stream → [filters / transforms / terminals] → Stream  → [filters / transforms / terminals] → Stream → Live hooks
                    ↓                                              ↓                                             ↓
                 Backend (ListBackend, SqliteBackend)           Backend                                      In Memory
```

**Store** owns a storage location (file, in-memory). **Session** manages named streams over a shared connection. **Stream** is the query/iteration surface — lazy until a terminal is called.

## Modules

| Module         | What                                                              |
|----------------|-------------------------------------------------------------------|
| `stream.py`    | Stream node — filters, transforms, terminals                      |
| `backend.py`   | Backend / LiveBackend protocols, VectorStore / BlobStore ABCs     |
| `filter.py`    | StreamQuery dataclass, filter types                               |
| `transform.py` | Transformer protocol, FnTransformer, QualityWindow                |
| `buffer.py`    | Backpressure buffers for live mode (KeepLast, Bounded, Unbounded) |
| `store.py`     | Store / Session base classes, StreamNamespace                     |
| `type.py`      | Observation, EmbeddedObservation dataclasses                      |
| `embed.py`     | EmbedImages / EmbedText transformers                              |

## Subpackages

| Package      | What                                                 | Docs                                             |
|--------------|------------------------------------------------------|--------------------------------------------------|
| `impl/`      | Backend implementations (ListBackend, SqliteBackend) | [impl/README.md](impl/README.md)                 |
| `blobstore/` | Pluggable blob storage (file, sqlite)                | [blobstore/blobstore.md](blobstore/blobstore.md) |
| `codecs/`    | Encode/decode for storage (pickle, JPEG, LCM)        | [codecs/README.md](codecs/README.md)             |

## Docs

| Doc | What |
|-----|------|
| [streaming.md](streaming.md) | Lazy vs materializing vs terminal — evaluation model, live safety |
| [embeddings.md](embeddings.md) | Embedding layer design — EmbeddedObservation, vector search, EmbedImages/EmbedText |
| [blobstore/blobstore.md](blobstore/blobstore.md) | BlobStore architecture — separate payload storage from metadata |

## Quick start

```python
from dimos.memory2 import MemoryStore

store = MemoryStore()
with store.session() as session:
    images = session.stream("images")

    # Write
    images.append(frame, ts=time.time(), pose=(x, y, z), tags={"camera": "front"})

    # Query
    recent = images.after(t).limit(10).fetch()
    nearest = images.near(pose, radius=2.0).fetch()
    latest = images.last()

    # Transform
    edges = images.transform(Canny()).save(session.stream("edges"))

    # Live
    for obs in images.live().transform(process):
        handle(obs)

    # Embed + search
    images.transform(EmbedImages(clip)).save(session.stream("embedded"))
    results = session.stream("embedded").search(query_vec, k=5).fetch()
```

## Implementations

| Backend         | Status   | Storage                                |
|-----------------|----------|----------------------------------------|
| `ListBackend`   | Complete | In-memory (lists + brute-force search) |
| `SqliteBackend` | Stub     | SQLite (WAL, FTS5, vec0)               |
