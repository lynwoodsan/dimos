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

from typing import Any

from dimos.memory2.stream import Stream
from dimos.memory2.transform import Transformer


class StreamModule:
    """Deploy a memory2 stream pipeline as a Module in a blueprint.

    Wraps any unbound :class:`Stream` chain (or a single :class:`Transformer`)
    into a Module with ``In``/``Out`` ports suitable for blueprint deployment::

        # Unbound stream pipeline:
        StreamModule.blueprint(
            pipeline=Stream().transform(VoxelMap(voxel_size=0.05)).map(postprocess),
            input=("lidar", PointCloud2),
            output=("global_map", PointCloud2),
        )

        # Single transformer shorthand:
        StreamModule.blueprint(
            pipeline=VoxelMap(voxel_size=0.05),
            input=("lidar", PointCloud2),
            output=("global_map", PointCloud2),
        )
    """

    @staticmethod
    def blueprint(
        *,
        pipeline: Transformer[Any, Any] | Stream[Any],
        input: tuple[str, type],
        output: tuple[str, type],
        **config_kwargs: Any,
    ) -> Any:  # returns Blueprint, but avoid circular import in annotation
        from reactivex.disposable import Disposable

        from dimos.core.blueprints import Blueprint
        from dimos.core.core import rpc
        from dimos.core.module import Module
        from dimos.core.stream import In, Out

        in_name, in_type = input
        out_name, out_type = output
        _pipeline = pipeline

        class _Module(Module):
            def __init__(self, **kwargs: Any) -> None:
                from dimos.memory2.store.memory import MemoryStore

                super().__init__(**kwargs)
                self._store = MemoryStore()

            @rpc
            def start(self) -> None:
                super().start()
                self._store.start()

                stream: Stream[Any] = self._store.stream(in_name, in_type)
                inp_port = getattr(self, in_name)
                out_port = getattr(self, out_name)

                unsub = inp_port.subscribe(lambda msg: stream.append(msg))
                self._disposables.add(Disposable(unsub))

                if isinstance(_pipeline, Stream):
                    bound = stream.live().chain(_pipeline)
                else:
                    bound = stream.live().transform(_pipeline)
                self._disposables.add(bound.publish(out_port))

            @rpc
            def stop(self) -> None:
                super().stop()
                self._store.stop()

        _Module.__name__ = "StreamModule"
        _Module.__qualname__ = "StreamModule"
        _Module.__annotations__[in_name] = In[in_type]  # type: ignore[valid-type]
        _Module.__annotations__[out_name] = Out[out_type]  # type: ignore[valid-type]

        return Blueprint.create(_Module, **config_kwargs)
