#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

"""
Shared memory transport that mimics LCMTransport interface.
"""

from __future__ import annotations

from typing import Callable, Optional, Tuple, List
import os
import threading
import time
import numpy as np

from dimos.core.transport import PubSubTransport
from dimos.utils.ipc_factory import CPU_IPC_Factory  # CUDA imported lazily
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.core.shared_memory")


class SharedMemoryImageTransport(PubSubTransport):
    """Drop-in replacement for LCMTransport using unified IPC channels (CPU/CUDA), Dask-safe."""

    def __init__(self, topic: str, shape: Tuple[int, ...], dtype, *, prefer: str = "cuda"):
        """
        Args:
            topic: semantic only (not used for SHM naming)
            shape: (H, W, C) etc.
            dtype: NumPy dtype or dtype string
            prefer: "auto" | "cuda" | "cpu" (env DIMOS_IPC_BACKEND overrides)
        """
        super().__init__(topic)
        self.shape = tuple(shape)
        self.dtype = np.dtype(dtype)

        backend = (os.getenv("DIMOS_IPC_BACKEND") or prefer or "auto").lower()
        self._is_cuda = False
        self._cp = None  # do NOT pickle module objects

        if backend in ("cuda", "auto"):
            try:
                import cupy as cp  # type: ignore
                from dimos.utils.ipc_factory import CUDA_IPC_Factory

                self._channel = CUDA_IPC_Factory.create(self.shape, dtype=self.dtype)
                self._is_cuda = True
                self._cp = cp
            except Exception:
                self._channel = CPU_IPC_Factory.create(self.shape, dtype=self.dtype)
        else:
            self._channel = CPU_IPC_Factory.create(self.shape, dtype=self.dtype)

        # Polling fanout (not pickled)
        self._subscribers: List[tuple[Callable, Optional[object]]] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_seq = -1

        # Cache a lightweight descriptor for pickling/reattach
        self._desc = (
            self._channel.descriptor()
        )  # includes kind, shape, dtype, shm names, epoch, etc.

    # ---------- Publish (LCM-style) ----------

    def broadcast(self, _, msg):
        """Broadcast a frame. Accepts NumPy or CuPy or an object with .data."""
        data = getattr(msg, "data", msg)

        if (
            tuple(getattr(data, "shape", ())) != self.shape
            or np.dtype(getattr(data, "dtype", None)) != self.dtype
        ):
            raise ValueError(
                f"Data shape/dtype mismatch: got {getattr(data, 'shape', None)} {getattr(data, 'dtype', None)}, "
                f"expected {self.shape} {self.dtype}"
            )

        if self._is_cuda:
            if not hasattr(data, "__cuda_array_interface__"):
                if self._cp is None:
                    raise RuntimeError("CUDA backend selected but CuPy unavailable")
                data = self._cp.asarray(data)  # H→D
        else:
            if hasattr(data, "__cuda_array_interface__"):
                try:
                    import cupy as cp  # type: ignore

                    data = cp.asnumpy(data)  # D→H
                except Exception:
                    data = np.array(data, copy=True)

        self._channel.publish(data)

    # ---------- Subscribe & fanout ----------

    def subscribe(self, callback: Callable, selfstream=None):
        """
        Subscribe to frames; callback receives NumPy arrays.
        If selfstream is provided, calls callback(selfstream, image_np).
        Returns an unsubscribe lambda.
        """
        self._subscribers.append((callback, selfstream))
        if self._thread is None:
            self._thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._thread.start()
        return lambda: self._unsubscribe(callback, selfstream)

    def _unsubscribe(self, callback: Callable, selfstream):
        try:
            self._subscribers.remove((callback, selfstream))
        except ValueError:
            pass
        if not self._subscribers and self._thread:
            self._stop.set()
            self._thread.join(timeout=0.5)
            self._thread = None
            self._stop.clear()

    def _poll_loop(self):
        while not self._stop.is_set():
            seq, ts_ns, view = self._channel.read(last_seq=self._last_seq, require_new=True)
            if view is None:
                time.sleep(0.001)
                continue
            self._last_seq = seq

            # Always deliver NumPy to consumers
            if self._is_cuda:
                try:
                    import cupy as cp  # type: ignore

                    img_np = cp.asnumpy(view)
                except Exception:
                    img_np = np.array(view, copy=True)
            else:
                img_np = np.array(view, copy=True)

            for cb, selfstream in list(self._subscribers):
                try:
                    cb(img_np) if selfstream is None else cb(selfstream, img_np)
                except Exception:
                    # best-effort; don't kill the loop
                    pass

    # ---------- Lifecycle ----------

    def close(self):
        try:
            if self._thread:
                self._stop.set()
                self._thread.join(timeout=0.5)
        finally:
            self._thread = None
            self._subscribers.clear()
            try:
                self._channel.close()
            except Exception:
                pass

    # ---------- Dask/cloudpickle support ----------

    def __getstate__(self):
        """
        Serialize a descriptor-only state; drop threads, locks, and module objects.
        """
        # Use cached descriptor to avoid regenerating CUDA IPC handles on reader side.
        desc = getattr(self, "_desc", None)
        if desc is None:
            desc = self._channel.descriptor()
        return {
            "topic": getattr(self, "topic", None),
            "shape": self.shape,
            "dtype": self.dtype.str,
            "backend": "cuda" if self._is_cuda else "cpu",
            "desc": desc,  # used to reattach on the other side
        }

    def __setstate__(self, state):
        """
        Rebuild channel from descriptor; recreate non-picklable fields.
        """
        # Basic fields
        topic = state.get("topic")
        logger.info(f"__setstate__ core/shared_memory STATE: {state}")
        if topic is not None:
            # PubSubTransport likely stores topic in base __dict__
            try:
                super().__init__(topic)  # type: ignore[misc]
            except Exception:
                self.topic = topic  # fallback if base ctor does anything else
        self.shape = tuple(state["shape"])
        self.dtype = np.dtype(state["dtype"])

        desc = state["desc"]
        kind = (desc.get("kind") or state.get("backend") or "cpu").lower()

        if kind == "cuda":
            try:
                from dimos.utils.ipc_factory import CUDA_IPC_Factory

                self._channel = CUDA_IPC_Factory.attach(desc)
                self._is_cuda = True
                import cupy as cp  # type: ignore

                self._cp = cp
            except Exception:
                # Fallback to CPU attach if CUDA unavailable here
                self._channel = CPU_IPC_Factory.attach(desc)
                self._is_cuda = False
                self._cp = None
        else:
            self._channel = CPU_IPC_Factory.attach(desc)
            self._is_cuda = False
            self._cp = None

        # Recreate non-serializable fields
        self._subscribers = []
        self._stop = threading.Event()
        self._thread = None
        self._last_seq = -1
        self._desc = desc

    # ---------- Optional: runtime resize ----------

    def reconfigure(self, shape: Tuple[int, ...], dtype) -> dict:
        """
        Safely change (shape, dtype). Returns a new descriptor the caller can propagate.
        Subscribers will continue after the caller updates any remote attachments.
        """
        new_shape = tuple(shape)
        new_dtype = np.dtype(dtype)
        desc = self._channel.reconfigure(new_shape, new_dtype)
        self.shape, self.dtype = new_shape, new_dtype
        self._last_seq = -1
        self._desc = desc
        return desc
