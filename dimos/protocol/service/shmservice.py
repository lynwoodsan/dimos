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

# dimos/protocol/service/shmservice.py
from __future__ import annotations

import os
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional

import numpy as np

from dimos.protocol.service.spec import Service
from dimos.utils.logging_config import setup_logger
from dimos.utils.ipc_factory import CPU_IPC_Factory

logger = setup_logger("dimos.protocol.service.shmservice")


@dataclass
class SharedMemoryConfig:
    """Configuration for SharedMemoryService."""

    prefer: str = "auto"  # "auto"|"cuda"|"cpu"
    default_capacity: int = 64 * 1024
    close_channels_on_stop: bool = True


class SharedMemoryService(Service[SharedMemoryConfig]):
    """
    Core SharedMemory/CUDA-IPC pub/sub service:
      * One frame channel per topic
      * Per-topic fanout thread
      * Byte payload wire format: [len:uint32_le] + payload (capacity-limited)
    """

    class _TopicState:
        __slots__ = (
            "channel",
            "subs",
            "stop",
            "thread",
            "last_seq",
            "shape",
            "dtype",
            "capacity",
            "is_cuda",
            "cp",
            "last_local_payload",
        )

        def __init__(self, channel, capacity: int, is_cuda: bool, cp_mod):
            self.channel = channel
            self.capacity = int(capacity)
            self.shape = (self.capacity + 4,)  # 4 bytes for length header
            self.dtype = np.uint8
            self.subs: list[Callable[[bytes, str], None]] = []
            self.stop = threading.Event()
            self.thread: Optional[threading.Thread] = None
            self.last_seq = 0
            self.is_cuda = is_cuda
            self.cp = cp_mod
            self.last_local_payload = None

    default_config = SharedMemoryConfig

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._topics: Dict[str, SharedMemoryService._TopicState] = {}
        self._lock = threading.Lock()

    # ---------------- lifecycle ----------------

    def start(self):
        pref = (self.config.prefer or "auto").lower()
        backend = os.getenv("DIMOS_IPC_BACKEND", pref).lower()
        logger.info(f"SharedMemoryService starting (backend={backend})")

    def stop(self):
        with self._lock:
            for st in list(self._topics.values()):
                # stop fanout
                try:
                    if st.thread:
                        st.stop.set()
                        st.thread.join(timeout=0.5)
                        st.thread = None
                except Exception:
                    pass
                # close channel
                if self.config.close_channels_on_stop:
                    try:
                        st.channel.close()
                    except Exception:
                        pass
            self._topics.clear()
        logger.info("SharedMemoryService stopped.")

    # ---------------- pub/sub (bytes) -----------

    def publish(self, topic: str, message: bytes) -> None:
        if not isinstance(message, (bytes, bytearray, memoryview)):
            raise TypeError(f"publish expects bytes-like, got {type(message)!r}")
        st = self._ensure_topic(topic)
        payload = memoryview(message)
        L = len(payload)
        if L > st.capacity:
            raise ValueError(f"Payload too large: {L} > capacity {st.capacity}")

        host = np.zeros(st.shape, dtype=st.dtype)
        host[:4] = np.frombuffer(struct.pack("<I", L), dtype=np.uint8)
        if L:
            host[4 : 4 + L] = np.frombuffer(payload, dtype=np.uint8)

        if st.is_cuda:
            try:
                d = st.cp.asarray(host)  # type: ignore[attr-defined]
                st.channel.publish(d)
            except Exception:
                st.channel.publish(host)
        else:
            st.channel.publish(host)

        payload_bytes = bytes(payload)  # snapshot before we drop 'payload' view
        for cb in list(st.subs):
            try:
                cb(payload_bytes, topic)
            except Exception:
                pass
        # mark so fanout can suppress the duplicate originating from this process
        st.last_local_payload = payload_bytes

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        st = self._ensure_topic(topic)
        st.subs.append(callback)
        if st.thread is None:
            st.thread = threading.Thread(target=self._fanout_loop, args=(topic, st), daemon=True)
            st.thread.start()

        def _unsub():
            try:
                st.subs.remove(callback)
            except ValueError:
                pass
            if not st.subs and st.thread:
                st.stop.set()
                st.thread.join(timeout=0.5)
                st.thread = None
                st.stop.clear()

        return _unsub

    def reconfigure(self, topic: str, *, capacity: int) -> dict:
        st = self._ensure_topic(topic)
        new_cap = int(capacity)
        new_shape = (new_cap + 4,)
        desc = st.channel.reconfigure(new_shape, np.uint8)
        st.capacity = new_cap
        st.shape = new_shape
        st.dtype = np.uint8
        st.last_seq = -1
        return desc

    # --------------- internals ------------------

    def _ensure_topic(self, topic: str) -> _TopicState:
        with self._lock:
            st = self._topics.get(topic)
            if st is not None:
                return st

            prefer = (os.getenv("DIMOS_IPC_BACKEND") or self.config.prefer or "auto").lower()
            shape = (int(self.config.default_capacity) + 4,)
            dtype = np.uint8

            is_cuda = False
            cp_mod = None
            if prefer in ("cuda", "auto"):
                try:
                    import cupy as cp  # type: ignore
                    from dimos.utils.ipc_factory import CUDA_IPC_Factory

                    ch = CUDA_IPC_Factory.create(shape, dtype=dtype)
                    is_cuda = True
                    cp_mod = cp
                except Exception:
                    ch = CPU_IPC_Factory.create(shape, dtype=dtype)
            else:
                ch = CPU_IPC_Factory.create(shape, dtype=dtype)

            st = SharedMemoryService._TopicState(
                ch, int(self.config.default_capacity), is_cuda, cp_mod
            )
            self._topics[topic] = st
            return st

    def _fanout_loop(self, topic: str, st: _TopicState):
        while not st.stop.is_set():
            seq, ts_ns, view = st.channel.read(last_seq=st.last_seq, require_new=True)
            if view is None:
                time.sleep(0.001)
                continue
            st.last_seq = seq

            if st.is_cuda:
                try:
                    host = st.cp.asnumpy(view)  # type: ignore[attr-defined]
                except Exception:
                    host = np.array(view, copy=True)
            else:
                host = np.array(view, copy=True)

            try:
                L = struct.unpack("<I", host[:4].tobytes())[0]
                if L == 0:
                    continue
                if L < 0 or L > st.capacity:
                    continue
                payload = host[4 : 4 + L].tobytes()
                if st.last_local_payload is not None and payload == st.last_local_payload:
                    st.last_local_payload = None
                    continue
            except Exception:
                continue

            for cb in list(st.subs):
                try:
                    cb(payload, topic)
                except Exception:
                    pass
