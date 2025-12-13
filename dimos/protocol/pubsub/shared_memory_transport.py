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

# ---------------------------------------------------------------------------
# SharedMemory Pub/Sub over unified IPC channels (CPU/CUDA)
# ---------------------------------------------------------------------------

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional

from dimos.protocol.pubsub.spec import PubSub, PubSubEncoderMixin, PickleEncoderMixin
from dimos.protocol.service.shmservice import SharedMemoryConfig, SharedMemoryService


@dataclass
class SHMTopic:
    """
    Simple topic wrapper to parallel LCM's Topic type.
    You can pass a plain string as well; __str__ handles both.
    """

    topic: str = ""

    def __str__(self) -> str:
        return self.topic


def _as_str_topic(t: str | SHMTopic) -> str:
    return t if isinstance(t, str) else str(t)


class SharedMemoryPubSubBase(PubSub[str | SHMTopic, Any], SharedMemoryService):
    """
    Pub/Sub over SharedMemory/CUDA-IPC, modeled after LCMPubSubBase.

    - Inherits the service for channel management, fanout, capacity, etc.
    - Exposes PubSub surface: publish(topic, bytes), subscribe(topic, cb), wait_for_message(...)
    - Encoders (bytes / pickle) are mixed in via PubSubEncoderMixin-like classes.
    """

    default_config = SharedMemoryConfig

    def __init__(self, **kwargs) -> None:
        # Initialize the service first (mirrors LCMPubSubBase)
        SharedMemoryService.__init__(self, **kwargs)
        # Initialize PubSub base (no-op but keeps symmetry with your LCM impl)
        super().__init__(**kwargs)

    # --- PubSub API (bytes on the wire) ------------------------------------

    def publish(self, topic: str | SHMTopic, message: bytes) -> None:
        """Publish a bytes payload to a topic."""
        SharedMemoryService.publish(self, _as_str_topic(topic), message)

    def subscribe(
        self, topic: str | SHMTopic, callback: Callable[[bytes, str | SHMTopic], Any]
    ) -> Callable[[], None]:
        """Subscribe with callback(message: bytes, topic). Returns unsubscribe fn."""
        t = _as_str_topic(topic)
        # Wrap so we give the *same* topic object back that we were called with (like LCM)
        return SharedMemoryService.subscribe(self, t, lambda msg, _t: callback(msg, topic))

    # --- Utility (mirrors LCMPubSubBase) -----------------------------------

    def wait_for_message(self, topic: str | SHMTopic, timeout: float = 1.0) -> Any:
        """
        Wait for one message on the topic. If this instance mixes in an encoder,
        it will decode before returning (just like your LCM wait_for_message).
        """
        received: Any = None
        evt = threading.Event()
        t = _as_str_topic(topic)

        def _handler(msg: bytes, _topic: str):
            nonlocal received
            try:
                if hasattr(self, "decode"):  # provided by encoder mixin
                    received = self.decode(msg, topic)  # type: ignore[misc]
                else:
                    received = msg
            finally:
                evt.set()

        unsub = self.subscribe(topic, _handler)
        try:
            evt.wait(timeout)
            return received
        finally:
            try:
                unsub()
            except Exception:
                pass


# ---- Encoders + concrete classes (parallel to LCM / PickleLCM) ------------


class SharedMemoryBytesEncoderMixin(PubSubEncoderMixin[str | SHMTopic, bytes]):
    """Identity encoder for raw bytes over SharedMemory."""

    def encode(self, msg: bytes, _: str | SHMTopic) -> bytes:
        if isinstance(msg, (bytes, bytearray, memoryview)):
            return bytes(msg)
        raise TypeError(f"SharedMemory expects bytes-like, got {type(msg)!r}")

    def decode(self, msg: bytes, _: str | SHMTopic) -> bytes:
        return msg


class SharedMemory(
    SharedMemoryBytesEncoderMixin,
    SharedMemoryPubSubBase,
):
    """SharedMemory pubsub that transports raw bytes."""

    ...


class PickleSharedMemory(
    PickleEncoderMixin[str | SHMTopic, Any],
    SharedMemoryPubSubBase,
):
    """SharedMemory pubsub that transports arbitrary Python objects via pickle."""

    ...
