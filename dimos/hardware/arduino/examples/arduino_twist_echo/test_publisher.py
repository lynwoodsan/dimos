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

"""TestPublisher: emits a Twist every 500ms, listens for echoes."""

from __future__ import annotations

import threading
import time

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TestPublisherConfig(ModuleConfig):
    publish_period_s: float = 0.5


class TestPublisher(Module):
    """Publishes a Twist on ``cmd_out`` every publish_period_s, prints any echo
    received on ``echo_in``."""

    config: TestPublisherConfig

    cmd_out: Out[Twist]
    echo_in: In[Twist]

    _thread: threading.Thread | None = None
    _stopping: bool = False
    _counter: int = 0

    @rpc
    def start(self) -> None:
        super().start()
        self._stopping = False
        self.echo_in.subscribe(self._on_echo)
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        logger.info("TestPublisher started")

    @rpc
    def stop(self) -> None:
        self._stopping = True
        if self._thread is not None:
            self._thread.join(timeout=2)
        super().stop()

    def _publish_loop(self) -> None:
        while not self._stopping:
            self._counter += 1
            twist = Twist(
                linear=Vector3(self._counter * 0.1, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, self._counter * 0.05),
            )
            self.cmd_out.publish(twist)
            logger.info(
                "Published twist",
                n=self._counter,
                linear_x=twist.linear.x,
                angular_z=twist.angular.z,
            )
            time.sleep(self.config.publish_period_s)

    def _on_echo(self, msg: Twist) -> None:
        logger.info(
            "Received echo",
            linear_x=msg.linear.x,
            linear_y=msg.linear.y,
            linear_z=msg.linear.z,
            angular_z=msg.angular.z,
        )
