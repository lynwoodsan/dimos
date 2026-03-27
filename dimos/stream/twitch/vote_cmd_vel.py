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

"""VoteCmdVel: demo bridge that converts VoteResult into Twist on cmd_vel."""

from __future__ import annotations

import time

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.stream.twitch.votes import VoteResult
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


_COMMAND_MAP: dict[str, tuple[float, float, float, float, float, float]] = {
    "forward": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "back": (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "left": (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    "right": (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
    "stop": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
}


def _command_to_twist(command: str, linear_speed: float, angular_speed: float) -> Twist:
    scales = _COMMAND_MAP.get(command, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    t = Twist()
    t.linear.x = scales[0] * linear_speed
    t.linear.y = scales[1] * linear_speed
    t.linear.z = scales[2] * linear_speed
    t.angular.x = scales[3] * angular_speed
    t.angular.y = scales[4] * angular_speed
    t.angular.z = scales[5] * angular_speed
    return t


class VoteCmdVelConfig(ModuleConfig):
    linear_speed: float = 0.3
    angular_speed: float = 0.5
    command_duration: float = 1.0


class VoteCmdVel(Module["VoteCmdVelConfig"]):
    """Demo bridge: subscribes to ``vote_results`` and publishes winning
    commands as Twist on ``cmd_vel`` for ``command_duration`` seconds."""

    default_config = VoteCmdVelConfig

    vote_results: In[VoteResult]
    cmd_vel: Out[Twist]

    @rpc
    def start(self) -> None:
        super().start()
        self.vote_results.subscribe(self._on_vote_result)
        logger.info("[VoteCmdVel] Listening for vote results")

    def _on_vote_result(self, result: VoteResult) -> None:
        if not result.winner:
            return

        twist = _command_to_twist(
            result.winner, self.config.linear_speed, self.config.angular_speed
        )
        logger.info("[VoteCmdVel] Executing: %s", result.winner)

        end_time = time.time() + self.config.command_duration
        while time.time() < end_time:
            self.cmd_vel.publish(twist)
            time.sleep(0.1)

        self.cmd_vel.publish(_command_to_twist("stop", 0.0, 0.0))


vote_cmd_vel = VoteCmdVel.blueprint
