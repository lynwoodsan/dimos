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

"""unitree-go2-twitch — Twitch Plays Go2 (demo).

Viewers in Twitch chat vote on robot commands (!forward, !left, etc.).
The winning command each voting window is sent to the Go2 via cmd_vel.

Usage::

    export DIMOS_TWITCH_TOKEN=oauth:your_token
    export DIMOS_CHANNEL_NAME=your_channel
    dimos run unitree-go2-twitch --robot-ip 192.168.123.161

Or with custom voting::

    dimos run unitree-go2-twitch --robot-ip 192.168.123.161 \\
        --vote-mode weighted_recent --vote-window-seconds 3
"""

from dimos.core.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.stream.twitch.vote_cmd_vel import VoteCmdVel
from dimos.stream.twitch.votes import TwitchVotes

unitree_go2_twitch = autoconnect(
    unitree_go2_basic,
    TwitchVotes.blueprint(),
    VoteCmdVel.blueprint(),
).global_config(n_workers=4, robot_model="unitree_go2")

__all__ = ["unitree_go2_twitch"]
