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

"""TwitchVotes: extends TwitchChat with vote tallying.

Collects chat commands over a configurable window and publishes the
winning command as a :class:`VoteResult`.

Voting modes: plurality, majority, weighted_recent, runoff.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
from enum import Enum
import re
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.stream.twitch.module import TwitchChat, TwitchChatConfig, TwitchMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class VoteResult:
    winner: str = ""
    total_votes: int = 0
    timestamp: float = 0.0


class VoteMode(str, Enum):
    PLURALITY = "plurality"
    MAJORITY = "majority"
    WEIGHTED_RECENT = "weighted_recent"
    RUNOFF = "runoff"


class TwitchVotesConfig(TwitchChatConfig):
    keywords: list[str] = ["forward", "back", "left", "right", "stop"]
    """Valid vote keywords. Also used to build regex patterns on the base TwitchChat."""

    vote_window_seconds: float = 5.0
    min_votes_threshold: int = 1
    vote_mode: VoteMode = VoteMode.PLURALITY


# ── Vote tallying ──


def _tally_plurality(votes: list[tuple[str, float, str]]) -> str | None:
    counts = Counter(cmd for cmd, _, _ in votes)
    if not counts:
        return None
    return counts.most_common(1)[0][0]


def _tally_majority(votes: list[tuple[str, float, str]]) -> str | None:
    counts = Counter(cmd for cmd, _, _ in votes)
    total = sum(counts.values())
    if total == 0:
        return None
    winner, count = counts.most_common(1)[0]
    return winner if count > total / 2 else None


def _tally_weighted_recent(
    votes: list[tuple[str, float, str]], window_start: float, window_end: float
) -> str | None:
    if not votes:
        return None
    duration = max(window_end - window_start, 0.001)
    weighted: Counter[str] = Counter()
    for cmd, ts, _ in votes:
        weight = 0.5 + 0.5 * ((ts - window_start) / duration)
        weighted[cmd] += int(weight * 1000)
    return weighted.most_common(1)[0][0] if weighted else None


def _tally_runoff(votes: list[tuple[str, float, str]]) -> str | None:
    counts = Counter(cmd for cmd, _, _ in votes)
    total = sum(counts.values())
    if total == 0:
        return None
    winner, count = counts.most_common(1)[0]
    if count > total / 2:
        return winner

    top2 = {cmd for cmd, _ in counts.most_common(2)}
    if len(top2) < 2:
        return winner

    # For each voter, use their latest vote if it's in top2
    latest: dict[str, str] = {}
    for cmd, _, voter in votes:
        latest[voter] = cmd

    runoff_counts: Counter[str] = Counter()
    for _voter, cmd in latest.items():
        if cmd in top2:
            runoff_counts[cmd] += 1

    return runoff_counts.most_common(1)[0][0] if runoff_counts else winner


class TwitchVotes(TwitchChat):
    """Extends TwitchChat with vote tallying.

    Chat messages matching configured keywords are collected over a time
    window, then the winning command is published as a :class:`VoteResult`
    on ``vote_results``.
    """

    default_config = TwitchVotesConfig
    config: TwitchVotesConfig  # type narrowing for mypy

    vote_results: Out[VoteResult]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._votes: deque[tuple[str, float, str]] = deque()
        self._votes_lock = threading.Lock()
        self._vote_thread: threading.Thread | None = None
        self._valid_keywords: frozenset[str] = frozenset()

    def _handle_message(self, message: Any) -> None:
        """Extend base to also record votes from matching messages."""
        super()._handle_message(message)

        content: str = message.content or ""
        prefix = self.config.bot_prefix
        if not content.strip().startswith(prefix):
            return
        cmd = content.strip()[len(prefix) :].strip().lower()
        author = message.author.name if message.author else "anonymous"

        if cmd in self._valid_keywords:
            with self._votes_lock:
                self._votes.append((cmd, time.time(), author))

    @rpc
    def start(self) -> None:
        # Auto-generate patterns from keywords so the base class filters correctly
        if self.config.keywords and not self.config.patterns:
            escaped = "|".join(re.escape(k) for k in self.config.keywords)
            self.config.patterns = [rf"^{re.escape(self.config.bot_prefix)}(?:{escaped})\b"]

        self._valid_keywords = frozenset(self.config.keywords)

        super().start()

        self._vote_thread = threading.Thread(
            target=self._vote_loop, daemon=True, name="twitch-vote"
        )
        self._vote_thread.start()
        logger.info(
            "[TwitchVotes] Vote loop started",
            vote_mode=self.config.vote_mode.value,
            window=self.config.vote_window_seconds,
        )

    @rpc
    def stop(self) -> None:
        if self._vote_thread is not None:
            self._vote_thread.join(timeout=2)
            self._vote_thread = None
        super().stop()

    def record_vote(self, command: str, voter: str = "anonymous") -> None:
        """Record a vote programmatically (for testing)."""
        cmd = command.lower().strip()
        if cmd not in self._valid_keywords:
            return
        with self._votes_lock:
            self._votes.append((cmd, time.time(), voter))

    def _vote_loop(self) -> None:
        while True:
            window_start = time.time()
            time.sleep(self.config.vote_window_seconds)
            window_end = time.time()

            # Atomically drain votes within window
            cutoff = window_end - self.config.vote_window_seconds
            with self._votes_lock:
                current_votes = [(cmd, ts, voter) for cmd, ts, voter in self._votes if ts >= cutoff]
                self._votes.clear()

            if len(current_votes) < self.config.min_votes_threshold:
                continue

            winner = self._tally(current_votes, window_start, window_end)
            if winner is None:
                continue

            logger.info(
                "[TwitchVotes] Winner: %s (%d votes)",
                winner,
                len(current_votes),
            )
            self.vote_results.publish(
                VoteResult(winner=winner, total_votes=len(current_votes), timestamp=window_end)
            )

    def _tally(
        self,
        votes: list[tuple[str, float, str]],
        window_start: float,
        window_end: float,
    ) -> str | None:
        mode = self.config.vote_mode
        if mode == VoteMode.PLURALITY:
            return _tally_plurality(votes)
        elif mode == VoteMode.MAJORITY:
            return _tally_majority(votes)
        elif mode == VoteMode.WEIGHTED_RECENT:
            return _tally_weighted_recent(votes, window_start, window_end)
        elif mode == VoteMode.RUNOFF:
            return _tally_runoff(votes)
        return _tally_plurality(votes)


twitch_votes = TwitchVotes.blueprint
