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

"""TwitchChat: connects to a Twitch channel and publishes chat messages.

Publishes all messages on ``raw_messages``, and a subset matching regex
patterns on ``filtered_messages``.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
import os
import re
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class TwitchMessage:
    author: str = ""
    content: str = ""
    channel: str = ""
    timestamp: float = 0.0
    is_subscriber: bool = False
    is_mod: bool = False
    badges: dict[str, str] = field(default_factory=dict)

    def __repr__(self) -> str:
        return f"TwitchMessage({self.author}: {self.content!r})"


class TwitchChatConfig(ModuleConfig):
    twitch_token: str = ""
    """OAuth token (oauth:xxx). Falls back to TWITCH_TOKEN env var."""

    channel_name: str = ""
    """Falls back to CHANNEL_NAME env var."""

    bot_prefix: str = "!"

    patterns: list[str] = []
    """Regex patterns for filtered_messages. If empty, all messages pass through."""


class TwitchChat(Module["TwitchChatConfig"]):
    """Connects to a Twitch channel and publishes chat messages.

    - ``raw_messages`` — every chat message
    - ``filtered_messages`` — messages matching configured regex patterns
    """

    default_config = TwitchChatConfig

    raw_messages: Out[TwitchMessage]
    filtered_messages: Out[TwitchMessage]

    def __init__(self, **kwargs: Any) -> None:
        self._bot: _TwitchBot | None = None
        self._bot_thread: threading.Thread | None = None
        self._bot_loop: asyncio.AbstractEventLoop | None = None
        self._compiled_patterns: list[re.Pattern[str]] = []
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        token = self.config.twitch_token or os.getenv("TWITCH_TOKEN", "")
        channel = self.config.channel_name or os.getenv("CHANNEL_NAME", "")

        if not token or not channel:
            logger.warning("[TwitchChat] No token/channel — running in local-only mode")
            return

        self._compiled_patterns = [re.compile(p, re.IGNORECASE) for p in self.config.patterns]

        self._bot_loop = asyncio.new_event_loop()
        self._bot_thread = threading.Thread(
            target=self._run_bot_loop,
            args=(token, channel),
            daemon=True,
            name="twitch-bot",
        )
        self._bot_thread.start()
        logger.info("[TwitchChat] Started", channel=channel)

    def _run_bot_loop(self, token: str, channel: str) -> None:
        assert self._bot_loop is not None
        asyncio.set_event_loop(self._bot_loop)
        try:
            self._bot = _TwitchBot(
                token=token,
                channel=channel,
                prefix=self.config.bot_prefix,
                on_message_cb=self._handle_message,
                on_ready_cb=self._handle_ready,
            )
            self._bot.run()
        except ImportError:
            logger.error("[TwitchChat] twitchio is not installed — run: uv pip install twitchio")
        except Exception:
            logger.exception("[TwitchChat] Bot crashed")

    @rpc
    def stop(self) -> None:
        if self._bot is not None and self._bot_loop is not None:

            async def _close() -> None:
                assert self._bot is not None
                await self._bot.close()

            asyncio.run_coroutine_threadsafe(_close(), self._bot_loop).result(timeout=5)

        if self._bot_loop is not None:
            self._bot_loop.call_soon_threadsafe(self._bot_loop.stop)

        if self._bot_thread is not None:
            self._bot_thread.join(timeout=5)

        self._bot = None
        self._bot_thread = None
        self._bot_loop = None
        super().stop()

    def _handle_ready(self) -> None:
        logger.info("[TwitchChat] Ready")

    def _handle_message(self, message: Any) -> None:
        """Convert a twitchio Message into a TwitchMessage and publish."""
        badges: dict[str, str] = {}
        if message.tags and "badges" in message.tags:
            raw = message.tags["badges"]
            if raw:
                for badge in raw.split(","):
                    parts = badge.split("/", 1)
                    if len(parts) == 2:
                        badges[parts[0]] = parts[1]

        msg = TwitchMessage(
            author=message.author.name if message.author else "",
            content=message.content or "",
            channel=message.channel.name if message.channel else "",
            timestamp=time.time(),
            is_subscriber="subscriber" in badges,
            is_mod="moderator" in badges,
            badges=badges,
        )

        self.raw_messages.publish(msg)
        self._publish_if_matched(msg)

    def _publish_if_matched(self, msg: TwitchMessage) -> None:
        """Publish to filtered_messages if msg matches patterns (or no patterns configured)."""
        if self._compiled_patterns:
            for pat in self._compiled_patterns:
                if pat.search(msg.content):
                    self.filtered_messages.publish(msg)
                    return
        else:
            self.filtered_messages.publish(msg)

    def inject_message(self, content: str, author: str = "anonymous") -> None:
        """Inject a chat message programmatically (for testing or local-only mode)."""
        msg = TwitchMessage(author=author, content=content, channel="local", timestamp=time.time())
        self.raw_messages.publish(msg)
        self._publish_if_matched(msg)


class _TwitchBot:
    """Thin twitchio wrapper that forwards messages via callbacks."""

    def __init__(
        self,
        token: str,
        channel: str,
        prefix: str,
        on_message_cb: Any,
        on_ready_cb: Any,
    ) -> None:
        from twitchio.ext import (  # type: ignore[import-not-found]
            commands as twitch_commands,  # type: ignore[import-untyped]
        )

        cb_message = on_message_cb
        cb_ready = on_ready_cb
        chan = channel

        class _Bot(twitch_commands.Bot):  # type: ignore[misc]
            def __init__(inner_self) -> None:  # noqa: N805
                super().__init__(token=token, prefix=prefix, initial_channels=[chan])

            async def event_ready(inner_self) -> None:  # noqa: N805
                logger.info("[TwitchChat] Bot connected as %s to #%s", inner_self.nick, chan)
                cb_ready()

            async def event_message(inner_self, message: Any) -> None:  # noqa: N805
                if message.echo:
                    return
                cb_message(message)

        self._bot = _Bot()

    def run(self) -> None:
        self._bot.run()

    async def close(self) -> None:
        await self._bot.close()
