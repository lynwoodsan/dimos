#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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
Phone Teleop FastAPI Server.

Replaces the Deno WebSocket-to-LCM bridge with a Python FastAPI server.
Extends RobotWebInterface to serve the phone teleop web app and forward
WebSocket binary LCM packets to the LCM network via UDP multicast.
"""

from __future__ import annotations

from pathlib import Path
import socket
import struct

from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from dimos.utils.logging_config import setup_logger
from dimos.web.robot_web_interface import RobotWebInterface

logger = setup_logger()

# LCM default multicast group and port
LCM_MULTICAST_GROUP = "239.255.76.67"
LCM_MULTICAST_PORT = 7667

STATIC_DIR = Path(__file__).parent / "static"


class PhoneTeleopServer(RobotWebInterface):
    """Phone teleoperation server built on RobotWebInterface.

    Adds a WebSocket endpoint that bridges binary LCM packets from the
    phone browser to the LCM UDP multicast network.
    """

    def __init__(
        self,
        port: int = 8444,
        lcm_multicast_group: str = LCM_MULTICAST_GROUP,
        lcm_multicast_port: int = LCM_MULTICAST_PORT,
    ) -> None:
        super().__init__(port=port)
        self.lcm_multicast_group = lcm_multicast_group
        self.lcm_multicast_port = lcm_multicast_port
        self._udp_sock: socket.socket | None = None

        self._setup_teleop_routes()

    # ------------------------------------------------------------------
    # LCM UDP socket
    # ------------------------------------------------------------------

    def _ensure_udp_socket(self) -> socket.socket:
        """Create (or reuse) a UDP socket for LCM multicast publishing."""
        if self._udp_sock is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, struct.pack("b", 1))
            self._udp_sock = sock
            logger.info("LCM UDP multicast socket created")
        return self._udp_sock

    def _publish_lcm_packet(self, data: bytes) -> None:
        """Send a raw LCM packet to the multicast group."""
        sock = self._ensure_udp_socket()
        sock.sendto(data, (self.lcm_multicast_group, self.lcm_multicast_port))

    # ------------------------------------------------------------------
    # Routes
    # ------------------------------------------------------------------

    def _setup_teleop_routes(self) -> None:
        """Add phone-teleop-specific routes on top of the base interface."""

        @self.app.get("/teleop", response_class=HTMLResponse)
        async def teleop_index() -> HTMLResponse:
            index_path = STATIC_DIR / "index.html"
            return HTMLResponse(content=index_path.read_text())

        if STATIC_DIR.is_dir():
            self.app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="teleop_static")

        @self.app.websocket("/ws")
        async def websocket_endpoint(ws: WebSocket) -> None:
            await ws.accept()
            logger.info("Phone client connected")
            try:
                while True:
                    data = await ws.receive_bytes()
                    try:
                        self._publish_lcm_packet(data)
                    except Exception:
                        logger.exception("Failed to forward LCM packet")
            except WebSocketDisconnect:
                logger.info("Phone client disconnected")
            except Exception:
                logger.exception("WebSocket error")


if __name__ == "__main__":
    server = PhoneTeleopServer()
    server.run()
