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

"""ArduinoModule: DimOS module for Arduino-based hardware.

An ArduinoModule generates a ``dimos_arduino.h`` header at build time,
compiles and flashes the user's Arduino sketch, then launches a generic
C++ bridge that relays structured data between the Arduino's USB serial
and the DimOS LCM bus.

Example usage::

    class MyArduinoBot(ArduinoModule):
        config: MyArduinoBotConfig
        imu: Out[Imu]
        motor_cmd: In[Twist]

See ``dimos/hardware/arduino/`` for the C headers, bridge binary, and
protocol documentation.
"""

from __future__ import annotations

from dataclasses import dataclass
import inspect
import json
from pathlib import Path
import subprocess
from typing import Any, ClassVar, get_args, get_origin, get_type_hints

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Path to the arduino hardware directory (relative to this file)
_ARDUINO_HW_DIR = Path(__file__).resolve().parent.parent / "hardware" / "arduino"
_COMMON_DIR = _ARDUINO_HW_DIR / "common"
_DSP_PROTOCOL_PATH = _COMMON_DIR / "dsp_protocol.h"


@dataclass
class CTypeGenerator:
    """Override for generating C struct/encode/decode for a message type."""

    struct_create: Any  # Callable[[str], str]  — (type_name) -> C code
    encode_create: Any | None = None  # Callable[[str, str, int], str]
    decode_create: Any | None = None  # Callable[[str, str, int], str]


# Registry of known Arduino-compatible message type header paths
_KNOWN_TYPE_HEADERS: dict[str, str] = {
    "std_msgs.Time": "std_msgs/Time.h",
    "std_msgs.Bool": "std_msgs/Bool.h",
    "std_msgs.Int32": "std_msgs/Int32.h",
    "std_msgs.Float32": "std_msgs/Float32.h",
    "std_msgs.Float64": "std_msgs/Float64.h",
    "std_msgs.ColorRGBA": "std_msgs/ColorRGBA.h",
    "geometry_msgs.Vector3": "geometry_msgs/Vector3.h",
    "geometry_msgs.Point": "geometry_msgs/Point.h",
    "geometry_msgs.Point32": "geometry_msgs/Point32.h",
    "geometry_msgs.Quaternion": "geometry_msgs/Quaternion.h",
    "geometry_msgs.Pose": "geometry_msgs/Pose.h",
    "geometry_msgs.Pose2D": "geometry_msgs/Pose2D.h",
    "geometry_msgs.Twist": "geometry_msgs/Twist.h",
    "geometry_msgs.Accel": "geometry_msgs/Accel.h",
    "geometry_msgs.Transform": "geometry_msgs/Transform.h",
    "geometry_msgs.Wrench": "geometry_msgs/Wrench.h",
    "geometry_msgs.Inertia": "geometry_msgs/Inertia.h",
    "geometry_msgs.PoseWithCovariance": "geometry_msgs/PoseWithCovariance.h",
    "geometry_msgs.TwistWithCovariance": "geometry_msgs/TwistWithCovariance.h",
    "geometry_msgs.AccelWithCovariance": "geometry_msgs/AccelWithCovariance.h",
}


class ArduinoModuleConfig(NativeModuleConfig):
    """Configuration for an Arduino module."""

    # Sketch
    sketch_path: str = "sketch/sketch.ino"
    board_fqbn: str = "arduino:avr:uno"

    # Bridge binary (generic, same for all modules)
    executable: str = "result/bin/arduino_bridge"
    build_command: str = "nix build .#arduino_bridge"
    cwd: str | None = None

    # Connection
    port: str | None = None
    baudrate: int = 115200
    auto_detect: bool = True
    auto_reconnect: bool = True
    reconnect_interval: float = 2.0

    # Virtual mode (QEMU emulator instead of real hardware)
    virtual: bool = False

    # Flash
    auto_flash: bool = True
    flash_timeout: float = 60.0

    # Fields to exclude from bridge CLI args (host-only config)
    cli_exclude: frozenset[str] = frozenset(
        {
            "sketch_path",
            "board_fqbn",
            "port",
            "auto_detect",
            "auto_flash",
            "flash_timeout",
            "auto_reconnect",
            "reconnect_interval",
            "virtual",
        }
    )

    # Fields to exclude from Arduino #define embedding
    arduino_config_exclude: frozenset[str] = frozenset(
        {
            "executable",
            "build_command",
            "cwd",
            "sketch_path",
            "board_fqbn",
            "port",
            "auto_detect",
            "auto_reconnect",
            "reconnect_interval",
            "auto_flash",
            "flash_timeout",
            "virtual",
            "extra_args",
            "extra_env",
            "shutdown_timeout",
            "log_format",
            "cli_exclude",
            "arduino_config_exclude",
        }
    )


class ArduinoModule(NativeModule):
    """Module that manages an Arduino board with a generated header, sketch
    compilation, flashing, and a C++ serial↔LCM bridge.

    Subclass this, declare In/Out ports, and set ``config`` to an
    :class:`ArduinoModuleConfig` subclass pointing at your sketch.
    """

    config: ArduinoModuleConfig

    # Override for custom message type C code generation
    c_type_generators: ClassVar[dict[type, CTypeGenerator]] = {}

    # Virtual mode state
    _qemu_proc: subprocess.Popen | None = None
    _virtual_pty: str | None = None

    @rpc
    def build(self) -> None:
        """Build step: generate header, compile sketch, build bridge, (flash)."""
        # 1. Detect port (only for physical hardware)
        if not self.config.virtual and self.config.auto_detect and not self.config.port:
            self.config.port = self._detect_port()
            logger.info("Auto-detected Arduino port", port=self.config.port)

        # 2. Generate dimos_arduino.h
        self._generate_header()

        # 3. Compile Arduino sketch
        self._compile_sketch()

        # 4. Build the C++ bridge binary if needed (shared across all
        # ArduinoModule subclasses — lives in dimos/hardware/arduino/)
        self._build_bridge()

        # Point NativeModule at the shared bridge binary for start()
        self.config.executable = str(_ARDUINO_HW_DIR / "result" / "bin" / "arduino_bridge")

        # 5. Flash Arduino (only for physical hardware)
        if not self.config.virtual and self.config.auto_flash and self.config.port:
            self._flash()

    def _build_bridge(self) -> None:
        """Build the shared C++ bridge binary via the arduino flake."""
        bridge_bin = _ARDUINO_HW_DIR / "result" / "bin" / "arduino_bridge"
        if bridge_bin.exists():
            return

        logger.info("Building arduino_bridge via nix flake")
        result = subprocess.run(
            ["nix", "build", ".#arduino_bridge"],
            cwd=str(_ARDUINO_HW_DIR),
            capture_output=True,
            text=True,
            timeout=600,
        )
        if result.returncode != 0:
            raise RuntimeError(f"arduino_bridge build failed:\n{result.stderr}\n{result.stdout}")
        if not bridge_bin.exists():
            raise RuntimeError(f"arduino_bridge build succeeded but binary missing: {bridge_bin}")
        logger.info("arduino_bridge built successfully", path=str(bridge_bin))

    @rpc
    def start(self) -> None:
        """Launch the C++ bridge subprocess (and QEMU if virtual)."""
        topics = self._collect_topics()
        topic_enum = self._build_topic_enum()

        # If virtual, launch QEMU first and use its PTY as the serial port
        if self.config.virtual:
            self._start_qemu()
            serial_port = self._virtual_pty
        else:
            serial_port = self.config.port or "/dev/ttyACM0"

        # Build extra CLI args for the bridge
        extra = [
            "--serial_port",
            serial_port,
            "--baudrate",
            str(self.config.baudrate),
            "--reconnect",
            str(self.config.auto_reconnect).lower(),
            "--reconnect_interval",
            str(self.config.reconnect_interval),
        ]

        for stream_name, topic_id in topic_enum.items():
            if stream_name not in topics:
                continue
            lcm_channel = topics[stream_name]
            if stream_name in self.outputs:
                extra.extend(["--topic_out", str(topic_id), lcm_channel])
            elif stream_name in self.inputs:
                extra.extend(["--topic_in", str(topic_id), lcm_channel])

        self.config.extra_args = extra
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()
        # Tear down QEMU if it was launched
        if self._qemu_proc is not None:
            try:
                self._qemu_proc.terminate()
                self._qemu_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._qemu_proc.kill()
            self._qemu_proc = None
            self._virtual_pty = None
            logger.info("QEMU virtual Arduino stopped")

    @rpc
    def flash(self) -> None:
        """Manual re-flash without full rebuild."""
        self._flash()

    # ------------------------------------------------------------------
    # Private methods
    # ------------------------------------------------------------------

    def _get_stream_types(self) -> dict[str, type]:
        """Get {stream_name: message_type} for all In/Out ports."""
        hints = get_type_hints(type(self))
        result: dict[str, type] = {}
        for name, hint in hints.items():
            origin = get_origin(hint)
            if origin is In or origin is Out:
                args = get_args(hint)
                if args:
                    result[name] = args[0]
        return result

    def _build_topic_enum(self) -> dict[str, int]:
        """Assign topic IDs to streams. Topic 0 is reserved for debug."""
        stream_types = self._get_stream_types()
        topic_enum: dict[str, int] = {}
        topic_id = 1
        for name in sorted(stream_types.keys()):
            topic_enum[name] = topic_id
            topic_id += 1
        return topic_enum

    def _detect_port(self) -> str:
        """Auto-detect Arduino port using arduino-cli."""
        try:
            result = subprocess.run(
                ["arduino-cli", "board", "list", "--format", "json"],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                raise RuntimeError(f"arduino-cli board list failed: {result.stderr}")

            boards = json.loads(result.stdout)
            # Search for matching FQBN
            for entry in boards.get("detected_ports", boards if isinstance(boards, list) else []):
                port_info = entry if isinstance(entry, dict) else {}
                address = port_info.get("port", {}).get("address", "")
                matching_boards = port_info.get("matching_boards", [])
                for board in matching_boards:
                    if board.get("fqbn", "") == self.config.board_fqbn:
                        return address

            # Fallback: scan for any ttyUSB/ttyACM
            import glob

            candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if len(candidates) == 1:
                logger.warning(
                    "No FQBN match, using only available serial port",
                    port=candidates[0],
                )
                return candidates[0]
            if candidates:
                logger.warning(
                    "No FQBN match, using first serial port",
                    port=candidates[0],
                    all_ports=candidates,
                )
                return candidates[0]

            raise RuntimeError(
                f"No Arduino board found matching FQBN '{self.config.board_fqbn}'. "
                f"Run 'arduino-cli board list' to see connected boards."
            )
        except FileNotFoundError:
            raise RuntimeError(
                "arduino-cli not found. Install it or enter the nix dev shell: "
                "cd dimos/hardware/arduino && nix develop"
            ) from None

    def _generate_header(self) -> None:
        """Generate dimos_arduino.h from stream declarations + config."""
        stream_types = self._get_stream_types()
        topic_enum = self._build_topic_enum()

        sections: list[str] = []

        # Header guard
        sections.append(
            "/* Auto-generated by DimOS ArduinoModule — do not edit */\n"
            "#ifndef DIMOS_ARDUINO_H\n"
            "#define DIMOS_ARDUINO_H\n"
        )

        # Config #defines
        sections.append("/* --- Config --- */")
        sections.append(f"#define DIMOS_BAUDRATE {self.config.baudrate}")
        ignore_fields = set(NativeModuleConfig.model_fields) | set(
            self.config.arduino_config_exclude
        )
        for field_name in self.config.__class__.model_fields:
            if field_name in ignore_fields:
                continue
            val = getattr(self.config, field_name)
            if val is None:
                continue
            c_name = f"DIMOS_{field_name.upper()}"
            if isinstance(val, bool):
                sections.append(f"#define {c_name} {'1' if val else '0'}")
            elif isinstance(val, int):
                sections.append(f"#define {c_name} {val}")
            elif isinstance(val, float):
                sections.append(f"#define {c_name} {val}f")
            elif isinstance(val, str):
                sections.append(f'#define {c_name} "{val}"')
        sections.append("")

        # Topic enum
        sections.append("/* --- Topic enum (shared with C++ bridge) --- */")
        sections.append("enum dimos_topic {")
        sections.append("    DIMOS_TOPIC_DEBUG = 0,")
        for name, tid in topic_enum.items():
            direction = "Out" if name in self.outputs else "In"
            msg_type = stream_types[name]
            sections.append(
                f"    DIMOS_TOPIC__{name.upper()} = {tid},  /* {direction}[{msg_type.__name__}] */"
            )
        sections.append("};")
        sections.append("")

        # Message type includes
        sections.append("/* --- Message type headers --- */")
        included_types: set[str] = set()
        for _name, msg_type in stream_types.items():
            msg_name = getattr(msg_type, "msg_name", None)
            if msg_name is None:
                msg_name = f"{msg_type.__module__}.{msg_type.__qualname__}"

            if msg_name in included_types:
                continue
            included_types.add(msg_name)

            header = _KNOWN_TYPE_HEADERS.get(msg_name)
            if header:
                sections.append(f'#include "{header}"')
            elif msg_type in self.c_type_generators:
                gen = self.c_type_generators[msg_type]
                sections.append(gen.struct_create(msg_type.__name__))
            else:
                raise TypeError(
                    f"No Arduino C header for message type '{msg_name}'. "
                    f"Either add it to arduino_msgs/ or set c_type_generators "
                    f"on your ArduinoModule subclass."
                )
        sections.append("")

        # DSP protocol core
        sections.append("/* --- DSP protocol core --- */")
        sections.append('#include "dsp_protocol.h"')
        sections.append("")

        # Close header guard
        sections.append("#endif /* DIMOS_ARDUINO_H */")

        # Write to sketch directory
        sketch_dir = self._resolve_sketch_dir()
        header_path = sketch_dir / "dimos_arduino.h"
        header_path.write_text("\n".join(sections))
        logger.info("Generated Arduino header", path=str(header_path))

    def _resolve_sketch_dir(self) -> Path:
        """Resolve the sketch directory path."""
        subclass_file = Path(inspect.getfile(type(self)))
        base_dir = subclass_file.parent
        if self.config.cwd:
            base_dir = base_dir / self.config.cwd
        sketch_path = base_dir / self.config.sketch_path
        return sketch_path.parent

    def _build_dir(self) -> Path:
        """Per-module build directory for compiled sketch artifacts."""
        sketch_dir = self._resolve_sketch_dir()
        return sketch_dir / "build"

    def _compile_sketch(self) -> None:
        """Compile the Arduino sketch using arduino-cli."""
        sketch_dir = self._resolve_sketch_dir()
        build_dir = self._build_dir()
        build_dir.mkdir(parents=True, exist_ok=True)

        common = str(_COMMON_DIR)
        msgs = str(_COMMON_DIR / "arduino_msgs")
        extra_flags = f"-I{common} -I{msgs} -DF_CPU=16000000UL"

        cmd = [
            "arduino-cli",
            "compile",
            "--fqbn",
            self.config.board_fqbn,
            "--build-property",
            f"compiler.cpp.extra_flags={extra_flags}",
            "--build-property",
            f"compiler.c.extra_flags={extra_flags}",
            "--build-path",
            str(build_dir),
            str(sketch_dir),
        ]

        logger.info("Compiling Arduino sketch", cmd=" ".join(cmd))
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120,
        )
        if result.returncode != 0:
            raise RuntimeError(
                f"Arduino sketch compilation failed:\n{result.stderr}\n{result.stdout}"
            )
        logger.info("Arduino sketch compiled successfully", build_dir=str(build_dir))

    def _start_qemu(self) -> None:
        """Launch qemu-system-avr with the compiled sketch and capture its PTY."""
        import re
        import tempfile

        build_dir = self._build_dir()
        # arduino-cli outputs <sketch_name>.ino.elf
        sketch_name = Path(self.config.sketch_path).stem
        elf_path = build_dir / f"{sketch_name}.ino.elf"
        if not elf_path.exists():
            raise RuntimeError(f"Compiled sketch not found: {elf_path}")

        # Map FQBN to QEMU machine type
        machine_map = {
            "arduino:avr:uno": "uno",
            "arduino:avr:mega": "mega",
            "arduino:avr:mega2560": "mega2560",
        }
        machine = machine_map.get(self.config.board_fqbn, "uno")

        # Capture stderr to a temp file so we can parse the PTY path
        log_file = tempfile.NamedTemporaryFile(
            prefix="dimos_qemu_", suffix=".log", delete=False, mode="w"
        )
        self._qemu_log_path = log_file.name
        log_file.close()

        cmd = [
            "qemu-system-avr",
            "-machine",
            machine,
            "-bios",
            str(elf_path),
            "-serial",
            "pty",
            "-monitor",
            "null",
            "-nographic",
        ]

        logger.info("Starting QEMU virtual Arduino", cmd=" ".join(cmd))
        log_fd = open(self._qemu_log_path, "w")
        self._qemu_proc = subprocess.Popen(
            cmd,
            stdout=log_fd,
            stderr=subprocess.STDOUT,
        )

        # Poll the log file for the PTY announcement (up to 5 seconds)
        import time as _time

        deadline = _time.time() + 5.0
        pty = None
        while _time.time() < deadline:
            if self._qemu_proc.poll() is not None:
                # QEMU exited
                with open(self._qemu_log_path) as f:
                    raise RuntimeError(f"QEMU exited unexpectedly:\n{f.read()}")
            with open(self._qemu_log_path) as f:
                content = f.read()
            m = re.search(r"/dev/pts/\d+", content)
            if m:
                pty = m.group(0)
                break
            _time.sleep(0.1)

        if pty is None:
            self._qemu_proc.terminate()
            raise RuntimeError("QEMU started but did not announce a PTY within 5 seconds")

        self._virtual_pty = pty
        logger.info("QEMU virtual Arduino running", pty=pty, pid=self._qemu_proc.pid)

    def _flash(self) -> None:
        """Flash the compiled sketch to the Arduino."""
        sketch_dir = self._resolve_sketch_dir()
        port = self.config.port
        if not port:
            raise RuntimeError("No port configured for flashing")

        cmd = [
            "arduino-cli",
            "upload",
            "-p",
            port,
            "--fqbn",
            self.config.board_fqbn,
            str(sketch_dir),
        ]

        logger.info("Flashing Arduino", cmd=" ".join(cmd), port=port)
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=self.config.flash_timeout,
        )
        if result.returncode != 0:
            raise RuntimeError(f"Arduino flash failed:\n{result.stderr}\n{result.stdout}")
        logger.info("Arduino flashed successfully", port=port)


__all__ = [
    "ArduinoModule",
    "ArduinoModuleConfig",
    "CTypeGenerator",
]
