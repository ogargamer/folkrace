#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: HIGH-SPEED MCU BINARY UART INGESTION BRIDGE
================================================================================
  Listens to binary UART telemetry from:
    1. ESP32 Main (Chassis Dynamics, 4-Wheel RPMs, IMU, Servo Feedback) @ 100 Hz
    2. ESP32 ToF Hub (VL53L8CX 16-zone distance arrays) @ 60 Hz

  Feeds parsed real-hardware frames directly to:
    - Path Planner Pipeline (Core 1)
    - Debug Server & Black Box Flight Recorder (Core 2) via UDP 127.0.0.1:9876
================================================================================
"""

import os
import sys
import time
import socket
import select
import json
import glob
import threading
from typing import Optional, Dict, Any, Tuple, List, Set

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

from mcu_telemetry_protocol import (
    MCU_SYNC,
    PI_SYNC,
    MSG_ID_CHASSIS_TELEM,
    MSG_ID_TOF_TELEM,
    MSG_ID_TOF_SUB_TELEM,
    CHASSIS_SIZE,
    TOF_SIZE,
    TOF_SUB_SIZE,
    TOF_SUB52_SIZE,
    TOF_SUB60_SIZE,
    CMD_SIZE,
    decode_chassis_packet,
    decode_tof_packet,
    decode_tof_sub_packet,
    encode_motion_command,
)
from tof_latency_compensator import ToFLatencyCompensator, SENSOR_NAMES

# Mapping of sensor_id (1..4) from ESP32_ToF to 16-channel array slots matching SENSOR_NAMES:
# Sensor 1: Front-Left Outer (49.1° to 82.9° Left)  -> slots 4..7
# Sensor 2: Front-Left Center (4.1° to 37.9° Left)   -> slots 0..3
# Sensor 3: Front-Right Outer (49.1° to 82.9° Right) -> slots 12..15
# Sensor 4: Front-Right Center (4.1° to 37.9° Right) -> slots 8..11
SENSOR_CHANNEL_SLOTS = {
    1: [4, 5, 6, 7],
    2: [0, 1, 2, 3],
    3: [12, 13, 14, 15],
    4: [8, 9, 10, 11],
}

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

UDP_DISPATCH_HOST = "127.0.0.1"
UDP_DISPATCH_PORT = 9876
UDP_CMD_PORT = 9877
UART_BAUD = 921600


class BinaryStreamParser:
    """Zero-allocation rolling stream parser for binary UART packets."""

    def __init__(self, is_tof_port: bool = False):
        self.is_tof_port = is_tof_port
        self.buffer = bytearray()

    def feed(self, chunk: bytes):
        self.buffer.extend(chunk)

    def extract_packets(self):
        """Yields (msg_type, decoded_dict) for all valid packets in buffer."""
        while len(self.buffer) >= 3:
            # 1. Search for sync header 0xAA 0x55
            idx = self.buffer.find(MCU_SYNC)
            if idx == -1:
                # Check for complete ASCII text lines from MCU (e.g. status or command responses)
                if not self.is_tof_port and b"\n" in self.buffer:
                    nl_idx = self.buffer.rfind(b"\n")
                    text_bytes = bytes(self.buffer[:nl_idx])
                    del self.buffer[:nl_idx + 1]
                    try:
                        text_str = text_bytes.decode("utf-8", errors="ignore")
                        for line in text_str.splitlines():
                            line = line.strip()
                            if line and len(line) >= 3 and all(c.isprintable() or c in "\r\n\t" for c in line):
                                yield ("TEXT", line)
                    except Exception:
                        pass
                else:
                    # Keep last byte in case header split across reads
                    self.buffer = self.buffer[-1:]
                break

            if idx > 0:
                if not self.is_tof_port:
                    prefix = bytes(self.buffer[:idx])
                    if b"\n" in prefix:
                        try:
                            text_str = prefix.decode("utf-8", errors="ignore")
                            for line in text_str.splitlines():
                                line = line.strip()
                                if line and len(line) >= 3 and all(c.isprintable() or c in "\r\n\t" for c in line):
                                    yield ("TEXT", line)
                        except Exception:
                            pass
                del self.buffer[:idx]

            if len(self.buffer) < 3:
                break

            msg_type_or_sensor_id = self.buffer[2]

            if not self.is_tof_port:
                # ESP32-Main Port
                if msg_type_or_sensor_id == MSG_ID_CHASSIS_TELEM:
                    if len(self.buffer) >= 4 and self.buffer[3] != (CHASSIS_SIZE - 5):
                        # False sync header inside payload; advance 2 bytes to rescan
                        del self.buffer[:2]
                        continue
                    if len(self.buffer) < CHASSIS_SIZE:
                        break
                    raw_pkt = bytes(self.buffer[:CHASSIS_SIZE])
                    decoded = decode_chassis_packet(raw_pkt)
                    if decoded:
                        del self.buffer[:CHASSIS_SIZE]
                        yield (MSG_ID_CHASSIS_TELEM, decoded)
                    else:
                        # Checksum mismatch on false sync; advance 2 bytes so nested header isn't lost
                        del self.buffer[:2]
                else:
                    del self.buffer[:2]
            else:
                # ESP32-ToF Port (supports 20-byte lean frame, 52-byte multi-target, and 78-byte full array)
                if 1 <= msg_type_or_sensor_id <= 4:
                    # 1. Check for 20-byte lean frame first (primary mode for Pi)
                    if len(self.buffer) >= TOF_SUB_SIZE:
                        chk_calc = 0
                        for b in self.buffer[: TOF_SUB_SIZE - 1]:
                            chk_calc ^= b
                        if chk_calc == self.buffer[TOF_SUB_SIZE - 1]:
                            raw_pkt = bytes(self.buffer[:TOF_SUB_SIZE])
                            del self.buffer[:TOF_SUB_SIZE]
                            decoded = decode_tof_sub_packet(raw_pkt)
                            if decoded:
                                yield (MSG_ID_TOF_SUB_TELEM, decoded)
                                continue

                    # 2. Check for 52-byte multi-target frame
                    if len(self.buffer) >= TOF_SUB52_SIZE:
                        chk_calc = 0
                        for b in self.buffer[: TOF_SUB52_SIZE - 1]:
                            chk_calc ^= b
                        if chk_calc == self.buffer[TOF_SUB52_SIZE - 1]:
                            raw_pkt = bytes(self.buffer[:TOF_SUB52_SIZE])
                            del self.buffer[:TOF_SUB52_SIZE]
                            decoded = decode_tof_sub_packet(raw_pkt)
                            if decoded:
                                yield (MSG_ID_TOF_SUB_TELEM, decoded)
                                continue

                    if len(self.buffer) >= TOF_SUB52_SIZE:
                        del self.buffer[:2]
                        continue
                    else:
                        break
                elif msg_type_or_sensor_id == MSG_ID_TOF_TELEM:
                    if len(self.buffer) < TOF_SIZE:
                        break
                    raw_pkt = bytes(self.buffer[:TOF_SIZE])
                    del self.buffer[:TOF_SIZE]
                    decoded = decode_tof_packet(raw_pkt)
                    if decoded:
                        yield (MSG_ID_TOF_TELEM, decoded)
                else:
                    del self.buffer[:2]


def open_low_latency_serial(port: str, baud: int = UART_BAUD) -> Any:
    """
    Opens a serial port with non-blocking raw I/O, clears HUPCL so closing/reopening
    never drops DTR/RTS to reset the ESP32, and enables kernel ASYNC_LOW_LATENCY.
    """
    ser = serial.Serial(port, baud, timeout=0.0, write_timeout=0.0)

    # Configure Linux termios (~HUPCL + VMIN=0/VTIME=0) and kernel low-latency flag
    if sys.platform == "linux":
        try:
            import fcntl
            import termios
            import array
            fd = ser.fileno()
            attrs = termios.tcgetattr(fd)
            # Clear HUPCL so closing the port never pulses DTR/RTS to reset the ESP32
            attrs[2] &= ~termios.HUPCL
            # Raw VMIN=0, VTIME=0 for instant non-blocking os.read()
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            try:
                # TIOCGSERIAL = 0x541E, TIOCSSERIAL = 0x541F, ASYNC_LOW_LATENCY = 0x2000
                buf = array.array("i", [0] * 32)
                fcntl.ioctl(fd, 0x541E, buf, True)
                buf[4] |= 0x2000
                fcntl.ioctl(fd, 0x541F, buf, False)
            except Exception:
                pass
        except Exception:
            pass
    return ser


def get_available_serial_ports() -> List[str]:
    """Returns sorted list of unique canonical (realpath) USB/ACM serial port device nodes."""
    ports: List[str] = []
    seen_real: Set[str] = set()

    def _add(path_candidate: str):
        if not path_candidate or not os.path.exists(path_candidate):
            return
        try:
            real_p = os.path.realpath(path_candidate)
        except Exception:
            real_p = path_candidate
        base = os.path.basename(real_p)
        # Skip onboard Pi SoC UARTs (/dev/ttyS0, /dev/ttyAMA0) when auto-scanning USB MCUs
        if base.startswith(("ttyS", "ttyAMA", "ttyprintk")):
            return
        if real_p not in seen_real and os.path.exists(real_p):
            seen_real.add(real_p)
            ports.append(real_p)

    if HAS_SERIAL:
        try:
            import serial.tools.list_ports
            for p in serial.tools.list_ports.comports():
                if p.device:
                    _add(p.device)
        except Exception:
            pass

    for pattern in ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/serial/by-id/*"]:
        for p in glob.glob(pattern):
            _add(p)

    return sorted(ports)


def probe_port_role(port: str, baud: int = UART_BAUD) -> Optional[str]:
    """
    Non-intrusively sniffs live binary frames on port (~10 ms typical) to identify ESP32 role
    using full XOR checksum validation before falling back to ASCII PING.
    """
    if not HAS_SERIAL or not os.path.exists(port):
        return None
    ser = None
    try:
        ser = open_low_latency_serial(port, baud)
        fd = ser.fileno()
        main_parser = BinaryStreamParser(is_tof_port=False)
        tof_parser = BinaryStreamParser(is_tof_port=True)
        accum = bytearray()

        # Phase 1: Passive binary/boot stream sniff (returns in ~10ms once streaming, up to 600ms on cold boot)
        t_deadline = time.perf_counter() + 0.600
        while time.perf_counter() < t_deadline:
            rem = max(0.001, t_deadline - time.perf_counter())
            rlist, _, _ = select.select([fd], [], [], rem)
            if rlist:
                chunk = os.read(fd, 4096)
                if chunk:
                    accum.extend(chunk)
                    main_parser.feed(chunk)
                    for msg_type, _ in main_parser.extract_packets():
                        if msg_type == MSG_ID_CHASSIS_TELEM:
                            return "main"
                    tof_parser.feed(chunk)
                    for msg_type, _ in tof_parser.extract_packets():
                        if msg_type in (MSG_ID_TOF_SUB_TELEM, MSG_ID_TOF_TELEM):
                            return "tof"
                    if (
                        b"PONG" in accum
                        or b"FOLKRACE" in accum
                        or b"STATUS REPORT" in accum
                        or b"[SPI Attempt" in accum
                        or b"STM32" in accum
                    ):
                        return "main"
                    if (
                        b"VL53L8CX" in accum
                        or b"[ToF " in accum
                        or b"Waking ToF" in accum
                        or b"RANGING" in accum
                    ):
                        return "tof"

        # Phase 2: Active ASCII ping fallback if stream was silent
        os.write(fd, b"PING\n")
        t_deadline = time.perf_counter() + 0.150
        while time.perf_counter() < t_deadline:
            rem = max(0.001, t_deadline - time.perf_counter())
            rlist, _, _ = select.select([fd], [], [], rem)
            if rlist:
                chunk = os.read(fd, 4096)
                if chunk:
                    accum.extend(chunk)
                    main_parser.feed(chunk)
                    for msg_type, _ in main_parser.extract_packets():
                        if msg_type == MSG_ID_CHASSIS_TELEM:
                            return "main"
                    tof_parser.feed(chunk)
                    for msg_type, _ in tof_parser.extract_packets():
                        if msg_type in (MSG_ID_TOF_SUB_TELEM, MSG_ID_TOF_TELEM):
                            return "tof"
                    if b"PONG" in accum or b"FOLKRACE" in accum or b"STATUS" in accum or b"STM32" in accum:
                        return "main"
                    if b"VL53L8CX" in accum or b"[ToF " in accum or b"Waking ToF" in accum or b"RANGING" in accum:
                        return "tof"
    except Exception:
        pass
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
    return None


def resolve_serial_ports(
    main_port: Optional[str] = "auto",
    tof_port: Optional[str] = "auto",
    baud: int = UART_BAUD
) -> Tuple[Optional[str], Optional[str]]:
    """Auto-detects and binds USB / UART serial ports to ESP32 Main and ESP32 ToF."""
    candidates = get_available_serial_ports()

    resolved_main = main_port if main_port and main_port != "auto" and os.path.exists(main_port) else None
    resolved_tof = tof_port if tof_port and tof_port != "auto" and os.path.exists(tof_port) else None

    # Filter out already explicitly claimed ports
    available = [c for c in candidates if c not in (resolved_main, resolved_tof) and os.path.exists(c)]

    # If both need detection, probe each available USB port
    if (resolved_main is None or resolved_tof is None) and available:
        probed_roles = {}
        for p in available:
            role = probe_port_role(p, baud)
            if role:
                probed_roles[p] = role

        # Assign based on probe findings
        for p, role in probed_roles.items():
            if role == "main" and resolved_main is None:
                resolved_main = p
            elif role == "tof" and resolved_tof is None:
                resolved_tof = p

        # Fallback for remaining unprobed or unassigned ports
        remaining = [p for p in available if p not in (resolved_main, resolved_tof)]
        if resolved_main is None and remaining:
            resolved_main = remaining.pop(0)
        if resolved_tof is None and remaining:
            resolved_tof = remaining.pop(0)

    return resolved_main, resolved_tof


class MCU_UART_Bridge:
    def __init__(
        self,
        main_port: Optional[str] = "auto",
        tof_port: Optional[str] = "auto",
        baud: int = UART_BAUD,
        udp_host: str = UDP_DISPATCH_HOST,
        udp_port: int = UDP_DISPATCH_PORT,
    ):
        self.main_port_arg = main_port
        self.tof_port_arg = tof_port
        self.baud = baud
        self.udp_host = udp_host
        self.udp_target = (udp_host, udp_port)
        self.running = False

        self.assigned_ports: Dict[str, Optional[str]] = {"ESP32_Main": None, "ESP32_ToF": None}
        self.last_known_ports: Dict[str, Optional[str]] = {"main": None, "tof": None}
        self.link_active: Dict[str, bool] = {"ESP32_Main": False, "ESP32_ToF": False}
        self.last_rx_timestamp: Dict[str, float] = {"ESP32_Main": 0.0, "ESP32_ToF": 0.0}
        self.force_reconnect_flag = False
        self.discovery_lock = threading.Lock()

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 524288)
        except Exception:
            pass
        self.udp_sock.setblocking(False)

        self.serial_lock = threading.Lock()
        self.main_serial: Optional[Any] = None
        self.tof_serial: Optional[Any] = None
        self.latest_mcu_msg: str = ""
        self.mcu_log: list = []

        # State aggregation & Option 1 Latency Compensator
        self.compensator = ToFLatencyCompensator()
        self.lock = threading.Lock()
        self.latest_chassis: Dict[str, Any] = {}
        self.latest_tof_ranges = [1.8] * 16
        self.latest_tof_ranges_raw = [1.8] * 16
        self._last_debug_dispatch = 0.0

    @property
    def main_port(self) -> Optional[str]:
        return self.assigned_ports.get("ESP32_Main")

    @property
    def tof_port(self) -> Optional[str]:
        return self.assigned_ports.get("ESP32_ToF")

    def trigger_reconnect(self):
        """Forces immediate re-probe and reconnect across all MCU serial links."""
        print("[MCU Bridge] Manual reconnect command received. Re-detecting all MCU links...")
        self.force_reconnect_flag = True
        def _clear():
            time.sleep(1.0)
            self.force_reconnect_flag = False
        threading.Thread(target=_clear, daemon=True).start()

    def find_matching_port(self, target_role: str, excluded_ports: Set[Optional[str]]) -> Optional[str]:
        """
        Dynamically searches available serial ports for a matching MCU role ('main' or 'tof').
        Excludes canonical realpath ports actively assigned to other workers.
        """
        excluded_real = set()
        for ep in excluded_ports:
            if ep:
                try:
                    excluded_real.add(os.path.realpath(ep))
                except Exception:
                    excluded_real.add(ep)

        candidates = get_available_serial_ports()
        available = [p for p in candidates if p and p not in excluded_real and os.path.exists(p)]
        if not available:
            return None

        # Prioritize last known good port for this role to minimize reconnection time
        preferred = self.last_known_ports.get(target_role)
        if preferred and preferred in available:
            available.remove(preferred)
            available.insert(0, preferred)

        # 1. Probe roles
        for p in available:
            role = probe_port_role(p, self.baud)
            if role:
                self.last_known_ports[role] = p
            if role == target_role:
                return p

        # 2. Check explicit configured arg if it exists and wasn't claimed
        configured = self.tof_port_arg if target_role == "tof" else self.main_port_arg
        if configured and configured != "auto":
            try:
                conf_real = os.path.realpath(configured)
            except Exception:
                conf_real = configured
            if conf_real in available:
                return conf_real

        return None

    def start(self):
        if not HAS_SERIAL:
            print("[MCU Bridge ERROR] pyserial is not installed on this system!")
            return

        self.running = True

        # Start Command Ingestion Worker (listens for uplink packets to forward to ESP32 Main)
        t_cmd = threading.Thread(target=self._command_listener_worker, name="MCUCommandWorker", daemon=True)
        t_cmd.start()

        # Start resilient supervisor workers for both ESP32 Main and ESP32 ToF Hub
        t_main = threading.Thread(target=self._mcu_worker, args=("ESP32_Main",), name="Worker_ESP32_Main", daemon=True)
        t_main.start()

        t_tof = threading.Thread(target=self._mcu_worker, args=("ESP32_ToF",), name="Worker_ESP32_ToF", daemon=True)
        t_tof.start()

    def stop(self):
        self.running = False

    def _command_listener_worker(self):
        """Listens on UDP 127.0.0.1:9877 for motion/motor commands from GUI or planner and writes to ESP32."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        except Exception:
            pass
        try:
            sock.bind((self.udp_host, UDP_CMD_PORT))
            print(f"[MCU Bridge] Uplink Command Listener active on {self.udp_host}:{UDP_CMD_PORT}")
        except Exception as e:
            print(f"[MCU Bridge ERROR] Cannot bind UDP command port {UDP_CMD_PORT}: {e}")
            return

        sock.setblocking(False)
        while self.running:
            try:
                rlist, _, _ = select.select([sock], [], [], 0.05)
                if not rlist:
                    continue

                latest_motion_cmd: Optional[bytes] = None
                # Drain socket so bursts of 60Hz binary motion commands coalesce to the freshest frame
                while True:
                    try:
                        data, _ = sock.recvfrom(2048)
                    except (BlockingIOError, socket.error):
                        break
                    if not data:
                        continue

                    # Fast-path for 13-byte binary MotionCommandPacket (0x55 0xAA)
                    if len(data) == CMD_SIZE and data[:2] == PI_SYNC:
                        latest_motion_cmd = data
                        continue

                    d_strip = data.strip()
                    if d_strip in (b"RECONNECT", b"RESET_PORTS", b"RESTART_BRIDGE"):
                        self.trigger_reconnect()
                        continue
                    try:
                        if d_strip.startswith(b"{"):
                            cmd_obj = json.loads(data.decode("utf-8", errors="ignore"))
                            if isinstance(cmd_obj, dict) and cmd_obj.get("action") in ("reconnect", "reset_ports"):
                                self.trigger_reconnect()
                                continue
                    except Exception:
                        pass

                    # Immediate dispatch for explicit ASCII/control commands
                    self.send_to_main_mcu(data)

                if latest_motion_cmd is not None:
                    self.send_to_main_mcu(latest_motion_cmd)
            except Exception:
                pass
        try:
            sock.close()
        except Exception:
            pass

    def send_to_tof_mcu(self, data: bytes):
        """Thread-safe transmission to ESP32 ToF Hub over USB Serial."""
        with self.serial_lock:
            if self.tof_serial and getattr(self.tof_serial, "is_open", False):
                try:
                    os.write(self.tof_serial.fileno(), data)
                except Exception as e:
                    print(f"[MCU Bridge ERROR] Failed writing to ESP32 ToF: {e}")

    def send_to_main_mcu(self, data: bytes):
        """Thread-safe zero-blocking transmission to ESP32 Main over USB Serial."""
        is_binary_motion = (len(data) == CMD_SIZE and data[:2] == PI_SYNC)
        with self.serial_lock:
            if self.main_serial and getattr(self.main_serial, "is_open", False):
                try:
                    os.write(self.main_serial.fileno(), data)
                    if not is_binary_motion:
                        preview = data.decode("utf-8", errors="replace").strip()
                        print(f"[MCU Bridge -> ESP32] Sent: {preview}")
                except Exception as e:
                    if not is_binary_motion:
                        print(f"[MCU Bridge ERROR] Failed writing to ESP32: {e}")
            elif not is_binary_motion:
                print(f"[MCU Bridge WARN] Cannot dispatch command (ESP32 serial offline)")

    def _mcu_worker(self, label: str):
        """
        Self-healing low-latency worker for an MCU connection.
        Uses event-driven select() + os.read() for <0.2ms packet ingestion latency
        (zero 256-byte buffering lag), detects disconnects, and reconnects in 250ms.
        """
        is_tof = (label == "ESP32_ToF")
        role_target = "tof" if is_tof else "main"
        configured_port = self.tof_port_arg if is_tof else self.main_port_arg

        while self.running:
            # 1. Port Acquisition Phase
            port = None
            if configured_port and configured_port != "auto" and os.path.exists(configured_port):
                try:
                    conf_real = os.path.realpath(configured_port)
                except Exception:
                    conf_real = configured_port
                with self.discovery_lock:
                    other_label = "ESP32_ToF" if label == "ESP32_Main" else "ESP32_Main"
                    if conf_real != self.assigned_ports.get(other_label):
                        port = conf_real
            else:
                with self.discovery_lock:
                    other_label = "ESP32_ToF" if label == "ESP32_Main" else "ESP32_Main"
                    excluded = {self.assigned_ports.get(other_label)}
                    port = self.find_matching_port(role_target, excluded)

            if not port:
                self.link_active[label] = False
                self.assigned_ports[label] = None
                time.sleep(0.5)
                continue

            self.assigned_ports[label] = port
            self.last_known_ports[role_target] = port
            print(f"[MCU Bridge] Connecting to {label} on {port} @ {self.baud} baud...")

            # 2. Connection and Low-Latency Event-Driven Streaming Phase
            parser = BinaryStreamParser(is_tof_port=is_tof)
            ser = None
            try:
                ser = open_low_latency_serial(port, self.baud)
                fd = ser.fileno()

                with self.serial_lock:
                    if is_tof:
                        self.tof_serial = ser
                    else:
                        self.main_serial = ser

                # Initialize MCU parameters without resetting or blocking
                try:
                    if is_tof:
                        os.write(fd, b"L\n")  # Lean mode: 20B
                    else:
                        os.write(fd, b"STATUS\n")
                except Exception:
                    pass

                print(f"[MCU Bridge] Connected to {label} ({port}). Sub-ms live streaming active.")
                self.link_active[label] = True
                last_pkt_time = time.time()
                self.last_rx_timestamp[label] = last_pkt_time

                while self.running:
                    if self.force_reconnect_flag:
                        print(f"[MCU Bridge] Force reconnect requested for {label}. Closing link...")
                        break

                    # Event-driven wait: wakes immediately (<0.1ms) when USB URB arrives
                    rlist, _, _ = select.select([fd], [], [], 0.020)
                    now = time.time()
                    if rlist:
                        data = os.read(fd, 4096)
                        if not data:
                            # EOF on tty device node (USB unplugged)
                            raise OSError("Serial device returned EOF (USB disconnected)")

                        parser.feed(data)
                        had_valid_packet = False
                        needs_dispatch = False
                        for msg_type, decoded in parser.extract_packets():
                            if self._ingest_packet(msg_type, decoded):
                                needs_dispatch = True
                            had_valid_packet = True

                        if had_valid_packet:
                            last_pkt_time = now
                            self.last_rx_timestamp[label] = now
                            self.link_active[label] = True

                        if needs_dispatch:
                            self._dispatch_telemetry(now)

                    # Watchdog: Silence detection (1.2 seconds with no valid telemetry packets)
                    if now - last_pkt_time > 1.2:
                        print(f"[MCU Bridge] Watchdog: No telemetry from {label} on {port} for 1.2s. Reconnecting...")
                        break

            except (serial.SerialException, OSError) as e:
                print(f"[MCU Bridge] Connection lost on {label} ({port}): {e}")
            except Exception as e:
                print(f"[MCU Bridge] Error in {label} worker ({port}): {e}")
            finally:
                with self.serial_lock:
                    if is_tof:
                        self.tof_serial = None
                    else:
                        self.main_serial = None
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                self.link_active[label] = False
                self.assigned_ports[label] = None
                if not is_tof:
                    with self.lock:
                        self.latest_chassis = {}
                self._dispatch_telemetry(time.time())
                print(f"[MCU Bridge] {label} disconnected. Re-probing in 0.25s...")
                time.sleep(0.25)

    def _ingest_packet(self, msg_type: Any, pkt: Any) -> bool:
        """Updates internal state for a single decoded packet. Returns True if telemetry should be dispatched."""
        with self.lock:
            if msg_type == "TEXT":
                text = str(pkt)
                self.latest_mcu_msg = text
                self.mcu_log.append(text)
                if len(self.mcu_log) > 50:
                    self.mcu_log = self.mcu_log[-50:]
                print(f"[MCU Main Text] {text}")
                return True

            if msg_type == MSG_ID_CHASSIS_TELEM:
                self.latest_chassis = pkt
                return True
            elif msg_type == MSG_ID_TOF_SUB_TELEM:
                sensor_id = pkt.get("sensor_id", 0)
                dists = pkt.get("distances_m", [])
                slots = SENSOR_CHANNEL_SLOTS.get(sensor_id, [])
                for i, slot in enumerate(slots):
                    if i < len(dists):
                        self.latest_tof_ranges_raw[slot] = dists[i]

                v_est = float(self.latest_chassis.get("v_est", self.latest_chassis.get("v_real", 0.0)))
                gyro_z = float(self.latest_chassis.get("gyro_z", 0.0))
                self.latest_tof_ranges = self.compensator.compensate_list(
                    self.latest_tof_ranges_raw, v_est=v_est, gyro_z=gyro_z, dt=0.01667
                )
                return True
            elif msg_type == MSG_ID_TOF_TELEM:
                dists = pkt.get("distances_m", [])
                if len(dists) == 16:
                    self.latest_tof_ranges_raw = list(dists)
                    v_est = float(self.latest_chassis.get("v_est", self.latest_chassis.get("v_real", 0.0)))
                    gyro_z = float(self.latest_chassis.get("gyro_z", 0.0))
                    self.latest_tof_ranges = self.compensator.compensate_list(
                        dists, v_est=v_est, gyro_z=gyro_z, dt=0.01667
                    )
                return True
        return False

    def _dispatch_telemetry(self, now: float):
        """Composes and dispatches telemetry once per read batch to Core 1 Planner and Core 2 Debug Server."""
        send_debug = (now - self._last_debug_dispatch >= 0.05)
        with self.lock:
            telemetry = dict(self.latest_chassis)
            telemetry["timestamp"] = now
            telemetry["tof_ranges"] = list(self.latest_tof_ranges)
            telemetry["tof_ranges_raw"] = list(self.latest_tof_ranges_raw)
            telemetry["f_tof"] = round(float(min(self.latest_tof_ranges)), 3) if self.latest_tof_ranges else 1.8
            telemetry["f_tof_raw"] = round(float(min(self.latest_tof_ranges_raw)), 3) if self.latest_tof_ranges_raw else 1.8
            telemetry["mcu_msg"] = self.latest_mcu_msg
            telemetry["mcu_log"] = list(self.mcu_log[-10:])
            telemetry["mcu_main_connected"] = bool(self.link_active.get("ESP32_Main", False))
            telemetry["mcu_tof_connected"] = bool(self.link_active.get("ESP32_ToF", False))
            telemetry["mcu_main_port"] = self.assigned_ports.get("ESP32_Main") or "Offline"
            telemetry["mcu_tof_port"] = self.assigned_ports.get("ESP32_ToF") or "Offline"

            if send_debug:
                self._last_debug_dispatch = now
                telemetry["tof"] = {
                    SENSOR_NAMES[i]: round(self.latest_tof_ranges[i], 3)
                    for i in range(min(len(SENSOR_NAMES), len(self.latest_tof_ranges)))
                }
                telemetry["tof_raw"] = {
                    SENSOR_NAMES[i]: round(self.latest_tof_ranges_raw[i], 3)
                    for i in range(min(len(SENSOR_NAMES), len(self.latest_tof_ranges_raw)))
                }
                v_curr = float(self.latest_chassis.get("v_est", 0.0))
                w_curr = float(self.latest_chassis.get("gyro_z", 0.0))
                telemetry["tof_anchor_proj"] = list(self.compensator.compute_projected_anchor(v_curr, w_curr, 0.01667))

        try:
            raw = json.dumps(telemetry).encode("utf-8")
            self.udp_sock.sendto(raw, (self.udp_host, 9875))  # Real-time feed to Path Planner Core 1
            if send_debug:
                self.udp_sock.sendto(raw, self.udp_target)
        except Exception:
            pass


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Folkrace MCU Binary UART Bridge")
    parser.add_argument("--main", default="auto", help="ESP32 Main serial device (auto-detects /dev/ttyUSB0, /dev/ttyACM0, etc.)")
    parser.add_argument("--tof", default="auto", help="ESP32 ToF Hub serial device (auto-detects /dev/ttyUSB1, /dev/ttyACM1, etc.)")
    parser.add_argument("--baud", type=int, default=UART_BAUD, help="UART baud rate")
    args = parser.parse_args()

    bridge = MCU_UART_Bridge(main_port=args.main, tof_port=args.tof, baud=args.baud)
    bridge.start()

    print("[MCU Bridge] Active. Press Ctrl+C to terminate.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        bridge.stop()
        print("\n[MCU Bridge] Terminated.")

