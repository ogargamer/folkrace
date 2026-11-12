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
import json
import glob
import threading
from typing import Optional, Dict, Any, Tuple, List, Set

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

from mcu_telemetry_protocol import (
    MCU_SYNC,
    MSG_ID_CHASSIS_TELEM,
    MSG_ID_TOF_TELEM,
    MSG_ID_TOF_SUB_TELEM,
    CHASSIS_SIZE,
    TOF_SIZE,
    TOF_SUB_SIZE,
    TOF_SUB52_SIZE,
    TOF_SUB60_SIZE,
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
                    if len(self.buffer) < CHASSIS_SIZE:
                        break
                    raw_pkt = bytes(self.buffer[:CHASSIS_SIZE])
                    del self.buffer[:CHASSIS_SIZE]
                    decoded = decode_chassis_packet(raw_pkt)
                    if decoded:
                        yield (MSG_ID_CHASSIS_TELEM, decoded)
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


def get_available_serial_ports() -> List[str]:
    """Returns sorted list of currently available serial port paths."""
    ports = []
    if HAS_SERIAL:
        try:
            import serial.tools.list_ports
            for p in serial.tools.list_ports.comports():
                if p.device and p.device not in ports:
                    ports.append(p.device)
        except Exception:
            pass

    for pattern in ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/serial/by-id/*"]:
        for p in glob.glob(pattern):
            try:
                real_p = os.path.realpath(p)
                if real_p not in ports:
                    ports.append(real_p)
                elif p not in ports:
                    ports.append(p)
            except Exception:
                if p not in ports:
                    ports.append(p)

    return sorted(ports)


def probe_port_role(port: str, baud: int = UART_BAUD) -> Optional[str]:
    """Sniffs incoming bytes on port for 200 ms to identify ESP32 role."""
    if not HAS_SERIAL or not os.path.exists(port):
        return None
    try:
        with serial.Serial(port, baud, timeout=0.25) as ser:
            ser.reset_input_buffer()
            # Send PING in case MCU is waiting in shell mode
            ser.write(b"PING\n")
            time.sleep(0.06)
            chunk = ser.read(512)
            if b"PONG" in chunk:
                return "main"
            if b"FOLKRACE ESP32-S3" in chunk or b"STATUS REPORT" in chunk:
                return "main"
            if b"VL53L8CX" in chunk or b"RANGING" in chunk:
                return "tof"

            idx = chunk.find(MCU_SYNC)
            if idx != -1 and len(chunk) > idx + 2:
                msg_id = chunk[idx + 2]
                if msg_id in (2, 3, 4) or msg_id in (MSG_ID_TOF_TELEM, MSG_ID_TOF_SUB_TELEM):
                    return "tof"
                elif msg_id == MSG_ID_CHASSIS_TELEM:
                    # msg_id == 0x01 can be Chassis or ToF Sensor 1; query status to verify
                    ser.write(b"STATUS\n")
                    time.sleep(0.06)
                    resp = ser.read(256)
                    if b"FOLKRACE" in resp or b"STATUS" in resp or b"PONG" in resp:
                        return "main"
                    return "tof"
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
        self.link_active: Dict[str, bool] = {"ESP32_Main": False, "ESP32_ToF": False}
        self.last_rx_timestamp: Dict[str, float] = {"ESP32_Main": 0.0, "ESP32_ToF": 0.0}
        self.force_reconnect_flag = False
        self.discovery_lock = threading.Lock()

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
        Excludes ports actively assigned to other workers.
        """
        candidates = get_available_serial_ports()
        available = [p for p in candidates if p and p not in excluded_ports and os.path.exists(p)]
        if not available:
            return None

        # 1. Probe roles
        for p in available:
            role = probe_port_role(p, self.baud)
            if role == target_role:
                return p

        # 2. Check explicit configured arg if it exists and wasn't claimed
        configured = self.tof_port_arg if target_role == "tof" else self.main_port_arg
        if configured and configured != "auto" and configured in available:
            return configured

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
            sock.bind((self.udp_host, UDP_CMD_PORT))
            print(f"[MCU Bridge] Uplink Command Listener active on {self.udp_host}:{UDP_CMD_PORT}")
        except Exception as e:
            print(f"[MCU Bridge ERROR] Cannot bind UDP command port {UDP_CMD_PORT}: {e}")
            return

        sock.settimeout(0.5)
        while self.running:
            try:
                data, _ = sock.recvfrom(2048)
                if data:
                    d_strip = data.strip()
                    # Reconnect / Reset command
                    if d_strip in (b"RECONNECT", b"RESET_PORTS", b"RESTART_BRIDGE"):
                        self.trigger_reconnect()
                        continue
                    try:
                        cmd_obj = json.loads(data.decode("utf-8", errors="ignore"))
                        if isinstance(cmd_obj, dict):
                            if "action" in cmd_obj and cmd_obj["action"] in ("reconnect", "reset_ports"):
                                self.trigger_reconnect()
                                continue
                    except Exception:
                        pass
                    self.send_to_main_mcu(data)
            except socket.timeout:
                continue
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
                    self.tof_serial.write(data)
                    self.tof_serial.flush()
                except Exception as e:
                    print(f"[MCU Bridge ERROR] Failed writing to ESP32 ToF: {e}")

    def send_to_main_mcu(self, data: bytes):
        """Thread-safe transmission to ESP32 Main over USB Serial."""
        with self.serial_lock:
            if self.main_serial and getattr(self.main_serial, "is_open", False):
                try:
                    self.main_serial.write(data)
                    self.main_serial.flush()
                    preview = data.decode("utf-8", errors="replace").strip() if b"\n" in data else f"{len(data)}b binary"
                    print(f"[MCU Bridge -> ESP32] Sent: {preview}")
                except Exception as e:
                    print(f"[MCU Bridge ERROR] Failed writing to ESP32: {e}")
            else:
                print(f"[MCU Bridge WARN] Cannot dispatch command (ESP32 serial offline)")

    def _mcu_worker(self, label: str):
        """
        Self-healing worker for an MCU connection.
        Continuously attempts connection, supervises live data flow,
        detects disconnects or silent lockups, and cleanly reconnects.
        """
        is_tof = (label == "ESP32_ToF")
        role_target = "tof" if is_tof else "main"
        configured_port = self.tof_port_arg if is_tof else self.main_port_arg

        while self.running:
            # 1. Port Acquisition Phase
            port = None
            if configured_port and configured_port != "auto" and os.path.exists(configured_port):
                with self.discovery_lock:
                    other_label = "ESP32_ToF" if label == "ESP32_Main" else "ESP32_Main"
                    if configured_port != self.assigned_ports.get(other_label):
                        port = configured_port
            else:
                with self.discovery_lock:
                    other_label = "ESP32_ToF" if label == "ESP32_Main" else "ESP32_Main"
                    excluded = {self.assigned_ports.get(other_label)}
                    port = self.find_matching_port(role_target, excluded)

            if not port:
                self.link_active[label] = False
                self.assigned_ports[label] = None
                time.sleep(2.0)  # Reliable 2.0s scan period
                continue

            self.assigned_ports[label] = port
            print(f"[MCU Bridge] Connecting to {label} on {port} @ {self.baud} baud...")

            # 2. Connection and Streaming Phase
            parser = BinaryStreamParser(is_tof_port=is_tof)
            try:
                with serial.Serial(port, self.baud, timeout=0.2) as ser:
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()

                    with self.serial_lock:
                        if is_tof:
                            self.tof_serial = ser
                        else:
                            self.main_serial = ser

                    # Initialize MCU parameters
                    if is_tof:
                        try:
                            ser.write(b"L\n")  # Lean mode: 20B
                            ser.flush()
                        except Exception:
                            pass
                    else:
                        try:
                            ser.write(b"STATUS\n")
                            ser.flush()
                        except Exception:
                            pass

                    print(f"[MCU Bridge] Connected to {label} ({port}). Live streaming active.")
                    self.link_active[label] = True
                    last_pkt_time = time.time()
                    self.last_rx_timestamp[label] = last_pkt_time

                    while self.running:
                        if self.force_reconnect_flag:
                            print(f"[MCU Bridge] Force reconnect requested for {label}. Closing link...")
                            break

                        data = ser.read(256)
                        now = time.time()
                        if data:
                            parser.feed(data)
                            had_valid_packet = False
                            for msg_type, decoded in parser.extract_packets():
                                self._handle_packet(msg_type, decoded)
                                had_valid_packet = True

                            if had_valid_packet:
                                last_pkt_time = now
                                self.last_rx_timestamp[label] = now
                                self.link_active[label] = True

                        # Watchdog: Silence detection (3.0 seconds with no valid telemetry packets)
                        if now - last_pkt_time > 3.0:
                            print(f"[MCU Bridge] Watchdog: No telemetry from {label} on {port} for 3.0s (link lost or MCU reset). Reconnecting...")
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
                self.link_active[label] = False
                self.assigned_ports[label] = None
                print(f"[MCU Bridge] {label} disconnected. Waiting 2.0s before re-detecting port...")
                time.sleep(2.0)  # Grace period for USB bus & ESP32 boot

    def _handle_packet(self, msg_type: Any, pkt: Any):
        with self.lock:
            if msg_type == "TEXT":
                text = str(pkt)
                self.latest_mcu_msg = text
                self.mcu_log.append(text)
                if len(self.mcu_log) > 50:
                    self.mcu_log = self.mcu_log[-50:]
                print(f"[MCU Main Text] {text}")
                return

            if msg_type == MSG_ID_CHASSIS_TELEM:
                self.latest_chassis = pkt
            elif msg_type == MSG_ID_TOF_SUB_TELEM:
                # 20-byte per-sensor frame from ESP32_ToF/src/main.cpp
                sensor_id = pkt.get("sensor_id", 0)
                dists = pkt.get("distances_m", [])
                slots = SENSOR_CHANNEL_SLOTS.get(sensor_id, [])
                for i, slot in enumerate(slots):
                    if i < len(dists):
                        self.latest_tof_ranges_raw[slot] = dists[i]

                # Option 1: Instantaneous Forward Extrapolation across all 16 beams
                v_est = float(self.latest_chassis.get("v_est", self.latest_chassis.get("v_real", 0.0)))
                gyro_z = float(self.latest_chassis.get("gyro_z", 0.0))
                self.latest_tof_ranges = self.compensator.compensate_list(
                    self.latest_tof_ranges_raw, v_est=v_est, gyro_z=gyro_z, dt=0.01667
                )
            elif msg_type == MSG_ID_TOF_TELEM:
                # 78-byte full 16-channel packet
                dists = pkt.get("distances_m", [])
                if len(dists) == 16:
                    self.latest_tof_ranges_raw = list(dists)
                    v_est = float(self.latest_chassis.get("v_est", self.latest_chassis.get("v_real", 0.0)))
                    gyro_z = float(self.latest_chassis.get("gyro_z", 0.0))
                    self.latest_tof_ranges = self.compensator.compensate_list(
                        dists, v_est=v_est, gyro_z=gyro_z, dt=0.01667
                    )

            # Merge and dispatch over UDP
            telemetry = dict(self.latest_chassis)
            telemetry["timestamp"] = time.time()
            telemetry["tof_ranges"] = list(self.latest_tof_ranges)
            telemetry["tof_ranges_raw"] = list(self.latest_tof_ranges_raw)
            telemetry["f_tof"] = round(float(min(self.latest_tof_ranges)), 3) if self.latest_tof_ranges else 1.8
            telemetry["f_tof_raw"] = round(float(min(self.latest_tof_ranges_raw)), 3) if self.latest_tof_ranges_raw else 1.8
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
            telemetry["mcu_msg"] = self.latest_mcu_msg
            telemetry["mcu_log"] = list(self.mcu_log[-10:])
            telemetry["mcu_main_connected"] = bool(self.link_active.get("ESP32_Main", False))
            telemetry["mcu_tof_connected"] = bool(self.link_active.get("ESP32_ToF", False))
            telemetry["mcu_main_port"] = self.assigned_ports.get("ESP32_Main") or "Offline"
            telemetry["mcu_tof_port"] = self.assigned_ports.get("ESP32_ToF") or "Offline"

        try:
            raw = json.dumps(telemetry).encode("utf-8")
            self.udp_sock.sendto(raw, (self.udp_host, 9875))  # Real-time feed to Path Planner Core 1
            # Rate-limit fallback telemetry to Debug Server (port 9876) to 20 Hz (prevents flooding Core 2 & SQLite)
            now = time.time()
            if now - self._last_debug_dispatch >= 0.05:
                self._last_debug_dispatch = now
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

