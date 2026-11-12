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
import threading
from typing import Optional, Dict, Any, Tuple

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

    def __init__(self, is_tof_port: bool = False, accept_s10: bool = True):
        self.is_tof_port = is_tof_port
        self.accept_s10 = accept_s10
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
                # ESP32-ToF Port (supports 52-byte multi-target 4-zone, 60-byte multi-target, 20-byte per-sensor, and 78-byte full array)
                if 1 <= msg_type_or_sensor_id <= 4:
                    if len(self.buffer) >= TOF_SUB52_SIZE:
                        raw_pkt = bytes(self.buffer[:TOF_SUB52_SIZE])
                        decoded = decode_tof_sub_packet(raw_pkt, accept_s10=self.accept_s10)
                        if decoded and decoded.get("dist_mm") is not None:
                            del self.buffer[:TOF_SUB52_SIZE]
                            yield (MSG_ID_TOF_SUB_TELEM, decoded)
                            continue

                    if len(self.buffer) >= TOF_SUB_SIZE:
                        raw_pkt = bytes(self.buffer[:TOF_SUB_SIZE])
                        decoded = decode_tof_sub_packet(raw_pkt, accept_s10=self.accept_s10)
                        if decoded:
                            del self.buffer[:TOF_SUB_SIZE]
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


def probe_port_role(port: str, baud: int = UART_BAUD) -> Optional[str]:
    """Sniffs incoming bytes on port for 200 ms to identify ESP32 role."""
    if not HAS_SERIAL or not os.path.exists(port):
        return None
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
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
    candidates = [
        p for p in [
            "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
            "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2",
            "/dev/ttyAMA0", "/dev/ttyAMA2"
        ] if os.path.exists(p)
    ]

    resolved_main = main_port if main_port and main_port != "auto" and os.path.exists(main_port) else None
    resolved_tof = tof_port if tof_port and tof_port != "auto" and os.path.exists(tof_port) else None

    # Filter out already explicitly claimed ports
    available = [c for c in candidates if c not in (resolved_main, resolved_tof)]

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
        accept_s10: bool = True,
    ):
        self.main_port, self.tof_port = resolve_serial_ports(main_port, tof_port, baud)
        self.baud = baud
        self.udp_host = udp_host
        self.udp_target = (udp_host, udp_port)
        self.accept_s10 = accept_s10
        self.running = False

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

    def start(self):
        if not HAS_SERIAL:
            print("[MCU Bridge ERROR] pyserial is not installed on this system!")
            return

        self.running = True

        # Start Command Ingestion Worker (listens for uplink packets to forward to ESP32 Main)
        t_cmd = threading.Thread(target=self._command_listener_worker, name="MCUCommandWorker", daemon=True)
        t_cmd.start()

        if self.main_port and os.path.exists(self.main_port):
            t_main = threading.Thread(target=self._uart_worker, args=(self.main_port, "ESP32_Main"), daemon=True)
            t_main.start()
        else:
            print(f"[MCU Bridge] ESP32 Main port not specified or offline: {self.main_port}")

        if self.tof_port and os.path.exists(self.tof_port):
            t_tof = threading.Thread(target=self._uart_worker, args=(self.tof_port, "ESP32_ToF"), daemon=True)
            t_tof.start()
        else:
            print(f"[MCU Bridge] ESP32 ToF port not specified or offline: {self.tof_port}")

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
                    # Intercept S10 toggle commands for ESP32 ToF Hub
                    d_strip = data.strip()
                    if d_strip in (b"S10_1", b"S10_ON", b"SET_S10_1"):
                        self.set_s10(True)
                        continue
                    elif d_strip in (b"S10_0", b"S10_OFF", b"SET_S10_0"):
                        self.set_s10(False)
                        continue
                    try:
                        cmd_obj = json.loads(data.decode("utf-8", errors="ignore"))
                        if isinstance(cmd_obj, dict) and "s10" in cmd_obj:
                            self.set_s10(bool(cmd_obj["s10"]))
                            continue
                    except Exception:
                        pass
                    self.send_to_main_mcu(data)
            except socket.timeout:
                continue
            except Exception as e:
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

    def set_s10(self, accept: bool):
        """Sets S10 acceptance and commands ESP32 ToF onboard processing via serial."""
        self.accept_s10 = accept
        self.send_to_tof_mcu(b"1\n" if accept else b"0\n")
        print(f"[MCU Bridge] ESP32 ToF S10 acceptance set to: {accept}")

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

    def _uart_worker(self, port_name: str, label: str):
        print(f"[MCU Bridge] Connecting to {label} on {port_name} @ {self.baud} baud...")
        is_tof = (label == "ESP32_ToF")
        parser = BinaryStreamParser(is_tof_port=is_tof, accept_s10=self.accept_s10)

        while self.running:
            try:
                with serial.Serial(port_name, self.baud, timeout=0.1) as ser:
                    print(f"[MCU Bridge] Connected to {label} ({port_name})")
                    if label == "ESP32_Main":
                        with self.serial_lock:
                            self.main_serial = ser
                    elif label == "ESP32_ToF":
                        with self.serial_lock:
                            self.tof_serial = ser
                        # Ensure ESP32 ToF Hub is in lean 20-byte mode for Pi and send S10 state
                        try:
                            ser.write(b"L\n")
                            ser.write(b"1\n" if self.accept_s10 else b"0\n")
                            ser.flush()
                        except Exception:
                            pass
                    try:
                        while self.running:
                            data = ser.read(256)
                            if data:
                                parser.feed(data)
                                for msg_type, decoded in parser.extract_packets():
                                    self._handle_packet(msg_type, decoded)
                    finally:
                        if label == "ESP32_Main":
                            with self.serial_lock:
                                self.main_serial = None
                        elif label == "ESP32_ToF":
                            with self.serial_lock:
                                self.tof_serial = None
            except Exception as e:
                print(f"[MCU Bridge] Error on {label} ({port_name}): {e}")
                time.sleep(1.0)

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
    parser.add_argument("--no-s10", dest="accept_s10", action="store_false", help="Reject ST Status 10 (Valid No Prev) in ToF arbitration")
    parser.set_defaults(accept_s10=True)
    args = parser.parse_args()

    bridge = MCU_UART_Bridge(main_port=args.main, tof_port=args.tof, baud=args.baud, accept_s10=args.accept_s10)
    bridge.start()

    print("[MCU Bridge] Active. Press Ctrl+C to terminate.")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        bridge.stop()
        print("\n[MCU Bridge] Terminated.")

