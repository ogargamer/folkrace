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
from typing import Optional, Dict, Any

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
                # Keep last byte in case header split across reads
                self.buffer = self.buffer[-1:]
                break

            if idx > 0:
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
                # ESP32-ToF Port (supports 60-byte multi-target, 20-byte per-sensor, and 78-byte full array)
                if 1 <= msg_type_or_sensor_id <= 4:
                    if len(self.buffer) >= TOF_SUB60_SIZE:
                        raw_pkt = bytes(self.buffer[:TOF_SUB60_SIZE])
                        decoded = decode_tof_sub_packet(raw_pkt)
                        if decoded:
                            del self.buffer[:TOF_SUB60_SIZE]
                            yield (MSG_ID_TOF_SUB_TELEM, decoded)
                            continue

                    if len(self.buffer) >= TOF_SUB_SIZE and len(self.buffer) < TOF_SUB60_SIZE:
                        raw_pkt = bytes(self.buffer[:TOF_SUB_SIZE])
                        decoded = decode_tof_sub_packet(raw_pkt)
                        if decoded:
                            del self.buffer[:TOF_SUB_SIZE]
                            yield (MSG_ID_TOF_SUB_TELEM, decoded)
                            continue

                    if len(self.buffer) < TOF_SUB60_SIZE:
                        break

                    del self.buffer[:2]
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


def resolve_serial_port(port: Optional[str], role: str = "tof", exclude: Optional[str] = None) -> Optional[str]:
    """Auto-detects and verifies available serial port for MCU communication."""
    if port and port != "auto" and os.path.exists(port):
        return port
    if role == "tof":
        candidates = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1", "/dev/ttyAMA2", "/dev/ttyAMA1"]
    else:
        candidates = ["/dev/ttyAMA0", "/dev/ttyUSB1", "/dev/ttyUSB0"]
    for c in candidates:
        if c != exclude and os.path.exists(c):
            return c
    return port if port != "auto" else None


class MCU_UART_Bridge:
    def __init__(
        self,
        main_port: Optional[str] = None,
        tof_port: Optional[str] = None,
        baud: int = UART_BAUD,
        udp_host: str = UDP_DISPATCH_HOST,
        udp_port: int = UDP_DISPATCH_PORT,
    ):
        self.tof_port = resolve_serial_port(tof_port, role="tof")
        self.main_port = resolve_serial_port(main_port, role="main", exclude=self.tof_port)
        self.baud = baud
        self.udp_host = udp_host
        self.udp_target = (udp_host, udp_port)
        self.running = False

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setblocking(False)

        # State aggregation & Option 1 Latency Compensator
        self.compensator = ToFLatencyCompensator()
        self.lock = threading.Lock()
        self.latest_chassis: Dict[str, Any] = {}
        self.latest_tof_ranges = [1.8] * 16
        self.latest_tof_ranges_raw = [1.8] * 16

    def start(self):
        if not HAS_SERIAL:
            print("[MCU Bridge ERROR] pyserial is not installed on this system!")
            return

        self.running = True
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

    def _uart_worker(self, port_name: str, label: str):
        print(f"[MCU Bridge] Connecting to {label} on {port_name} @ {self.baud} baud...")
        is_tof = (label == "ESP32_ToF")
        parser = BinaryStreamParser(is_tof_port=is_tof)

        while self.running:
            try:
                with serial.Serial(port_name, self.baud, timeout=0.1) as ser:
                    print(f"[MCU Bridge] Connected to {label} ({port_name})")
                    while self.running:
                        data = ser.read(256)
                        if data:
                            parser.feed(data)
                            for msg_type, decoded in parser.extract_packets():
                                self._handle_packet(msg_type, decoded)
            except Exception as e:
                print(f"[MCU Bridge] Error on {label} ({port_name}): {e}")
                time.sleep(1.0)

    def _handle_packet(self, msg_type: int, pkt: Dict[str, Any]):
        with self.lock:
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

        try:
            raw = json.dumps(telemetry).encode("utf-8")
            self.udp_sock.sendto(raw, (self.udp_host, 9875))  # Real-time feed to Path Planner Core 1
            self.udp_sock.sendto(raw, self.udp_target)        # Telemetry to Debug Server Core 2
        except Exception:
            pass


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Folkrace MCU Binary UART Bridge")
    parser.add_argument("--main", default="/dev/ttyAMA0", help="ESP32 Main serial device (or auto)")
    parser.add_argument("--tof", default="auto", help="ESP32 ToF Hub serial device (auto-detects /dev/ttyACM0, /dev/ttyUSB0, /dev/ttyAMA2, etc.)")
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

