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
    CHASSIS_SIZE,
    TOF_SIZE,
    decode_chassis_packet,
    decode_tof_packet,
    encode_motion_command,
)

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

    def __init__(self):
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

            msg_type = self.buffer[2]
            if msg_type == MSG_ID_CHASSIS_TELEM:
                if len(self.buffer) < CHASSIS_SIZE:
                    break
                raw_pkt = bytes(self.buffer[:CHASSIS_SIZE])
                del self.buffer[:CHASSIS_SIZE]
                decoded = decode_chassis_packet(raw_pkt)
                if decoded:
                    yield (MSG_ID_CHASSIS_TELEM, decoded)

            elif msg_type == MSG_ID_TOF_TELEM:
                if len(self.buffer) < TOF_SIZE:
                    break
                raw_pkt = bytes(self.buffer[:TOF_SIZE])
                del self.buffer[:TOF_SIZE]
                decoded = decode_tof_packet(raw_pkt)
                if decoded:
                    yield (MSG_ID_TOF_TELEM, decoded)
            else:
                # Unknown message ID: advance past sync header
                del self.buffer[:2]


class MCU_UART_Bridge:
    def __init__(
        self,
        main_port: Optional[str] = None,
        tof_port: Optional[str] = None,
        baud: int = UART_BAUD,
        udp_host: str = UDP_DISPATCH_HOST,
        udp_port: int = UDP_DISPATCH_PORT,
    ):
        self.main_port = main_port
        self.tof_port = tof_port
        self.baud = baud
        self.udp_target = (udp_host, udp_port)
        self.running = False

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setblocking(False)

        # State aggregation
        self.lock = threading.Lock()
        self.latest_chassis: Dict[str, Any] = {}
        self.latest_tof_ranges = [1.8] * 16

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
        parser = BinaryStreamParser()

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
            elif msg_type == MSG_ID_TOF_TELEM:
                # Update ToF distances
                dists = pkt.get("distances_m", [])
                if len(dists) == 16:
                    self.latest_tof_ranges = dists

            # Merge and dispatch over UDP
            telemetry = dict(self.latest_chassis)
            telemetry["timestamp"] = time.time()
            telemetry["tof_ranges"] = list(self.latest_tof_ranges)

        try:
            raw = json.dumps(telemetry).encode("utf-8")
            self.udp_sock.sendto(raw, self.udp_target)
        except Exception:
            pass


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Folkrace MCU Binary UART Bridge")
    parser.add_argument("--main", default="/dev/ttyAMA0", help="ESP32 Main serial device")
    parser.add_argument("--tof", default="/dev/ttyAMA2", help="ESP32 ToF Hub serial device")
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

