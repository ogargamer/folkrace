#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE — BNO086 9-DOF IMU 3D MISSION DASHBOARD & TELEMETRY SERVER
================================================================================
  Hardware: ESP32-S3 Master + Soldered GY-BNO086 (SPI2 Master)
  Pinout:   CS=10, INT=9, SCK=12, MOSI=11, MISO=13, RST=14
  Protocol: 921,600 Baud USB Serial (42-byte binary frames & text fallback)
  Features:
    - Three.js 3D Attitude Orientation (SLERP filtered)
    - PFD Artificial Horizon & Compass Heading Dial
    - G-Force Vector Accelerometer Gauge
    - Live 9-DOF Telemetry Grid (Euler, Accel, Gyro, Quat)
    - Real-Time Rolling Waveform Oscilloscope
    - Hardware Tare / Zero Yaw and Soft Reset
================================================================================
"""

import os
import sys
import time
import json
import struct
import math
import re
import argparse
import threading
import webbrowser

import serial
import serial.tools.list_ports

import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.httpserver
import tornado.netutil

# Configuration Defaults
DEFAULT_HTTP_PORT = 8000
DEFAULT_BAUD = 921600
RAD_TO_DEG = 180.0 / math.pi

# Global State
active_clients = set()
latest_telemetry = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
    "ax": 0.0, "ay": 0.0, "az": 9.81,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "ts": time.time()
}
state_lock = threading.Lock()
serial_lock = threading.Lock()
running = True
serial_instance = None
active_port_name = "AUTO"

# Fallback Text Parsing Regex
QUAT_REGEX = re.compile(r'W\s*:\s*([+-]?\d+\.?\d*)\s+X\s*:\s*([+-]?\d+\.?\d*)\s+Y\s*:\s*([+-]?\d+\.?\d*)\s+Z\s*:\s*([+-]?\d+\.?\d*)', re.IGNORECASE)
RPY_REGEX = re.compile(r'R\s*:\s*([+-]?\d+\.?\d*)[°\s]+P\s*:\s*([+-]?\d+\.?\d*)[°\s]+Y\s*:\s*([+-]?\d+\.?\d*)', re.IGNORECASE)
ACCEL_REGEX = re.compile(r'Accel.*?\[\s*([+-]?\d+\.?\d*)\s+([+-]?\d+\.?\d*)\s+([+-]?\d+\.?\d*)\s*\]', re.IGNORECASE)
GYRO_REGEX = re.compile(r'Gyro.*?\[\s*([+-]?\d+\.?\d*)\s+([+-]?\d+\.?\d*)\s+([+-]?\d+\.?\d*)\s*\]', re.IGNORECASE)


def auto_detect_esp32_port():
    """Detects ESP32 COM port based on description / VID:PID."""
    ports = list(serial.tools.list_ports.comports())
    # Priority 1: Common USB-UART bridges used by ESP32 (CH343, CP210x, FTDI, Espressif)
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(keyword in desc or keyword in hwid for keyword in ["ch343", "ch340", "cp210", "espressif", "usb-enhanced-serial", "uart"]):
            return p.device
    # Priority 2: Fallback to COM5 if present
    for p in ports:
        if p.device.upper() == "COM5":
            return "COM5"
    # Priority 3: First available non-Bluetooth serial port
    for p in ports:
        if "bluetooth" not in (p.description or "").lower():
            return p.device
    return None


def send_esp32_command(cmd_char):
    """Sends a single-byte command to ESP32 over Serial."""
    global serial_instance
    with serial_lock:
        if serial_instance and serial_instance.is_open:
            try:
                serial_instance.write(cmd_char.encode('utf-8'))
                serial_instance.flush()
                print(f"[UART TX] Sent command to ESP32: '{cmd_char}'")
                return True
            except Exception as e:
                print(f"[UART TX Error] Failed to send '{cmd_char}': {e}")
    return False


def serial_reader_thread(port_name, baud_rate):
    global running, latest_telemetry, serial_instance, active_port_name

    while running:
        chosen_port = port_name
        if not chosen_port or chosen_port == "AUTO":
            chosen_port = auto_detect_esp32_port()

        if not chosen_port:
            print("[Serial Reader] Waiting for ESP32 serial port...")
            time.sleep(1.5)
            continue

        active_port_name = chosen_port
        print(f"[Serial Reader] Opening {chosen_port} @ {baud_rate} Baud...")

        try:
            ser = serial.Serial(chosen_port, baud_rate, timeout=0.1)
            ser.reset_input_buffer()
            with serial_lock:
                serial_instance = ser
            print(f"[Serial Reader] CONNECTED to {chosen_port} @ {baud_rate} Baud!")

            # Enable 50 Hz USB IMU JSON stream on ESP32_Main
            time.sleep(0.1)
            ser.write(b"IMU_STREAM 1\n")

            line_buf = ""
            raw_buf = bytearray()

            while running:
                available = ser.in_waiting
                chunk = ser.read(available if available > 0 else 1)
                if not chunk:
                    continue

                raw_buf.extend(chunk)

                # 1. Parse Packets
                while len(raw_buf) >= 42:
                    # Scan for sync sequence 0xAA, 0x55
                    sync_idx = raw_buf.find(b'\xAA\x55')
                    if sync_idx == -1:
                        raw_buf = raw_buf[-1:]
                        break
                    elif sync_idx > 0:
                        pre_slice = raw_buf[:sync_idx]
                        raw_buf = raw_buf[sync_idx:]
                        try:
                            line_buf += pre_slice.decode('utf-8', errors='ignore')
                        except Exception:
                            pass

                    # Fast-path skip 54-byte Chassis Telemetry packet if present
                    if len(raw_buf) >= 3 and raw_buf[2] == 0x01:
                        if len(raw_buf) >= 54:
                            raw_buf = raw_buf[54:]
                            continue
                        else:
                            break

                    if len(raw_buf) < 42:
                        break

                    try:
                        qw, qx, qy, qz, ax, ay, az, gx, gy, gz = struct.unpack_from("<ffffffffff", raw_buf, 2)

                        # Sanity check unit quaternion norm
                        norm_sq = qw*qw + qx*qx + qy*qy + qz*qz
                        if 0.5 <= norm_sq <= 1.5:
                            # Valid 42-byte IMU binary frame!
                            sinr_cosp = 2.0 * (qw * qx + qy * qz)
                            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                            roll = math.atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG

                            sinp = 2.0 * (qw * qy - qz * qx)
                            pitch = math.copysign(90.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp) * RAD_TO_DEG

                            siny_cosp = 2.0 * (qw * qz + qx * qy)
                            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                            yaw = math.atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG

                            with state_lock:
                                latest_telemetry["qw"] = round(float(qw), 4)
                                latest_telemetry["qx"] = round(float(qx), 4)
                                latest_telemetry["qy"] = round(float(qy), 4)
                                latest_telemetry["qz"] = round(float(qz), 4)
                                latest_telemetry["roll"] = round(float(roll), 2)
                                latest_telemetry["pitch"] = round(float(pitch), 2)
                                latest_telemetry["yaw"] = round(float(yaw), 2)
                                latest_telemetry["ax"] = round(float(ax), 2)
                                latest_telemetry["ay"] = round(float(ay), 2)
                                latest_telemetry["az"] = round(float(az), 2)
                                latest_telemetry["gx"] = round(float(gx) * RAD_TO_DEG, 1)
                                latest_telemetry["gy"] = round(float(gy) * RAD_TO_DEG, 1)
                                latest_telemetry["gz"] = round(float(gz) * RAD_TO_DEG, 1)
                                latest_telemetry["ts"] = time.time()

                            raw_buf = raw_buf[42:]
                            continue
                        else:
                            raw_buf = raw_buf[1:]
                    except Exception:
                        raw_buf = raw_buf[1:]

                # 2. Parse JSON Stream & Fallback Text Telemetry
                if '\n' in line_buf:
                    lines = line_buf.split('\n')
                    line_buf = lines[-1]
                    for l in lines[:-1]:
                        line = l.strip()
                        if not line or line.startswith("[400Hz BENCH]"):
                            continue
                        if line.startswith("{"):
                            try:
                                obj = json.loads(line)
                                if obj.get("imu") == 1 and obj.get("online", 1) == 1:
                                    with state_lock:
                                        latest_telemetry["qw"] = round(float(obj.get("qw", 1.0)), 4)
                                        latest_telemetry["qx"] = round(float(obj.get("qx", 0.0)), 4)
                                        latest_telemetry["qy"] = round(float(obj.get("qy", 0.0)), 4)
                                        latest_telemetry["qz"] = round(float(obj.get("qz", 0.0)), 4)
                                        latest_telemetry["roll"] = round(float(obj.get("r", 0.0)), 2)
                                        latest_telemetry["pitch"] = round(float(obj.get("p", 0.0)), 2)
                                        latest_telemetry["yaw"] = round(float(obj.get("y", 0.0)), 2)
                                        latest_telemetry["ax"] = round(float(obj.get("ax", 0.0)), 2)
                                        latest_telemetry["ay"] = round(float(obj.get("ay", 0.0)), 2)
                                        latest_telemetry["az"] = round(float(obj.get("az", 9.81)), 2)
                                        latest_telemetry["gx"] = round(float(obj.get("gx", 0.0)) * RAD_TO_DEG, 1)
                                        latest_telemetry["gy"] = round(float(obj.get("gy", 0.0)) * RAD_TO_DEG, 1)
                                        latest_telemetry["gz"] = round(float(obj.get("gz", 0.0)) * RAD_TO_DEG, 1)
                                        latest_telemetry["ts"] = time.time()
                                    continue
                            except Exception:
                                pass
                        qm = QUAT_REGEX.search(line)
                        rm = RPY_REGEX.search(line)
                        am = ACCEL_REGEX.search(line)
                        gm = GYRO_REGEX.search(line)
                        if qm or rm or am or gm:
                            with state_lock:
                                if qm:
                                    latest_telemetry["qw"] = float(qm.group(1))
                                    latest_telemetry["qx"] = float(qm.group(2))
                                    latest_telemetry["qy"] = float(qm.group(3))
                                    latest_telemetry["qz"] = float(qm.group(4))
                                if rm:
                                    latest_telemetry["roll"] = float(rm.group(1))
                                    latest_telemetry["pitch"] = float(rm.group(2))
                                    latest_telemetry["yaw"] = float(rm.group(3))
                                if am:
                                    latest_telemetry["ax"] = float(am.group(1))
                                    latest_telemetry["ay"] = float(am.group(2))
                                    latest_telemetry["az"] = float(am.group(3))
                                if gm:
                                    latest_telemetry["gx"] = float(gm.group(1))
                                    latest_telemetry["gy"] = float(gm.group(2))
                                    latest_telemetry["gz"] = float(gm.group(3))
                                latest_telemetry["ts"] = time.time()

        except Exception as e:
            print(f"[Serial Reader Error] {e}. Reconnecting in 2s...")
            with serial_lock:
                serial_instance = None
            time.sleep(2.0)


def broadcast_tick():
    global active_clients, latest_telemetry
    if not active_clients:
        return

    with state_lock:
        packet = dict(latest_telemetry)

    msg_str = json.dumps(packet)
    for client in list(active_clients):
        try:
            client.write_message(msg_str)
        except Exception:
            pass


class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("static/index.html")


class IMUWebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        active_clients.add(self)
        try:
            if hasattr(self, 'stream') and self.stream:
                self.stream.set_nodelay(True)
            if hasattr(self, 'ws_connection') and self.ws_connection and hasattr(self.ws_connection, 'stream'):
                self.ws_connection.stream.set_nodelay(True)
        except Exception:
            pass

    def on_message(self, message):
        try:
            data = json.loads(message)
            action = data.get("action")
            if action == "tare":
                send_esp32_command('t')
            elif action == "reset":
                send_esp32_command('r')
            elif action == "set_rate":
                hz = int(data.get("hz", 100))
                if hz >= 200:
                    send_esp32_command('+')
                elif hz <= 50:
                    send_esp32_command('-')
                else:
                    send_esp32_command('0')
            elif action == "set_mode":
                mode = str(data.get("mode", "1"))
                send_esp32_command(mode)
        except Exception as e:
            print(f"[WS Command Error] {e}")

    def on_close(self):
        active_clients.discard(self)


class TareHandler(tornado.web.RequestHandler):
    def post(self):
        success = send_esp32_command('t')
        self.write({"status": "success" if success else "error", "action": "tare"})


class SetRateHandler(tornado.web.RequestHandler):
    def post(self):
        try:
            hz = int(self.get_argument("hz", "100"))
            if hz >= 200:
                send_esp32_command('+')
            elif hz <= 50:
                send_esp32_command('-')
            else:
                send_esp32_command('0')
            self.write({"status": "success", "rate_hz": hz})
        except Exception as e:
            self.write({"status": "error", "message": str(e)})


class SetModeHandler(tornado.web.RequestHandler):
    def post(self):
        try:
            mode = self.get_argument("mode", "1")
            send_esp32_command(mode)
            self.write({"status": "success", "mode": mode})
        except Exception as e:
            self.write({"status": "error", "message": str(e)})


class ResetHandler(tornado.web.RequestHandler):
    def post(self):
        success = send_esp32_command('r')
        self.write({"status": "success" if success else "error", "action": "reset"})


class ToggleFeatureHandler(tornado.web.RequestHandler):
    def post(self):
        feature = self.get_argument("feature", "")
        # Mode 1 is game rotation vector, mode 2 is 9-axis rotation vector
        if "rotation" in feature.lower():
            send_esp32_command('1')
        elif "gyro" in feature.lower() or "accel" in feature.lower():
            send_esp32_command('3')
        self.write({"status": "success", "feature": feature})


def make_app():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    static_dir = os.path.join(base_dir, "static")
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/ws", IMUWebSocketHandler),
        (r"/api/tare", TareHandler),
        (r"/api/set_rate", SetRateHandler),
        (r"/api/set_mode", SetModeHandler),
        (r"/api/reset", ResetHandler),
        (r"/api/toggle_feature", ToggleFeatureHandler),
        (r"/static/(.*)", tornado.web.StaticFileHandler, {"path": static_dir}),
    ], template_path=base_dir)


def main():
    parser = argparse.ArgumentParser(description="BNO086 9-DOF IMU 3D Mission Dashboard")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Serial Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--web-port", type=int, default=DEFAULT_HTTP_PORT, help=f"HTTP Port (default: {DEFAULT_HTTP_PORT})")
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically launch web browser")
    args = parser.parse_args()

    app = make_app()
    app.listen(args.web_port, address="0.0.0.0")
    main_ioloop = tornado.ioloop.IOLoop.current()

    # Start serial background reader
    t = threading.Thread(target=serial_reader_thread, args=(args.port, args.baud), daemon=True)
    t.start()

    # High-rate WebSocket telemetry broadcaster (every 10 ms = 100 Hz)
    stream_timer = tornado.ioloop.PeriodicCallback(broadcast_tick, 10)
    stream_timer.start()

    url = f"http://localhost:{args.web_port}"
    print("=" * 72)
    print("   FOLKRACE — BNO086 9-DOF IMU 3D MISSION DASHBOARD")
    print(f"   URL: {url}")
    print(f"   Serial Port: {args.port or 'AUTO-DETECT'} @ {args.baud} Baud")
    print("=" * 72)

    if not args.no_browser:
        def open_browser():
            time.sleep(1.0)
            print(f"[Launcher] Opening web dashboard in browser: {url}")
            webbrowser.open(url)
        threading.Thread(target=open_browser, daemon=True).start()

    try:
        main_ioloop.start()
    except KeyboardInterrupt:
        print("\n[Server] Shutting down...")


if __name__ == "__main__":
    main()
