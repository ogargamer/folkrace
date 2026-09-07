#!/usr/bin/env python3
"""
BNO08X IMU Real-Time 3D Telemetry Server (ESP32 High-Speed UART2 -> Pi 5 -> WebSocket Port 8000)
Supports 200Hz Binary Packets (42-byte header + 10 floats: Quat, Accel, Gyro),
Bidirectional command routing (Tare, Dynamic Rate, Report Modes, Reset),
and Zero-Latency Three.js Web Dashboard.
"""

import os
import sys
import time
import json
import struct
import math
import re
import glob
import threading
import serial
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.httpserver
import tornado.netutil

# Configuration
PORT = 8000
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
active_port_name = "/dev/ttyAMA2"

# Regex for Fallback Text Telemetry
QUAT_REGEX = re.compile(r'W\s*:\s*([+-]?\d+\.?\d*)\s+X\s*:\s*([+-]?\d+\.?\d*)\s+Y\s*:\s*([+-]?\d+\.?\d*)\s+Z\s*:\s*([+-]?\d+\.?\d*)', re.IGNORECASE)
RPY_REGEX = re.compile(r'R\s*:\s*([+-]?\d+\.?\d*)[°\s]+P\s*:\s*([+-]?\d+\.?\d*)[°\s]+Y\s*:\s*([+-]?\d+\.?\d*)', re.IGNORECASE)
ACCEL_REGEX = re.compile(r'Accel.*?\[\s*([+-]?\d+\.?\d*)\s+([+-]?\d+\.?\d*)\s+([+-]?\d+\.?\d*)\s*\]', re.IGNORECASE)
GYRO_REGEX = re.compile(r'Gyro.*?\[\s*([+-]?\d+\.?\d*)\s+([+-]?\d+\.?\d*)\s+([+-]?\d+\.?\d*)\s*\]', re.IGNORECASE)

def send_esp32_command(cmd_char):
    """Sends a single-byte command to ESP32 over UART2."""
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

def uart_reader_thread():
    global running, latest_telemetry, serial_instance, active_port_name

    port = "/dev/ttyAMA2" if os.path.exists("/dev/ttyAMA2") else "/dev/ttyAMA0"
    active_port_name = port
    print(f"[UART Reader] Starting reader on {port} @ {DEFAULT_BAUD} Baud...")

    while running:
        try:
            ser = serial.Serial(port, DEFAULT_BAUD, timeout=0.1)
            ser.reset_input_buffer()
            with serial_lock:
                serial_instance = ser
            print(f"[UART Reader] Connected to {port} @ {DEFAULT_BAUD} Baud!")
            line_buf = ""

            while running:
                raw = ser.read(ser.in_waiting or 42)
                if not raw:
                    continue

                # 1. Parse 42-Byte Binary Frame (0xAA 0x55 + 10 Floats: qw, qx, qy, qz, ax, ay, az, gx, gy, gz)
                idx = 0
                while idx <= len(raw) - 42:
                    if raw[idx] == 0xAA and raw[idx+1] == 0x55:
                        try:
                            qw, qx, qy, qz, ax, ay, az, gx, gy, gz = struct.unpack_from("<ffffffffff", raw, idx + 2)
                            
                            # Fast Euler Conversion
                            sinr_cosp = 2.0 * (qw * qx + qy * qz)
                            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                            roll = math.atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG

                            sinp = 2.0 * (qw * qy - qz * qx)
                            pitch = math.copysign(90.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp) * RAD_TO_DEG

                            siny_cosp = 2.0 * (qw * qz + qx * qy)
                            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                            yaw = math.atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG

                            with state_lock:
                                latest_telemetry["qw"] = round(qw, 4)
                                latest_telemetry["qx"] = round(qx, 4)
                                latest_telemetry["qy"] = round(qy, 4)
                                latest_telemetry["qz"] = round(qz, 4)
                                latest_telemetry["roll"] = round(roll, 2)
                                latest_telemetry["pitch"] = round(pitch, 2)
                                latest_telemetry["yaw"] = round(yaw, 2)
                                latest_telemetry["ax"] = round(ax, 2)
                                latest_telemetry["ay"] = round(ay, 2)
                                latest_telemetry["az"] = round(az, 2)
                                latest_telemetry["gx"] = round(gx * RAD_TO_DEG, 1)
                                latest_telemetry["gy"] = round(gy * RAD_TO_DEG, 1)
                                latest_telemetry["gz"] = round(gz * RAD_TO_DEG, 1)
                                latest_telemetry["ts"] = time.time()
                        except Exception:
                            pass
                        idx += 42
                    elif raw[idx] == 0xAA and raw[idx+1] == 0x55 and len(raw) - idx >= 30:
                        # Legacy 30-Byte Frame Fallback
                        try:
                            qw, qx, qy, qz, ax, ay, az = struct.unpack_from("<fffffff", raw, idx + 2)
                            sinr_cosp = 2.0 * (qw * qx + qy * qz)
                            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                            roll = math.atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG

                            sinp = 2.0 * (qw * qy - qz * qx)
                            pitch = math.copysign(90.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp) * RAD_TO_DEG

                            siny_cosp = 2.0 * (qw * qz + qx * qy)
                            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                            yaw = math.atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG

                            with state_lock:
                                latest_telemetry["qw"] = round(qw, 4)
                                latest_telemetry["qx"] = round(qx, 4)
                                latest_telemetry["qy"] = round(qy, 4)
                                latest_telemetry["qz"] = round(qz, 4)
                                latest_telemetry["roll"] = round(roll, 2)
                                latest_telemetry["pitch"] = round(pitch, 2)
                                latest_telemetry["yaw"] = round(yaw, 2)
                                latest_telemetry["ax"] = round(ax, 2)
                                latest_telemetry["ay"] = round(ay, 2)
                                latest_telemetry["az"] = round(az, 2)
                                latest_telemetry["ts"] = time.time()
                        except Exception:
                            pass
                        idx += 30
                    else:
                        idx += 1

                # 2. Text String Parsing Fallback
                try:
                    text = raw.decode('utf-8', errors='ignore')
                    line_buf += text
                    if '\n' in line_buf:
                        lines = line_buf.split('\n')
                        line_buf = lines[-1]
                        for line in lines[:-1]:
                            line = line.strip()
                            if not line or line.startswith("[ESP32-S3]"):
                                continue

                            qm = QUAT_REGEX.search(line)
                            rm = RPY_REGEX.search(line)
                            am = ACCEL_REGEX.search(line)
                            gm = GYRO_REGEX.search(line)

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
                except Exception:
                    pass

        except Exception as e:
            print(f"[UART Reader Error] {e}. Reconnecting in 2s...")
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
        (r"/static/(.*)", tornado.web.StaticFileHandler, {"path": static_dir}),
    ], template_path=base_dir)

if __name__ == "__main__":
    app = make_app()
    sockets = tornado.netutil.bind_sockets(PORT, address="0.0.0.0", reuse_port=True)
    server = tornado.httpserver.HTTPServer(app)
    server.add_sockets(sockets)
    main_ioloop = tornado.ioloop.IOLoop.current()

    # Start UART background reader thread
    threading.Thread(target=uart_reader_thread, daemon=True).start()

    # Start 200Hz WebSocket broadcaster (every 5ms)
    stream_timer = tornado.ioloop.PeriodicCallback(broadcast_tick, 5)
    stream_timer.start()

    print(f"================================================================")
    print(f"  ESP32 UART IMU DASHBOARD READY AT: http://10.43.8.42:{PORT}")
    print(f"================================================================")
    main_ioloop.start()
