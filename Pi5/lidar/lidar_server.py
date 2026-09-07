#!/usr/bin/env python3
"""
MS200 LiDAR Real-Time 2D Visualizer Server
Runs on Raspberry Pi 5. Serves an interactive 2D Radar & Point Cloud GUI via HTTP & WebSocket on port 8080.
"""

import sys
import os
import time
import struct
import math
import asyncio
import threading
import json
import tornado.ioloop
import tornado.web
import tornado.websocket
import serial

PORT = 8080
SERIAL_PORT = "/dev/ttyAMA4"
BAUDRATE = 230400

# MS200 CRC-8 Table
CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
]

def cal_crc8(data):
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc

def xor_check(data):
    chk = 0
    for b in data:
        chk ^= b
    return chk

# Shared state
ws_clients = set()
latest_points = []
state_lock = threading.Lock()
telemetry = {
    "hz": 10.0,
    "speed_raw": 3600,
    "fps": 0,
    "points_per_sec": 0,
    "points_in_frame": 0,
    "min_dist": 0,
    "max_dist": 0,
    "is_running": True,
    "port": SERIAL_PORT
}

ser_instance = None

def send_motor_cmd(cmd_type, value):
    global ser_instance
    if not ser_instance or not ser_instance.is_open:
        return False
    try:
        if cmd_type == "speed":
            hz = int(value)
            hz = max(5, min(15, hz))
            header = struct.pack("<HBBB", 0xF5A5, 0xA1, 0xC1, 2)
            payload = struct.pack("<H", hz)
            chk = xor_check(header + payload)
            pkt = header + payload + bytes([chk, 0x31, 0xF2])
            ser_instance.write(pkt)
            ser_instance.flush()
            print(f"Sent speed change to {hz} Hz")
            return True
        elif cmd_type == "mode":
            mode_byte = 0x81 if value else 0x80
            header = struct.pack("<HBBB", 0xF5A5, 0xA2, 0xC1, 1)
            payload = bytes([mode_byte])
            chk = xor_check(header + payload)
            pkt = header + payload + bytes([chk, 0x31, 0xF2])
            ser_instance.write(pkt)
            ser_instance.flush()
            print(f"Sent motor mode: {'START' if value else 'STOP'}")
            return True
    except Exception as e:
        print(f"Error sending serial command: {e}")
    return False

def lidar_worker():
    global ser_instance, latest_points, telemetry
    print(f"Starting LiDAR Reader on {SERIAL_PORT} @ {BAUDRATE}...")
    
    while True:
        try:
            ser_instance = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1.0)
            print(f"Serial port {SERIAL_PORT} opened successfully!")
            
            buffer = bytearray()
            current_revolution = []
            last_start_angle = 0.0
            frame_count = 0
            pt_count = 0
            last_stat_time = time.time()
            
            while True:
                chunk = ser_instance.read(ser_instance.in_waiting or 1)
                if chunk:
                    buffer.extend(chunk)
                
                while len(buffer) >= 47:
                    if buffer[0] == 0x54 and buffer[1] == 0x2C:
                        frame_data = buffer[:47]
                        expected_crc = frame_data[46]
                        if cal_crc8(frame_data[:46]) == expected_crc:
                            frame_count += 1
                            speed, start_angle = struct.unpack_from("<HH", frame_data, 2)
                            end_angle, timestamp = struct.unpack_from("<HH", frame_data, 42)
                            
                            start_deg = start_angle / 100.0
                            end_deg = end_angle / 100.0
                            
                            # Handle 360 wrap
                            step_deg = (end_deg - start_deg) / 11.0 if end_deg >= start_deg else (end_deg + 360.0 - start_deg) / 11.0
                            
                            # Parse 12 points
                            for i in range(12):
                                dist, conf = struct.unpack_from("<HB", frame_data, 6 + i * 3)
                                angle = (start_deg + step_deg * i) % 360.0
                                if dist > 0:
                                    # distance in mm, angle in degrees, confidence 0-255
                                    current_revolution.append([round(angle, 1), int(dist), int(conf)])
                                    pt_count += 1
                            
                            # Check if a full 360 revolution completed (angle wrapped)
                            if start_deg < last_start_angle and len(current_revolution) > 50:
                                with state_lock:
                                    latest_points = list(current_revolution)
                                    dists = [p[1] for p in latest_points if p[1] > 0]
                                    telemetry["min_dist"] = min(dists) if dists else 0
                                    telemetry["max_dist"] = max(dists) if dists else 0
                                    telemetry["points_in_frame"] = len(latest_points)
                                    telemetry["hz"] = round(speed / 360.0, 1)
                                    telemetry["speed_raw"] = speed
                                
                                current_revolution = []
                            
                            last_start_angle = start_deg
                            buffer = buffer[47:]
                        else:
                            buffer.pop(0)
                    else:
                        buffer.pop(0)
                
                # Update FPS and telemetry every second
                now = time.time()
                if now - last_stat_time >= 1.0:
                    with state_lock:
                        telemetry["fps"] = frame_count
                        telemetry["points_per_sec"] = pt_count
                    frame_count = 0
                    pt_count = 0
                    last_stat_time = now
                    
        except Exception as e:
            print(f"LiDAR Serial Error: {e}. Retrying in 2 seconds...")
            time.sleep(2)

class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        ws_clients.add(self)
        # Send initial config/status
        with state_lock:
            self.write_message(json.dumps({"type": "telemetry", "data": telemetry}))

    def on_message(self, message):
        try:
            req = json.loads(message)
            action = req.get("action")
            if action == "set_speed":
                hz = req.get("hz", 10)
                ok = send_motor_cmd("speed", hz)
                self.write_message(json.dumps({"type": "cmd_result", "action": "set_speed", "success": ok, "hz": hz}))
            elif action == "set_mode":
                running = req.get("running", True)
                ok = send_motor_cmd("mode", running)
                with state_lock:
                    telemetry["is_running"] = running
                self.write_message(json.dumps({"type": "cmd_result", "action": "set_mode", "success": ok, "running": running}))
        except Exception as e:
            print(f"WS error: {e}")

    def on_close(self):
        ws_clients.discard(self)

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html")

def broadcast_lidar_stream():
    if not ws_clients:
        return
    with state_lock:
        data_payload = {
            "type": "scan",
            "telemetry": telemetry,
            "points": latest_points
        }
    msg = json.dumps(data_payload)
    for c in list(ws_clients):
        try:
            c.write_message(msg)
        except:
            ws_clients.discard(c)

def make_app():
    static_path = os.path.join(os.path.dirname(__file__), "web")
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/ws", WebSocketHandler),
        (r"/(.*)", tornado.web.StaticFileHandler, {"path": static_path}),
    ], template_path=static_path)

def main():
    # Start background serial worker
    worker_thread = threading.Thread(target=lidar_worker, daemon=True)
    worker_thread.start()

    app = make_app()
    app.listen(PORT, address="0.0.0.0")
    print(f"==================================================================")
    print(f"  MS200 LIDAR 2D VISUALIZER RUNNING AT: http://0.0.0.0:{PORT}")
    print(f"==================================================================")
    
    # Broadcast scan updates every 33ms (~30 FPS)
    tornado.ioloop.PeriodicCallback(broadcast_lidar_stream, 33).start()
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
