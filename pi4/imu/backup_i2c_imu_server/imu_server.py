#!/usr/bin/env python3
"""
BNO08X IMU High-Precision Constant-Rate Telemetry Server (Raspberry Pi 5)
Zero-Jitter Architecture: Dedicated Hardware Thread + Non-Blocking WebSocket Streamer.
"""

import os
import json
import time
import threading
import tornado.ioloop
import tornado.web
import tornado.websocket

# Import our thread-safe BNO08X I2C driver
from bno08x_i2c import BNO08xI2C

# Global State
active_clients = set()
current_rate_hz = 100
stream_timer = None
main_ioloop = None
running = True

latest_telemetry = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
    "ax": 0.0, "ay": 0.0, "az": 9.81,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "ts": time.time()
}
state_lock = threading.Lock()

# Initialize IMU
print("[IMU Server] Initializing BNO08X on I2C bus 1 (Address 0x4B)...")
imu = BNO08xI2C(bus_id=1, address=0x4B, rst_pin=21, int_pin=26)
imu.reset()

def reconfigure_sensor(hz):
    global current_rate_hz, imu, stream_timer
    current_rate_hz = max(5, min(200, int(hz)))
    interval_us = int(1000000 / current_rate_hz)
    
    print(f"[IMU Server] Reconfiguring BNO08X reports to {current_rate_hz} Hz ({interval_us} µs)...")
    if stream_timer:
        stream_timer.stop()

    # Re-apply 3 core reports with command spacing
    imu.enable_feature(BNO08xI2C.REPORT_ARVR_STABILIZED_GRV, interval_us)
    imu.enable_feature(BNO08xI2C.REPORT_ACCELEROMETER, interval_us)
    imu.enable_feature(BNO08xI2C.REPORT_GYROSCOPE, interval_us)
    
    # Broadcast timer tick (e.g. 10ms for 100Hz, 20ms for 50Hz)
    period_ms = max(5, int(1000 / current_rate_hz))
    stream_timer = tornado.ioloop.PeriodicCallback(broadcast_imu_tick, period_ms)
    stream_timer.start()
    print(f"[IMU Server] Constant stream active @ {period_ms}ms interval ({current_rate_hz} Hz)")

def hardware_reader_thread():
    global imu, running, latest_telemetry
    print("[IMU Server] Hardware I2C Reader thread active!")
    while running:
        try:
            if imu.update():
                d = imu.data
                now = time.time()
                with state_lock:
                    latest_telemetry["roll"] = round(d["roll"], 2)
                    latest_telemetry["pitch"] = round(d["pitch"], 2)
                    latest_telemetry["yaw"] = round(d["yaw"], 2)
                    latest_telemetry["qw"] = round(d["quat_w"], 4)
                    latest_telemetry["qx"] = round(d["quat_x"], 4)
                    latest_telemetry["qy"] = round(d["quat_y"], 4)
                    latest_telemetry["qz"] = round(d["quat_z"], 4)
                    latest_telemetry["ax"] = round(d["accel_x"], 2)
                    latest_telemetry["ay"] = round(d["accel_y"], 2)
                    latest_telemetry["az"] = round(d["accel_z"], 2)
                    latest_telemetry["gx"] = round(d["gyro_x"] * 57.2958, 1)
                    latest_telemetry["gy"] = round(d["gyro_y"] * 57.2958, 1)
                    latest_telemetry["gz"] = round(d["gyro_z"] * 57.2958, 1)
                    latest_telemetry["ts"] = now
            time.sleep(0.001)
        except Exception:
            time.sleep(0.002)

def broadcast_imu_tick():
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

# Tornado Handlers
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
        print(f"[WebSocket] Client connected with TCP_NODELAY ({len(active_clients)} active).")

    def on_close(self):
        active_clients.discard(self)
        print(f"[WebSocket] Client disconnected ({len(active_clients)} active).")

class SetRateHandler(tornado.web.RequestHandler):
    def post(self):
        try:
            hz = int(self.get_argument("hz", "100"))
            reconfigure_sensor(hz)
            self.write({"status": "success", "hz": current_rate_hz})
        except Exception as e:
            self.write({"status": "error", "message": str(e)})

class ResetHandler(tornado.web.RequestHandler):
    def post(self):
        global imu, current_rate_hz
        try:
            print("[Settings] Performing Sensor Hardware Reset & Re-arm...")
            imu.reset()
            reconfigure_sensor(current_rate_hz)
            self.write({"status": "success", "message": "Sensor reset completed."})
        except Exception as e:
            self.write({"status": "error", "message": str(e)})

def make_app():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    static_dir = os.path.join(base_dir, "static")
    return tornado.web.Application([
        (r"/", MainHandler),
        (r"/ws", IMUWebSocketHandler),
        (r"/api/set_rate", SetRateHandler),
        (r"/api/reset", ResetHandler),
        (r"/static/(.*)", tornado.web.StaticFileHandler, {"path": static_dir}),
    ], template_path=base_dir)

if __name__ == "__main__":
    PORT = 8000
    app = make_app()
    app.listen(PORT, address="0.0.0.0")
    main_ioloop = tornado.ioloop.IOLoop.current()
    
    # Start background I2C reader thread
    threading.Thread(target=hardware_reader_thread, daemon=True).start()
    
    # Start initial rate stream @ 100Hz
    reconfigure_sensor(100)
    
    print(f"================================================================")
    print(f"  BNO08X TELEMETRY DASHBOARD READY AT: http://10.43.8.42:{PORT}")
    print(f"================================================================")
    main_ioloop.start()
