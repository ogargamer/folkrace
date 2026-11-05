#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: REAL-TIME TACTICAL DEBUG SERVER & FLIGHT RECORDER
================================================================================
  Hardware Isolation:
    - Strictly pinned to dedicated CPU Core 2 (leaving Core 1 100% to Path Planner).
    - Asynchronous, zero-stall telemetry ingestion.
    - Zero interference with the 60 Hz real-time SL-MPC planning pipeline.

  Capabilities:
    - UDP Telemetry Listener on Port 9876 (accepts planner & sensor packets).
    - SQLite WAL Black Box Flight Recorder (persists to 64GB SD card).
    - Embedded Mission Control Web Server on Port 8080 (HTML5 tactical dashboard).
    - Frame-by-frame time travel API: allows scrubbing back to any exact moment.
    - Anomaly index: instant jump to unexpected slip, contact, or latency events.
================================================================================
"""

import os
import sys
import time
import json
import zlib
import base64
import socket
import shutil
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from typing import Dict, List, Optional, Any

# Ensure pi4 folder is in sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

from core_manager import set_core_affinity, get_core_affinity, get_system_cpu_stats
from debug_recorder import TelemetryFlightRecorder, TelemetryLogReader

PORT_HTTP = 8080
PORT_UDP_RECV = 9876
HOST_BIND = "0.0.0.0"

HTML_PATH = os.path.join(CURR_DIR, "web_debug_gui.html")


class DebugServerState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_pkt: Dict[str, Any] = {}
        self.total_frames_received = 0
        self.last_pkt_time = 0.0
        self.loop_hz = 0.0
        self._hz_counter = 0
        self._last_hz_calc = time.time()
        self.recorder: Optional[TelemetryFlightRecorder] = None


STATE = DebugServerState()


def decompress_costmap_for_web(pkt: Dict[str, Any]):
    """
    Decodes zlib base64 costmap on Core 2 to offload processing from browser.
    Creates a 2D array representation in pkt['grid_unpacked'].
    """
    if "grid_data" in pkt and "grid_shape" in pkt and "grid_unpacked" not in pkt:
        try:
            shape = tuple(pkt["grid_shape"])  # e.g. (95, 125)
            raw_b = zlib.decompress(base64.b64decode(pkt["grid_data"]))
            # Downsample / convert to list of lists for easy JSON delivery
            # Unpack bytes into uint8 array
            vals = list(raw_b)
            rows, cols = shape
            grid_2d = []
            for r in range(rows):
                row = vals[r * cols : (r + 1) * cols]
                # Normalize 0..255 to 0.0..1.0
                grid_2d.append([round(v / 255.0, 2) for v in row])
            pkt["grid_unpacked"] = grid_2d
        except Exception:
            pass


class DebugHTTPRequestHandler(BaseHTTPRequestHandler):
    """Handles HTTP requests for Mission Control UI and REST endpoints."""

    def log_message(self, format, *args):
        # Silence routine request logging to prevent terminal spam
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ("/", "/index.html"):
            self._serve_file(HTML_PATH, "text/html")

        elif path == "/api/status":
            self._serve_status()

        elif path == "/api/live":
            self._serve_live()

        elif path.startswith("/api/frame/"):
            try:
                frame_id = int(path.split("/")[-1])
                self._serve_frame(frame_id)
            except ValueError:
                self._send_json({"error": "Invalid frame ID"}, 400)

        elif path == "/api/anomalies":
            self._serve_anomalies()

        elif path == "/api/download":
            self._serve_download()

        else:
            self._send_json({"error": "Not Found"}, 404)

    def _serve_file(self, filepath: str, content_type: str):
        if not os.path.exists(filepath):
            self.send_error(404, "File not found")
            return
        try:
            with open(filepath, "rb") as f:
                content = f.read()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(content)
        except Exception as e:
            self.send_error(500, f"Error reading file: {e}")

    def _send_json(self, data: Any, code: int = 200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_status(self):
        stats = get_system_cpu_stats()
        disk_total, disk_used, disk_free = shutil.disk_usage(STATE.recorder.output_dir if STATE.recorder else CURR_DIR)

        db_size_mb = 0.0
        if STATE.recorder and os.path.exists(STATE.recorder.db_path):
            db_size_mb = round(os.path.getsize(STATE.recorder.db_path) / (1024 * 1024), 2)

        data = {
            "session_id": STATE.recorder.session_id if STATE.recorder else "none",
            "total_recorded": STATE.recorder.total_frames_recorded if STATE.recorder else 0,
            "dropped_frames": STATE.recorder.dropped_frames if STATE.recorder else 0,
            "pruned_frames": STATE.recorder.pruned_frames_count if STATE.recorder else 0,
            "driving_seconds": round(STATE.recorder.total_driving_seconds, 1) if STATE.recorder else 0.0,
            "driving_minutes": round(STATE.recorder.total_driving_seconds / 60.0, 1) if STATE.recorder else 0.0,
            "max_buffer_minutes": round((STATE.recorder.max_driving_seconds / 60.0), 1) if STATE.recorder else 30.0,
            "file_size_mb": db_size_mb,
            "disk_free_gb": round(disk_free / (1024**3), 2),
            "temp_c": stats.get("temp_c", 0.0),
            "loop_hz": round(STATE.loop_hz, 1),
            "cpu_affinity": list(get_core_affinity()),
        }
        self._send_json(data)

    def _serve_live(self):
        with STATE.lock:
            pkt = dict(STATE.latest_pkt)
            pkt["_total_frames"] = STATE.recorder.total_frames_recorded if STATE.recorder else STATE.total_frames_received
        self._send_json(pkt)

    def _serve_frame(self, frame_id: int):
        if not STATE.recorder:
            self._send_json({"error": "No active recorder"}, 500)
            return

        reader = TelemetryLogReader(STATE.recorder.db_path)
        frame_data = reader.get_frame(frame_id)
        if frame_data:
            decompress_costmap_for_web(frame_data)
            self._send_json(frame_data)
        else:
            self._send_json({"error": f"Frame {frame_id} not found"}, 404)

    def _serve_anomalies(self):
        if not STATE.recorder:
            self._send_json([])
            return
        reader = TelemetryLogReader(STATE.recorder.db_path)
        anoms = reader.get_anomalies()
        self._send_json(anoms)

    def _serve_download(self):
        if not STATE.recorder or not os.path.exists(STATE.recorder.db_path):
            self.send_error(404, "Recording not found")
            return

        filename = os.path.basename(STATE.recorder.db_path)
        try:
            with open(STATE.recorder.db_path, "rb") as f:
                content = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
        except Exception as e:
            self.send_error(500, f"Error sending file: {e}")


def udp_listener_worker():
    """
    Dedicated UDP telemetry ingestion thread running on Core 2.
    Receives packets from Core 1 Planner and records to SQLite.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind((HOST_BIND, PORT_UDP_RECV))
        print(f"[DebugServer] UDP Telemetry Listener active on {HOST_BIND}:{PORT_UDP_RECV}")
    except Exception as e:
        print(f"[DebugServer FATAL] Failed to bind UDP {HOST_BIND}:{PORT_UDP_RECV}: {e}")
        return

    sock.settimeout(1.0)

    while True:
        try:
            data, _ = sock.recvfrom(65535)
            pkt = json.loads(data.decode("utf-8"))

            now = time.time()
            STATE.total_frames_received += 1
            STATE._hz_counter += 1

            if now - STATE._last_hz_calc >= 1.0:
                STATE.loop_hz = STATE._hz_counter / (now - STATE._last_hz_calc)
                STATE._hz_counter = 0
                STATE._last_hz_calc = now

            # Decompress costmap on Core 2 for immediate Web consumption
            decompress_costmap_for_web(pkt)

            with STATE.lock:
                STATE.latest_pkt = pkt
                STATE.last_pkt_time = now

            # Non-blocking write to Flight Recorder
            if STATE.recorder:
                STATE.recorder.record_frame(pkt)

        except socket.timeout:
            continue
        except Exception as e:
            # Drop bad packets without crashing
            continue


def run_debug_server():
    """Main launcher for Debug Server & Flight Recorder on Core 2."""
    print("=" * 70)
    print("  FOLKRACE PI 4: REAL-TIME DEBUG SERVER & FLIGHT RECORDER")
    print("=" * 70)

    # 1. HARD HARDWARE ISOLATION: Pin strictly to Core 2 (or 2+3 for I/O)
    set_core_affinity({2, 3}, "DebugServer")

    # 2. Initialize Flight Recorder
    recordings_dir = os.path.join(CURR_DIR, "data", "recordings")
    STATE.recorder = TelemetryFlightRecorder(output_dir=recordings_dir)
    STATE.recorder.start()

    # 3. Start UDP Listener Thread on Core 2
    udp_thread = threading.Thread(target=udp_listener_worker, name="UDPListener", daemon=True)
    udp_thread.start()

    # 4. Start HTTP Web Server on Core 2
    server_address = (HOST_BIND, PORT_HTTP)
    httpd = HTTPServer(server_address, DebugHTTPRequestHandler)
    print(f"[DebugServer] Web Mission Control GUI: http://localhost:{PORT_HTTP} or http://robot:{PORT_HTTP}")
    print(f"[DebugServer] Real-time Flight Black Box saving to: {STATE.recorder.db_path}")
    print("=" * 70)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[DebugServer] Shutting down...")
    finally:
        httpd.server_close()
        if STATE.recorder:
            STATE.recorder.stop()


if __name__ == "__main__":
    run_debug_server()

