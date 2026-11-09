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
import struct
import shutil
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from typing import Dict, List, Optional, Any, Set, Tuple

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
        self.last_planner_pkt_time = 0.0
        self.loop_hz = 0.0
        self._hz_counter = 0
        self._last_hz_calc = time.time()
        self.recorder: Optional[TelemetryFlightRecorder] = None
        # Forward targets registered dynamically by connecting client IPs
        self.forward_targets: Set[Tuple[str, int]] = set()
        # Fast cached binary & JSON representations (zero redundant work across requests)
        self._cached_bin: Optional[bytes] = None
        self._cached_json: Optional[bytes] = None

    def register_client_target(self, target: Tuple[str, int]):
        with self.lock:
            self.forward_targets.add(target)

    def invalidate_caches(self):
        self._cached_bin = None
        self._cached_json = None

    def get_cached_bin(self) -> bytes:
        if self._cached_bin is None:
            self._cached_bin = build_live_binary_packet(self.latest_pkt)
        return self._cached_bin

    def get_cached_json(self) -> bytes:
        if self._cached_json is None:
            pkt = dict(self.latest_pkt)
            pkt["_total_frames"] = self.recorder.total_frames_recorded if self.recorder else self.total_frames_received
            self._cached_json = json.dumps(pkt).encode("utf-8")
        return self._cached_json


STATE = DebugServerState()


def build_live_binary_packet(pkt: Dict[str, Any]) -> bytes:
    """
    Constructs a high-speed compact binary telemetry packet for laptop workstations.
    Packs 232-byte header, compressed costmap binary bytes (~1.8 KB), and meta JSON.
    Total transfer size: ~8-12 KB. Packing time on Pi 4: ~0.02 ms.
    """
    prob_comp_bytes = b""
    haz_comp_bytes = b""
    grid_shape = pkt.get("grid_shape", [0, 0])
    rows = int(grid_shape[0]) if len(grid_shape) >= 1 else 0
    cols = int(grid_shape[1]) if len(grid_shape) >= 2 else 0
    raw_costmap_len = rows * cols

    if "grid_data" in pkt:
        try:
            prob_comp_bytes = base64.b64decode(pkt["grid_data"])
        except Exception:
            prob_comp_bytes = b""
    if "hazard_data" in pkt:
        try:
            haz_comp_bytes = base64.b64decode(pkt["hazard_data"])
        except Exception:
            haz_comp_bytes = b""

    meta = {
        "status": pkt.get("status", "OK"),
        "mode": pkt.get("mode", "DRIVE"),
        "slip_state": pkt.get("slip_state", "NORMAL"),
        "recovery_state": pkt.get("escape_state", pkt.get("recovery_state", "DRIVE")),
        "best_line": pkt.get("best_line", []),
        "candidate_lines": pkt.get("candidate_lines", []),
        "guide_line": pkt.get("guide_line", []),
        "corridor_left": pkt.get("corridor_left", []),
        "corridor_right": pkt.get("corridor_right", []),
        "shield": pkt.get("shield", {}),
        "targets": pkt.get("targets", []),
        "trail": pkt.get("trail", []),
        "sightlines": pkt.get("sightlines", []),
        "is_wrong_way": pkt.get("is_wrong_way", False),
        "target_yaw": pkt.get("target_yaw", 0.0),
        "total_frames": pkt.get("_total_frames", 0),
    }
    meta_bytes = json.dumps(meta).encode("utf-8")

    rpms_cmd = pkt.get("rpms_cmd", [0.0, 0.0, 0.0, 0.0])
    rpms_act = pkt.get("rpms_act", [0.0, 0.0, 0.0, 0.0])
    tof_ranges = pkt.get("tof_ranges", [1.8] * 16)
    if len(tof_ranges) < 16:
        tof_ranges = list(tof_ranges) + [1.8] * (16 - len(tof_ranges))
    gap = pkt.get("gap") or [0.0, 0.0, 0.0]
    bounds = pkt.get("grid_bounds") or [-1.25, 1.25, 0.0, 1.90]

    header_fmt = "<4s I I d 16f 4f 4f 16f 3f 2H 4f 5I"
    header_size = struct.calcsize(header_fmt)
    total_len = header_size + len(prob_comp_bytes) + len(haz_comp_bytes) + len(meta_bytes)

    header = struct.pack(
        header_fmt,
        b"FOLK",
        total_len,
        int(pkt.get("time_step", 0)),
        float(pkt.get("timestamp", time.time())),
        float(pkt.get("sim_time", 0.0)),
        float(pkt.get("v_cmd", 0.0)),
        float(pkt.get("v_real", pkt.get("v_est", 0.0))),
        float(pkt.get("steer_cmd_deg", pkt.get("steer_deg", 0.0))),
        float(pkt.get("steer_actual_deg", pkt.get("steer_deg", 0.0))),
        float(pkt.get("yaw_deg", 0.0)),
        float(pkt.get("pitch_deg", 0.0)),
        float(pkt.get("roll_deg", 0.0)),
        float(pkt.get("gyro_z", 0.0)),
        float(pkt.get("accel_fwd", 0.0)),
        float(pkt.get("accel_lat", 0.0)),
        float(pkt.get("slip_ratio", 0.0)),
        float(pkt.get("f_tof", 1.8)),
        float(pkt.get("plan_ms", 0.0)),
        float(pkt.get("map_ms", 0.0)),
        float(pkt.get("scout_ms", 0.0)),
        float(rpms_cmd[0] if len(rpms_cmd) > 0 else 0.0),
        float(rpms_cmd[1] if len(rpms_cmd) > 1 else 0.0),
        float(rpms_cmd[2] if len(rpms_cmd) > 2 else 0.0),
        float(rpms_cmd[3] if len(rpms_cmd) > 3 else 0.0),
        float(rpms_act[0] if len(rpms_act) > 0 else 0.0),
        float(rpms_act[1] if len(rpms_act) > 1 else 0.0),
        float(rpms_act[2] if len(rpms_act) > 2 else 0.0),
        float(rpms_act[3] if len(rpms_act) > 3 else 0.0),
        *[float(tof_ranges[i]) for i in range(16)],
        float(gap[0] if len(gap) > 0 else 0.0),
        float(gap[1] if len(gap) > 1 else 0.0),
        float(gap[2] if len(gap) > 2 else 0.0),
        int(rows),
        int(cols),
        float(bounds[0]), float(bounds[1]), float(bounds[2]), float(bounds[3]),
        len(prob_comp_bytes),
        raw_costmap_len,
        len(haz_comp_bytes),
        raw_costmap_len,
        len(meta_bytes)
    )

    return header + prob_comp_bytes + haz_comp_bytes + meta_bytes


class DebugHTTPRequestHandler(BaseHTTPRequestHandler):
    """Handles HTTP requests for Mission Control UI and REST endpoints."""

    def log_message(self, format, *args):
        # Silence routine request logging to prevent terminal spam
        pass

    def do_GET(self):
        # Auto-register external client IP for direct high-speed UDP streaming
        client_ip = self.client_address[0]
        if client_ip not in ("127.0.0.1", "localhost", "::1"):
            STATE.register_client_target((client_ip, PORT_UDP_RECV))

        parsed = urlparse(self.path)
        path = parsed.path

        if path in ("/", "/index.html"):
            self._serve_file(HTML_PATH, "text/html")

        elif path == "/api/status":
            self._serve_status()

        elif path == "/api/live":
            self._serve_live()

        elif path == "/api/live_bin":
            self._serve_live_bin()

        elif path.startswith("/api/register_client"):
            qs = parse_qs(parsed.query)
            target_ip = qs.get("ip", [client_ip])[0]
            port = PORT_UDP_RECV
            if "port" in qs:
                try:
                    port = int(qs["port"][0])
                except ValueError:
                    pass
            STATE.register_client_target((target_ip, port))
            self._send_json({"status": "registered", "target": [target_ip, port]})

        elif path.startswith("/api/frame_bin/"):
            try:
                frame_id = int(path.split("/")[-1])
                self._serve_frame_bin(frame_id)
            except ValueError:
                self.send_error(400, "Invalid frame ID")

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
            body = STATE.get_cached_json()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_live_bin(self):
        with STATE.lock:
            body = STATE.get_cached_bin()
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_frame(self, frame_id: int):
        if not STATE.recorder:
            self._send_json({"error": "No active recorder"}, 500)
            return

        reader = TelemetryLogReader(STATE.recorder.db_path)
        frame_data = reader.get_frame(frame_id)
        if frame_data:
            self._send_json(frame_data)
        else:
            self._send_json({"error": f"Frame {frame_id} not found"}, 404)

    def _serve_frame_bin(self, frame_id: int):
        if not STATE.recorder:
            self.send_error(500, "No active recorder")
            return

        reader = TelemetryLogReader(STATE.recorder.db_path)
        frame_data = reader.get_frame(frame_id)
        if frame_data:
            body = build_live_binary_packet(frame_data)
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404, f"Frame {frame_id} not found")

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
    Dedicated UDP telemetry ingestion and high-speed laptop forwarder on Core 2.
    Receives packets from Core 1 Planner, records to SQLite WAL flight recorder,
    and forwards full-fidelity 1cm telemetry directly to laptop workstation.
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

    # UDP Forwarder socket to laptop (broadcast + direct unicast)
    fwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        fwd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception:
        pass
    fwd_sock.setblocking(False)

    while True:
        try:
            data, _ = sock.recvfrom(65535)

            # Direct binary UDP forward to laptop (zero re-encoding or serialization overhead)
            with STATE.lock:
                targets = list(STATE.forward_targets)
            for target in targets:
                try:
                    fwd_sock.sendto(data, target)
                except Exception:
                    pass

            pkt = json.loads(data.decode("utf-8"))

            now = time.time()
            STATE.total_frames_received += 1
            STATE._hz_counter += 1

            if now - STATE._last_hz_calc >= 1.0:
                STATE.loop_hz = STATE._hz_counter / (now - STATE._last_hz_calc)
                STATE._hz_counter = 0
                STATE._last_hz_calc = now

            is_planner = ("best_line" in pkt) or ("grid_data" in pkt) or ("v_cmd" in pkt)

            with STATE.lock:
                if is_planner or (now - STATE.last_planner_pkt_time > 1.0):
                    STATE.latest_pkt = pkt
                    if is_planner:
                        STATE.last_planner_pkt_time = now
                STATE.last_pkt_time = now
                STATE.invalidate_caches()

            # Non-blocking write to Flight Recorder (all planner frames, bounded sensor frames)
            if STATE.recorder:
                if is_planner or (now - getattr(STATE, "_last_db_record", 0.0) >= 0.05):
                    STATE._last_db_record = now
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

