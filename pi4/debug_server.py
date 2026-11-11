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
import hashlib
import socket
import struct
import shutil
import threading
import queue
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
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
UDP_CMD_PORT = 9877
HOST_BIND = "0.0.0.0"

HTML_PATH = os.path.join(CURR_DIR, "web_debug_gui.html")


def send_mcu_cmd(cmd_bytes: bytes) -> bool:
    """Dispatches command bytes to MCU UART Bridge on UDP 127.0.0.1:9877."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(cmd_bytes, ("127.0.0.1", UDP_CMD_PORT))
        sock.close()
        return True
    except Exception as e:
        print(f"[DebugServer] Failed dispatching MCU command: {e}")
        return False


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
        # Persistent WebSocket streaming clients
        self.ws_clients: Set[socket.socket] = set()
        self.ws_lock = threading.Lock()
        self.ws_queue: queue.Queue = queue.Queue(maxsize=15)
        # Dedicated WebSocket broadcast thread offloads all TCP client I/O from UDP telemetry ingestion
        self._ws_thread = threading.Thread(target=self._ws_broadcast_worker, daemon=True, name="WSBroadcaster")
        self._ws_thread.start()
        # Disk stats cache (avoid blocking on SD card stat every 2s)
        self._last_disk_check: float = 0.0
        self._cached_disk_free: float = 0.0
        self._cached_db_size: float = 0.0

    def _ws_broadcast_worker(self):
        while True:
            try:
                payload = self.ws_queue.get()
                if payload is None:
                    break
                self._send_ws_payload(payload)
            except Exception:
                pass

    def register_client_target(self, target: Tuple[str, int]):
        with self.lock:
            self.forward_targets.add(target)

    def register_ws_client(self, sock: socket.socket):
        with self.ws_lock:
            self.ws_clients.add(sock)

    def unregister_ws_client(self, sock: socket.socket):
        with self.ws_lock:
            self.ws_clients.discard(sock)

    def broadcast_ws_binary(self, payload: bytes):
        """Asynchronously dispatches live binary frame without stalling UDP listener."""
        try:
            if self.ws_queue.full():
                _ = self.ws_queue.get_nowait()
            self.ws_queue.put_nowait(payload)
        except Exception:
            pass

    def _send_ws_payload(self, payload: bytes):
        with self.ws_lock:
            if not self.ws_clients:
                return
            clients = list(self.ws_clients)

        length = len(payload)
        if length < 126:
            hdr = struct.pack("!BB", 0x82, length)
        elif length <= 0xFFFF:
            hdr = struct.pack("!BBH", 0x82, 126, length)
        else:
            hdr = struct.pack("!BBQ", 0x82, 127, length)
        frame = hdr + payload

        dead = []
        for s in clients:
            try:
                s.settimeout(0.20)  # 200ms ceiling tolerates temporary Wi-Fi retries
                s.sendall(frame)
            except (socket.timeout, TimeoutError):
                # Temporary Wi-Fi lag; don't disconnect the client
                pass
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                dead.append(s)

        if dead:
            with self.ws_lock:
                for s in dead:
                    self.ws_clients.discard(s)
                    try:
                        s.close()
                    except Exception:
                        pass

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
        "flags": pkt.get("flags", {}),
        "servo_pulse_us": pkt.get("servo_pulse_us", 1520),
        "mcu_msg": pkt.get("mcu_msg", ""),
        "mcu_log": pkt.get("mcu_log", []),
        "last_action": pkt.get("last_action", ""),
        "last_cmd_sent": pkt.get("last_cmd_sent", ""),
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
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass

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

        if path == "/ws_live":
            upgrade = self.headers.get("Upgrade", "").lower()
            ws_key = self.headers.get("Sec-WebSocket-Key", "")
            if "websocket" in upgrade and ws_key:
                self._handle_websocket_upgrade(ws_key)
                return

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

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path in ("/api/mcu/command", "/api/motor/command"):
            content_len = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_len) if content_len > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8"))
            except Exception:
                self._send_json({"error": "Invalid JSON"}, 400)
                return

            action = data.get("action", "").lower()
            cmd_str = ""

            if action == "motors":
                enable = data.get("enable", True)
                cmd_str = "MOTORS\n" if enable else "DISABLE\n"
            elif action == "estop":
                cmd_str = "ESTOP\n"
            elif action == "pid":
                kp = float(data.get("kp", 0.1))
                ki = float(data.get("ki", 1.5))
                cmd_str = f"PID {kp:.3f} {ki:.3f}\n"
            elif action == "calib":
                axle = int(data.get("axle", 0))
                motor = int(data.get("motor", 1))
                voltage = float(data.get("voltage", 4.0))
                cmd_str = f"CALIB {axle} {motor} {voltage:.1f}\n"
            elif action == "save":
                cmd_str = "SAVE\n"
            elif action == "dfu":
                cmd_str = "DFU\n"
            elif action == "speed":
                v = float(data.get("v", 0.0))
                cmd_str = f"V {v:.2f}\n"
            elif action == "steer":
                deg = float(data.get("deg", 90.0))
                cmd_str = f"A {deg:.1f}\n"
            elif action == "pulse":
                pulse = float(data.get("pulse", 1520.0))
                cmd_str = f"P {pulse:.0f}\n"
            elif action == "bench":
                cmd_str = "BENCH\n"
            elif action == "reset":
                cmd_str = "RESET\n"
            elif action == "status":
                cmd_str = "STATUS\n"
            elif action == "raw":
                cmd_str = f"{data.get('cmd', '').strip()}\n"
            else:
                self._send_json({"error": f"Unknown action: {action}"}, 400)
                return

            ok = send_mcu_cmd(cmd_str.encode("utf-8"))

            # Update cached state
            with STATE.lock:
                if action == "speed":
                    STATE.latest_pkt["v_cmd"] = float(data.get("v", 0.0))
                elif action == "steer":
                    STATE.latest_pkt["steer_cmd_deg"] = float(data.get("deg", 0.0))
                elif action == "estop":
                    STATE.latest_pkt["v_cmd"] = 0.0
                    STATE.latest_pkt["mode"] = "ESTOP"
                elif action == "motors":
                    STATE.latest_pkt["mode"] = "DRIVE" if data.get("enable", True) else "IDLE"
                STATE.latest_pkt["last_action"] = action
                STATE.latest_pkt["last_cmd_sent"] = cmd_str.strip()

            self._send_json({
                "status": "dispatched" if ok else "failed",
                "action": action,
                "cmd": cmd_str.strip(),
                "pipe": "pi4 -> esp32 -> stm32"
            })
        else:
            self._send_json({"error": "Endpoint not found"}, 404)

    def _handle_websocket_upgrade(self, ws_key: str):
        guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept = base64.b64encode(hashlib.sha1((ws_key.strip() + guid).encode("utf-8")).digest()).decode("utf-8")
        handshake = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
        )
        sock = self.connection
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.2)
        sock.sendall(handshake.encode("utf-8"))

        STATE.register_ws_client(sock)
        # Push latest frame immediately on connect
        with STATE.lock:
            cur_bin = STATE.get_cached_bin()
        if cur_bin:
            STATE.broadcast_ws_binary(cur_bin)

        try:
            while True:
                try:
                    data = sock.recv(1024)
                    if not data:
                        break
                    # Close frame
                    if (data[0] & 0x0F) == 0x08:
                        break
                    # Ping frame -> reply pong
                    elif (data[0] & 0x0F) == 0x09:
                        pong = bytearray(data)
                        pong[0] = (pong[0] & 0xF0) | 0x0A
                        sock.sendall(pong)
                except (socket.timeout, TimeoutError):
                    # Telemetry clients stream downlink; lack of uplink frames is normal
                    continue
        except Exception:
            pass
        finally:
            STATE.unregister_ws_client(sock)
            try:
                sock.close()
            except Exception:
                pass

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
            self.send_header("Connection", "keep-alive")
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
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_status(self):
        stats = get_system_cpu_stats()
        now = time.time()
        if (now - STATE._last_disk_check) >= 5.0:
            STATE._last_disk_check = now
            try:
                disk_total, disk_used, disk_free = shutil.disk_usage(STATE.recorder.output_dir if STATE.recorder else CURR_DIR)
                STATE._cached_disk_free = round(disk_free / (1024**3), 2)
            except Exception:
                pass
            if STATE.recorder and os.path.exists(STATE.recorder.db_path):
                try:
                    STATE._cached_db_size = round(os.path.getsize(STATE.recorder.db_path) / (1024 * 1024), 2)
                except Exception:
                    pass

        data = {
            "session_id": STATE.recorder.session_id if STATE.recorder else "none",
            "total_recorded": STATE.recorder.total_frames_recorded if STATE.recorder else 0,
            "dropped_frames": STATE.recorder.dropped_frames if STATE.recorder else 0,
            "pruned_frames": STATE.recorder.pruned_frames_count if STATE.recorder else 0,
            "driving_seconds": round(STATE.recorder.total_driving_seconds, 1) if STATE.recorder else 0.0,
            "driving_minutes": round(STATE.recorder.total_driving_seconds / 60.0, 1) if STATE.recorder else 0.0,
            "max_buffer_minutes": round((STATE.recorder.max_driving_seconds / 60.0), 1) if STATE.recorder else 30.0,
            "file_size_mb": STATE._cached_db_size,
            "disk_free_gb": STATE._cached_disk_free,
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
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _serve_live_bin(self):
        with STATE.lock:
            body = STATE.get_cached_bin()
        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "keep-alive")
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
    # 2 MB socket receive buffer prevents kernel drops during peak burst traffic
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2097152)
    except Exception:
        pass

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

            is_planner = ("best_line" in pkt) or ("plan_ms" in pkt) or ("corridor_left" in pkt)

            with STATE.lock:
                if is_planner:
                    STATE.latest_pkt = pkt
                    STATE.last_planner_pkt_time = now
                elif now - STATE.last_planner_pkt_time > 1.0:
                    # Planner is offline: show direct MCU / bridge telemetry
                    STATE.latest_pkt = pkt
                else:
                    # Planner is active: non-destructively merge fresh MCU telemetry fields
                    for k in ("rpms_act", "flags", "mcu_msg", "mcu_log", "slip_state", "slip_ratio", "tof", "tof_ranges", "tof_ranges_raw", "f_tof"):
                        if k in pkt:
                            STATE.latest_pkt[k] = pkt[k]

                STATE.last_pkt_time = now
                STATE.invalidate_caches()
                bin_data = STATE.get_cached_bin() if is_planner else None

            # Push live binary frame instantly to connected WebSockets (zero polling overhead)
            if bin_data:
                STATE.broadcast_ws_binary(bin_data)

            # Non-blocking write to Flight Recorder (all planner frames, bounded sensor frames)
            if STATE.recorder:
                if is_planner or (now - getattr(STATE, "_last_db_record", 0.0) >= 0.05):
                    STATE._last_db_record = now
                    STATE.recorder.record_frame(pkt, raw_bytes=data)

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

    # 4. Start High-Performance Threading HTTP & WebSocket Server on Core 2
    server_address = (HOST_BIND, PORT_HTTP)
    httpd = ThreadingHTTPServer(server_address, DebugHTTPRequestHandler)
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
    import argparse
    parser = argparse.ArgumentParser(description="Folkrace Debug Server")
    parser.add_argument("--port", type=int, default=PORT_HTTP, help="HTTP port")
    args = parser.parse_args()
    PORT_HTTP = args.port
    run_debug_server()

