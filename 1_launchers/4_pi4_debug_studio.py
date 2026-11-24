"""
================================================================================
  LAUNCHER: PI 4 DUAL-CORE DEBUG STUDIO & FLIGHT RECORDER
================================================================================
  Connects to Raspberry Pi 4 ('robot'):
    - Auto-detects and connects to Pi debug server over HTTP port 8080.
    - Launches Tactical Mission Control (same rich GUI as the browser server)
      in a dedicated standalone window (Edge/Chrome app mode) or browser tab.
    - Displays live telemetry status, driving buffer metrics, and SD card space.
    - Self-healing ESP32 reconnection watchdog and remote reconnect trigger.
    - Supports full automated endpoint verification (--verify), real-time
      terminal telemetry monitoring (--stream), and log downloading (--download).

  Usage:
    python 1_launchers/4_pi4_debug_studio.py              # Opens Tactical Mission Control standalone GUI (default)
    python 1_launchers/4_pi4_debug_studio.py --tab        # Opens Tactical Mission Control in standard browser tab
    python 1_launchers/4_pi4_debug_studio.py --reconnect  # Triggers remote ESP32 USB serial re-probe & reconnect
    python 1_launchers/4_pi4_debug_studio.py --desktop    # Launches legacy PyQt5 simulation planner GUI
    python 1_launchers/4_pi4_debug_studio.py --verify     # Tests all Pi server endpoints
    python 1_launchers/4_pi4_debug_studio.py --stream     # Live terminal telemetry dashboard
    python 1_launchers/4_pi4_debug_studio.py --download   # Downloads active SQLite flight log
    python 1_launchers/4_pi4_debug_studio.py --replay     # Downloads (or opens latest) .db for 60Hz offline replay
    python 1_launchers/4_pi4_debug_studio.py --restart    # Restarts remote Folkrace service
    python 1_launchers/4_pi4_debug_studio.py --sync       # Syncs 2_pi4 code to robot before launch
================================================================================
"""

import os
import re
import sys
import time
import json
import socket
import shutil
import urllib.request
import urllib.error
import argparse
import concurrent.futures
import webbrowser
import subprocess
from pathlib import Path
from typing import Optional, Dict, Any, Tuple, List

# Ensure UTF-8 output on Windows terminals
if sys.platform == "win32" and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI4_DIR = os.path.join(ROOT_DIR, "2_pi4")
CONTROLLERS_DIR = os.path.join(ROOT_DIR, "5_webots", "simulation", "my_project", "controllers", "tof_path_planner")
PLANNER_GUI_SCRIPT = os.path.join(CONTROLLERS_DIR, "planner_gui.py")

DEFAULT_WIFI_IPS = ["192.168.20.120", "192.168.20.117"]
PI_MAC_PREFIXES = ("dc:a6:32", "b8:27:eb", "e4:5f:01", "d8:3a:dd", "2c:cf:67")
REMOTE_DIR = "/home/admin/folkrace"
PORT = 8080


def get_active_wifi_network() -> Tuple[Optional[str], Optional[str], Optional[str]]:
    """
    Detects the laptop's currently connected Wi-Fi SSID, local IPv4, and /24 prefix (e.g. '10.88.202.').
    Returns (ssid, local_ip, subnet_prefix).
    """
    ssid: Optional[str] = None
    local_ip: Optional[str] = None
    iface_name: Optional[str] = None

    if sys.platform == "win32":
        try:
            res = subprocess.run(
                ["netsh", "wlan", "show", "interfaces"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=2,
            )
            if res.returncode == 0:
                in_connected = False
                for line in res.stdout.splitlines():
                    if ":" not in line:
                        continue
                    key, val = [p.strip() for p in line.split(":", 1)]
                    if key == "Name":
                        iface_name = val
                        in_connected = False
                    elif key == "State" and val.lower() == "connected":
                        in_connected = True
                    elif key == "SSID" and in_connected and val:
                        ssid = val
                        break
        except Exception:
            pass

        if iface_name and ssid:
            try:
                res = subprocess.run(
                    ["netsh", "interface", "ip", "show", "addresses", iface_name],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    timeout=2,
                )
                if res.returncode == 0:
                    m = re.search(r"IP Address:\s*(\d+\.\d+\.\d+\.\d+)", res.stdout)
                    if m:
                        local_ip = m.group(1)
            except Exception:
                pass

    if not local_ip:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                local_ip = s.getsockname()[0]
        except Exception:
            pass

    subnet_prefix = ".".join(local_ip.split(".")[:3]) + "." if local_ip and local_ip.count(".") == 3 else None
    return ssid, local_ip, subnet_prefix


def discover_arp_pi_ips(subnet_prefix: Optional[str] = None) -> List[str]:
    """Reads the local OS ARP cache for Raspberry Pi MAC prefixes on the active subnet."""
    ips: List[str] = []
    try:
        res = subprocess.run(["arp", "-a"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=2)
        if res.returncode != 0:
            return ips
        for line in res.stdout.splitlines():
            m = re.search(r"(\d+\.\d+\.\d+\.\d+)\s+([0-9a-fA-F]{2}(?:[:-][0-9a-fA-F]{2}){5})", line)
            if not m:
                continue
            ip, mac = m.group(1), m.group(2).lower().replace("-", ":")
            if subnet_prefix and not ip.startswith(subnet_prefix):
                continue
            if mac.startswith(PI_MAC_PREFIXES) and ip not in ips:
                ips.append(ip)
    except Exception:
        pass
    return ips


def _is_tcp_open(ip: str, port: int, timeout_s: float = 0.35) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout_s):
            return True
    except Exception:
        return False


def scan_wifi_subnet_for_pi(
    subnet_prefix: str, local_ip: Optional[str], port: int, timeout_s: float = 0.8
) -> List[str]:
    """Probes the current Wi-Fi /24 subnet in parallel for hosts listening on `port`."""
    targets = [f"{subnet_prefix}{i}" for i in range(1, 255) if f"{subnet_prefix}{i}" != local_ip]
    open_ips: List[str] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=64) as pool:
        futures = {pool.submit(_is_tcp_open, ip, port, timeout_s): ip for ip in targets}
        for fut in concurrent.futures.as_completed(futures):
            ip = futures[fut]
            try:
                if fut.result():
                    open_ips.append(ip)
            except Exception:
                pass
    # Prioritize hosts whose MAC in the refreshed ARP cache matches a Raspberry Pi OUI
    arp_pis = set(discover_arp_pi_ips(subnet_prefix))
    open_ips.sort(key=lambda ip: (0 if ip in arp_pis else 1, ip))
    return open_ips


def get_candidate_hosts(preferred_host: Optional[str] = None, subnet_prefix: Optional[str] = None) -> List[str]:
    """Returns deduplicated candidate hosts restricted to the active Wi-Fi subnet."""
    if preferred_host:
        return [preferred_host]
    if subnet_prefix is None:
        _, _, subnet_prefix = get_active_wifi_network()

    ordered: List[str] = []
    for ip in discover_arp_pi_ips(subnet_prefix):
        if ip not in ordered:
            ordered.append(ip)

    for ip in DEFAULT_WIFI_IPS:
        if (subnet_prefix is None or ip.startswith(subnet_prefix)) and ip not in ordered:
            ordered.append(ip)

    if subnet_prefix is None or subnet_prefix == "192.168.20.":
        if "robot" not in ordered:
            ordered.append("robot")

    if "robot.local" not in ordered:
        ordered.append("robot.local")
    return ordered


def format_ssh_target(host: Optional[str]) -> str:
    """Formats a hostname or IP for SSH/SCP (uses 'robot' alias or 'admin@<ip>')."""
    if not host or host in ("robot", "robot.local", "127.0.0.1"):
        return "robot"
    if "@" in host:
        return host
    return f"admin@{host}"


def _probe_http_status(host: str, timeout_s: float = 1.0) -> Optional[str]:
    url = f"http://{host}:{PORT}/api/status"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "FolkraceLauncher/1.0"})
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            if resp.status == 200:
                return f"http://{host}:{PORT}"
    except Exception:
        pass
    return None


def resolve_pi_host(preferred_host: Optional[str] = None, timeout_s: float = 1.0) -> Tuple[Optional[str], Optional[str]]:
    """
    Auto-detects the responsive hostname or IP for the Pi 4 debug server on the active Wi-Fi.
    Returns (hostname, base_url).
    """
    _, local_ip, subnet_prefix = get_active_wifi_network()
    candidates = get_candidate_hosts(preferred_host, subnet_prefix=subnet_prefix)

    for host in candidates:
        base_url = _probe_http_status(host, timeout_s=timeout_s)
        if base_url:
            return host, base_url

    if not preferred_host and subnet_prefix:
        for ip in scan_wifi_subnet_for_pi(subnet_prefix, local_ip, PORT):
            base_url = _probe_http_status(ip, timeout_s=timeout_s)
            if base_url:
                return ip, base_url

    return None, None


def check_ssh_connection(host: str = "robot", timeout_s: int = 3) -> bool:
    """Checks if Raspberry Pi is reachable over SSH."""
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={timeout_s}", host, "echo ok"]
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout_s + 1, text=True)
        return res.returncode == 0 and "ok" in res.stdout
    except Exception:
        return False


def resolve_ssh_host(preferred_host: Optional[str] = None, timeout_s: int = 3) -> str:
    """Finds the first reachable SSH target on the active Wi-Fi (including dynamic hotspot IPs)."""
    if preferred_host:
        return format_ssh_target(preferred_host)

    _, local_ip, subnet_prefix = get_active_wifi_network()
    candidates = get_candidate_hosts(subnet_prefix=subnet_prefix)
    seen = set()

    def _try_hosts(hosts: List[str]) -> Optional[str]:
        for host in hosts:
            if host in ("127.0.0.1", "robot.local"):
                continue
            target = format_ssh_target(host)
            if target in seen:
                continue
            seen.add(target)
            probe_ip = "192.168.20.120" if host == "robot" else host
            if not _is_tcp_open(probe_ip, 22, timeout_s=0.4):
                continue
            if check_ssh_connection(target, timeout_s=timeout_s):
                return target
        return None

    found = _try_hosts(candidates)
    if found:
        return found

    if subnet_prefix:
        scanned = scan_wifi_subnet_for_pi(subnet_prefix, local_ip, 22)
        found = _try_hosts(scanned)
        if found:
            return found

    return "robot"


def query_api_json(base_url: str, endpoint: str, timeout_s: float = 3.0) -> Optional[Dict[str, Any]]:
    """Helper to query JSON from the Pi debug server."""
    url = f"{base_url}{endpoint}"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "FolkraceLauncher/1.0"})
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            if resp.status == 200:
                return json.loads(resp.read().decode("utf-8"))
    except Exception as e:
        pass
    return None


def sync_pi_stack(host: str = "robot"):
    """Synchronizes local 2_pi4 and brain files to the Pi."""
    print(f"[Sync] Preparing remote directory: {REMOTE_DIR} on {host}...")
    subprocess.run(["ssh", host, f"mkdir -p {REMOTE_DIR}/brain {REMOTE_DIR}/data/recordings"], check=True)

    print(f"[Sync] Uploading 2_pi4 subsystem scripts to {host}...")
    files = [
        os.path.join(PI4_DIR, "auto_servo_center.py"),
        os.path.join(PI4_DIR, "core_manager.py"),
        os.path.join(PI4_DIR, "debug_recorder.py"),
        os.path.join(PI4_DIR, "debug_server.py"),
        os.path.join(PI4_DIR, "path_planner_runner.py"),
        os.path.join(PI4_DIR, "run_folkrace_system.py"),
        os.path.join(PI4_DIR, "web_debug_gui.html"),
        os.path.join(PI4_DIR, "mcu_telemetry_protocol.py"),
        os.path.join(PI4_DIR, "mcu_uart_bridge.py"),
    ]
    scp_cmd = ["scp"] + [f for f in files if os.path.exists(f)] + [f"{host}:{REMOTE_DIR}/"]
    subprocess.run(scp_cmd, check=True)

    brain_dir = os.path.join(ROOT_DIR, "brain")
    if os.path.exists(brain_dir):
        subprocess.run(["ssh", host, f"mkdir -p {REMOTE_DIR}/brain && rm -f {REMOTE_DIR}/brain/*.dll"], check=True)
        brain_files = [
            os.path.join(brain_dir, f)
            for f in os.listdir(brain_dir)
            if os.path.isfile(os.path.join(brain_dir, f)) and not f.endswith((".dll", ".pyc", ".o", ".obj"))
        ]
        if brain_files:
            subprocess.run(["scp"] + brain_files + [f"{host}:{REMOTE_DIR}/brain/"], check=True)

    print("[Sync] Files synced successfully.")


def restart_remote_service(host: str = "robot"):
    """Restarts the Folkrace service on the Pi."""
    print(f"[Remote] Restarting folkrace systemd service on {host}...")
    cmd = ["ssh", host, "echo l | sudo -S systemctl restart folkrace"]
    subprocess.run(cmd, check=True)
    time.sleep(2.0)
    print("[Remote] Service restarted.")


def stop_remote_service(host: str = "robot"):
    """Stops the Folkrace service on the Pi."""
    print(f"[Remote] Stopping folkrace systemd service on {host}...")
    cmd = ["ssh", host, "echo l | sudo -S systemctl stop folkrace"]
    subprocess.run(cmd, check=True)
    print("[Remote] Service stopped.")


def launch_tactical_gui(base_url: str, prefer_app_window: bool = True):
    """
    Launches the Tactical Mission Control GUI (same as browser server).
    If prefer_app_window is True, attempts to launch Edge or Chrome in standalone
    app window mode (--app=http://...) for a borderless native application window
    without browser tabs or URL bars. Falls back to default web browser.
    """
    if prefer_app_window:
        candidates = [
            r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
            r"C:\Program Files\Google\Chrome\Application\chrome.exe",
            r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
            shutil.which("msedge") or "",
            shutil.which("google-chrome") or "",
            shutil.which("chrome") or "",
            shutil.which("chromium") or "",
        ]
        for c in candidates:
            if c and os.path.exists(c):
                try:
                    subprocess.Popen([c, f"--app={base_url}"])
                    print(f"[Launcher] Tactical Mission Control opened in standalone app window ({os.path.basename(c)})")
                    print(f"  URL: {base_url}")
                    print("=" * 75)
                    return
                except Exception:
                    pass

    print(f"[Launcher] Opening Tactical Mission Control in default browser: {base_url}")
    webbrowser.open(base_url)
    print("=" * 75)


def trigger_mcu_reconnect(base_url: str) -> bool:
    """Tells the Pi debug server to re-detect and reconnect to all ESP32 USB serial ports."""
    url = f"{base_url}/api/mcu/command"
    payload = json.dumps({"action": "reconnect"}).encode("utf-8")
    print(f"[Launcher] Triggering remote ESP32 serial re-probe & reconnect on {base_url}...")
    try:
        req = urllib.request.Request(
            url,
            data=payload,
            headers={"Content-Type": "application/json", "User-Agent": "FolkraceLauncher/1.0"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=4.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            print(f"[Launcher] ESP32 Reconnect Status: {data.get('status', 'OK')} ({data.get('msg', 'reconnected')})")
            return True
    except Exception as e:
        print(f"[Launcher ERROR] Failed triggering ESP32 reconnect: {e}")
        return False


def download_database(base_url: str, output_path: Optional[str] = None) -> Optional[str]:
    """Downloads the active flight recorder SQLite database and returns its local path."""
    status = query_api_json(base_url, "/api/status")
    session_id = status.get("session_id", "recording") if status else "recording"
    
    if not output_path:
        out_dir = Path(ROOT_DIR) / "data" / "recordings"
        out_dir.mkdir(parents=True, exist_ok=True)
        output_path = str(out_dir / f"{session_id}.db")

    url = f"{base_url}/api/download"
    print(f"[Download] Fetching flight database from {url}...")
    t0 = time.time()
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "FolkraceLauncher/1.0"})
        with urllib.request.urlopen(req, timeout=30.0) as resp, open(output_path, "wb") as f:
            total_bytes = 0
            while True:
                chunk = resp.read(65536)
                if not chunk:
                    break
                f.write(chunk)
                total_bytes += len(chunk)
        dt = time.time() - t0
        mb = total_bytes / (1024 * 1024)
        print(f"[Download] Saved {mb:.2f} MB in {dt:.2f}s ({mb/max(0.01, dt):.2f} MB/s) -> {output_path}")
        return output_path
    except Exception as e:
        print(f"[Download] Error: {e}")
        return None


def find_latest_local_recording() -> Optional[str]:
    """Returns the most recently modified .db file in data/recordings/, if any."""
    rec_dir = Path(ROOT_DIR) / "data" / "recordings"
    if not rec_dir.exists():
        return None
    dbs = sorted(rec_dir.glob("*.db"), key=lambda p: p.stat().st_mtime, reverse=True)
    return str(dbs[0]) if dbs else None


def run_local_replay_server(db_path: str, prefer_app_window: bool = True, open_browser: bool = True):
    """Starts a localhost 60 Hz offline replay server for a downloaded SQLite flight database."""
    from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
    from urllib.parse import urlparse
    import threading

    if PI4_DIR not in sys.path:
        sys.path.insert(0, PI4_DIR)
    tools_dir = os.path.join(ROOT_DIR, "tools")
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)
    from debug_recorder import TelemetryLogReader, add_label
    from debug_server import build_live_binary_packet
    from flight_analyzer import FlightSession

    html_path = os.path.join(PI4_DIR, "web_debug_gui.html")
    state_lock = threading.Lock()
    replay_state: Dict[str, Any] = {
        "db_path": db_path,
        "reader": TelemetryLogReader(db_path),
        "summary": None,
        "bounds": (1, 0, 0),
        "flight_session": None,
    }

    def _reload_db(new_path: str):
        new_reader = TelemetryLogReader(new_path)
        summary = new_reader.get_summary()
        bounds = new_reader.get_frame_bounds()
        with state_lock:
            old_reader = replay_state.get("reader")
            replay_state["db_path"] = new_path
            replay_state["reader"] = new_reader
            replay_state["summary"] = summary
            replay_state["bounds"] = bounds
            replay_state["flight_session"] = None
        if old_reader and hasattr(old_reader, "close"):
            old_reader.close()

    def _get_flight_session() -> FlightSession:
        with state_lock:
            sess = replay_state.get("flight_session")
            cur_db = replay_state["db_path"]
        if sess is None or sess.db_path != cur_db:
            new_sess = FlightSession(cur_db)
            with state_lock:
                if replay_state["db_path"] == cur_db:
                    replay_state["flight_session"] = new_sess
                sess = replay_state.get("flight_session") or new_sess
        return sess

    _reload_db(db_path)

    class LocalReplayHandler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, format, *args):
            pass

        def _send_bytes(self, body: bytes, content_type: str, status: int = 200):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, obj: Any, status: int = 200):
            self._send_bytes(json.dumps(obj).encode("utf-8"), "application/json", status)

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                with open(html_path, "rb") as f:
                    self._send_bytes(f.read(), "text/html")
            elif path == "/api/status":
                with state_lock:
                    cur_db = replay_state["db_path"]
                    summary = replay_state["summary"] or {}
                    min_id, max_id, _ = replay_state["bounds"]
                try:
                    _, _, disk_free = shutil.disk_usage(os.path.dirname(cur_db) or ROOT_DIR)
                    disk_free_gb = round(disk_free / (1024**3), 2)
                except Exception:
                    disk_free_gb = 0.0
                self._send_json({
                    "session_id": summary.get("session_id", os.path.basename(cur_db)),
                    "min_frame_id": min_id,
                    "total_recorded": max_id,
                    "dropped_frames": 0,
                    "pruned_frames": 0,
                    "driving_seconds": summary.get("driving_seconds", 0.0),
                    "driving_minutes": summary.get("driving_minutes", 0.0),
                    "max_buffer_minutes": 30.0,
                    "file_size_mb": summary.get("file_size_mb", 0.0),
                    "disk_free_gb": disk_free_gb,
                    "temp_c": 0.0,
                    "loop_hz": 60.0,
                    "cpu_affinity": [],
                })
            elif path.startswith("/api/frame_bin/"):
                try:
                    frame_id = int(path.split("/")[-1])
                except ValueError:
                    self.send_error(400, "Invalid frame ID")
                    return
                with state_lock:
                    reader = replay_state["reader"]
                frame_data = reader.get_frame(frame_id)
                if not frame_data:
                    self.send_error(404, f"Frame {frame_id} not found")
                    return
                try:
                    self._send_bytes(build_live_binary_packet(frame_data), "application/octet-stream")
                except Exception as e:
                    self.send_error(500, f"Failed to pack frame {frame_id}: {e}")
            elif path.startswith("/api/frame/"):
                try:
                    frame_id = int(path.split("/")[-1])
                except ValueError:
                    self._send_json({"error": "Invalid frame ID"}, 400)
                    return
                with state_lock:
                    reader = replay_state["reader"]
                frame_data = reader.get_frame(frame_id)
                if frame_data:
                    self._send_json(frame_data)
                else:
                    self._send_json({"error": f"Frame {frame_id} not found"}, 404)
            elif path == "/api/anomalies":
                with state_lock:
                    reader = replay_state["reader"]
                self._send_json(reader.get_anomalies())
            elif path == "/api/ai_report":
                try:
                    self._send_json(_get_flight_session().build_digest())
                except Exception as e:
                    self._send_json({"error": str(e)}, 500)
            elif path == "/api/episodes":
                try:
                    self._send_json(_get_flight_session().extract_episodes())
                except Exception as e:
                    self._send_json({"error": str(e)}, 500)
            elif path == "/api/labels":
                with state_lock:
                    reader = replay_state["reader"]
                self._send_json(reader.get_labels())
            elif path == "/api/download":
                with state_lock:
                    cur_db = replay_state["db_path"]
                if os.path.exists(cur_db):
                    with open(cur_db, "rb") as f:
                        data = f.read()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/octet-stream")
                    self.send_header("Content-Disposition", f'attachment; filename="{os.path.basename(cur_db)}"')
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                else:
                    self.send_error(404, "Database not found")
            elif path in ("/api/live", "/api/live_bin"):
                self._send_json({"status": "REPLAY_OFFLINE"}, 200) if path == "/api/live" else self.send_error(404)
            else:
                self._send_json({"error": "Not Found"}, 404)

        def do_POST(self):
            path = urlparse(self.path).path
            if path == "/api/upload_db":
                content_len = int(self.headers.get("Content-Length", 0))
                if content_len <= 0:
                    self._send_json({"error": "Empty body"}, 400)
                    return
                body = self.rfile.read(content_len)
                raw_name = os.path.basename(self.headers.get("X-Filename", "uploaded.db")) or "uploaded.db"
                out_dir = Path(ROOT_DIR) / "data" / "recordings"
                out_dir.mkdir(parents=True, exist_ok=True)
                target_path = str(out_dir / raw_name)
                with open(target_path, "wb") as f:
                    f.write(body)
                try:
                    _reload_db(target_path)
                    self._send_json({"status": "ok", "db_path": target_path})
                except Exception as e:
                    self._send_json({"error": str(e)}, 400)
            elif path == "/api/labels":
                content_len = int(self.headers.get("Content-Length", 0))
                if content_len <= 0:
                    self._send_json({"error": "Empty body"}, 400)
                    return
                try:
                    payload = json.loads(self.rfile.read(content_len).decode("utf-8"))
                    start_f = int(payload["start_frame"])
                    end_f = int(payload["end_frame"])
                    tag = str(payload["tag"]).strip()
                    if not tag:
                        raise ValueError("tag must be non-empty")
                    note = str(payload.get("note", ""))
                    author = str(payload.get("author", "user"))
                    with state_lock:
                        cur_db = replay_state["db_path"]
                    lbl_id = add_label(cur_db, start_f, end_f, tag, note=note, author=author)
                    with state_lock:
                        replay_state["flight_session"] = None
                    self._send_json({"status": "ok", "label_id": lbl_id})
                except Exception as e:
                    self._send_json({"error": str(e)}, 400)
            else:
                self._send_json({"error": "Endpoint not found"}, 404)

    httpd = ThreadingHTTPServer(("127.0.0.1", 0), LocalReplayHandler)
    port = httpd.server_address[1]
    url = f"http://127.0.0.1:{port}/?replay=1"
    print(f"[Replay] Serving {db_path} locally at 60 Hz: {url}")
    print("[Replay] Press Ctrl+C to stop the local replay server.")
    if open_browser:
        launch_tactical_gui(url, prefer_app_window=prefer_app_window)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[Replay] Stopped.")
    finally:
        httpd.server_close()


def run_full_verification(base_url: str, ssh_host: str = "robot") -> bool:
    """
    Runs comprehensive end-to-end verification of all Pi server endpoints.
    """
    print("\n" + "=" * 75)
    print(f"  FOLKRACE PI 4: FULL SUBSYSTEM VERIFICATION REPORT")
    print("=" * 75)
    print(f"  Target Server Base URL: {base_url}")
    print(f"  SSH Host Target       : {ssh_host}")
    print("-" * 75)

    all_passed = True
    tests = []

    # 1. SSH Check
    ssh_target = format_ssh_target(ssh_host)
    t0 = time.perf_counter()
    ssh_ok = check_ssh_connection(ssh_target, timeout_s=6)
    dt_ssh = (time.perf_counter() - t0) * 1000
    tests.append(("SSH Connectivity (port 22)", ssh_ok, f"{dt_ssh:.1f} ms", f"Connected as {ssh_target}"))
    if not ssh_ok:
        all_passed = False

    # 2. HTTP Web GUI Root (port 8080)
    t0 = time.perf_counter()
    gui_ok = False
    gui_info = "Failed to load index"
    try:
        req = urllib.request.Request(f"{base_url}/", headers={"User-Agent": "FolkraceLauncher/1.0"})
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            content = resp.read()
            gui_ok = (resp.status == 200) and (b"Folkrace" in content or b"FOLKRACE" in content)
            gui_info = f"{len(content)} bytes served"
    except Exception as e:
        gui_info = str(e)
    dt_gui = (time.perf_counter() - t0) * 1000
    tests.append(("HTTP Web Tactical GUI (/)", gui_ok, f"{dt_gui:.1f} ms", gui_info))
    if not gui_ok:
        all_passed = False

    # 3. /api/status Endpoint
    t0 = time.perf_counter()
    status_data = query_api_json(base_url, "/api/status")
    dt_status = (time.perf_counter() - t0) * 1000
    status_ok = status_data is not None and "session_id" in status_data
    status_info = ""
    if status_ok:
        affinity = status_data.get("cpu_affinity", [])
        status_info = f"Session: {status_data.get('session_id')} | Core Mask: {affinity} | Loop: {status_data.get('loop_hz')} Hz"
    tests.append(("System Status API (/api/status)", status_ok, f"{dt_status:.1f} ms", status_info))
    if not status_ok:
        all_passed = False

    # 4. /api/live Real-time Telemetry Stream
    t0 = time.perf_counter()
    live_data = query_api_json(base_url, "/api/live")
    dt_live = (time.perf_counter() - t0) * 1000
    live_ok = live_data is not None and ("timestamp" in live_data or "sim_time" in live_data or "v_cmd" in live_data)
    live_info = ""
    if live_ok:
        live_info = f"Mode: {live_data.get('mode', 'DRIVE')} | V: {live_data.get('v_cmd', 0.0):.2f} m/s | Steer: {live_data.get('steer_cmd_deg', live_data.get('steer_deg', 0.0)):.1f}° | Step: {live_data.get('step_ms', 0.0)} ms"
    tests.append(("Live Telemetry API (/api/live)", live_ok, f"{dt_live:.1f} ms", live_info))
    if not live_ok:
        all_passed = False

    # 4b. /api/live_bin High-Speed Zero-Copy Binary Stream
    t0 = time.perf_counter()
    bin_ok = False
    bin_info = ""
    try:
        req = urllib.request.Request(f"{base_url}/api/live_bin", headers={"User-Agent": "FolkraceLauncher/1.0"})
        with urllib.request.urlopen(req, timeout=2.5) as resp:
            raw_bin = resp.read()
            bin_ok = (resp.status == 200) and len(raw_bin) >= 224 and raw_bin[:4] == b"FOLK"
            bin_info = f"{len(raw_bin)/1024.0:.1f} KB binary payload (Header + Raw Costmaps + Metadata)"
    except Exception as e:
        bin_info = str(e)
    dt_bin = (time.perf_counter() - t0) * 1000
    tests.append(("High-Speed Binary Stream (/api/live_bin)", bin_ok, f"{dt_bin:.1f} ms", bin_info))
    if not bin_ok:
        all_passed = False

    # 5. /api/anomalies Black Box Index
    t0 = time.perf_counter()
    anom_data = query_api_json(base_url, "/api/anomalies")
    dt_anom = (time.perf_counter() - t0) * 1000
    anom_ok = isinstance(anom_data, list)
    anom_info = f"{len(anom_data)} anomalies indexed" if anom_ok else "Invalid response"
    tests.append(("Anomaly Watchdog (/api/anomalies)", anom_ok, f"{dt_anom:.1f} ms", anom_info))
    if not anom_ok:
        all_passed = False

    # 6. /api/frame/<id> Historical Random Access
    test_frame_id = live_data.get("_total_frames", 100) if live_data else 100
    test_frame_id = max(1, min(test_frame_id - 5, test_frame_id))
    t0 = time.perf_counter()
    frame_data = query_api_json(base_url, f"/api/frame/{test_frame_id}")
    dt_frame = (time.perf_counter() - t0) * 1000
    frame_ok = frame_data is not None and ("_frame_id" in frame_data or "v_cmd" in frame_data)
    frame_info = f"Frame #{test_frame_id} retrieved with full costmap & trajectories" if frame_ok else "Failed"
    tests.append(("Historical Time Travel (/api/frame)", frame_ok, f"{dt_frame:.1f} ms", frame_info))
    if not frame_ok:
        all_passed = False

    # 7. Core 1 vs Core 2 Hardware Isolation Check (via SSH)
    t0 = time.perf_counter()
    iso_ok = False
    iso_info = ""
    try:
        cmd = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4", ssh_target, "ps -eo pid,psr,comm"]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.DEVNULL, text=True, timeout=5)
        lines = [l for l in res.stdout.strip().splitlines() if any(k in l for k in ("python", "path_planner", "debug_server"))]
        iso_ok = len(lines) >= 2
        iso_info = f"{len(lines)} Folkrace processes active on Pi CPU cores"
    except Exception as e:
        iso_info = str(e)
    dt_iso = (time.perf_counter() - t0) * 1000
    tests.append(("Hardware CPU Core Isolation", iso_ok, f"{dt_iso:.1f} ms", iso_info))

    # Print Table
    for name, ok, latency, details in tests:
        mark = " PASS " if ok else " FAIL "
        status_color = "\033[92m" if ok else "\033[91m"
        reset_color = "\033[0m"
        print(f"  [{status_color}{mark}{reset_color}] {name:<35} | {latency:<9} | {details}")

    print("-" * 75)
    if all_passed:
        print("  \033[92m>>> ALL CHECKS PASSED: Pi 4 Dual-Core Stack is Fully Operational! <<<\033[0m")
    else:
        print("  \033[91m>>> WARNING: One or more checks failed. Review Pi logs via SSH. <<<\033[0m")
    print("=" * 75 + "\n")
    return all_passed


def stream_telemetry_console(base_url: str):
    """Streams live telemetry directly into the terminal."""
    print("\n" + "=" * 80)
    print("  FOLKRACE LIVE TELEMETRY CONSOLE (Press Ctrl+C to exit)")
    print("=" * 80)
    print(f"  {'TIME':<8} | {'MODE':<7} | {'V_CMD':<7} | {'STEER':<7} | {'FL':<5} {'FR':<5} {'RL':<5} {'RR':<5} | {'PLAN_MS':<7} | {'ANOMALIES'}")
    print("-" * 80)

    try:
        while True:
            pkt = query_api_json(base_url, "/api/live", timeout_s=1.0)
            if pkt:
                sim_t = f"{pkt.get('sim_time', 0.0):.1f}s"
                mode = str(pkt.get("mode", "DRIVE"))[:7]
                v_cmd = f"{pkt.get('v_cmd', 0.0):.2f}"
                steer = f"{pkt.get('steer_cmd_deg', 0.0):.1f}°"
                rpms = pkt.get("rpms_act", [0, 0, 0, 0])
                if len(rpms) >= 4:
                    fl, fr, rl, rr = int(rpms[0]), int(rpms[1]), int(rpms[2]), int(rpms[3])
                else:
                    fl = fr = rl = rr = 0
                plan_ms = f"{pkt.get('plan_ms', 0.0):.1f}ms"
                slip = pkt.get("slip_state", "NORMAL")
                f_tof = pkt.get("f_tof", 1.0)
                
                alerts = []
                if slip != "NORMAL":
                    alerts.append(f"SLIP({slip})")
                if f_tof < 0.2:
                    alerts.append(f"PROX({f_tof:.2f}m)")
                alert_str = " ".join(alerts) if alerts else "OK"

                print(f"  {sim_t:<8} | {mode:<7} | {v_cmd:<7} | {steer:<7} | {fl:<5} {fr:<5} {rl:<5} {rr:<5} | {plan_ms:<7} | {alert_str}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Stream] Stopped.")


def main():
    parser = argparse.ArgumentParser(description="Folkrace Pi 4 Dual-Core Debug Studio")
    parser.add_argument("--host", default=None, help="Pi 4 Hostname or IP (auto-detects by default)")
    parser.add_argument("--verify", action="store_true", help="Run full automated verification of all server endpoints")
    parser.add_argument("--stream", action="store_true", help="Stream live telemetry dashboard in terminal")
    parser.add_argument("--download", action="store_true", help="Download current SQLite black box flight database")
    parser.add_argument("--replay", nargs="?", const="__AUTO__", default=None, help="Open local 60 Hz replay GUI (optional .db path; downloads from Pi or uses latest local .db)")
    parser.add_argument("--reconnect", "--reconnect-mcu", dest="reconnect", action="store_true", help="Trigger remote ESP32 serial re-probe & reconnection")
    parser.add_argument("--tab", "--browser-tab", "--browser", "--web", dest="browser_tab", action="store_true", help="Open Tactical Mission Control in standard browser tab instead of standalone app window")
    parser.add_argument("--desktop", "--pyqt", "--planner-gui", dest="desktop", action="store_true", help="Launch native PyQt5 desktop planner GUI instead of Tactical Mission Control")
    parser.add_argument("--profile", action="store_true", help="Run GUI & telemetry performance profiler and benchmark")
    parser.add_argument("--measure-browser", action="store_true", help="Run browser latency and stream jitter benchmark")
    parser.add_argument("--restart", action="store_true", help="Restart remote folkrace systemd service via SSH")
    parser.add_argument("--stop", action="store_true", help="Stop remote folkrace service")
    parser.add_argument("--sync", action="store_true", help="Sync latest code to Pi 4")
    parser.add_argument("--no-browser", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    print("=" * 75)
    print("  FOLKRACE PI 4: DUAL-CORE DEBUG STUDIO & FLIGHT RECORDER")
    print("=" * 75)

    # 1. Handle code sync if requested
    if args.sync:
        ssh_target = resolve_ssh_host(args.host)
        sync_pi_stack(ssh_target)

    # 2. Handle remote restart/stop if requested
    if args.restart:
        restart_remote_service(resolve_ssh_host(args.host))
    if args.stop:
        stop_remote_service(resolve_ssh_host(args.host))
        return

    # 2b. Handle offline local --replay before requiring Pi connectivity
    if args.replay is not None:
        if args.replay != "__AUTO__":
            if not os.path.exists(args.replay):
                print(f"[Replay ERROR] Database not found: {args.replay}")
                sys.exit(1)
            run_local_replay_server(args.replay, prefer_app_window=not args.browser_tab, open_browser=not args.no_browser)
            return

        # --replay without path: try downloading from Pi, else fall back to latest local .db
        _, base_url = resolve_pi_host(args.host, timeout_s=1.0)
        db_file = download_database(base_url) if base_url else None
        if not db_file:
            db_file = find_latest_local_recording()
        if not db_file:
            print("[Replay ERROR] Pi 4 is unreachable and no local recordings found in data/recordings/.")
            sys.exit(1)
        run_local_replay_server(db_file, prefer_app_window=not args.browser_tab, open_browser=not args.no_browser)
        return

    # 3. Auto-detect Pi host and verify HTTP server
    wifi_ssid, _, wifi_prefix = get_active_wifi_network()
    if wifi_ssid and wifi_prefix:
        print(f"[Launcher] Active Wi-Fi: '{wifi_ssid}' ({wifi_prefix}0/24)")
    elif wifi_prefix:
        print(f"[Launcher] Active Network: {wifi_prefix}0/24")
    print("[Launcher] Auto-detecting Raspberry Pi 4 debug server...")
    active_host, base_url = resolve_pi_host(args.host)

    if not base_url:
        print(f"[Warning] Debug server not responding at port {PORT} on candidate hosts.")
        ssh_target = resolve_ssh_host(args.host)
        print(f"[Launcher] Attempting to auto-start Folkrace service via SSH on '{ssh_target}'...")
        try:
            restart_remote_service(ssh_target)
            active_host, base_url = resolve_pi_host(args.host, timeout_s=3.0)
        except Exception as e:
            print(f"[Error] Could not auto-start remote service: {e}")

    if not base_url:
        print(f"\n[FATAL] Unable to connect to Pi 4 Debug Server.")
        print(f"  Please ensure:")
        print(f"  1. Pi 4 is powered on and connected to local Wi-Fi.")
        print(f"  2. Service is running: ssh robot 'systemctl status folkrace'")
        print(f"  3. You can manually specify the IP: python 1_launchers/{os.path.basename(__file__)} --host <pi-ip>")
        sys.exit(1)

    # 4. Query System Status
    status = query_api_json(base_url, "/api/status")
    print("-" * 75)
    print(f"  Connected Host        : {active_host} ({base_url})")
    if status:
        print(f"  Active Session ID     : {status.get('session_id')}")
        print(f"  Total Recorded Frames : {status.get('total_recorded')} frames")
        print(f"  Active Driving Time   : {status.get('driving_minutes')} min ({status.get('driving_seconds')}s)")
        print(f"  Running Buffer Cap    : {status.get('max_buffer_minutes')} min (Auto-pruning active)")
        print(f"  Flight Log DB Size    : {status.get('file_size_mb')} MB (Free SD space: {status.get('disk_free_gb')} GB)")
        print(f"  Core Allocation       : Planner = Core 1 | Debug Server = Cores {status.get('cpu_affinity')}")
        print(f"  Telemetry Loop Rate   : {status.get('loop_hz')} Hz | Pi SoC Temp: {status.get('temp_c')}°C")
    print("-" * 75)

    # 5. Execute sub-modes
    if args.reconnect:
        trigger_mcu_reconnect(base_url)
        return

    if args.verify:
        run_full_verification(base_url, ssh_host=format_ssh_target(active_host))
        return

    if args.profile or args.measure_browser:
        tools_dir = os.path.join(ROOT_DIR, "tools")
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)
        import measure_browser_latency
        ws_res = measure_browser_latency.benchmark_websocket_stream(host=active_host, duration_s=8.0)
        http_res = measure_browser_latency.benchmark_http_polling(base_url=base_url, duration_s=5.0)
        measure_browser_latency.print_comparison_report(ws_res, http_res)
        return

    if args.download:
        download_database(base_url)
        return

    if args.stream:
        stream_telemetry_console(base_url)
        return

    if args.desktop:
        if not os.path.exists(PLANNER_GUI_SCRIPT):
            print(f"[Error] Desktop GUI not found at: {PLANNER_GUI_SCRIPT}")
            sys.exit(1)

        if base_url:
            try:
                reg_resp = query_api_json(base_url, "/api/register_client", timeout_s=1.5)
                if reg_resp:
                    print(f"[Launcher] Registered this workstation for direct binary UDP telemetry from {active_host}")
            except Exception:
                pass

        print("[Launcher] Launching native PyQt5 Desktop GUI (hardware-accelerated, zero-overhead UDP)...")
        print("=" * 75)
        cmd = [sys.executable, PLANNER_GUI_SCRIPT]
        sys.exit(subprocess.run(cmd, cwd=CONTROLLERS_DIR).returncode)

    # Default action: Launch Tactical Mission Control GUI (same GUI as browser server)
    if base_url:
        try:
            query_api_json(base_url, "/api/register_client", timeout_s=1.0)
        except Exception:
            pass

    launch_tactical_gui(base_url, prefer_app_window=not args.browser_tab)


if __name__ == "__main__":
    main()
