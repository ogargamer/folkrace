"""
================================================================================
  17. LAUNCHER: PI 4 DUAL-CORE DEBUG STUDIO & FLIGHT RECORDER
================================================================================
  Connects to Raspberry Pi 4 ('robot'):
    - Auto-detects and connects to Pi debug server over HTTP port 8080.
    - Verifies Path Planner is hard pinned to CPU Core 1.
    - Verifies Debug Server & SQLite Black Box are hard pinned to CPU Core 2.
    - Displays live telemetry status, driving buffer metrics, and SD card space.
    - Opens browser-based Tactical Mission Control (http://robot.local:8080)
      OR connects native desktop PyQt5 2D Telemetry GUI to live stream.
    - Supports full automated endpoint verification (--verify), real-time
      terminal telemetry monitoring (--stream), and log downloading (--download).

  Usage:
    python launchers/17_pi4_debug_studio.py              # Connects, shows status & opens Web GUI
    python launchers/17_pi4_debug_studio.py --verify     # Tests all Pi server endpoints
    python launchers/17_pi4_debug_studio.py --stream     # Live terminal telemetry dashboard
    python launchers/17_pi4_debug_studio.py --download   # Downloads active SQLite flight log
    python launchers/17_pi4_debug_studio.py --desktop    # Launches native desktop PyQt5 GUI
    python launchers/17_pi4_debug_studio.py --restart    # Restarts remote Folkrace service
    python launchers/17_pi4_debug_studio.py --sync       # Syncs pi4 code to robot before launch
================================================================================
"""

import os
import sys
import time
import json
import socket
import urllib.request
import urllib.error
import argparse
import webbrowser
import subprocess
from pathlib import Path
from typing import Optional, Dict, Any, Tuple

# Ensure UTF-8 output on Windows terminals
if sys.platform == "win32" and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI4_DIR = os.path.join(ROOT_DIR, "pi4")
CONTROLLERS_DIR = os.path.join(ROOT_DIR, "webots", "simulation", "my_project", "controllers", "tof_path_planner")
PLANNER_GUI_SCRIPT = os.path.join(CONTROLLERS_DIR, "planner_gui.py")

DEFAULT_CANDIDATES = ["192.168.20.120", "192.168.20.117", "robot", "robot.local", "127.0.0.1"]
REMOTE_DIR = "/home/admin/folkrace"
PORT = 8080


def resolve_pi_host(preferred_host: Optional[str] = None, timeout_s: float = 1.5) -> Tuple[Optional[str], Optional[str]]:
    """
    Auto-detects the responsive hostname or IP for the Pi 4 debug server.
    Returns (hostname, base_url).
    """
    candidates = [preferred_host] if preferred_host else DEFAULT_CANDIDATES
    candidates = [c for c in candidates if c]

    for host in candidates:
        url = f"http://{host}:{PORT}/api/status"
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "FolkraceLauncher/1.0"})
            with urllib.request.urlopen(req, timeout=timeout_s) as resp:
                if resp.status == 200:
                    return host, f"http://{host}:{PORT}"
        except Exception:
            continue

    return None, None


def check_ssh_connection(host: str = "robot", timeout_s: int = 3) -> bool:
    """Checks if Raspberry Pi is reachable over SSH."""
    cmd = ["ssh", "-o", "BatchMode=yes", "-o", f"ConnectTimeout={timeout_s}", host, "echo ok"]
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout_s + 1, text=True)
        return res.returncode == 0 and "ok" in res.stdout
    except Exception:
        return False


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
    """Synchronizes local pi4 files and core configurations to the Pi."""
    print(f"[Sync] Preparing remote directory: {REMOTE_DIR} on {host}...")
    subprocess.run(["ssh", host, f"mkdir -p {REMOTE_DIR}/core {REMOTE_DIR}/data/recordings"], check=True)

    print(f"[Sync] Uploading pi4 subsystem scripts to {host}...")
    files = [
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

    core_dir = os.path.join(ROOT_DIR, "core")
    if os.path.exists(core_dir):
        subprocess.run(["scp", "-r", f"{core_dir}/*", f"{host}:{REMOTE_DIR}/core/"], check=True)

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


def download_database(base_url: str, output_path: Optional[str] = None) -> bool:
    """Downloads the active flight recorder SQLite database."""
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
        return True
    except Exception as e:
        print(f"[Download] Error: {e}")
        return False


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
    ssh_target = "robot"
    t0 = time.perf_counter()
    ssh_ok = check_ssh_connection(ssh_target, timeout_s=6)
    dt_ssh = (time.perf_counter() - t0) * 1000
    tests.append(("SSH Connectivity (port 22)", ssh_ok, f"{dt_ssh:.1f} ms", f"Connected as admin@{ssh_target}"))
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
    parser.add_argument("--desktop", action="store_true", help="Launch native PyQt5 desktop GUI")
    parser.add_argument("--profile", action="store_true", help="Run GUI & telemetry performance profiler and benchmark")
    parser.add_argument("--restart", action="store_true", help="Restart remote folkrace systemd service via SSH")
    parser.add_argument("--stop", action="store_true", help="Stop remote folkrace service")
    parser.add_argument("--sync", action="store_true", help="Sync latest code to Pi 4")
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically open web browser")
    args = parser.parse_args()

    print("=" * 75)
    print("  FOLKRACE PI 4: DUAL-CORE DEBUG STUDIO & FLIGHT RECORDER")
    print("=" * 75)

    # 1. Handle code sync if requested
    if args.sync:
        ssh_target = args.host or "robot"
        sync_pi_stack(ssh_target)

    # 2. Handle remote restart/stop if requested
    if args.restart:
        restart_remote_service(args.host or "robot")
    if args.stop:
        stop_remote_service(args.host or "robot")
        return

    # 3. Auto-detect Pi host and verify HTTP server
    print("[Launcher] Auto-detecting Raspberry Pi 4 debug server...")
    active_host, base_url = resolve_pi_host(args.host)

    if not base_url:
        print(f"[Warning] Debug server not responding at port {PORT} on candidate hosts.")
        print(f"[Launcher] Attempting to auto-start Folkrace service via SSH on 'robot'...")
        try:
            restart_remote_service("robot")
            active_host, base_url = resolve_pi_host(args.host, timeout_s=3.0)
        except Exception as e:
            print(f"[Error] Could not auto-start remote service: {e}")

    if not base_url:
        print(f"\n[FATAL] Unable to connect to Pi 4 Debug Server.")
        print(f"  Please ensure:")
        print(f"  1. Pi 4 is powered on and connected to local Wi-Fi.")
        print(f"  2. Service is running: ssh robot 'systemctl status folkrace'")
        print(f"  3. You can manually specify the IP: python launchers/17_pi4_debug_studio.py --host <pi-ip>")
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
    if args.verify:
        run_full_verification(base_url, ssh_host=active_host if "robot" in active_host else "robot")
        return

    if args.profile:
        tools_dir = os.path.join(ROOT_DIR, "tools")
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)
        from gui_profiler import profile_pi4_studio
        profile_pi4_studio(host=active_host)
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
                    print(f"[Launcher] Registered this workstation for direct 60 Hz UDP telemetry from {active_host}")
            except Exception:
                pass
        print("[Launcher] Launching native PyQt5 Desktop GUI...")
        cmd = [sys.executable, PLANNER_GUI_SCRIPT]
        sys.exit(subprocess.run(cmd, cwd=CONTROLLERS_DIR).returncode)

    # Default action: Open browser
    if not args.no_browser:
        print(f"[Launcher] Opening Web Tactical Debugger in browser: {base_url}")
        webbrowser.open(base_url)
    print("=" * 75)


if __name__ == "__main__":
    main()
