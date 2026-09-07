"""
================================================================================
  WEBOTS TELEMETRY STREAM & BI-DIRECTIONAL VERIFICATION TEST
================================================================================
  1. Binds UDP port 9876 to listen for live telemetry packets from Webots.
  2. Sends ping/commands to Webots on port 9877 to verify bi-directional control.
  3. Displays real-time streaming frequency, chassis kinematics, pillar states,
     and the live mathematical thought process of all 4 Anti-Wrong-Way options.
  4. Automatically diagnoses connection state (e.g. paused simulation, controller
     offline, or active stream).
================================================================================
"""

import os
import sys
import time
import json
import socket
import select
import subprocess
from typing import Optional

UDP_RECV_PORT = 9876
UDP_CMD_PORT = 9877

def check_system_processes():
    """Checks if Webots and its controller are currently running."""
    webots_running = False
    controller_running = False
    try:
        res = subprocess.run(
            ["powershell", "-NoProfile", "-Command",
             "Get-CimInstance Win32_Process | Select-Object ProcessId, Name, CommandLine | ConvertTo-Json -Depth 2"],
            capture_output=True, text=True, timeout=5
        )
        if res.stdout:
            data = json.loads(res.stdout)
            if isinstance(data, dict):
                data = [data]
            for p in data:
                cmd = (p.get("CommandLine") or "").lower()
                name = (p.get("Name") or "").lower()
                if "webots" in name or "webots" in cmd:
                    webots_running = True
                if "champion_live_preview" in cmd:
                    controller_running = True
    except Exception:
        pass
    return webots_running, controller_running

def run_test():
    print("=" * 78)
    print("   WEBOTS TELEMETRY & ANTI-WRONG-WAY VERIFICATION TEST")
    print("=" * 78)

    # 1. System Process Diagnostic
    print("\n[Step 1/3] Checking Running Processes...")
    w_run, c_run = check_system_processes()
    print(f"  * Webots Simulator:   {'[RUNNING]' if w_run else '[NOT DETECTED]'}")
    print(f"  * Champion Controller: {'[RUNNING]' if c_run else '[NOT DETECTED]'}")

    if not w_run:
        print("\n  [!] Webots is not running. Please open test_wrong_way.wbt in Webots first.")
    elif not c_run:
        print("\n  [!] Controller champion_live_preview is not running. Press Ctrl+Shift+R in Webots to reload.")

    # 2. Bind UDP Receiver Socket
    print("\n[Step 2/3] Binding UDP Telemetry Receiver on Port 9876...")
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    bound = False
    for ip in ["127.0.0.1", "0.0.0.0"]:
        try:
            recv_sock.bind((ip, UDP_RECV_PORT))
            print(f"  [SUCCESS] Bound UDP listening socket to {ip}:{UDP_RECV_PORT}")
            bound = True
            break
        except Exception as e:
            print(f"  [NOTE] Could not bind {ip}:{UDP_RECV_PORT}: {e}")

    if not bound:
        print("  [ERROR] Port 9876 is busy! If another GUI instance is open, close it first.")
        return

    recv_sock.settimeout(0.5)

    # 3. Create Command Socket
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("\n[Step 3/3] Testing Bi-directional Telemetry Stream...")
    print("  * Sending 'play' command to Webots on port 9877 to wake controller...")
    try:
        cmd_sock.sendto(json.dumps({"cmd": "play"}).encode("utf-8"), ("127.0.0.1", UDP_CMD_PORT))
        print("  * Play command dispatched to 127.0.0.1:9877 successfully.")
    except Exception as e:
        print(f"  [WARN] Failed to send UDP command: {e}")

    print("\n" + "-" * 78)
    print("  STREAMING LIVE TELEMETRY (Press Ctrl+C to exit test):")
    print("-" * 78)

    packet_count = 0
    start_time = time.time()
    last_stat_time = time.time()
    consecutive_timeouts = 0

    while True:
        try:
            data, addr = recv_sock.recvfrom(65535)
            packet_count += 1
            consecutive_timeouts = 0
            now = time.time()

            try:
                pkt = json.loads(data.decode("utf-8"))
            except Exception as e:
                print(f"  [JSON ERROR] Could not decode packet: {e}")
                continue

            sim_time = pkt.get("time", 0.0)
            mode = pkt.get("mode", "UNKNOWN")
            v_est = pkt.get("v_est", 0.0)
            v_cmd = pkt.get("v_cmd", 0.0)
            steer_deg = pkt.get("steer_deg", 0.0)
            dist_m = pkt.get("dist_m", 0.0)
            lap = pkt.get("lap", 0)
            fp = pkt.get("front_pillar", False)
            bp = pkt.get("back_pillar", False)
            aww = pkt.get("aww", {})

            fps = packet_count / max(0.001, (now - start_time))

            # Print concise summary every 0.35s
            if now - last_stat_time >= 0.35 or packet_count <= 2:
                last_stat_time = now

                is_ww = aww.get("is_wrong_way", False)
                det_by = aww.get("detected_by", "NONE")
                action = aww.get("action", mode)

                opt1 = aww.get("opt1", {})
                opt2 = aww.get("opt2", {})
                opt3 = aww.get("opt3", {})
                opt4 = aww.get("opt4", {})

                status_tag = f"[WRONG-WAY: {det_by}]" if is_ww else "[NORMAL CLEAR]"

                print(f"\n>> PKT #{packet_count:05d} | {fps:5.1f} Hz | {len(data)}B | Sim T: {sim_time:5.2f}s | {status_tag}")
                print(f"   Kinematics:  v_est = {v_est:4.2f} m/s (cmd: {v_cmd:4.2f}) | Steer = {steer_deg:+5.1f} deg | Dist = {dist_m:5.1f}m (Lap {lap})")
                print(f"   Pillars:     Front = {'[RAISED]' if fp else '[LOWERED]'} | Back = {'[RAISED]' if bp else '[LOWERED]'}")
                print(f"   Verdict:     Action = {action} | Detected By = {det_by}")
                print(f"   Option 1 (Breadcrumb):  Score = {opt1.get('score', 0.0):.2f} | Tripped = {opt1.get('is_tripped', False)} | {opt1.get('thought', '')}")
                print(f"   Option 2 (Turn Radius): R = {opt2.get('radius_m', 0.0):.2f}m | Tripped = {opt2.get('is_tripped', False)} | {opt2.get('thought', '')}")
                print(f"   Option 3 (Flow Costmap): Dot = {opt3.get('dot_product', 0.0):+.2f} | Tripped = {opt3.get('is_tripped', False)} | {opt3.get('thought', '')}")
                print(f"   Option 4 (Lateral Sep): InTurn = {opt4.get('in_turn', False)} | Lat = {opt4.get('lateral_m', 0.0):.2f}m | {opt4.get('thought', '')}")

        except socket.timeout:
            consecutive_timeouts += 1
            if consecutive_timeouts % 4 == 0:
                elapsed = time.time() - start_time
                print(f"\n[WAITING FOR WEBOTS] No packet received in last 2.0s (Total received: {packet_count}, Elapsed: {elapsed:.1f}s)")
                print("  -> DIAGNOSTIC: If Webots is open, make sure the simulation is NOT paused!")
                print("  -> Press the PLAY button in Webots toolbar or press Spacebar / Ctrl+2 to run.")
                print("  -> If you recently edited code, reload Webots world with Ctrl+Shift+R.")
                # Send play command again to try waking controller
                try:
                    cmd_sock.sendto(json.dumps({"cmd": "play"}).encode("utf-8"), ("127.0.0.1", UDP_CMD_PORT))
                except Exception:
                    pass

        except KeyboardInterrupt:
            print("\n\n[TEST FINISHED] Stopped by user.")
            break
        except Exception as e:
            print(f"\n[TEST EXCEPTION] {e}")

    recv_sock.close()
    cmd_sock.close()

if __name__ == "__main__":
    run_test()
