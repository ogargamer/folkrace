#!/usr/bin/env python3
"""
================================================================================
  9. LAUNCHER: MODULAR RACE TRACK STUDIO & CARTOGRAPHER MAP EXPORTER
================================================================================
  Interactive 1200mm Tile Snapping, Custom Obstacle Drawing & 1-Click Pi 5 Deploy.
  Runs a zero-dependency local web server on port 8088 and opens your browser.

  Usage:
    python 9_track_studio.py
================================================================================
"""

import os
import sys
import json
import base64
import time
import webbrowser
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
import urllib.parse

PORT = 8088
PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATIC_DIR = os.path.join(ROOT_DIR, "pi4", "cartographer", "static")


class TrackStudioHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC_DIR, **kwargs)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path in ["/", "/index.html"]:
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            with open(os.path.join(STATIC_DIR, "track_builder.html"), "rb") as f:
                self.wfile.write(f.read())
            return
        elif parsed.path in ["/track_builder.css"]:
            self.send_response(200)
            self.send_header("Content-Type", "text/css; charset=utf-8")
            self.end_headers()
            with open(os.path.join(STATIC_DIR, "track_builder.css"), "rb") as f:
                self.wfile.write(f.read())
            return
        elif parsed.path in ["/track_builder.js"]:
            self.send_response(200)
            self.send_header("Content-Type", "application/javascript; charset=utf-8")
            self.end_headers()
            with open(os.path.join(STATIC_DIR, "track_builder.js"), "rb") as f:
                self.wfile.write(f.read())
            return

        return super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/deploy_to_pi":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)

            try:
                data = json.loads(body.decode("utf-8"))
                png_b64 = data.get("png_b64", "")
                yaml_content = data.get("yaml", "")
                track_json = data.get("track_json", "")

                # 1. Save Locally in maps/ directory
                local_map_dir = os.path.join(ROOT_DIR, "maps")
                os.makedirs(local_map_dir, exist_ok=True)

                local_png = os.path.join(local_map_dir, "track_map.png")
                local_yaml = os.path.join(local_map_dir, "track_map.yaml")
                local_json = os.path.join(local_map_dir, "track_layout.json")

                with open(local_png, "wb") as f:
                    f.write(base64.b64decode(png_b64))

                with open(local_yaml, "w") as f:
                    f.write(yaml_content)

                with open(local_json, "w") as f:
                    f.write(track_json)

                print(f"[OK] Saved map files locally in: {local_map_dir}")

                # 2. Upload to Raspberry Pi 5 over SFTP (optional)
                pi_deployed = False
                try:
                    import paramiko
                    ssh = paramiko.SSHClient()
                    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=3)
                    ssh.exec_command("mkdir -p /home/admin/cartographer/maps")
                    time.sleep(0.2)

                    sftp = ssh.open_sftp()
                    sftp.put(local_png, "/home/admin/cartographer/maps/track_map.png")
                    sftp.put(local_yaml, "/home/admin/cartographer/maps/track_map.yaml")
                    sftp.put(local_json, "/home/admin/cartographer/maps/track_layout.json")
                    sftp.close()
                    ssh.close()
                    pi_deployed = True
                except Exception as pi_err:
                    print(f"[!] Pi 5 offline or unreachable: {pi_err}")

                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()

                if pi_deployed:
                    resp_data = {"status": "ok", "message": "Map saved locally & uploaded to Pi 5 (/home/admin/cartographer/maps/)"}
                else:
                    resp_data = {"status": "ok", "message": "Saved locally in maps/ folder (Pi 5 offline)"}

                self.wfile.write(json.dumps(resp_data).encode("utf-8"))

            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))
            return

        return super().do_POST()


def run_server():
    print("=" * 70)
    print("  MODULAR RACE TRACK STUDIO & CARTOGRAPHER MAP BUILDER")
    print(f"  WEB STUDIO READY AT: http://localhost:{PORT}")
    print("=" * 70)

    server = HTTPServer(("0.0.0.0", PORT), TrackStudioHandler)

    def open_browser():
        time.sleep(0.5)
        webbrowser.open(f"http://localhost:{PORT}")

    threading.Thread(target=open_browser, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Track Studio Stopped]")


if __name__ == "__main__":
    run_server()
