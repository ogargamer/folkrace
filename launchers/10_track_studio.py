#!/usr/bin/env python3
"""
================================================================================
  10. LAUNCHER: MODULAR RACE TRACK STUDIO & WEBOTS/MAP EXPORTER
================================================================================
  Interactive 1200mm Tile Snapping, Custom Obstacle Drawing & Local Bundle Export.
  Runs a zero-dependency local web server on port 8088 and opens your browser.

  Usage:
    python launchers/10_track_studio.py
================================================================================
"""

import argparse
import base64
import json
import os
import sys
import threading
import time
import urllib.parse
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

DEFAULT_PORT = 8088
DEFAULT_HOST = "0.0.0.0"

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STATIC_DIR = os.path.join(ROOT_DIR, "webots", "assets", "track_builder")
DEFAULT_OUTPUT_DIR = os.path.join(ROOT_DIR, "data", "maps")
REQUIRED_ASSETS = ("track_builder.html", "track_builder.css", "track_builder.js")


class TrackStudioServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, server_address, handler_class, output_dir: str):
        self.output_dir = os.path.abspath(output_dir)
        super().__init__(server_address, handler_class)


class TrackStudioHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC_DIR, **kwargs)

    def _send_json(self, status_code: int, payload: dict) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _serve_static_file(self, filename: str, content_type: str) -> None:
        filepath = os.path.join(STATIC_DIR, filename)
        with open(filepath, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._serve_static_file("track_builder.html", "text/html; charset=utf-8")
            return
        if parsed.path == "/track_builder.css":
            self._serve_static_file("track_builder.css", "text/css; charset=utf-8")
            return
        if parsed.path == "/track_builder.js":
            self._serve_static_file("track_builder.js", "application/javascript; charset=utf-8")
            return

        return super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path not in ("/api/save_local", "/api/deploy_to_pi"):
            self._send_json(404, {"status": "error", "message": "Endpoint not found"})
            return

        try:
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length)
            data = json.loads(body.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as e:
            self._send_json(400, {"status": "error", "message": f"Invalid JSON payload: {e}"})
            return

        try:
            output_dir = self.server.output_dir
            os.makedirs(output_dir, exist_ok=True)
            saved_files = []

            png_b64 = data.get("png_b64", "")
            if png_b64:
                try:
                    png_bytes = base64.b64decode(png_b64, validate=True)
                except Exception as e:
                    self._send_json(400, {"status": "error", "message": f"Invalid base64 PNG data: {e}"})
                    return
                with open(os.path.join(output_dir, "track_map.png"), "wb") as f:
                    f.write(png_bytes)
                saved_files.append("track_map.png")

            yaml_content = data.get("yaml", "")
            if yaml_content:
                with open(os.path.join(output_dir, "track_map.yaml"), "w", encoding="utf-8") as f:
                    f.write(yaml_content)
                saved_files.append("track_map.yaml")

            track_json = data.get("track_json", "")
            if track_json:
                with open(os.path.join(output_dir, "track_layout.json"), "w", encoding="utf-8") as f:
                    f.write(track_json)
                saved_files.append("track_layout.json")

            wbt_content = data.get("wbt", "")
            if wbt_content:
                with open(os.path.join(output_dir, "track_world.wbt"), "w", encoding="utf-8") as f:
                    f.write(wbt_content)
                saved_files.append("track_world.wbt")

            try:
                display_dir = os.path.relpath(output_dir, ROOT_DIR)
            except ValueError:
                display_dir = output_dir

            print(f"[OK] Saved track bundle ({', '.join(saved_files)}) in: {output_dir}")
            self._send_json(
                200,
                {
                    "status": "ok",
                    "message": f"Saved track bundle to {display_dir}",
                    "files": saved_files,
                },
            )
        except Exception as e:
            self._send_json(500, {"status": "error", "message": str(e)})


def validate_static_assets() -> bool:
    if not os.path.isdir(STATIC_DIR):
        print(f"[ERROR] Track Studio static directory not found: {STATIC_DIR}", file=sys.stderr)
        return False
    missing = [name for name in REQUIRED_ASSETS if not os.path.isfile(os.path.join(STATIC_DIR, name))]
    if missing:
        print(
            f"[ERROR] Missing required Track Studio assets in {STATIC_DIR}: {', '.join(missing)}",
            file=sys.stderr,
        )
        return False
    return True


def run_server(host: str, port: int, output_dir: str, open_browser_tab: bool = True) -> None:
    if not validate_static_assets():
        sys.exit(1)

    print("=" * 70)
    print("  10. MODULAR RACE TRACK STUDIO & WEBOTS/MAP EXPORTER")
    print(f"  WEB STUDIO READY AT: http://localhost:{port}")
    print(f"  LOCAL BUNDLE DIR:    {os.path.abspath(output_dir)}")
    print("=" * 70)

    server = TrackStudioServer((host, port), TrackStudioHandler, output_dir=output_dir)

    if open_browser_tab:
        def open_browser():
            time.sleep(0.5)
            webbrowser.open(f"http://localhost:{port}")

        threading.Thread(target=open_browser, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Track Studio Stopped]")
    finally:
        server.server_close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Folkrace Modular Race Track Studio (Launcher 10)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"HTTP port (default: {DEFAULT_PORT})")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Bind address (default: {DEFAULT_HOST})")
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help=f"Local export bundle directory (default: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument("--no-browser", action="store_true", help="Do not open a browser window automatically")
    args = parser.parse_args()

    run_server(
        host=args.host,
        port=args.port,
        output_dir=args.output_dir,
        open_browser_tab=not args.no_browser,
    )


if __name__ == "__main__":
    main()
