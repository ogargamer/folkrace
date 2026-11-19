"""
================================================================================
  19. LAUNCHER: OFFLINE 60 HZ FLIGHT LOG REPLAY VIEWER
================================================================================
  Opens downloaded SQLite flight recordings (.db) locally on your laptop
  at full 60 Hz using the Tactical Mission Control GUI—without requiring
  a connection to the Raspberry Pi 4.

  Usage:
    python launchers/19_local_flight_replay.py                  # Opens latest .db in Downloads or data/recordings/
    python launchers/19_local_flight_replay.py path/to/file.db  # Opens a specific .db file
    python launchers/19_local_flight_replay.py --list           # Lists available local recordings
    python launchers/19_local_flight_replay.py --pick           # Choose a recording interactively
    python launchers/19_local_flight_replay.py --tab            # Opens in standard browser tab
================================================================================
"""

import os
import sys
import argparse
import importlib.util
from pathlib import Path
from typing import List, Optional

if sys.platform == "win32" and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

ROOT_DIR = Path(__file__).resolve().parent.parent
DOWNLOADS_DIR = Path.home() / "Downloads"
RECORDINGS_DIR = ROOT_DIR / "data" / "recordings"
SEARCH_DIRS = (DOWNLOADS_DIR, RECORDINGS_DIR)


def _load_debug_studio_module():
    """Imports 17_pi4_debug_studio.py to reuse its local replay server."""
    studio_path = Path(__file__).resolve().parent / "17_pi4_debug_studio.py"
    spec = importlib.util.spec_from_file_location("pi4_debug_studio", str(studio_path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def list_local_recordings(directories=SEARCH_DIRS) -> List[Path]:
    """Returns all .db files in the given directories sorted newest first."""
    dbs: List[Path] = []
    seen = set()
    for directory in directories:
        if not directory.exists():
            continue
        for p in directory.glob("*.db"):
            resolved = p.resolve()
            if resolved not in seen:
                seen.add(resolved)
                dbs.append(p)
    return sorted(dbs, key=lambda p: p.stat().st_mtime, reverse=True)


def get_latest_downloads_recording() -> Optional[Path]:
    """Returns the newest .db file in Downloads, or None if none exist."""
    downloads_dbs = list_local_recordings((DOWNLOADS_DIR,))
    return downloads_dbs[0] if downloads_dbs else None


def print_recordings(dbs: List[Path]):
    """Prints available local .db recordings with size and frame count."""
    pi4_dir = str(ROOT_DIR / "pi4")
    if pi4_dir not in sys.path:
        sys.path.insert(0, pi4_dir)
    from debug_recorder import TelemetryLogReader

    print("\nAvailable local flight recordings (Downloads & data/recordings/):")
    print("-" * 90)
    for idx, p in enumerate(dbs, 1):
        size_mb = p.stat().st_size / (1024 * 1024)
        frames_str = "?"
        try:
            reader = TelemetryLogReader(str(p))
            _, _, total = reader.get_frame_bounds()
            reader.close()
            frames_str = f"{total} frames"
        except Exception:
            pass
        source = "Downloads" if p.parent.resolve() == DOWNLOADS_DIR.resolve() else "data/recordings"
        print(f"  [{idx}] {p.name:<34} | {source:<15} | {size_mb:6.2f} MB | {frames_str}")
    print("-" * 90)


def main():
    parser = argparse.ArgumentParser(description="Offline 60 Hz Local Flight Log Replay Viewer")
    parser.add_argument("db_path", nargs="?", default=None, help="Path to a downloaded .db flight recording")
    parser.add_argument("--list", action="store_true", help="List available .db files in Downloads and data/recordings/")
    parser.add_argument("--pick", action="store_true", help="Interactively pick a recording from Downloads or data/recordings/")
    parser.add_argument("--tab", dest="browser_tab", action="store_true", help="Open in standard browser tab instead of standalone app window")
    parser.add_argument("--digest", action="store_true", help="Print AI debugging digest for the recording and exit")
    parser.add_argument("--export", action="store_true", help="Export AI/ML bundle to data/ai_exports/ and exit")
    parser.add_argument("--no-browser", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    print("=" * 75)
    print("  FOLKRACE: OFFLINE 60 HZ FLIGHT LOG REPLAY VIEWER")
    print("=" * 75)

    dbs = list_local_recordings()

    if args.list:
        if not dbs:
            print(f"[Replay] No recordings found in {DOWNLOADS_DIR} or {RECORDINGS_DIR}")
            print("  Download one from the Pi with: python launchers/17_pi4_debug_studio.py --download")
        else:
            print_recordings(dbs)
        return

    target_db = args.db_path
    if not target_db:
        if not dbs:
            print(f"[Replay ERROR] No .db recordings found in {DOWNLOADS_DIR} or {RECORDINGS_DIR}")
            print("  Download one from the Pi with: python launchers/17_pi4_debug_studio.py --download")
            sys.exit(1)

        if args.pick and len(dbs) > 1:
            print_recordings(dbs)
            choice = input(f"Select recording [1-{len(dbs)}] (default 1): ").strip()
            idx = int(choice) - 1 if choice.isdigit() and 1 <= int(choice) <= len(dbs) else 0
            target_db = str(dbs[idx])
        else:
            latest_download = get_latest_downloads_recording()
            chosen = latest_download if latest_download is not None else dbs[0]
            target_db = str(chosen)
            source_label = "Downloads" if latest_download is not None else "data/recordings"
            print(f"[Replay] Auto-selected latest recording from {source_label}: {target_db}")

    if not os.path.exists(target_db):
        print(f"[Replay ERROR] Database not found: {target_db}")
        sys.exit(1)

    if args.digest or args.export:
        import json
        tools_dir = str(ROOT_DIR / "tools")
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)
        from flight_analyzer import FlightSession, format_digest_markdown

        if sys.platform == "win32" and hasattr(sys.stdout, "reconfigure"):
            try:
                sys.stdout.reconfigure(encoding="utf-8", errors="replace")
            except Exception:
                pass
        sess = FlightSession(target_db)
        if args.digest:
            print(format_digest_markdown(sess.build_digest()))
        if args.export:
            outputs = sess.export_bundle()
            print(json.dumps(outputs, indent=2))
        return

    studio = _load_debug_studio_module()
    studio.run_local_replay_server(
        target_db,
        prefer_app_window=not args.browser_tab,
        open_browser=not args.no_browser,
    )


if __name__ == "__main__":
    main()
