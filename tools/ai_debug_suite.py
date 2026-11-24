"""
================================================================================
  AI FLIGHT DEBUGGING SUITE & TELEMETRY ANALYZER
================================================================================
  Zero-boilerplate CLI for analyzing Pi 4 SQLite flight recordings (.db) by
  humans and AI agents. Works on both v1 and v2 recording schemas.

  Usage:
    python tools/ai_debug_suite.py list
    python tools/ai_debug_suite.py digest [db]
    python tools/ai_debug_suite.py episodes [db] [--category ...] [--severity ...]
    python tools/ai_debug_suite.py window [db] <start> <end> [--by-time] [--stride N]
    python tools/ai_debug_suite.py window [db] --around <frame> [--radius 15]
    python tools/ai_debug_suite.py frame [db] <frame_id> [--no-radar] [--raw]
    python tools/ai_debug_suite.py query [db] [--mode ...] [--shield] [--max-tof ...]
    python tools/ai_debug_suite.py compare <db_a> <db_b>
    python tools/ai_debug_suite.py export [db] [--out-dir ...] [--formats md,json,csv,npz]
    python tools/ai_debug_suite.py tag [db] <start_frame> <end_frame> <tag> [--note ...]
    python tools/ai_debug_suite.py labels [db]
    python tools/ai_debug_suite.py pull [--host ...] [--digest] [--export]
================================================================================
"""

from __future__ import annotations

import os
import sys
import json
import argparse
import importlib.util
from pathlib import Path
from typing import Any, List, Optional, Tuple

if sys.platform == "win32" and hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

ROOT_DIR = Path(__file__).resolve().parent.parent
TOOLS_DIR = ROOT_DIR / "tools"
PI4_DIR = ROOT_DIR / "pi4"
for p in (str(ROOT_DIR), str(TOOLS_DIR), str(PI4_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

from debug_recorder import add_label  # noqa: E402
from flight_analyzer import (  # noqa: E402
    FlightSession,
    compare_sessions,
    format_comparison_markdown,
    format_digest_markdown,
    format_episodes_markdown,
    format_frame_markdown,
    format_window_table,
    list_recordings,
    resolve_db_path,
)


def _print_json(obj: Any):
    print(json.dumps(obj, indent=2, ensure_ascii=False))


def _load_debug_studio_module():
    launchers_dir = ROOT_DIR / "launchers"
    matches = sorted(launchers_dir.glob("*pi4_debug_studio.py"))
    if not matches:
        raise FileNotFoundError(f"Could not locate *pi4_debug_studio.py in {launchers_dir}")
    studio_path = matches[0]
    spec = importlib.util.spec_from_file_location("pi4_debug_studio", str(studio_path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _cmd_list(args):
    recs = list_recordings(include_counts=True)
    if args.json:
        _print_json(recs)
        return
    if not recs:
        print("No .db recordings found in Downloads or data/recordings/.")
        return
    print(f"Found {len(recs)} local flight recording(s):")
    print("-" * 106)
    print("  # | File                               | Source          | Schema | Size MB | Frames | Driving(s) | Anom/Clust")
    print("-" * 106)
    for r in recs:
        print(
            f" {r['index']:2d} | {r['name']:<34} | {r['source']:<15} |   v{r['schema_version']}   | "
            f"{r['size_mb']:7.2f} | {r['total_frames']:6d} | {r['driving_seconds']:10.1f} | "
            f"{r['anomalies_count']:4d} / {r['anomaly_clusters']:3d}"
        )
    print("-" * 106)


def _cmd_digest(args):
    db_path = resolve_db_path(args.db)
    sess = FlightSession(db_path)
    digest = sess.build_digest(max_episodes=args.max_episodes)
    text = json.dumps(digest, indent=2, ensure_ascii=False) if args.json else format_digest_markdown(digest)
    if args.out:
        out_p = Path(args.out)
        out_p.parent.mkdir(parents=True, exist_ok=True)
        out_p.write_text(text, encoding="utf-8")
        print(f"[Digest] Saved to {out_p}")
    else:
        print(text)


def _cmd_episodes(args):
    db_path = resolve_db_path(args.db)
    sess = FlightSession(db_path)
    eps = sess.extract_episodes(include_slip=args.include_slip)
    if args.category:
        cat_q = args.category.upper()
        eps = [e for e in eps if cat_q in e["category"].upper()]
    if args.severity:
        sev_q = args.severity.upper()
        eps = [e for e in eps if e["severity"].upper() == sev_q]
    if args.limit > 0:
        eps = eps[: args.limit]
    if args.json:
        _print_json(eps)
    else:
        print(format_episodes_markdown(eps, db_path.name))


def _parse_window_args(args, parser: argparse.ArgumentParser) -> Tuple[Optional[str], float, float]:
    pos: List[str] = list(args.pos or [])
    if args.around is not None:
        if len(pos) > 1:
            parser.error("With --around, provide at most one positional argument: [db].")
        db_spec = pos[0] if len(pos) == 1 else None
        lo = float(args.around) - float(args.radius)
        hi = float(args.around) + float(args.radius)
        return db_spec, lo, hi

    if len(pos) == 2:
        return None, float(pos[0]), float(pos[1])
    if len(pos) == 3:
        return pos[0], float(pos[1]), float(pos[2])
    parser.error("Usage: window [db] <start> <end>  OR  window [db] --around <val> [--radius 15]")
    raise AssertionError("unreachable")


def _cmd_window(args, parser: argparse.ArgumentParser):
    db_spec, start, end = _parse_window_args(args, parser)
    db_path = resolve_db_path(db_spec)
    sess = FlightSession(db_path)
    rows = sess.inspect_window(start, end, by_time=args.by_time, stride=args.stride)
    if args.json:
        _print_json(rows)
    else:
        kind = "sim_time" if args.by_time else "frame_id"
        print(f"# Window [{start}..{end}] ({kind}, stride={args.stride}) — {db_path.name}")
        print(format_window_table(rows))


def _cmd_frame(args, parser: argparse.ArgumentParser):
    pos: List[str] = list(args.pos)
    if len(pos) == 1:
        db_spec, fid = None, int(pos[0])
    elif len(pos) == 2:
        db_spec, fid = pos[0], int(pos[1])
    else:
        parser.error("Usage: frame [db] <frame_id>")
        raise AssertionError("unreachable")

    db_path = resolve_db_path(db_spec)
    sess = FlightSession(db_path)
    if args.raw:
        idx = sess.frame_index_by_id.get(fid)
        if idx is None:
            raise KeyError(f"Frame {fid} not found in {db_path}")
        _print_json(sess.frames[idx]["payload"])
        return

    info = sess.inspect_frame(fid, include_radar=not args.no_radar)
    if args.json:
        _print_json(info)
    else:
        print(format_frame_markdown(info))


def _cmd_query(args):
    db_path = resolve_db_path(args.db)
    sess = FlightSession(db_path)
    rows = sess.query_frames(
        mode=args.mode,
        status=args.status,
        flag=args.flag,
        driving_only=args.driving,
        shield_only=args.shield,
        wrong_way_only=args.wrong_way,
        on_ramp_only=args.on_ramp,
        link_drop_only=args.link_drop,
        min_v=args.min_v,
        max_v=args.max_v,
        max_tof=args.max_tof,
        min_slip=args.min_slip,
        min_plan_ms=args.min_plan_ms,
        min_steer_err=args.min_steer_err,
        frame_range=tuple(args.frame_range) if args.frame_range else None,
        sim_range=tuple(args.sim_range) if args.sim_range else None,
        limit=args.limit,
    )
    if args.json:
        _print_json(rows)
        return
    print(f"# Query Matches ({len(rows)} shown, limit={args.limit}) — {db_path.name}")
    if not rows:
        print("(no matching frames)")
        return
    header = "  fid |  sim_t | mode                    | status                 |  v_cmd / real | str c /  a | f_tof |  slip | plan_ms | shield      | flags"
    print(header)
    print("-" * len(header))
    for r in rows:
        fl = ",".join(r["reasons"]) if r["reasons"] else "-"
        print(
            f"{r['frame_id']:5d} | {r['sim_time']:6.2f} | {r['mode']:<23} | {r['status']:<22} | "
            f"{r['v_cmd']:5.2f} /{r['v_real']:5.2f} | {r['steer_cmd_deg']:5.1f} /{r['steer_actual_deg']:5.1f} | "
            f"{r['f_tof_min']:5.2f} | {r['slip_ratio']:5.2f} | {r['plan_ms']:7.2f} | {r['shield']:<11} | {fl}"
        )


def _cmd_compare(args):
    p_a = resolve_db_path(args.db_a)
    p_b = resolve_db_path(args.db_b)
    comp = compare_sessions(p_a, p_b)
    if args.json:
        _print_json(comp)
    else:
        print(format_comparison_markdown(comp))


def _cmd_export(args):
    db_path = resolve_db_path(args.db)
    sess = FlightSession(db_path)
    formats = [x.strip() for x in args.formats.split(",") if x.strip()]
    outputs = sess.export_bundle(out_dir=args.out_dir, formats=formats)
    _print_json(outputs)


def _cmd_tag(args, parser: argparse.ArgumentParser):
    pos: List[str] = list(args.pos)
    if len(pos) == 3:
        db_spec, s_fid, e_fid, tag_name = None, int(pos[0]), int(pos[1]), pos[2]
    elif len(pos) == 4:
        db_spec, s_fid, e_fid, tag_name = pos[0], int(pos[1]), int(pos[2]), pos[3]
    else:
        parser.error("Usage: tag [db] <start_frame> <end_frame> <tag>")
        raise AssertionError("unreachable")

    db_path = resolve_db_path(db_spec)
    lbl_id = add_label(str(db_path), s_fid, e_fid, tag_name, note=args.note, author=args.author)
    print(f"Added label #{lbl_id}: frames {s_fid}..{e_fid} tag={tag_name!r} in {db_path.name}")


def _cmd_labels(args):
    db_path = resolve_db_path(args.db)
    sess = FlightSession(db_path)
    if args.json:
        _print_json(sess.labels)
        return
    print(f"# Labels in {db_path.name} ({len(sess.labels)}):")
    if not sess.labels:
        print("(none)")
        return
    for lbl in sess.labels:
        print(
            f"  [{lbl['label_id']}] frames {lbl['start_frame']}..{lbl['end_frame']} | "
            f"{lbl['tag']} (by {lbl['author']} @ {lbl['created_at']}) — {lbl['note']}"
        )


def _cmd_pull(args):
    studio = _load_debug_studio_module()
    host, base_url = studio.resolve_pi_host(preferred_host=args.host)
    if not base_url:
        print("[Pull ERROR] Pi 4 Debug Server unreachable.")
        sys.exit(1)
    print(f"[Pull] Connected to Pi 4 at {host} ({base_url})")
    saved_path = studio.download_database(base_url, output_path=args.out)
    if not saved_path:
        sys.exit(1)

    if args.digest:
        sess = FlightSession(saved_path)
        print("")
        print(format_digest_markdown(sess.build_digest()))
    if args.export:
        sess = FlightSession(saved_path)
        outputs = sess.export_bundle()
        print("\n[Export]", json.dumps(outputs, indent=2))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Folkrace AI Flight Debugging Suite & Telemetry Analyzer"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list", help="List available local .db flight recordings")
    p_list.add_argument("--json", action="store_true", help="Output JSON")

    p_dig = sub.add_parser("digest", help="Generate compact AI debugging briefing")
    p_dig.add_argument("db", nargs="?", default=None, help="Path, index, or 'latest'")
    p_dig.add_argument("--max-episodes", type=int, default=15, help="Max episodes to show")
    p_dig.add_argument("--out", default=None, help="Optional output file path")
    p_dig.add_argument("--json", action="store_true", help="Output JSON")

    p_ep = sub.add_parser("episodes", help="List grouped incident episodes")
    p_ep.add_argument("db", nargs="?", default=None, help="Path, index, or 'latest'")
    p_ep.add_argument("--category", default=None, help="Filter category substring (e.g. SHIELD, STALL, LINK_DROP)")
    p_ep.add_argument("--severity", default=None, help="Filter severity (CRITICAL, WARN, INFO)")
    p_ep.add_argument("--include-slip", action="store_true", help="Include |slip_ratio|>0.35 episodes")
    p_ep.add_argument("--limit", type=int, default=0, help="Max episodes (0=all)")
    p_ep.add_argument("--json", action="store_true", help="Output JSON")

    p_win = sub.add_parser("window", help="Inspect compact frame table over [start..end] or --around")
    p_win.add_argument("pos", nargs="*", help="[db] <start> <end>  (or [db] with --around)")
    p_win.add_argument("--around", type=float, default=None, help="Center frame_id (or sim_time with --by-time)")
    p_win.add_argument("--radius", type=float, default=15.0, help="Window half-width around --around")
    p_win.add_argument("--by-time", action="store_true", help="Interpret range as sim_time seconds")
    p_win.add_argument("--stride", type=int, default=1, help="Subsample stride")
    p_win.add_argument("--json", action="store_true", help="Output JSON")

    p_fr = sub.add_parser("frame", help="Inspect single frame details and ASCII radar")
    p_fr.add_argument("pos", nargs="+", help="[db] <frame_id>")
    p_fr.add_argument("--no-radar", action="store_true", help="Omit top-down ASCII radar")
    p_fr.add_argument("--raw", action="store_true", help="Print raw decompressed JSON payload")
    p_fr.add_argument("--json", action="store_true", help="Output JSON")

    p_q = sub.add_parser("query", help="Filter telemetry frames by condition")
    p_q.add_argument("db", nargs="?", default=None, help="Path, index, or 'latest'")
    p_q.add_argument("--mode", default=None, help="Mode substring")
    p_q.add_argument("--status", default=None, help="Planner status substring")
    p_q.add_argument("--flag", default=None, help="Anomaly reason substring (e.g. SHIELD, PROXIMITY, SLIP)")
    p_q.add_argument("--driving", action="store_true", help="Only is_driving=1 frames")
    p_q.add_argument("--shield", action="store_true", help="Only shield-active frames")
    p_q.add_argument("--wrong-way", action="store_true", help="Only wrong-way frames")
    p_q.add_argument("--on-ramp", action="store_true", help="Only is_on_ramp frames")
    p_q.add_argument("--link-drop", action="store_true", help="Only frames with offline MCU/STM32/IMU")
    p_q.add_argument("--min-v", type=float, default=None, help="Min v_real (m/s)")
    p_q.add_argument("--max-v", type=float, default=None, help="Max v_real (m/s)")
    p_q.add_argument("--max-tof", type=float, default=None, help="Max f_tof_min (m)")
    p_q.add_argument("--min-slip", type=float, default=None, help="Min |slip_ratio|")
    p_q.add_argument("--min-plan-ms", type=float, default=None, help="Min plan_ms")
    p_q.add_argument("--min-steer-err", type=float, default=None, help="Min |steer_actual - steer_cmd| deg")
    p_q.add_argument("--frame-range", nargs=2, type=int, metavar=("START", "END"), default=None)
    p_q.add_argument("--sim-range", nargs=2, type=float, metavar=("T0", "T1"), default=None)
    p_q.add_argument("--limit", type=int, default=50, help="Max rows")
    p_q.add_argument("--json", action="store_true", help="Output JSON")

    p_cmp = sub.add_parser("compare", help="A/B compare two flight recordings")
    p_cmp.add_argument("db_a", help="Baseline recording A (path, index, or 'latest')")
    p_cmp.add_argument("db_b", help="Candidate recording B (path, index, or 'latest')")
    p_cmp.add_argument("--json", action="store_true", help="Output JSON")

    p_exp = sub.add_parser("export", help="Export AI/ML bundle (md, json, csv, npz, parquet)")
    p_exp.add_argument("db", nargs="?", default=None, help="Path, index, or 'latest'")
    p_exp.add_argument("--out-dir", default=None, help="Destination folder (default data/ai_exports/<stem>/)")
    p_exp.add_argument("--formats", default="md,json,csv,npz,parquet", help="Comma-separated formats")

    p_tag = sub.add_parser("tag", help="Attach persistent frame-range label to .db")
    p_tag.add_argument("pos", nargs="+", help="[db] <start_frame> <end_frame> <tag>")
    p_tag.add_argument("--note", default="", help="Optional note")
    p_tag.add_argument("--author", default="ai", help="Author tag (default: ai)")

    p_lbl = sub.add_parser("labels", help="List stored labels in .db")
    p_lbl.add_argument("db", nargs="?", default=None, help="Path, index, or 'latest'")
    p_lbl.add_argument("--json", action="store_true", help="Output JSON")

    p_pull = sub.add_parser("pull", help="Download active flight .db from Pi 4")
    p_pull.add_argument("--host", default=None, help="Pi 4 hostname or IP")
    p_pull.add_argument("--out", default=None, help="Output .db path")
    p_pull.add_argument("--digest", action="store_true", help="Print AI digest immediately after download")
    p_pull.add_argument("--export", action="store_true", help="Export AI/ML bundle after download")

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    cmd = args.command
    if cmd == "list":
        _cmd_list(args)
    elif cmd == "digest":
        _cmd_digest(args)
    elif cmd == "episodes":
        _cmd_episodes(args)
    elif cmd == "window":
        _cmd_window(args, parser)
    elif cmd == "frame":
        _cmd_frame(args, parser)
    elif cmd == "query":
        _cmd_query(args)
    elif cmd == "compare":
        _cmd_compare(args)
    elif cmd == "export":
        _cmd_export(args)
    elif cmd == "tag":
        _cmd_tag(args, parser)
    elif cmd == "labels":
        _cmd_labels(args)
    elif cmd == "pull":
        _cmd_pull(args)


if __name__ == "__main__":
    main()
