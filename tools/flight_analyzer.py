"""
================================================================================
  FOLKRACE AI FLIGHT DEBUGGING & ANALYSIS ENGINE
================================================================================
  Offline telemetry ingestion, diagnostic auditing, incident episode clustering,
  ASCII top-down ToF/MPC radar rendering, A/B run comparison, and ML/AI export.

  Backward-compatible with both v1 (32-col frames) and v2 (40-col + events/labels)
  SQLite flight databases produced by pi4/debug_recorder.py.
================================================================================
"""

from __future__ import annotations

import os
import sys
import csv
import json
import math
import zlib
import sqlite3
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

TOOLS_DIR = Path(__file__).resolve().parent
ROOT_DIR = TOOLS_DIR.parent
PI4_DIR = ROOT_DIR / "pi4"
DOWNLOADS_DIR = Path.home() / "Downloads"
RECORDINGS_DIR = ROOT_DIR / "data" / "recordings"
AI_EXPORTS_DIR = ROOT_DIR / "data" / "ai_exports"

if str(PI4_DIR) not in sys.path:
    sys.path.insert(0, str(PI4_DIR))

from debug_recorder import (  # noqa: E402
    AnomalyFlag,
    TelemetryLogReader,
    _classify_log_severity,
    _decode_anomaly_flags,
    _extract_f_tof_min,
    _open_sqlite_ro,
    add_label,
)

# Exact 16-channel cross-eyed VL53L8CX mounting geometry from brain/tof_gap_scout.py
# Format: (sensor_name, mount_x_m, mount_y_m, heading_rad)  [+X right, +Y forward]
FRONT_TOF_SENSOR_RIG: List[Tuple[str, float, float, float]] = [
    ("tof_front_left_4.1deg",   -0.058, 0.092, 1.5512),
    ("tof_front_left_15.4deg",  -0.058, 0.092, 1.3548),
    ("tof_front_left_26.6deg",  -0.058, 0.092, 1.1585),
    ("tof_front_left_37.9deg",  -0.058, 0.092, 0.9621),
    ("tof_front_left_49.1deg",  -0.057, 0.090, 0.7658),
    ("tof_front_left_60.4deg",  -0.057, 0.090, 0.5694),
    ("tof_front_left_71.6deg",  -0.057, 0.090, 0.3731),
    ("tof_front_left_82.9deg",  -0.057, 0.090, 0.1767),
    ("tof_front_right_4.1deg",   0.058, 0.092, 1.5904),
    ("tof_front_right_15.4deg",  0.058, 0.092, 1.7868),
    ("tof_front_right_26.6deg",  0.058, 0.092, 1.9831),
    ("tof_front_right_37.9deg",  0.058, 0.092, 2.1795),
    ("tof_front_right_49.1deg",  0.057, 0.090, 2.3758),
    ("tof_front_right_60.4deg",  0.057, 0.090, 2.5722),
    ("tof_front_right_71.6deg",  0.057, 0.090, 2.7685),
    ("tof_front_right_82.9deg",  0.057, 0.090, 2.9649),
]

TOF_SENSOR_NAMES = [s[0] for s in FRONT_TOF_SENSOR_RIG]
_SEVERITY_RANK = {"CRITICAL": 0, "WARN": 1, "INFO": 2}
_EMERGENCY_KEYWORDS = ("PIVOT", "REVERSE", "DECEL", "STUCK", "EMERGENCY")
_IDLE_MODES = ("STOPPED", "STANDBY", "OFFLINE", "IDLE", "WARMUP", "PAUSED")


def _safe_float(val: Any, default: float = 0.0) -> float:
    if val is None:
        return default
    try:
        f = float(val)
        return f if math.isfinite(f) else default
    except (TypeError, ValueError):
        return default


def _safe_int(val: Any, default: int = 0) -> int:
    if val is None:
        return default
    try:
        return int(val)
    except (TypeError, ValueError):
        return default


def _percentile_summary(arr: np.ndarray) -> Dict[str, float]:
    """Computes compact min/mean/p95/max summary for finite values."""
    if arr.size == 0:
        return {"min": 0.0, "mean": 0.0, "p95": 0.0, "max": 0.0}
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        return {"min": 0.0, "mean": 0.0, "p95": 0.0, "max": 0.0}
    return {
        "min": round(float(np.min(finite)), 3),
        "mean": round(float(np.mean(finite)), 3),
        "p95": round(float(np.percentile(finite, 95)), 3),
        "max": round(float(np.max(finite)), 3),
    }


def _count_contiguous_clusters(frame_ids: Sequence[int], max_gap: int = 12) -> int:
    if not frame_ids:
        return 0
    clusters = 1
    prev = frame_ids[0]
    for fid in frame_ids[1:]:
        if fid - prev > max_gap + 1:
            clusters += 1
        prev = fid
    return clusters


def list_recordings(
    directories: Sequence[Path] = (DOWNLOADS_DIR, RECORDINGS_DIR),
    include_counts: bool = True,
) -> List[Dict[str, Any]]:
    """Lists local .db recordings across Downloads and data/recordings/, newest first."""
    seen = set()
    candidates: List[Path] = []
    for d in directories:
        if not d.exists():
            continue
        for p in d.glob("*.db"):
            rp = p.resolve()
            if rp not in seen:
                seen.add(rp)
                candidates.append(p)
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)

    results: List[Dict[str, Any]] = []
    for idx, p in enumerate(candidates, 1):
        st = p.stat()
        source = "Downloads" if p.parent.resolve() == DOWNLOADS_DIR.resolve() else "data/recordings"
        entry: Dict[str, Any] = {
            "index": idx,
            "path": str(p),
            "name": p.name,
            "source": source,
            "size_mb": round(st.st_size / (1024 * 1024), 2),
            "mtime": st.st_mtime,
            "schema_version": 1,
            "total_frames": 0,
            "driving_seconds": 0.0,
            "anomalies_count": 0,
            "anomaly_clusters": 0,
        }
        if include_counts:
            try:
                reader = TelemetryLogReader(str(p))
                summary = reader.get_summary()
                entry["schema_version"] = summary.get("schema_version", 1)
                entry["total_frames"] = summary.get("total_frames", 0)
                entry["driving_seconds"] = summary.get("driving_seconds", 0.0)
                entry["anomalies_count"] = summary.get("anomalies_count", 0)
                reader.close()

                conn = _open_sqlite_ro(str(p))
                anom_ids = [
                    r[0] for r in conn.execute(
                        "SELECT frame_id FROM frames WHERE anomaly_flags > 0 ORDER BY frame_id ASC;"
                    ).fetchall()
                ]
                conn.close()
                entry["anomaly_clusters"] = _count_contiguous_clusters(anom_ids, max_gap=12)
            except Exception:
                pass
        results.append(entry)
    return results


def resolve_db_path(spec: Optional[str]) -> Path:
    """Resolves a recording path from explicit path, 'latest', None, or 1-based index."""
    if spec is None or str(spec).strip().lower() in ("", "latest"):
        recs = list_recordings(include_counts=False)
        if not recs:
            raise FileNotFoundError(
                f"No .db recordings found in {DOWNLOADS_DIR} or {RECORDINGS_DIR}."
            )
        return Path(recs[0]["path"])

    s = str(spec).strip()
    if s.isdigit():
        recs = list_recordings(include_counts=False)
        idx = int(s) - 1
        if 0 <= idx < len(recs):
            return Path(recs[idx]["path"])
        raise FileNotFoundError(f"Recording index {s} out of range (1..{len(recs)}).")

    p = Path(s)
    if p.exists():
        return p
    for d in (RECORDINGS_DIR, DOWNLOADS_DIR):
        cand = d / s
        if cand.exists():
            return cand
    raise FileNotFoundError(f"Recording database not found: {spec}")


class FlightSession:
    """
    Loads a v1 or v2 SQLite flight recording into normalized frames and typed NumPy arrays
    for fast diagnostics, incident clustering, visualization, comparison, and export.
    """

    def __init__(self, db_path: str | Path):
        self.db_path = str(db_path)
        if not os.path.exists(self.db_path):
            raise FileNotFoundError(f"Flight database not found: {self.db_path}")

        reader = TelemetryLogReader(self.db_path)
        self.session_info = reader.get_session_info()
        self.summary_raw = reader.get_summary()
        self.labels = reader.get_labels()
        self.raw_events = reader.get_events(limit=5000)
        reader.close()

        self.schema_version = int(self.session_info.get("schema_version") or 1)
        self.git_commit = str(self.session_info.get("git_commit") or "")
        self.config = self.session_info.get("config") or {}

        self.frames: List[Dict[str, Any]] = []
        self.frame_index_by_id: Dict[int, int] = {}
        self.mode_transitions: List[Dict[str, Any]] = []
        self.mcu_logs: List[Dict[str, Any]] = []

        self._load_frames()
        self._build_arrays()
        self._build_timelines()

    def _load_frames(self):
        conn = _open_sqlite_ro(self.db_path)
        try:
            cur = conn.cursor()
            cols = [r[1] for r in cur.execute("PRAGMA table_info(frames);").fetchall()]
            col_set = set(cols)
            base_cols = [
                "frame_id", "timestamp", "sim_time", "driving_time", "is_driving", "mode",
                "v_cmd", "v_real", "steer_cmd_deg", "steer_actual_deg",
                "fl_rpm_cmd", "fr_rpm_cmd", "rl_rpm_cmd", "rr_rpm_cmd",
                "fl_rpm_act", "fr_rpm_act", "rl_rpm_act", "rr_rpm_act",
                "yaw_deg", "pitch_deg", "roll_deg", "gyro_z", "accel_fwd", "accel_lat",
                "slip_state", "slip_ratio", "plan_ms", "map_ms", "osqp_iters",
                "f_tof_min", "anomaly_flags", "payload_compressed",
            ]
            v2_cols = [
                "v_tgt", "step_ms", "scout_ms", "servo_pulse_us", "status",
                "shield_active", "is_on_ramp", "is_wrong_way",
            ]
            select_cols = [c for c in base_cols + v2_cols if c in col_set]
            cur.execute(f"SELECT {', '.join(select_cols)} FROM frames ORDER BY frame_id ASC;")
            rows = cur.fetchall()
        finally:
            conn.close()

        for row in rows:
            rec = dict(zip(select_cols, row))
            blob = rec.pop("payload_compressed", None)
            payload: Dict[str, Any] = {}
            if blob:
                try:
                    payload = json.loads(zlib.decompress(blob).decode("utf-8"))
                except Exception:
                    payload = {}

            fid = _safe_int(rec.get("frame_id"))
            mode = str(rec.get("mode") or payload.get("mode") or "UNKNOWN")
            flags_dict = payload.get("flags") if isinstance(payload.get("flags"), dict) else {}
            shield_dict = payload.get("shield") if isinstance(payload.get("shield"), dict) else {}

            tof_raw = payload.get("tof_ranges_raw")
            if not isinstance(tof_raw, list) or len(tof_raw) == 0:
                tof_raw = [0.0] * 16
            tof_clean = payload.get("tof_ranges")
            if not isinstance(tof_clean, list) or len(tof_clean) == 0:
                tof_clean = tof_raw

            tof_raw_16 = [_safe_float(x, 0.0) for x in (tof_raw + [0.0] * 16)[:16]]
            tof_clean_16 = [_safe_float(x, 0.0) for x in (tof_clean + [0.0] * 16)[:16]]

            # Reconstruct f_tof_min accurately even if v1 recorded default 1.0 when f_tof was absent
            f_tof_min = _extract_f_tof_min(payload) if payload else _safe_float(rec.get("f_tof_min"), 1.0)
            if payload.get("f_tof") is None and payload.get("f_tof_raw") is None and rec.get("f_tof_min") is not None:
                db_ftof = _safe_float(rec.get("f_tof_min"), 1.0)
                if db_ftof < f_tof_min and db_ftof > 0.001:
                    f_tof_min = db_ftof

            v_cmd = _safe_float(rec.get("v_cmd"), _safe_float(payload.get("v_cmd")))
            v_real = _safe_float(rec.get("v_real"), _safe_float(payload.get("v_real", payload.get("v_est"))))
            v_tgt = _safe_float(
                rec.get("v_tgt") if "v_tgt" in rec else payload.get("v_tgt"),
                v_cmd,
            )

            shield_active = bool(
                rec["shield_active"]
                if "shield_active" in rec and rec["shield_active"] is not None
                else (shield_dict.get("active") or shield_dict.get("reverse_suggested"))
            )
            is_on_ramp = bool(
                rec["is_on_ramp"]
                if "is_on_ramp" in rec and rec["is_on_ramp"] is not None
                else payload.get("is_on_ramp")
            )
            is_wrong_way = bool(
                rec["is_wrong_way"]
                if "is_wrong_way" in rec and rec["is_wrong_way"] is not None
                else payload.get("is_wrong_way")
            )

            mcu_main_conn = payload.get("mcu_main_connected")
            mcu_tof_conn = payload.get("mcu_tof_connected")
            stm32_front = flags_dict.get("stm32_front")
            stm32_rear = flags_dict.get("stm32_rear")
            imu_online = flags_dict.get("imu_online")
            override_active = bool(flags_dict.get("override_active", False))
            fake_data = bool(flags_dict.get("fake_data", False))
            planner_running = bool(
                payload.get("planner_running")
                if "planner_running" in payload
                else flags_dict.get("planner_running", mode not in ("STOPPED", "STANDBY", "OFFLINE"))
            )

            raw_flags = _safe_int(rec.get("anomaly_flags"))
            eff_flags = raw_flags
            if shield_active:
                eff_flags |= AnomalyFlag.SHIELD
            if is_wrong_way:
                eff_flags |= AnomalyFlag.WRONG_WAY
            if (
                planner_running or mode not in _IDLE_MODES
            ) and (
                mcu_main_conn is False
                or mcu_tof_conn is False
                or stm32_front is False
                or stm32_rear is False
                or imu_online is False
            ):
                eff_flags |= AnomalyFlag.LINK_DROP
            if (
                mode not in ("STOPPED", "STANDBY", "OFFLINE")
                and mcu_tof_conn is not False
                and 0.001 < f_tof_min < 0.12
            ):
                eff_flags |= AnomalyFlag.PROXIMITY

            plan_ms = _safe_float(rec.get("plan_ms"), _safe_float(payload.get("plan_ms")))
            step_ms = _safe_float(rec.get("step_ms") if "step_ms" in rec else payload.get("step_ms"))
            if plan_ms > 16.6 or step_ms > 16.6:
                eff_flags |= AnomalyFlag.LATENCY_SPIKE

            frame_entry: Dict[str, Any] = {
                "frame_id": fid,
                "timestamp": _safe_float(rec.get("timestamp")),
                "sim_time": _safe_float(rec.get("sim_time")),
                "driving_time": _safe_float(rec.get("driving_time")),
                "is_driving": int(_safe_int(rec.get("is_driving"))),
                "mode": mode,
                "status": str(rec.get("status") if "status" in rec and rec["status"] is not None else (payload.get("status") or "")),
                "planner_running": planner_running,
                "v_cmd": v_cmd,
                "v_tgt": v_tgt,
                "v_real": v_real,
                "steer_cmd_deg": _safe_float(rec.get("steer_cmd_deg")),
                "steer_actual_deg": _safe_float(rec.get("steer_actual_deg")),
                "fl_rpm_cmd": _safe_float(rec.get("fl_rpm_cmd")),
                "fr_rpm_cmd": _safe_float(rec.get("fr_rpm_cmd")),
                "rl_rpm_cmd": _safe_float(rec.get("rl_rpm_cmd")),
                "rr_rpm_cmd": _safe_float(rec.get("rr_rpm_cmd")),
                "fl_rpm_act": _safe_float(rec.get("fl_rpm_act")),
                "fr_rpm_act": _safe_float(rec.get("fr_rpm_act")),
                "rl_rpm_act": _safe_float(rec.get("rl_rpm_act")),
                "rr_rpm_act": _safe_float(rec.get("rr_rpm_act")),
                "yaw_deg": _safe_float(rec.get("yaw_deg")),
                "pitch_deg": _safe_float(rec.get("pitch_deg")),
                "roll_deg": _safe_float(rec.get("roll_deg")),
                "gyro_z": _safe_float(rec.get("gyro_z")),
                "accel_fwd": _safe_float(rec.get("accel_fwd")),
                "accel_lat": _safe_float(rec.get("accel_lat")),
                "slip_state": str(rec.get("slip_state") or "NORMAL"),
                "slip_ratio": _safe_float(rec.get("slip_ratio")),
                "plan_ms": plan_ms,
                "step_ms": step_ms,
                "map_ms": _safe_float(rec.get("map_ms")),
                "scout_ms": _safe_float(rec.get("scout_ms") if "scout_ms" in rec else payload.get("scout_ms")),
                "opt_prim_ms": _safe_float(payload.get("opt_prim_ms")),
                "opt_alt_ms": _safe_float(payload.get("opt_alt_ms")),
                "osqp_iters": _safe_int(rec.get("osqp_iters")),
                "servo_pulse_us": _safe_int(
                    rec.get("servo_pulse_us") if "servo_pulse_us" in rec else payload.get("servo_pulse_us"),
                    1510,
                ),
                "f_tof_min": f_tof_min,
                "raw_anomaly_flags": raw_flags,
                "anomaly_flags": eff_flags,
                "anomaly_reasons": _decode_anomaly_flags(eff_flags),
                "shield_active": shield_active,
                "shield_chosen": str(shield_dict.get("chosen_name") or ""),
                "shield_reverse": bool(shield_dict.get("reverse_suggested", False)),
                "is_on_ramp": is_on_ramp,
                "is_wrong_way": is_wrong_way,
                "recovery_state": str(payload.get("recovery_state") or ""),
                "mcu_main_connected": True if mcu_main_conn is None else bool(mcu_main_conn),
                "mcu_tof_connected": True if mcu_tof_conn is None else bool(mcu_tof_conn),
                "stm32_front": True if stm32_front is None else bool(stm32_front),
                "stm32_rear": True if stm32_rear is None else bool(stm32_rear),
                "imu_online": True if imu_online is None else bool(imu_online),
                "override_active": override_active,
                "fake_data": fake_data,
                "tof_ranges": tof_clean_16,
                "tof_ranges_raw": tof_raw_16,
                "gap": payload.get("gap") if isinstance(payload.get("gap"), list) else [],
                "pose": payload.get("pose") if isinstance(payload.get("pose"), list) else [],
                "best_line": payload.get("best_line") if isinstance(payload.get("best_line"), list) else [],
                "corridor_left": payload.get("corridor_left") if isinstance(payload.get("corridor_left"), list) else [],
                "corridor_right": payload.get("corridor_right") if isinstance(payload.get("corridor_right"), list) else [],
                "targets": payload.get("targets") if isinstance(payload.get("targets"), list) else [],
                "payload": payload,
            }
            self.frame_index_by_id[fid] = len(self.frames)
            self.frames.append(frame_entry)

    def _build_arrays(self):
        n = len(self.frames)
        self.n_frames = n
        if n == 0:
            self.frame_ids = np.zeros(0, dtype=np.int64)
            self.timestamps = np.zeros(0, dtype=np.float64)
            self.sim_times = np.zeros(0, dtype=np.float64)
            self.is_driving = np.zeros(0, dtype=np.int8)
            self.v_cmd = np.zeros(0, dtype=np.float64)
            self.v_tgt = np.zeros(0, dtype=np.float64)
            self.v_real = np.zeros(0, dtype=np.float64)
            self.steer_cmd = np.zeros(0, dtype=np.float64)
            self.steer_act = np.zeros(0, dtype=np.float64)
            self.rpms_cmd = np.zeros((0, 4), dtype=np.float64)
            self.rpms_act = np.zeros((0, 4), dtype=np.float64)
            self.pitch_deg = np.zeros(0, dtype=np.float64)
            self.roll_deg = np.zeros(0, dtype=np.float64)
            self.gyro_z = np.zeros(0, dtype=np.float64)
            self.accel_fwd = np.zeros(0, dtype=np.float64)
            self.accel_lat = np.zeros(0, dtype=np.float64)
            self.slip_ratio = np.zeros(0, dtype=np.float64)
            self.plan_ms = np.zeros(0, dtype=np.float64)
            self.step_ms = np.zeros(0, dtype=np.float64)
            self.map_ms = np.zeros(0, dtype=np.float64)
            self.scout_ms = np.zeros(0, dtype=np.float64)
            self.opt_prim_ms = np.zeros(0, dtype=np.float64)
            self.opt_alt_ms = np.zeros(0, dtype=np.float64)
            self.f_tof_min = np.zeros(0, dtype=np.float64)
            self.tof_ranges = np.zeros((0, 16), dtype=np.float64)
            self.tof_ranges_raw = np.zeros((0, 16), dtype=np.float64)
            self.shield_active = np.zeros(0, dtype=bool)
            self.is_on_ramp = np.zeros(0, dtype=bool)
            self.is_wrong_way = np.zeros(0, dtype=bool)
            self.planner_running = np.zeros(0, dtype=bool)
            return

        self.frame_ids = np.array([f["frame_id"] for f in self.frames], dtype=np.int64)
        self.timestamps = np.array([f["timestamp"] for f in self.frames], dtype=np.float64)
        self.sim_times = np.array([f["sim_time"] for f in self.frames], dtype=np.float64)
        self.is_driving = np.array([f["is_driving"] for f in self.frames], dtype=np.int8)
        self.v_cmd = np.array([f["v_cmd"] for f in self.frames], dtype=np.float64)
        self.v_tgt = np.array([f["v_tgt"] for f in self.frames], dtype=np.float64)
        self.v_real = np.array([f["v_real"] for f in self.frames], dtype=np.float64)
        self.steer_cmd = np.array([f["steer_cmd_deg"] for f in self.frames], dtype=np.float64)
        self.steer_act = np.array([f["steer_actual_deg"] for f in self.frames], dtype=np.float64)
        self.rpms_cmd = np.array(
            [[f["fl_rpm_cmd"], f["fr_rpm_cmd"], f["rl_rpm_cmd"], f["rr_rpm_cmd"]] for f in self.frames],
            dtype=np.float64,
        )
        self.rpms_act = np.array(
            [[f["fl_rpm_act"], f["fr_rpm_act"], f["rl_rpm_act"], f["rr_rpm_act"]] for f in self.frames],
            dtype=np.float64,
        )
        self.pitch_deg = np.array([f["pitch_deg"] for f in self.frames], dtype=np.float64)
        self.roll_deg = np.array([f["roll_deg"] for f in self.frames], dtype=np.float64)
        self.gyro_z = np.array([f["gyro_z"] for f in self.frames], dtype=np.float64)
        self.accel_fwd = np.array([f["accel_fwd"] for f in self.frames], dtype=np.float64)
        self.accel_lat = np.array([f["accel_lat"] for f in self.frames], dtype=np.float64)
        self.slip_ratio = np.array([f["slip_ratio"] for f in self.frames], dtype=np.float64)
        self.plan_ms = np.array([f["plan_ms"] for f in self.frames], dtype=np.float64)
        self.step_ms = np.array([f["step_ms"] for f in self.frames], dtype=np.float64)
        self.map_ms = np.array([f["map_ms"] for f in self.frames], dtype=np.float64)
        self.scout_ms = np.array([f["scout_ms"] for f in self.frames], dtype=np.float64)
        self.opt_prim_ms = np.array([f["opt_prim_ms"] for f in self.frames], dtype=np.float64)
        self.opt_alt_ms = np.array([f["opt_alt_ms"] for f in self.frames], dtype=np.float64)
        self.f_tof_min = np.array([f["f_tof_min"] for f in self.frames], dtype=np.float64)
        self.tof_ranges = np.array([f["tof_ranges"] for f in self.frames], dtype=np.float64)
        self.tof_ranges_raw = np.array([f["tof_ranges_raw"] for f in self.frames], dtype=np.float64)
        self.shield_active = np.array([f["shield_active"] for f in self.frames], dtype=bool)
        self.is_on_ramp = np.array([f["is_on_ramp"] for f in self.frames], dtype=bool)
        self.is_wrong_way = np.array([f["is_wrong_way"] for f in self.frames], dtype=bool)
        self.planner_running = np.array([f["planner_running"] for f in self.frames], dtype=bool)

    def _build_timelines(self):
        if not self.frames:
            return

        # Mode transitions
        prev_mode: Optional[str] = None
        for f in self.frames:
            m = f["mode"]
            if prev_mode is not None and m != prev_mode:
                self.mode_transitions.append({
                    "frame_id": f["frame_id"],
                    "sim_time": round(f["sim_time"], 3),
                    "from_mode": prev_mode,
                    "to_mode": m,
                })
            prev_mode = m

        # Prefer v2 events table if it contains MCU_LOG rows; otherwise reconstruct from frame payloads
        v2_mcu = [e for e in self.raw_events if e.get("event_type") == "MCU_LOG"]
        if v2_mcu:
            for e in v2_mcu:
                msg = str((e.get("details") or {}).get("msg") or e.get("label") or "").strip()
                if msg:
                    self.mcu_logs.append({
                        "frame_id": _safe_int(e.get("frame_id")),
                        "sim_time": round(_safe_float(e.get("sim_time")), 3),
                        "severity": str(e.get("severity") or "INFO"),
                        "message": msg,
                    })
            return

        seen_window: List[str] = []
        for f in self.frames:
            p = f["payload"]
            candidates: List[str] = []
            raw_log = p.get("mcu_log")
            if isinstance(raw_log, list):
                for item in raw_log:
                    if item is not None:
                        s = str(item).strip()
                        if s:
                            candidates.append(s)
            msg = p.get("mcu_msg")
            if msg is not None:
                s_msg = str(msg).strip()
                if s_msg and s_msg not in candidates:
                    candidates.append(s_msg)

            for s in candidates:
                if s in seen_window:
                    continue
                seen_window.append(s)
                if len(seen_window) > 100:
                    seen_window.pop(0)
                sev = _classify_log_severity(s)
                self.mcu_logs.append({
                    "frame_id": f["frame_id"],
                    "sim_time": round(f["sim_time"], 3),
                    "severity": sev,
                    "message": s,
                })

    def _compute_continuity_and_distance(self) -> Dict[str, Any]:
        if self.n_frames < 2:
            return {
                "wall_duration_s": 0.0,
                "driving_seconds": float(self.summary_raw.get("driving_seconds") or 0.0),
                "effective_hz": 0.0,
                "gap_count": 0,
                "max_gap_s": 0.0,
                "distance_m": 0.0,
            }
        dt = np.diff(self.timestamps)
        pos_dt = dt[dt > 0]
        wall_dur = float(self.timestamps[-1] - self.timestamps[0])
        median_dt = float(np.median(pos_dt)) if pos_dt.size > 0 else (1.0 / 60.0)
        eff_hz = (1.0 / median_dt) if median_dt > 0 else 0.0
        gaps = dt[dt > 0.05]
        dt_clipped = np.clip(dt, 0.0, 0.1)
        drv_mask = self.is_driving[1:] == 1
        dist_m = float(np.sum(np.abs(self.v_real[1:][drv_mask]) * dt_clipped[drv_mask])) if np.any(drv_mask) else 0.0
        drv_s = float(np.sum(dt_clipped[drv_mask])) if np.any(drv_mask) else 0.0
        return {
            "wall_duration_s": round(max(0.0, wall_dur), 2),
            "driving_seconds": round(drv_s, 2),
            "effective_hz": round(eff_hz, 1),
            "gap_count": int(gaps.size),
            "max_gap_s": round(float(np.max(gaps)), 3) if gaps.size > 0 else 0.0,
            "distance_m": round(dist_m, 2),
        }

    def run_diagnostics(self) -> List[Dict[str, Any]]:
        """Runs Folkrace-specific automated health & physics diagnostics."""
        findings: List[Dict[str, Any]] = []
        if self.n_frames == 0:
            return [{
                "severity": "CRITICAL",
                "category": "RECORDING",
                "code": "EMPTY_DB",
                "title": "Database contains zero telemetry frames",
                "evidence": "0 rows in frames table.",
                "sample_frames": [],
            }]

        drv = self.is_driving == 1
        n_drv = int(np.sum(drv))
        active = self.planner_running | drv
        n_act = int(np.sum(active))

        cont = self._compute_continuity_and_distance()

        # 1. Wheel RPM asymmetry & tracking
        if n_drv > 0:
            straight_strict = drv & (np.abs(self.v_cmd) > 0.3) & (np.abs(self.steer_cmd) < 5.0)
            straight_loose = drv & (np.abs(self.v_cmd) > 0.3) & (np.abs(self.steer_cmd) < 12.0)
            if int(np.sum(straight_strict)) >= 20:
                rpm_mask = straight_strict
                scope_note = f"{int(np.sum(rpm_mask))} straight (|steer|<5 deg) frames"
                is_strict_straight = True
            elif int(np.sum(straight_loose)) >= 20:
                rpm_mask = straight_loose
                scope_note = f"{int(np.sum(rpm_mask))} near-straight (|steer|<12 deg) frames"
                is_strict_straight = False
            else:
                rpm_mask = drv
                scope_note = f"{n_drv} driving frames (cornering included)"
                is_strict_straight = False

            wheel_names = ["FL", "FR", "RL", "RR"]
            means = [float(np.mean(np.abs(self.rpms_act[rpm_mask, i]))) for i in range(4)]
            cmd_means = [float(np.mean(np.abs(self.rpms_cmd[rpm_mask, i]))) for i in range(4)]
            maes = [float(np.mean(np.abs(self.rpms_cmd[drv, i] - self.rpms_act[drv, i]))) for i in range(4)]
            med_rpm = float(np.median(means))
            med_cmd = float(np.median(cmd_means))
            med_mae = float(np.median(maes))
            if med_rpm > 10.0:
                lags = [(med_rpm - m) / med_rpm * 100.0 for m in means]
                cmd_lags = [(med_cmd - c) / max(1.0, med_cmd) * 100.0 for c in cmd_means]
                worst_lag_idx = int(np.argmax(lags))
                worst_mae_idx = int(np.argmax(maes))
                worst_lag = lags[worst_lag_idx]
                worst_mae = maes[worst_mae_idx]
                mae_outlier = worst_mae > max(80.0, 2.0 * med_mae)
                straight_uncommanded_lag = (
                    is_strict_straight and worst_lag > 15.0 and cmd_lags[worst_lag_idx] < 10.0
                )
                means_str = ", ".join(f"{w}={m:.0f}" for w, m in zip(wheel_names, means))
                mae_str = ", ".join(f"{w}={e:.0f}" for w, e in zip(wheel_names, maes))
                if mae_outlier or straight_uncommanded_lag:
                    focus_idx = worst_mae_idx if mae_outlier else worst_lag_idx
                    sev = "CRITICAL" if (worst_lag > 25.0 and cmd_lags[worst_lag_idx] < 10.0) or worst_mae > 200.0 else "WARN"
                    findings.append({
                        "severity": sev,
                        "category": "POWERTRAIN",
                        "code": "WHEEL_RPM_ASYMMETRY",
                        "title": (
                            f"{wheel_names[focus_idx]} wheel tracking/RPM anomaly "
                            f"(MAE={maes[focus_idx]:.0f} vs median {med_mae:.0f}, lag={lags[focus_idx]:.1f}%)"
                        ),
                        "evidence": f"Mean |RPM_act| ({scope_note}): [{means_str}] (median={med_rpm:.0f}). Tracking MAE: [{mae_str}].",
                        "sample_frames": [int(f["frame_id"]) for f in np.array(self.frames)[rpm_mask][:3]],
                    })

            # 2. Speed tracking & stalls
            v_err = self.v_real[drv] - self.v_cmd[drv]
            v_rmse = float(np.sqrt(np.mean(v_err ** 2)))
            v_bias = float(np.mean(v_err))
            stall_mask = (np.abs(self.v_cmd) > 0.5) & (np.abs(self.v_real) < 0.15)
            stall_runs = self._find_runs(stall_mask, min_len=15)
            if stall_runs:
                total_stall_frames = sum(e - s + 1 for s, e in stall_runs)
                samples = [int(self.frame_ids[s]) for s, _ in stall_runs[:5]]
                ranges = ", ".join(f"#{self.frame_ids[s]}-#{self.frame_ids[e]}" for s, e in stall_runs[:4])
                findings.append({
                    "severity": "CRITICAL",
                    "category": "POWERTRAIN",
                    "code": "MECHANICAL_STALL",
                    "title": f"{len(stall_runs)} stall episode(s) detected ({total_stall_frames} frames with |v_cmd|>0.5, |v_real|<0.15)",
                    "evidence": f"Ranges: {ranges}. Driving v_real-v_cmd RMSE={v_rmse:.2f} m/s, bias={v_bias:+.2f} m/s.",
                    "sample_frames": samples,
                })
            elif v_rmse > 0.65:
                findings.append({
                    "severity": "WARN",
                    "category": "POWERTRAIN",
                    "code": "SPEED_TRACKING_ERROR",
                    "title": f"Elevated speed tracking error (RMSE {v_rmse:.2f} m/s)",
                    "evidence": f"Mean bias (v_real - v_cmd) = {v_bias:+.2f} m/s across {n_drv} driving frames.",
                    "sample_frames": [],
                })

            # 3. Steering tracking, saturation, oscillation
            str_err = self.steer_act[drv] - self.steer_cmd[drv]
            str_rmse = float(np.sqrt(np.mean(str_err ** 2)))
            sat_pct = float(np.mean(np.abs(self.steer_cmd[drv]) >= 28.0) * 100.0)
            # Count rapid sign flips across +-5 deg
            drv_steer = self.steer_cmd[drv]
            signs = np.where(drv_steer > 5.0, 1, np.where(drv_steer < -5.0, -1, 0))
            nz = signs[signs != 0]
            flips = int(np.sum(nz[1:] != nz[:-1])) if nz.size > 1 else 0
            drv_s = cont["driving_seconds"] if cont["driving_seconds"] > 0.0 else (n_drv / 60.0)
            flip_hz = flips / max(1e-3, drv_s)
            if str_rmse > 5.0 or sat_pct > 25.0 or flip_hz > 2.5:
                findings.append({
                    "severity": "WARN" if str_rmse <= 10.0 else "CRITICAL",
                    "category": "STEERING",
                    "code": "STEERING_STRESS",
                    "title": f"Steering error/saturation (RMSE {str_rmse:.1f} deg, {sat_pct:.1f}% saturated >=28 deg)",
                    "evidence": f"Large steering sign reversals (+/-5 deg): {flips} (~{flip_hz:.2f}/s over {drv_s:.1f}s driving).",
                    "sample_frames": [],
                })

        # 4. Hardware / link health
        link_issues: List[Tuple[str, List[int]]] = []
        checks = [
            ("mcu_main_connected", False, "ESP32 Main link offline"),
            ("mcu_tof_connected", False, "ESP32 ToF Hub link offline"),
            ("stm32_front", False, "STM32 Front axle offline"),
            ("stm32_rear", False, "STM32 Rear axle offline"),
            ("imu_online", False, "BNO086 IMU offline"),
        ]
        for key, bad_val, label in checks:
            fids = [f["frame_id"] for f in self.frames if f[key] is bad_val and (f["planner_running"] or f["is_driving"])]
            if fids:
                link_issues.append((f"{label}: {len(fids)} active frames", fids))
        if link_issues:
            all_samples = sorted({fids[0] for _, fids in link_issues})
            findings.append({
                "severity": "CRITICAL",
                "category": "HARDWARE_LINK",
                "code": "LINK_DROP",
                "title": f"Hardware link outage during active run ({len(link_issues)} subsystem(s))",
                "evidence": "; ".join(txt for txt, _ in link_issues),
                "sample_frames": all_samples[:6],
            })

        override_fids = [f["frame_id"] for f in self.frames if f["override_active"]]
        if override_fids:
            findings.append({
                "severity": "INFO",
                "category": "SESSION",
                "code": "OVERRIDE_ACTIVE",
                "title": f"Manual/GUI override_active flag set on {len(override_fids)}/{self.n_frames} frames",
                "evidence": "Control authority or safety interlocks may reflect manual bench/GUI override.",
                "sample_frames": override_fids[:3],
            })

        fake_fids = [f["frame_id"] for f in self.frames if f["fake_data"]]
        if fake_fids:
            findings.append({
                "severity": "INFO",
                "category": "SESSION",
                "code": "FAKE_DATA",
                "title": f"Synthetic bench fake_data flag active on {len(fake_fids)}/{self.n_frames} frames",
                "evidence": "Recorded values include simulated bench packets.",
                "sample_frames": fake_fids[:3],
            })

        # 5. ToF 16-channel health
        if n_act > 0:
            tof_act = self.tof_ranges_raw[active]
            zero_pcts = np.mean(tof_act <= 0.001, axis=0) * 100.0
            sat_pcts = np.mean(tof_act >= 1.79, axis=0) * 100.0
            dead_chs = [(i, TOF_SENSOR_NAMES[i], float(zero_pcts[i])) for i in range(16) if zero_pcts[i] > 20.0]
            sat_chs = [(i, TOF_SENSOR_NAMES[i], float(sat_pcts[i])) for i in range(16) if sat_pcts[i] > 98.0]
            if dead_chs:
                desc = ", ".join(f"ch{i}({name.replace('tof_front_', '')}: {pct:.0f}%)" for i, name, pct in dead_chs[:6])
                findings.append({
                    "severity": "WARN" if len(dead_chs) < 4 else "CRITICAL",
                    "category": "SENSORS",
                    "code": "TOF_ZERO_CHANNELS",
                    "title": f"{len(dead_chs)} ToF channel(s) stuck at 0.0m for >20% of active frames",
                    "evidence": desc,
                    "sample_frames": [],
                })
            if sat_chs:
                desc = ", ".join(f"ch{i}({name.replace('tof_front_', '')}: {pct:.0f}%)" for i, name, pct in sat_chs[:6])
                findings.append({
                    "severity": "INFO",
                    "category": "SENSORS",
                    "code": "TOF_SATURATED_CHANNELS",
                    "title": f"{len(sat_chs)} ToF channel(s) saturated >=1.79m (>98% of active frames)",
                    "evidence": desc,
                    "sample_frames": [],
                })

        # 6. Planner timing budget
        plan_mask = self.plan_ms > 0.0
        if np.any(plan_mask):
            p_arr = self.plan_ms[plan_mask]
            s_arr = self.step_ms[plan_mask]
            over_plan = self.frame_ids[self.plan_ms > 16.6]
            over_step = self.frame_ids[self.step_ms > 16.6]
            if over_plan.size > 0 or over_step.size > 0:
                findings.append({
                    "severity": "WARN" if over_plan.size < 10 else "CRITICAL",
                    "category": "TIMING",
                    "code": "DEADLINE_OVERRUN",
                    "title": f"60 Hz budget (>16.6ms) exceeded: plan={over_plan.size} frames, step={over_step.size} frames",
                    "evidence": f"plan_ms p95={np.percentile(p_arr, 95):.2f} max={np.max(p_arr):.2f}; step_ms p95={np.percentile(s_arr, 95):.2f} max={np.max(s_arr):.2f}.",
                    "sample_frames": [int(x) for x in (over_plan[:5] if over_plan.size else over_step[:5])],
                })

        # 7. Safety shield, emergency recovery & wrong-way
        if n_drv > 0:
            sh_cnt = int(np.sum(self.shield_active & drv))
            sh_pct = sh_cnt / n_drv * 100.0
            em_cnt = sum(1 for f in self.frames if f["is_driving"] and any(k in f["mode"].upper() for k in _EMERGENCY_KEYWORDS))
            em_pct = em_cnt / n_drv * 100.0
            ww_fids = [f["frame_id"] for f in self.frames if f["is_wrong_way"]]
            if ww_fids:
                findings.append({
                    "severity": "CRITICAL",
                    "category": "NAVIGATION",
                    "code": "WRONG_WAY_TRIGGERED",
                    "title": f"Wrong-way watchdog triggered on {len(ww_fids)} frame(s)",
                    "evidence": f"Frames: {ww_fids[:8]}",
                    "sample_frames": ww_fids[:6],
                })
            if sh_pct > 10.0 or em_pct > 10.0:
                chosen_counts: Dict[str, int] = {}
                for f in self.frames:
                    if f["shield_active"] and f["is_driving"]:
                        cname = f["shield_chosen"] or "unknown"
                        chosen_counts[cname] = chosen_counts.get(cname, 0) + 1
                chosen_txt = ", ".join(f"{k}:{v}" for k, v in sorted(chosen_counts.items(), key=lambda x: -x[1])) or "none"
                findings.append({
                    "severity": "WARN",
                    "category": "NAVIGATION",
                    "code": "HIGH_INTERVENTION_RATE",
                    "title": f"Frequent safety intervention: shield={sh_pct:.1f}% ({sh_cnt}), recovery/pivot={em_pct:.1f}% ({em_cnt})",
                    "evidence": f"Shield chosen modes: {chosen_txt}.",
                    "sample_frames": [],
                })

            # 8. Ramp detector bias check
            ramp_cnt = int(np.sum(self.is_on_ramp & drv))
            ramp_pct = ramp_cnt / n_drv * 100.0
            mean_abs_pitch = float(np.mean(np.abs(self.pitch_deg[drv])))
            mean_pitch = float(np.mean(self.pitch_deg[drv]))
            if ramp_pct > 40.0 and mean_abs_pitch < 5.0:
                findings.append({
                    "severity": "WARN",
                    "category": "SENSORS",
                    "code": "RAMP_FALSE_POSITIVE_BIAS",
                    "title": f"is_on_ramp=True for {ramp_pct:.1f}% of driving despite flat mean |pitch|={mean_abs_pitch:.2f} deg",
                    "evidence": f"pitch_deg min={np.min(self.pitch_deg[drv]):.2f}, mean={mean_pitch:.2f}, max={np.max(self.pitch_deg[drv]):.2f} deg. Check IMU pitch zero/tare or ramp threshold.",
                    "sample_frames": [],
                })

            # 9. Slip ratio vs slip_state calibration
            slip_state_cnt = sum(1 for f in self.frames if f["is_driving"] and f["slip_state"].upper() != "NORMAL")
            slip_ratio_cnt = int(np.sum((np.abs(self.slip_ratio) > 0.35) & drv))
            if slip_ratio_cnt > max(15, int(0.15 * n_drv)) and slip_state_cnt < max(3, int(0.02 * n_drv)):
                findings.append({
                    "severity": "INFO",
                    "category": "DYNAMICS",
                    "code": "SLIP_RATIO_NOISE_MISMATCH",
                    "title": f"|slip_ratio|>0.35 on {slip_ratio_cnt}/{n_drv} driving frames ({slip_ratio_cnt/n_drv*100:.1f}%), but slip_state!=NORMAL only {slip_state_cnt} times",
                    "evidence": "0.35 slip_ratio threshold triggers frequently during normal cornering/wheel mismatch; use slip_state or higher threshold for incidents.",
                    "sample_frames": [],
                })

        # 10. Continuity
        if cont["gap_count"] > 0 and cont["max_gap_s"] > 0.5:
            findings.append({
                "severity": "WARN",
                "category": "RECORDING",
                "code": "RECORDING_GAPS",
                "title": f"{cont['gap_count']} timestamp gap(s) >50ms (max {cont['max_gap_s']:.2f}s)",
                "evidence": f"Effective median rate={cont['effective_hz']:.1f} Hz across {self.n_frames} frames.",
                "sample_frames": [],
            })

        findings.sort(key=lambda x: _SEVERITY_RANK.get(x["severity"], 9))
        return findings

    @staticmethod
    def _find_runs(mask: np.ndarray, min_len: int = 1) -> List[Tuple[int, int]]:
        runs: List[Tuple[int, int]] = []
        n = int(mask.size)
        i = 0
        while i < n:
            if not mask[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            if (j - i + 1) >= min_len:
                runs.append((i, j))
            i = j + 1
        return runs

    def extract_episodes(self, gap_frames: int = 12, include_slip: bool = False) -> List[Dict[str, Any]]:
        """Clusters anomalous/incident frames into discrete episodes with lead-in & outcome."""
        if self.n_frames == 0:
            return []

        stall_indices = set()
        stall_mask = (np.abs(self.v_cmd) > 0.5) & (np.abs(self.v_real) < 0.15)
        for s_idx, e_idx in self._find_runs(stall_mask, min_len=10):
            stall_indices.update(range(s_idx, e_idx + 1))

        cat_defs: List[Tuple[str, str]] = [
            ("WRONG_WAY", "CRITICAL"),
            ("LINK_DROP", "CRITICAL"),
            ("EMERGENCY_RECOVERY", "CRITICAL"),
            ("STALL", "CRITICAL"),
            ("NEAR_COLLISION", "WARN"),
            ("SHIELD_EVADE", "WARN"),
            ("LATENCY_SPIKE", "WARN"),
            ("SEVERE_SLIP", "INFO"),
        ]

        cat_indices: Dict[str, List[int]] = {c: [] for c, _ in cat_defs}
        for idx, f in enumerate(self.frames):
            mode_u = f["mode"].upper()
            active = f["planner_running"] or f["is_driving"] == 1
            if f["is_wrong_way"]:
                cat_indices["WRONG_WAY"].append(idx)
            if active and (
                not f["mcu_main_connected"]
                or not f["mcu_tof_connected"]
                or not f["stm32_front"]
                or not f["stm32_rear"]
                or not f["imu_online"]
            ):
                cat_indices["LINK_DROP"].append(idx)
            if any(k in mode_u for k in _EMERGENCY_KEYWORDS):
                cat_indices["EMERGENCY_RECOVERY"].append(idx)
            if idx in stall_indices:
                cat_indices["STALL"].append(idx)
            if (
                mode_u not in ("STOPPED", "STANDBY", "OFFLINE")
                and f["mcu_tof_connected"]
                and 0.001 < f["f_tof_min"] < 0.12
            ):
                cat_indices["NEAR_COLLISION"].append(idx)
            if f["shield_active"]:
                cat_indices["SHIELD_EVADE"].append(idx)
            if f["plan_ms"] > 16.6 or f["step_ms"] > 16.6:
                cat_indices["LATENCY_SPIKE"].append(idx)

            slip_u = f["slip_state"].upper()
            is_state_slip = any(k in slip_u for k in ("OVERSTEER", "UNDERSTEER", "SPIN"))
            is_ratio_slip = include_slip and abs(f["slip_ratio"]) > 0.35
            if is_state_slip or is_ratio_slip:
                cat_indices["SEVERE_SLIP"].append(idx)

        episodes: List[Dict[str, Any]] = []
        for cat, sev in cat_defs:
            idxs = cat_indices[cat]
            if not idxs:
                continue
            groups: List[List[int]] = [[idxs[0]]]
            for idx in idxs[1:]:
                if (self.frames[idx]["frame_id"] - self.frames[groups[-1][-1]]["frame_id"]) <= gap_frames + 1:
                    groups[-1].append(idx)
                else:
                    groups.append([idx])

            for g in groups:
                i0, i1 = g[0], g[-1]
                f0, f1 = self.frames[i0], self.frames[i1]
                sl = slice(i0, i1 + 1)
                modes_visited: List[str] = []
                for fr in self.frames[i0:i1 + 1]:
                    if not modes_visited or modes_visited[-1] != fr["mode"]:
                        modes_visited.append(fr["mode"])

                lead_idx = max(0, i0 - 10)
                out_idx = min(self.n_frames - 1, i1 + 10)
                lead_f = self.frames[lead_idx]
                out_f = self.frames[out_idx]

                # MCU logs within [start_frame - 10, end_frame + 10]
                lo_fid = lead_f["frame_id"]
                hi_fid = out_f["frame_id"]
                ep_logs = [
                    m for m in self.mcu_logs
                    if lo_fid <= m["frame_id"] <= hi_fid
                ][:6]

                nz_sims = [fr["sim_time"] for fr in self.frames[i0:i1 + 1] if fr["sim_time"] > 0.0]
                sim_start = min(nz_sims) if nz_sims else float(f0["sim_time"])
                sim_end = max(nz_sims) if nz_sims else float(f1["sim_time"])
                sim_dur = max(0.0, sim_end - sim_start) if len(nz_sims) >= 2 else 0.0
                wall_dur = max(0.0, float(f1["timestamp"] - f0["timestamp"]))
                if sim_dur > 0.0 and (wall_dur == 0.0 or abs(sim_dur - wall_dur) < max(5.0, wall_dur * 2.0)):
                    dur_s = sim_dur
                elif 0.0 < wall_dur <= max(10.0, (i1 - i0 + 1) * 0.5):
                    dur_s = wall_dur
                else:
                    eff_hz = self._compute_continuity_and_distance()["effective_hz"] or 60.0
                    dur_s = (i1 - i0) / max(1.0, eff_hz)

                episodes.append({
                    "episode_id": "",
                    "category": cat,
                    "severity": sev,
                    "start_frame": f0["frame_id"],
                    "end_frame": f1["frame_id"],
                    "frame_count": i1 - i0 + 1,
                    "trigger_frames": len(g),
                    "duration_s": round(dur_s, 2),
                    "sim_start": round(sim_start, 2),
                    "sim_end": round(sim_end, 2),
                    "modes_visited": modes_visited,
                    "peak_metrics": {
                        "min_f_tof": round(float(np.min(self.f_tof_min[sl])), 3),
                        "min_v_real": round(float(np.min(self.v_real[sl])), 2),
                        "max_v_real": round(float(np.max(self.v_real[sl])), 2),
                        "max_abs_steer": round(float(np.max(np.abs(self.steer_cmd[sl]))), 1),
                        "max_abs_slip": round(float(np.max(np.abs(self.slip_ratio[sl]))), 2),
                        "max_plan_ms": round(float(np.max(self.plan_ms[sl])), 2),
                        "max_step_ms": round(float(np.max(self.step_ms[sl])), 2),
                        "max_abs_gyro_z": round(float(np.max(np.abs(self.gyro_z[sl]))), 2),
                        "max_abs_accel_lat": round(float(np.max(np.abs(self.accel_lat[sl]))), 2),
                    },
                    "lead_in": {
                        "frame_id": lead_f["frame_id"],
                        "mode": lead_f["mode"],
                        "v_real": round(lead_f["v_real"], 2),
                        "steer_deg": round(lead_f["steer_cmd_deg"], 1),
                        "f_tof_min": round(lead_f["f_tof_min"], 2),
                    },
                    "outcome": {
                        "frame_id": out_f["frame_id"],
                        "mode": out_f["mode"],
                        "v_real": round(out_f["v_real"], 2),
                        "steer_deg": round(out_f["steer_cmd_deg"], 1),
                        "f_tof_min": round(out_f["f_tof_min"], 2),
                    },
                    "mcu_logs": ep_logs,
                })

        episodes.sort(key=lambda e: (_SEVERITY_RANK.get(e["severity"], 9), -e["duration_s"], -e["frame_count"]))
        for idx, ep in enumerate(episodes, 1):
            ep["episode_id"] = f"EP{idx:02d}"
        return episodes

    def _compute_kpis(self) -> Dict[str, Any]:
        cont = self._compute_continuity_and_distance()
        if self.n_frames == 0:
            return {"continuity": cont, "modes": {}, "statuses": {}}

        drv = self.is_driving == 1
        drv_sl = drv if np.any(drv) else slice(None)
        active = self.planner_running | drv
        act_sl = active if np.any(active) else slice(None)

        mode_counts: Dict[str, int] = {}
        status_counts: Dict[str, int] = {}
        for f in self.frames:
            mode_counts[f["mode"]] = mode_counts.get(f["mode"], 0) + 1
            st = f["status"] or "NONE"
            status_counts[st] = status_counts.get(st, 0) + 1

        wheel_means = [round(float(np.mean(np.abs(self.rpms_act[drv_sl, i]))), 1) for i in range(4)]
        wheel_maes = [round(float(np.mean(np.abs(self.rpms_cmd[drv_sl, i] - self.rpms_act[drv_sl, i]))), 1) for i in range(4)]

        tof_act = self.tof_ranges_raw[act_sl]
        tof_channels = []
        for i in range(16):
            ch = tof_act[:, i]
            tof_channels.append({
                "ch": i,
                "name": TOF_SENSOR_NAMES[i],
                "min": round(float(np.min(ch)), 3),
                "p10": round(float(np.percentile(ch, 10)), 3),
                "median": round(float(np.median(ch)), 3),
                "max": round(float(np.max(ch)), 3),
                "zero_pct": round(float(np.mean(ch <= 0.001) * 100.0), 1),
                "sat_pct": round(float(np.mean(ch >= 1.79) * 100.0), 1),
            })

        return {
            "continuity": cont,
            "v_cmd": _percentile_summary(self.v_cmd[drv_sl]),
            "v_real": _percentile_summary(self.v_real[drv_sl]),
            "steer_err_rmse": round(float(np.sqrt(np.mean((self.steer_act[drv_sl] - self.steer_cmd[drv_sl]) ** 2))), 2),
            "v_err_rmse": round(float(np.sqrt(np.mean((self.v_real[drv_sl] - self.v_cmd[drv_sl]) ** 2))), 2),
            "rpm_act_abs_mean": {"FL": wheel_means[0], "FR": wheel_means[1], "RL": wheel_means[2], "RR": wheel_means[3]},
            "rpm_tracking_mae": {"FL": wheel_maes[0], "FR": wheel_maes[1], "RL": wheel_maes[2], "RR": wheel_maes[3]},
            "imu": {
                "pitch_deg": _percentile_summary(self.pitch_deg[drv_sl]),
                "roll_deg": _percentile_summary(self.roll_deg[drv_sl]),
                "gyro_z": _percentile_summary(np.abs(self.gyro_z[drv_sl])),
                "accel_lat": _percentile_summary(np.abs(self.accel_lat[drv_sl])),
                "is_on_ramp_pct": round(float(np.mean(self.is_on_ramp[drv_sl]) * 100.0), 1),
            },
            "timing_ms": {
                "plan_ms": _percentile_summary(self.plan_ms[self.plan_ms > 0] if np.any(self.plan_ms > 0) else self.plan_ms),
                "step_ms": _percentile_summary(self.step_ms[self.step_ms > 0] if np.any(self.step_ms > 0) else self.step_ms),
                "map_ms": _percentile_summary(self.map_ms[self.map_ms > 0] if np.any(self.map_ms > 0) else self.map_ms),
                "scout_ms": _percentile_summary(self.scout_ms[self.scout_ms > 0] if np.any(self.scout_ms > 0) else self.scout_ms),
                "opt_prim_ms": _percentile_summary(self.opt_prim_ms[self.opt_prim_ms > 0] if np.any(self.opt_prim_ms > 0) else self.opt_prim_ms),
                "opt_alt_ms": _percentile_summary(self.opt_alt_ms[self.opt_alt_ms > 0] if np.any(self.opt_alt_ms > 0) else self.opt_alt_ms),
            },
            "f_tof_min": _percentile_summary(self.f_tof_min[act_sl]),
            "shield_pct": round(float(np.mean(self.shield_active[drv_sl]) * 100.0), 1),
            "wrong_way_frames": int(np.sum(self.is_wrong_way)),
            "modes": dict(sorted(mode_counts.items(), key=lambda x: -x[1])),
            "statuses": dict(sorted(status_counts.items(), key=lambda x: -x[1])),
            "tof_channels": tof_channels,
        }

    def build_digest(self, max_episodes: int = 15) -> Dict[str, Any]:
        """Builds structured AI debugging briefing for the session."""
        diagnostics = self.run_diagnostics()
        episodes = self.extract_episodes()
        kpis = self._compute_kpis()
        cont = kpis["continuity"]

        return {
            "session": {
                "db_path": self.db_path,
                "filename": os.path.basename(self.db_path),
                "session_id": self.summary_raw.get("session_id", Path(self.db_path).stem),
                "created_at": self.summary_raw.get("created_at", ""),
                "schema_version": self.schema_version,
                "git_commit": self.git_commit,
                "total_frames": self.n_frames,
                "driving_frames": int(np.sum(self.is_driving == 1)),
                "frame_range": [int(self.frame_ids[0]), int(self.frame_ids[-1])] if self.n_frames else [0, 0],
                "wall_duration_s": cont["wall_duration_s"],
                "driving_seconds": cont["driving_seconds"],
                "distance_m": cont["distance_m"],
                "effective_hz": cont["effective_hz"],
                "override_frames": sum(1 for f in self.frames if f["override_active"]),
                "link_drop_frames": sum(
                    1 for f in self.frames
                    if (f["planner_running"] or f["is_driving"]) and (
                        not f["mcu_main_connected"] or not f["mcu_tof_connected"] or not f["stm32_front"] or not f["stm32_rear"] or not f["imu_online"]
                    )
                ),
                "file_size_mb": self.summary_raw.get("file_size_mb", 0.0),
            },
            "diagnostics": diagnostics,
            "kpis": kpis,
            "episodes_total": len(episodes),
            "episodes": episodes[:max_episodes],
            "mode_transitions_count": len(self.mode_transitions),
            "mcu_logs": self.mcu_logs,
            "labels": self.labels,
        }

    def inspect_window(
        self,
        start: float,
        end: float,
        by_time: bool = False,
        stride: int = 1,
    ) -> List[Dict[str, Any]]:
        """Returns compact frame summaries within a frame_id or sim_time window."""
        stride = max(1, int(stride))
        matched: List[Dict[str, Any]] = []
        for f in self.frames:
            key_val = f["sim_time"] if by_time else f["frame_id"]
            if start <= key_val <= end:
                matched.append(f)
        sliced = matched[::stride]

        rows: List[Dict[str, Any]] = []
        for f in sliced:
            gap_str = f"{f['gap'][0]:+.2f}m" if len(f["gap"]) >= 1 else "-"
            sh_str = f["shield_chosen"] or ("ON" if f["shield_active"] else "-")
            if f["shield_active"] and f["shield_reverse"]:
                sh_str += "(REV)"
            rows.append({
                "frame_id": f["frame_id"],
                "sim_time": round(f["sim_time"], 2),
                "mode": f["mode"],
                "v_cmd": round(f["v_cmd"], 2),
                "v_real": round(f["v_real"], 2),
                "steer_cmd_deg": round(f["steer_cmd_deg"], 1),
                "steer_actual_deg": round(f["steer_actual_deg"], 1),
                "f_tof_min": round(f["f_tof_min"], 2),
                "slip_ratio": round(f["slip_ratio"], 2),
                "slip_state": f["slip_state"],
                "plan_ms": round(f["plan_ms"], 2),
                "shield": sh_str,
                "gap": gap_str,
                "reasons": f["anomaly_reasons"],
            })
        return rows

    def inspect_frame(self, frame_id: int, include_radar: bool = True) -> Dict[str, Any]:
        """Returns deep single-frame inspection plus optional 2D ASCII radar."""
        idx = self.frame_index_by_id.get(int(frame_id))
        if idx is None:
            raise KeyError(f"Frame {frame_id} not found in {self.db_path}")
        f = self.frames[idx]
        p = f["payload"]
        shield = p.get("shield") if isinstance(p.get("shield"), dict) else {}

        # Shield branch safety summary
        shield_branches = {}
        for branch in ("k1_left", "k1_right", "k2_left", "k2_right", "straight", "straight_mpc", "straight_zero"):
            s_key = f"{branch}_safe"
            if s_key in shield:
                shield_branches[branch] = bool(shield[s_key])

        # Corridor width samples along best_line (near/mid/far)
        corridor_samples: List[Dict[str, Any]] = []
        cl, cr, bl = f["corridor_left"], f["corridor_right"], f["best_line"]
        n_pts = min(len(cl), len(cr))
        if n_pts > 0:
            for label, pt_idx in (("near", 0), ("mid", n_pts // 2), ("far", n_pts - 1)):
                lx, ly = float(cl[pt_idx][0]), float(cl[pt_idx][1])
                rx, ry = float(cr[pt_idx][0]), float(cr[pt_idx][1])
                width = math.hypot(rx - lx, ry - ly)
                bx, by = (float(bl[pt_idx][0]), float(bl[pt_idx][1])) if pt_idx < len(bl) else (0.0, 0.0)
                corridor_samples.append({
                    "zone": label,
                    "idx": pt_idx,
                    "width_m": round(width, 3),
                    "left": [round(lx, 3), round(ly, 3)],
                    "right": [round(rx, 3), round(ry, 3)],
                    "best": [round(bx, 3), round(by, 3)],
                })

        tof_table = []
        for i, (name, mx, my, hdg) in enumerate(FRONT_TOF_SENSOR_RIG):
            tof_table.append({
                "ch": i,
                "name": name,
                "raw_m": round(f["tof_ranges_raw"][i], 3),
                "clean_m": round(f["tof_ranges"][i], 3),
                "heading_deg": round(math.degrees(hdg), 1),
            })

        out = {
            "frame_id": f["frame_id"],
            "timestamp": f["timestamp"],
            "sim_time": f["sim_time"],
            "driving_time": f["driving_time"],
            "is_driving": bool(f["is_driving"]),
            "mode": f["mode"],
            "status": f["status"],
            "recovery_state": f["recovery_state"],
            "motion": {
                "v_cmd": round(f["v_cmd"], 3),
                "v_tgt": round(f["v_tgt"], 3),
                "v_real": round(f["v_real"], 3),
                "steer_cmd_deg": round(f["steer_cmd_deg"], 2),
                "steer_actual_deg": round(f["steer_actual_deg"], 2),
                "servo_pulse_us": f["servo_pulse_us"],
                "pose": f["pose"],
            },
            "wheels": {
                "FL": {"cmd": round(f["fl_rpm_cmd"], 1), "act": round(f["fl_rpm_act"], 1), "err": round(f["fl_rpm_act"] - f["fl_rpm_cmd"], 1)},
                "FR": {"cmd": round(f["fr_rpm_cmd"], 1), "act": round(f["fr_rpm_act"], 1), "err": round(f["fr_rpm_act"] - f["fr_rpm_cmd"], 1)},
                "RL": {"cmd": round(f["rl_rpm_cmd"], 1), "act": round(f["rl_rpm_act"], 1), "err": round(f["rl_rpm_act"] - f["rl_rpm_cmd"], 1)},
                "RR": {"cmd": round(f["rr_rpm_cmd"], 1), "act": round(f["rr_rpm_act"], 1), "err": round(f["rr_rpm_act"] - f["rr_rpm_cmd"], 1)},
            },
            "imu": {
                "yaw_deg": round(f["yaw_deg"], 2),
                "pitch_deg": round(f["pitch_deg"], 2),
                "roll_deg": round(f["roll_deg"], 2),
                "gyro_z": round(f["gyro_z"], 3),
                "accel_fwd": round(f["accel_fwd"], 3),
                "accel_lat": round(f["accel_lat"], 3),
                "slip_state": f["slip_state"],
                "slip_ratio": round(f["slip_ratio"], 3),
            },
            "timing_ms": {
                "plan_ms": round(f["plan_ms"], 2),
                "step_ms": round(f["step_ms"], 2),
                "map_ms": round(f["map_ms"], 2),
                "scout_ms": round(f["scout_ms"], 2),
                "opt_prim_ms": round(f["opt_prim_ms"], 2),
                "opt_alt_ms": round(f["opt_alt_ms"], 2),
                "osqp_iters": f["osqp_iters"],
            },
            "shield": {
                "active": f["shield_active"],
                "chosen_name": f["shield_chosen"],
                "reverse_suggested": f["shield_reverse"],
                "branches_safe": shield_branches,
            },
            "planner": {
                "gap": f["gap"],
                "targets": f["targets"],
                "corridor_samples": corridor_samples,
                "best_line_len": len(f["best_line"]),
            },
            "links": {
                "mcu_main_connected": f["mcu_main_connected"],
                "mcu_tof_connected": f["mcu_tof_connected"],
                "stm32_front": f["stm32_front"],
                "stm32_rear": f["stm32_rear"],
                "imu_online": f["imu_online"],
                "override_active": f["override_active"],
                "fake_data": f["fake_data"],
                "planner_running": f["planner_running"],
                "is_on_ramp": f["is_on_ramp"],
                "is_wrong_way": f["is_wrong_way"],
            },
            "f_tof_min": round(f["f_tof_min"], 3),
            "anomaly_reasons": f["anomaly_reasons"],
            "tof_channels": tof_table,
            "mcu_msg": str(p.get("mcu_msg") or ""),
            "mcu_log": [str(x) for x in (p.get("mcu_log") or [])][-5:],
        }
        if include_radar:
            out["ascii_radar"] = render_ascii_radar(f)
        return out

    def query_frames(
        self,
        mode: Optional[str] = None,
        status: Optional[str] = None,
        flag: Optional[str] = None,
        driving_only: bool = False,
        shield_only: bool = False,
        wrong_way_only: bool = False,
        on_ramp_only: bool = False,
        link_drop_only: bool = False,
        min_v: Optional[float] = None,
        max_v: Optional[float] = None,
        max_tof: Optional[float] = None,
        min_slip: Optional[float] = None,
        min_plan_ms: Optional[float] = None,
        min_steer_err: Optional[float] = None,
        frame_range: Optional[Tuple[int, int]] = None,
        sim_range: Optional[Tuple[float, float]] = None,
        limit: int = 50,
    ) -> List[Dict[str, Any]]:
        """Filters frames across scalar and reconstructed telemetry fields."""
        mode_q = mode.upper() if mode else None
        status_q = status.upper() if status else None
        flag_q = flag.upper() if flag else None

        matches: List[Dict[str, Any]] = []
        for f in self.frames:
            if frame_range and not (frame_range[0] <= f["frame_id"] <= frame_range[1]):
                continue
            if sim_range and not (sim_range[0] <= f["sim_time"] <= sim_range[1]):
                continue
            if driving_only and f["is_driving"] != 1:
                continue
            if mode_q and mode_q not in f["mode"].upper():
                continue
            if status_q and status_q not in f["status"].upper():
                continue
            if flag_q and not any(flag_q in r.upper() for r in f["anomaly_reasons"]):
                continue
            if shield_only and not f["shield_active"]:
                continue
            if wrong_way_only and not f["is_wrong_way"]:
                continue
            if on_ramp_only and not f["is_on_ramp"]:
                continue
            if link_drop_only and not (
                not f["mcu_main_connected"]
                or not f["mcu_tof_connected"]
                or not f["stm32_front"]
                or not f["stm32_rear"]
                or not f["imu_online"]
            ):
                continue
            if min_v is not None and f["v_real"] < min_v:
                continue
            if max_v is not None and f["v_real"] > max_v:
                continue
            if max_tof is not None and f["f_tof_min"] > max_tof:
                continue
            if min_slip is not None and abs(f["slip_ratio"]) < min_slip:
                continue
            if min_plan_ms is not None and f["plan_ms"] < min_plan_ms:
                continue
            if min_steer_err is not None and abs(f["steer_actual_deg"] - f["steer_cmd_deg"]) < min_steer_err:
                continue

            matches.append({
                "frame_id": f["frame_id"],
                "sim_time": round(f["sim_time"], 2),
                "mode": f["mode"],
                "status": f["status"],
                "v_cmd": round(f["v_cmd"], 2),
                "v_real": round(f["v_real"], 2),
                "steer_cmd_deg": round(f["steer_cmd_deg"], 1),
                "steer_actual_deg": round(f["steer_actual_deg"], 1),
                "f_tof_min": round(f["f_tof_min"], 2),
                "slip_ratio": round(f["slip_ratio"], 2),
                "plan_ms": round(f["plan_ms"], 2),
                "shield": f["shield_chosen"] or ("ON" if f["shield_active"] else "-"),
                "reasons": f["anomaly_reasons"],
            })
            if len(matches) >= limit:
                break
        return matches

    def export_bundle(
        self,
        out_dir: Optional[str | Path] = None,
        formats: Sequence[str] = ("md", "json", "csv", "npz", "parquet"),
    ) -> Dict[str, str]:
        """Exports AI digest + flattened CSV + typed NPZ (+ optional Parquet) bundle."""
        stem = Path(self.db_path).stem
        dest = Path(out_dir) if out_dir else (AI_EXPORTS_DIR / stem)
        dest.mkdir(parents=True, exist_ok=True)

        req_formats = {str(fmt).strip().lower() for fmt in formats}
        outputs: Dict[str, str] = {"out_dir": str(dest)}
        digest = self.build_digest()

        if "md" in req_formats:
            md_path = dest / "report.md"
            md_path.write_text(format_digest_markdown(digest), encoding="utf-8")
            outputs["md"] = str(md_path)

        if "json" in req_formats:
            json_path = dest / "digest.json"
            json_path.write_text(json.dumps(digest, indent=2), encoding="utf-8")
            outputs["json"] = str(json_path)

        scalar_cols = [
            "frame_id", "timestamp", "sim_time", "driving_time", "is_driving",
            "mode", "status", "v_cmd", "v_tgt", "v_real",
            "steer_cmd_deg", "steer_actual_deg",
            "fl_rpm_cmd", "fr_rpm_cmd", "rl_rpm_cmd", "rr_rpm_cmd",
            "fl_rpm_act", "fr_rpm_act", "rl_rpm_act", "rr_rpm_act",
            "yaw_deg", "pitch_deg", "roll_deg", "gyro_z", "accel_fwd", "accel_lat",
            "slip_state", "slip_ratio", "plan_ms", "step_ms", "map_ms", "scout_ms",
            "opt_prim_ms", "opt_alt_ms", "osqp_iters", "servo_pulse_us", "f_tof_min",
            "anomaly_flags", "shield_active", "is_on_ramp", "is_wrong_way",
            "mcu_main_connected", "mcu_tof_connected", "stm32_front", "stm32_rear",
            "imu_online", "override_active",
        ]

        if "csv" in req_formats:
            csv_path = dest / "frames.csv"
            with open(csv_path, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                tof_cols = [f"tof_raw_{i}" for i in range(16)]
                writer.writerow(scalar_cols + tof_cols)
                for f in self.frames:
                    row = [
                        int(f[c]) if isinstance(f[c], bool) else f[c]
                        for c in scalar_cols
                    ] + [round(x, 3) for x in f["tof_ranges_raw"]]
                    writer.writerow(row)
            outputs["csv"] = str(csv_path)

        if "npz" in req_formats:
            n = self.n_frames
            best_arr = np.full((n, 31, 2), np.nan, dtype=np.float32)
            cl_arr = np.full((n, 31, 2), np.nan, dtype=np.float32)
            cr_arr = np.full((n, 31, 2), np.nan, dtype=np.float32)
            pose_arr = np.full((n, 3), np.nan, dtype=np.float32)

            for i, f in enumerate(self.frames):
                for src, dst in (
                    (f["best_line"], best_arr),
                    (f["corridor_left"], cl_arr),
                    (f["corridor_right"], cr_arr),
                ):
                    for j, pt in enumerate(src[:31]):
                        if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                            dst[i, j, 0] = _safe_float(pt[0], math.nan)
                            dst[i, j, 1] = _safe_float(pt[1], math.nan)
                if isinstance(f["pose"], (list, tuple)) and len(f["pose"]) >= 3:
                    pose_arr[i, 0] = _safe_float(f["pose"][0], math.nan)
                    pose_arr[i, 1] = _safe_float(f["pose"][1], math.nan)
                    pose_arr[i, 2] = _safe_float(f["pose"][2], math.nan)

            npz_path = dest / "arrays.npz"
            np.savez_compressed(
                npz_path,
                frame_id=self.frame_ids,
                timestamp=self.timestamps,
                sim_time=self.sim_times,
                driving_time=np.array([f["driving_time"] for f in self.frames], dtype=np.float64),
                is_driving=self.is_driving,
                mode=np.array([f["mode"] for f in self.frames]),
                status=np.array([f["status"] for f in self.frames]),
                v_cmd=self.v_cmd,
                v_tgt=self.v_tgt,
                v_real=self.v_real,
                steer_cmd_deg=self.steer_cmd,
                steer_actual_deg=self.steer_act,
                rpms_cmd=self.rpms_cmd,
                rpms_act=self.rpms_act,
                yaw_deg=np.array([f["yaw_deg"] for f in self.frames], dtype=np.float64),
                pitch_deg=self.pitch_deg,
                roll_deg=self.roll_deg,
                gyro_z=self.gyro_z,
                accel_fwd=self.accel_fwd,
                accel_lat=self.accel_lat,
                slip_ratio=self.slip_ratio,
                plan_ms=self.plan_ms,
                step_ms=self.step_ms,
                map_ms=self.map_ms,
                scout_ms=self.scout_ms,
                f_tof_min=self.f_tof_min,
                anomaly_flags=np.array([f["anomaly_flags"] for f in self.frames], dtype=np.int32),
                shield_active=self.shield_active,
                is_on_ramp=self.is_on_ramp,
                is_wrong_way=self.is_wrong_way,
                tof_ranges=self.tof_ranges,
                tof_ranges_raw=self.tof_ranges_raw,
                best_line=best_arr,
                corridor_left=cl_arr,
                corridor_right=cr_arr,
                pose=pose_arr,
            )
            outputs["npz"] = str(npz_path)

        if "parquet" in req_formats:
            try:
                import pyarrow  # noqa: F401
                import pandas as pd

                records = []
                for f in self.frames:
                    d = {c: (int(f[c]) if isinstance(f[c], bool) else f[c]) for c in scalar_cols}
                    for i in range(16):
                        d[f"tof_raw_{i}"] = round(f["tof_ranges_raw"][i], 3)
                    records.append(d)
                pq_path = dest / "frames.parquet"
                pd.DataFrame.from_records(records).to_parquet(pq_path, index=False, engine="pyarrow")
                outputs["parquet"] = str(pq_path)
            except ImportError:
                outputs["parquet"] = "skipped (pyarrow not installed)"
            except Exception as exc:
                outputs["parquet"] = f"skipped ({exc})"

        return outputs


def render_ascii_radar(
    frame: Dict[str, Any],
    width: int = 49,
    height: int = 23,
) -> str:
    """
    Renders a 2D car-frame top-down ASCII radar (+Y up/forward, +X right).
    Bounds: x in [-1.25, +1.25] m, y in [0.0, 2.20] m.
    Legend: A=car (0,0), #=ToF hit, <=corridor_left, >=corridor_right, *=best_line, G=gap/target, !=shield.
    """
    x_min, x_max = -1.25, 1.25
    y_min, y_max = 0.0, 2.20
    grid = [[" " for _ in range(width)] for _ in range(height)]

    def _plot(x: float, y: float, ch: str, overwrite: bool = True):
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        if x < x_min or x > x_max or y < y_min or y > y_max:
            return
        col = int(round((x - x_min) / (x_max - x_min) * (width - 1)))
        row = int(round((y_max - y) / (y_max - y_min) * (height - 1)))
        col = max(0, min(width - 1, col))
        row = max(0, min(height - 1, row))
        if overwrite or grid[row][col] == " ":
            grid[row][col] = ch

    # Subtle centerline guide
    mid_col = int(round((0.0 - x_min) / (x_max - x_min) * (width - 1)))
    for r in range(height):
        grid[r][mid_col] = ":"

    # Corridor boundaries
    for pt in frame.get("corridor_left") or []:
        if isinstance(pt, (list, tuple)) and len(pt) >= 2:
            _plot(_safe_float(pt[0]), _safe_float(pt[1]), "<")
    for pt in frame.get("corridor_right") or []:
        if isinstance(pt, (list, tuple)) and len(pt) >= 2:
            _plot(_safe_float(pt[0]), _safe_float(pt[1]), ">")

    # Best line trajectory
    for pt in frame.get("best_line") or []:
        if isinstance(pt, (list, tuple)) and len(pt) >= 2:
            _plot(_safe_float(pt[0]), _safe_float(pt[1]), "*")

    # Active shield trajectory if engaged
    payload = frame.get("payload") if isinstance(frame.get("payload"), dict) else {}
    shield = payload.get("shield") if isinstance(payload.get("shield"), dict) else {}
    if shield.get("active"):
        chosen_name = shield.get("chosen_name")
        if chosen_name and isinstance(shield.get(chosen_name), list):
            for pt in shield[chosen_name]:
                if isinstance(pt, (list, tuple)) and len(pt) >= 2:
                    _plot(_safe_float(pt[0]), _safe_float(pt[1]), "!")

    # Gap / primary target
    targets = frame.get("targets") or []
    if targets and isinstance(targets[0], (list, tuple)) and len(targets[0]) >= 2:
        _plot(_safe_float(targets[0][0]), _safe_float(targets[0][1]), "G")
    elif isinstance(frame.get("gap"), (list, tuple)) and len(frame["gap"]) >= 2:
        _plot(_safe_float(frame["gap"][0]), _safe_float(frame["gap"][1]), "G")

    # 16 cross-eyed ToF obstacle hits
    tof_vals = frame.get("tof_ranges") or frame.get("tof_ranges_raw") or []
    for i, (_, mx, my, hdg) in enumerate(FRONT_TOF_SENSOR_RIG):
        if i >= len(tof_vals):
            break
        d = _safe_float(tof_vals[i], 0.0)
        if 0.01 < d < 1.78:
            hx = mx + d * math.cos(hdg)
            hy = my + d * math.sin(hdg)
            _plot(hx, hy, "#")

    # Vehicle origin
    _plot(0.0, 0.0, "A")

    lines = [
        f"Top-Down Radar (Frame #{frame['frame_id']} | mode={frame['mode']} | x=[-1.25..+1.25]m, y=[0..2.2]m)",
        "+" + "-" * width + "+",
    ]
    for r, row_chars in enumerate(grid):
        y_val = y_max - r / max(1, height - 1) * (y_max - y_min)
        suffix = f" {y_val:.1f}m" if r in (0, height // 2, height - 1) else ""
        lines.append("|" + "".join(row_chars) + "|" + suffix)
    lines.append("+" + "-" * width + "+")
    lines.append("Legend: A=robot  #=ToF hit  <=left_wall  >=right_wall  *=best_line  G=target/gap  !=shield")
    return "\n".join(lines)


def compare_sessions(db_a: str | Path, db_b: str | Path) -> Dict[str, Any]:
    """Computes side-by-side KPI deltas and config/git diff between session A and session B."""
    sa = FlightSession(db_a)
    sb = FlightSession(db_b)
    da = sa.build_digest()
    db = sb.build_digest()

    ka, kb = da["kpis"], db["kpis"]
    ca, cb = ka["continuity"], kb["continuity"]

    def _rate_per_min(ep_list: List[Dict[str, Any]], cat: str, drv_s: float) -> float:
        cnt = sum(1 for e in ep_list if e["category"] == cat)
        return round(cnt / max(1e-3, drv_s / 60.0), 2)

    eps_a = sa.extract_episodes()
    eps_b = sb.extract_episodes()

    metrics = [
        ("Driving Seconds (s)", ca["driving_seconds"], cb["driving_seconds"]),
        ("Distance (m)", ca["distance_m"], cb["distance_m"]),
        ("Mean Speed v_real (m/s)", ka.get("v_real", {}).get("mean", 0.0), kb.get("v_real", {}).get("mean", 0.0)),
        ("Max Speed v_real (m/s)", ka.get("v_real", {}).get("max", 0.0), kb.get("v_real", {}).get("max", 0.0)),
        ("Speed RMSE (m/s)", ka.get("v_err_rmse", 0.0), kb.get("v_err_rmse", 0.0)),
        ("Steer RMSE (deg)", ka.get("steer_err_rmse", 0.0), kb.get("steer_err_rmse", 0.0)),
        ("Plan Time p95 (ms)", ka.get("timing_ms", {}).get("plan_ms", {}).get("p95", 0.0), kb.get("timing_ms", {}).get("plan_ms", {}).get("p95", 0.0)),
        ("Shield Active (%)", ka.get("shield_pct", 0.0), kb.get("shield_pct", 0.0)),
        ("On-Ramp (%)", ka.get("imu", {}).get("is_on_ramp_pct", 0.0), kb.get("imu", {}).get("is_on_ramp_pct", 0.0)),
        ("Emergency Eps / min", _rate_per_min(eps_a, "EMERGENCY_RECOVERY", ca["driving_seconds"]), _rate_per_min(eps_b, "EMERGENCY_RECOVERY", cb["driving_seconds"])),
        ("Shield Eps / min", _rate_per_min(eps_a, "SHIELD_EVADE", ca["driving_seconds"]), _rate_per_min(eps_b, "SHIELD_EVADE", cb["driving_seconds"])),
        ("Stall Eps / min", _rate_per_min(eps_a, "STALL", ca["driving_seconds"]), _rate_per_min(eps_b, "STALL", cb["driving_seconds"])),
    ]

    comparisons = []
    for name, va, vb in metrics:
        delta = round(float(vb) - float(va), 3)
        pct = round(delta / abs(float(va)) * 100.0, 1) if abs(float(va)) > 1e-6 else None
        comparisons.append({"metric": name, "a": va, "b": vb, "delta": delta, "pct_change": pct})

    all_modes = sorted(set(ka.get("modes", {})) | set(kb.get("modes", {})))
    mode_shares = []
    na = max(1, sa.n_frames)
    nb = max(1, sb.n_frames)
    for m in all_modes:
        pa = round(ka.get("modes", {}).get(m, 0) / na * 100.0, 1)
        pb = round(kb.get("modes", {}).get(m, 0) / nb * 100.0, 1)
        mode_shares.append({"mode": m, "a_pct": pa, "b_pct": pb, "delta_pct": round(pb - pa, 1)})

    return {
        "session_a": da["session"],
        "session_b": db["session"],
        "metrics": comparisons,
        "mode_shares": mode_shares,
        "git_diff": {
            "a": da["session"].get("git_commit", ""),
            "b": db["session"].get("git_commit", ""),
        },
        "config_diff": _diff_dicts(sa.config, sb.config),
    }


def _diff_dicts(a: Dict[str, Any], b: Dict[str, Any], prefix: str = "") -> List[Dict[str, Any]]:
    diffs: List[Dict[str, Any]] = []
    for k in sorted(set(a) | set(b)):
        key_path = f"{prefix}.{k}" if prefix else str(k)
        va, vb = a.get(k), b.get(k)
        if isinstance(va, dict) and isinstance(vb, dict):
            diffs.extend(_diff_dicts(va, vb, key_path))
        elif va != vb:
            diffs.append({"key": key_path, "a": va, "b": vb})
    return diffs


# ==============================================================================
# COMPACT MARKDOWN / TABLE FORMATTERS FOR AI & TERMINAL INGESTION
# ==============================================================================

def format_digest_markdown(digest: Dict[str, Any]) -> str:
    s = digest["session"]
    k = digest["kpis"]
    diags = digest["diagnostics"]
    eps = digest["episodes"]

    lines: List[str] = []
    lines.append(f"# Flight Debug Digest: {s['filename']}")
    lines.append("")
    lines.append("## 1. Session Overview")
    lines.append(
        f"- **Session ID**: `{s['session_id']}` | **Schema**: v{s['schema_version']} | "
        f"**Git**: `{s['git_commit'] or 'unknown'}` | **Size**: {s['file_size_mb']} MB"
    )
    lines.append(
        f"- **Frames**: {s['total_frames']} total (`{s['frame_range'][0]}..{s['frame_range'][1]}`), "
        f"{s['driving_frames']} driving | **Rate**: {s['effective_hz']} Hz"
    )
    lines.append(
        f"- **Duration & Distance**: {s['wall_duration_s']}s wall, {s['driving_seconds']}s driving, "
        f"est. **{s['distance_m']} m**"
    )
    lines.append(
        f"- **Authority / Links**: override_active={s['override_frames']} frames | "
        f"active_link_drops={s['link_drop_frames']} frames | incidents={digest['episodes_total']}"
    )
    lines.append("")

    lines.append("## 2. Automated Diagnostics")
    if not diags:
        lines.append("- `[INFO]` No significant faults detected.")
    else:
        for d in diags:
            samp = f" (frames: {d['sample_frames']})" if d["sample_frames"] else ""
            lines.append(f"- **[{d['severity']}] `{d['code']}`**: {d['title']}{samp}")
            lines.append(f"  - {d['evidence']}")
    lines.append("")

    lines.append("## 3. KPI Summary")
    lines.append("| Subsystem | Metric | Value |")
    lines.append("| :--- | :--- | :--- |")
    if k.get("v_real"):
        vr = k["v_real"]
        vc = k["v_cmd"]
        lines.append(f"| Speed | v_real (min/mean/p95/max) | {vr['min']} / {vr['mean']} / {vr['p95']} / {vr['max']} m/s (cmd mean={vc['mean']}, RMSE={k['v_err_rmse']}) |")
        lines.append(f"| Steering | tracking RMSE | {k['steer_err_rmse']} deg |")
        rm = k["rpm_act_abs_mean"]
        re = k["rpm_tracking_mae"]
        lines.append(f"| Wheels | mean |RPM_act| | FL={rm['FL']}, FR={rm['FR']}, RL={rm['RL']}, RR={rm['RR']} (MAE: {re['FL']}/{re['FR']}/{re['RL']}/{re['RR']}) |")
        imu = k["imu"]
        lines.append(f"| IMU / Ramp | pitch / |gyro_z| / |a_lat| | pitch={imu['pitch_deg']['mean']} [{imu['pitch_deg']['min']}..{imu['pitch_deg']['max']}] deg, gyro_z_p95={imu['gyro_z']['p95']}, a_lat_p95={imu['accel_lat']['p95']}, on_ramp={imu['is_on_ramp_pct']}% |")
        tm = k["timing_ms"]
        lines.append(f"| Timing (ms) | plan / step / map / scout | plan={tm['plan_ms']['mean']}/{tm['plan_ms']['p95']}/{tm['plan_ms']['max']}, step={tm['step_ms']['mean']}/{tm['step_ms']['p95']}/{tm['step_ms']['max']}, map={tm['map_ms']['mean']}, scout={tm['scout_ms']['mean']} |")
        lines.append(f"| Safety | f_tof_min / shield / wrong_way | tof_min={k['f_tof_min']['min']} (p95={k['f_tof_min']['p95']}), shield={k['shield_pct']}%, wrong_way={k['wrong_way_frames']}f |")
    top_modes = ", ".join(f"{m}:{cnt}" for m, cnt in list(k.get("modes", {}).items())[:8])
    top_status = ", ".join(f"{st}:{cnt}" for st, cnt in list(k.get("statuses", {}).items())[:6])
    lines.append(f"| Modes | distribution | {top_modes} |")
    lines.append(f"| Status | planner status | {top_status} |")
    lines.append("")

    lines.append(f"## 4. Top Incident Episodes ({len(eps)} shown of {digest['episodes_total']})")
    if not eps:
        lines.append("- None detected.")
    else:
        lines.append("| ID | Severity | Category | Frames | Sim t (s) | Dur | min_tof | v_real | Modes | Lead-in -> Outcome |")
        lines.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |")
        for e in eps:
            pm = e["peak_metrics"]
            m_str = "->".join(e["modes_visited"][:3]) + ("..." if len(e["modes_visited"]) > 3 else "")
            li = e["lead_in"]
            oc = e["outcome"]
            lo_str = f"{li['mode']}({li['v_real']}m/s) -> {oc['mode']}({oc['v_real']}m/s)"
            lines.append(
                f"| {e['episode_id']} | {e['severity']} | {e['category']} | {e['start_frame']}..{e['end_frame']} ({e['frame_count']}f) | "
                f"{e['sim_start']}..{e['sim_end']} | {e['duration_s']}s | {pm['min_f_tof']}m | {pm['min_v_real']}..{pm['max_v_real']} | {m_str} | {lo_str} |"
            )
    lines.append("")

    lines.append("## 5. ToF 16-Channel Health")
    chs = k.get("tof_channels") or []
    if chs:
        left_meds = [f"{c['ch']}:{c['median']:.2f}" for c in chs[:8]]
        right_meds = [f"{c['ch']}:{c['median']:.2f}" for c in chs[8:]]
        lines.append(f"- **Left Pod (0..7) medians (m)**: {', '.join(left_meds)}")
        lines.append(f"- **Right Pod (8..15) medians (m)**: {', '.join(right_meds)}")
        bad = [c for c in chs if c["zero_pct"] > 5.0]
        if bad:
            lines.append("- **Non-trivial zero readings (>5%)**: " + ", ".join(f"ch{c['ch']}={c['zero_pct']}%" for c in bad))
    lines.append("")

    lines.append("## 6. Deduplicated MCU / System Logs")
    logs = digest.get("mcu_logs") or []
    non_servo = [m for m in logs if not m["message"].startswith('{"pin":')]
    shown = non_servo[:20] if non_servo else logs[:20]
    if not shown:
        lines.append("- None recorded.")
    else:
        for m in shown:
            lines.append(f"- `f#{m['frame_id']} (t={m['sim_time']}s)` [{m['severity']}] {m['message']}")
        if len(logs) > len(shown):
            lines.append(f"- _({len(logs) - len(shown)} additional log entries omitted; see --json or export)_")
    lines.append("")

    if digest.get("labels"):
        lines.append("## 7. Stored Labels")
        for lbl in digest["labels"]:
            lines.append(f"- `#{lbl['start_frame']}..#{lbl['end_frame']}` **{lbl['tag']}** ({lbl['author']}): {lbl['note']}")
        lines.append("")

    if eps:
        lines.append("## 8. Next Drill-Down Commands")
        lead_frames = max(15, int(round(1.5 * (s["effective_hz"] or 60.0))))
        for e in eps[:2]:
            w0 = max(1, e["start_frame"] - lead_frames)
            w1 = e["end_frame"] + 15
            lines.append(f"- `python launchers/20_ai_debug_suite.py window \"{s['db_path']}\" {w0} {w1}`  # Inspect {e['episode_id']} ({e['category']} + ~1.5s lead-in)")
            lines.append(f"- `python launchers/20_ai_debug_suite.py frame \"{s['db_path']}\" {e['start_frame']}`  # Radar at episode start")
    return "\n".join(lines)


def format_episodes_markdown(episodes: List[Dict[str, Any]], db_name: str) -> str:
    lines = [f"# Incident Episodes: {db_name} ({len(episodes)} total)", ""]
    if not episodes:
        lines.append("No incident episodes matched filters.")
        return "\n".join(lines)
    for e in episodes:
        pm = e["peak_metrics"]
        li = e["lead_in"]
        oc = e["outcome"]
        lines.append(
            f"### {e['episode_id']} [{e['severity']}] {e['category']} — frames {e['start_frame']}..{e['end_frame']} "
            f"({e['frame_count']}f, {e['duration_s']}s, sim_t={e['sim_start']}..{e['sim_end']}s)"
        )
        lines.append(
            f"- **Modes**: {' -> '.join(e['modes_visited'])} | **Peaks**: min_tof={pm['min_f_tof']}m, "
            f"v_real=[{pm['min_v_real']}..{pm['max_v_real']}] m/s, max|steer|={pm['max_abs_steer']} deg, "
            f"max|slip|={pm['max_abs_slip']}, max_plan={pm['max_plan_ms']} ms, max_step={pm['max_step_ms']} ms"
        )
        lines.append(
            f"- **Lead-in (#{li['frame_id']})**: {li['mode']} v={li['v_real']}m/s str={li['steer_deg']}deg tof={li['f_tof_min']}m -> "
            f"**Outcome (#{oc['frame_id']})**: {oc['mode']} v={oc['v_real']}m/s str={oc['steer_deg']}deg tof={oc['f_tof_min']}m"
        )
        if e["mcu_logs"]:
            lines.append("- **Logs**: " + " | ".join(f"f#{m['frame_id']}: {m['message']}" for m in e["mcu_logs"][:3]))
        lines.append("")
    return "\n".join(lines)


def format_window_table(rows: List[Dict[str, Any]]) -> str:
    if not rows:
        return "(no frames in range)"
    header = "  fid |  sim_t | mode                    | v_cmd / real | str c /  a | f_tof |  slip | plan_ms | shield        | gap     | flags"
    sep = "-" * len(header)
    out = [header, sep]
    for r in rows:
        fl = ",".join(r["reasons"]) if r["reasons"] else "-"
        out.append(
            f"{r['frame_id']:5d} | {r['sim_time']:6.2f} | {r['mode']:<23} | "
            f"{r['v_cmd']:5.2f} /{r['v_real']:5.2f} | {r['steer_cmd_deg']:5.1f} /{r['steer_actual_deg']:5.1f} | "
            f"{r['f_tof_min']:5.2f} | {r['slip_ratio']:5.2f} | {r['plan_ms']:7.2f} | "
            f"{r['shield']:<13} | {r['gap']:<7} | {fl}"
        )
    return "\n".join(out)


def format_frame_markdown(info: Dict[str, Any]) -> str:
    m = info["motion"]
    w = info["wheels"]
    imu = info["imu"]
    tm = info["timing_ms"]
    sh = info["shield"]
    pl = info["planner"]
    lk = info["links"]

    lines = [
        f"# Frame #{info['frame_id']} (sim_t={info['sim_time']:.3f}s | mode={info['mode']} | status={info['status'] or 'NONE'})",
        "",
        f"- **Motion**: v_cmd={m['v_cmd']} | v_tgt={m['v_tgt']} | v_real={m['v_real']} m/s | steer_cmd={m['steer_cmd_deg']} | steer_act={m['steer_actual_deg']} deg | servo={m['servo_pulse_us']} us | pose={m['pose']}",
        f"- **Wheels (cmd -> act, err)**: FL={w['FL']['cmd']}->{w['FL']['act']} ({w['FL']['err']:+.0f}) | FR={w['FR']['cmd']}->{w['FR']['act']} ({w['FR']['err']:+.0f}) | RL={w['RL']['cmd']}->{w['RL']['act']} ({w['RL']['err']:+.0f}) | RR={w['RR']['cmd']}->{w['RR']['act']} ({w['RR']['err']:+.0f})",
        f"- **IMU / Slip**: yaw={imu['yaw_deg']} pitch={imu['pitch_deg']} roll={imu['roll_deg']} deg | gyro_z={imu['gyro_z']} | a_fwd={imu['accel_fwd']} a_lat={imu['accel_lat']} | slip={imu['slip_state']} ({imu['slip_ratio']})",
        f"- **Timing (ms)**: plan={tm['plan_ms']} step={tm['step_ms']} map={tm['map_ms']} scout={tm['scout_ms']} opt_p={tm['opt_prim_ms']} opt_a={tm['opt_alt_ms']} iters={tm['osqp_iters']}",
        f"- **Shield**: active={sh['active']} chosen={sh['chosen_name'] or '-'} rev={sh['reverse_suggested']} | branches={sh['branches_safe']}",
        f"- **Links / Flags**: main={lk['mcu_main_connected']} tof={lk['mcu_tof_connected']} stmF={lk['stm32_front']} stmR={lk['stm32_rear']} imu={lk['imu_online']} | ramp={lk['is_on_ramp']} wrong_way={lk['is_wrong_way']} override={lk['override_active']}",
        f"- **Anomalies**: {', '.join(info['anomaly_reasons']) if info['anomaly_reasons'] else 'NONE'} | f_tof_min={info['f_tof_min']}m",
        f"- **Gap / Targets**: gap={pl['gap']} | targets={pl['targets']}",
    ]
    if pl["corridor_samples"]:
        cs = ", ".join(f"{c['zone']}(w={c['width_m']}m)" for c in pl["corridor_samples"])
        lines.append(f"- **Corridor**: {cs} | best_line_pts={pl['best_line_len']}")
    if info["mcu_msg"] or info["mcu_log"]:
        lines.append(f"- **MCU msg**: `{info['mcu_msg']}` | recent_log={info['mcu_log']}")
    lines.append("")
    lines.append("Left Pod (ch 0..7):  " + ", ".join(f"c{c['ch']}={c['raw_m']:.2f}" for c in info["tof_channels"][:8]))
    lines.append("Right Pod (ch 8..15): " + ", ".join(f"c{c['ch']}={c['raw_m']:.2f}" for c in info["tof_channels"][8:]))
    if "ascii_radar" in info:
        lines.append("")
        lines.append(info["ascii_radar"])
    return "\n".join(lines)


def format_comparison_markdown(comp: Dict[str, Any]) -> str:
    sa = comp["session_a"]
    sb = comp["session_b"]
    lines = [
        f"# Session A/B Comparison",
        f"- **A**: `{sa['filename']}` (frames={sa['total_frames']}, git=`{sa['git_commit'] or 'unknown'}`)",
        f"- **B**: `{sb['filename']}` (frames={sb['total_frames']}, git=`{sb['git_commit'] or 'unknown'}`)",
        "",
        "| Metric | Session A | Session B | Delta (B - A) | % Change |",
        "| :--- | ---: | ---: | ---: | ---: |",
    ]
    for m in comp["metrics"]:
        pct_s = f"{m['pct_change']:+.1f}%" if m["pct_change"] is not None else "-"
        lines.append(f"| {m['metric']} | {m['a']} | {m['b']} | {m['delta']:+.3f} | {pct_s} |")
    lines.append("")
    lines.append("## Mode Share Changes")
    lines.append("| Mode | A % | B % | Delta |")
    lines.append("| :--- | ---: | ---: | ---: |")
    for ms in comp["mode_shares"]:
        lines.append(f"| {ms['mode']} | {ms['a_pct']}% | {ms['b_pct']}% | {ms['delta_pct']:+.1f}% |")
    if comp["config_diff"]:
        lines.append("")
        lines.append("## Config Differences")
        for d in comp["config_diff"][:20]:
            lines.append(f"- `{d['key']}`: `{d['a']}` -> `{d['b']}`")
    return "\n".join(lines)
