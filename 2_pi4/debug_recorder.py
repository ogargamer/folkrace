"""
================================================================================
  FOLKRACE PI 4: HIGH-RATE FLIGHT RECORDER & BLACK BOX (SQLite WAL)
================================================================================
  Stores full-fidelity, frame-by-frame autonomous racing telemetry to the 64GB
  SD card without blocking or interrupting the real-time path planner.

  Key Specifications:
    - Database Engine: SQLite3 in Write-Ahead-Log (WAL) mode with NORMAL sync.
    - Asynchronous Pipeline: Dedicated I/O worker thread with a bounded queue.
    - Indexing: Instant O(1) random access by frame_id or timestamp.
    - Anomaly Bitmask: Fast filtering of unexpected robot behaviors:
        Bit 0 (0x01): Vehicle Traction Loss / Wheel Slip / Spin
        Bit 1 (0x02): Emergency Stop / Reverse Backup Triggered
        Bit 2 (0x04): Obstacle Proximity Contact Alert (< 0.12m)
        Bit 3 (0x08): Planner Latency Spike (> 16.6ms deadline overrun)
        Bit 4 (0x10): Steering Rate / Servo Saturation Spike
    - Storage Safeguard: Automatically tracks SD card free space; keeps >= 5GB free.
================================================================================
"""

import os
import re
import sys
import time
import json
import math
import zlib
import shutil
import sqlite3
import datetime
import threading
import subprocess
from collections import deque
from pathlib import Path
import queue
from typing import Dict, List, Optional, Tuple, Any

# Default storage location
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DATA_DIR = os.path.join(BASE_DIR, "data", "recordings")


class AnomalyFlag:
    NONE = 0
    SLIP = 1 << 0           # 0x01: Traction loss / spin
    EMERGENCY = 1 << 1      # 0x02: Emergency stop / recovery mode
    PROXIMITY = 1 << 2      # 0x04: Obstacle dangerously close
    LATENCY_SPIKE = 1 << 3  # 0x08: Planner frame exceeded 16.6ms
    STEER_JERK = 1 << 4     # 0x10: Severe steering rate / servo jump
    SHIELD = 1 << 5         # 0x20: Safety shield active / reverse suggested
    WRONG_WAY = 1 << 6      # 0x40: Anti-wrong-way watchdog triggered
    LINK_DROP = 1 << 7      # 0x80: MCU / ToF / STM32 / IMU link offline


def _open_sqlite_ro(db_path: str) -> sqlite3.Connection:
    """Opens SQLite database read-only, falling back cleanly on Windows WAL locks."""
    try:
        uri = Path(db_path).resolve().as_uri() + "?mode=ro"
        conn = sqlite3.connect(uri, uri=True)
        # Probe schema read to catch WAL/SHM read-only permission quirks early
        conn.execute("SELECT name FROM sqlite_master LIMIT 1;")
        return conn
    except Exception:
        return sqlite3.connect(db_path)


def _extract_f_tof_min(telemetry: Dict[str, Any]) -> float:
    """Extracts minimum front/ray ToF distance (m), falling back to tof_ranges / tof_ranges_raw."""
    for key in ("f_tof", "f_tof_raw"):
        val = telemetry.get(key)
        if val is not None:
            try:
                fval = float(val)
                if math.isfinite(fval):
                    return fval
            except (TypeError, ValueError):
                pass

    if telemetry.get("mcu_tof_connected", True) is not False and telemetry.get("tof_front_ok", True) is not False:
        for arr_key in ("tof_ranges", "tof_ranges_raw"):
            arr = telemetry.get(arr_key)
            if isinstance(arr, (list, tuple)) and arr:
                valid = []
                for x in arr:
                    try:
                        fx = float(x)
                        if math.isfinite(fx) and fx > 0.001:
                            valid.append(fx)
                    except (TypeError, ValueError):
                        pass
                if valid:
                    return min(valid)
    return 1.0


def _collect_session_metadata() -> Tuple[str, str]:
    """Collects best-effort git commit hash, vehicle_config.json, and planner constants once at init."""
    git_commit = ""
    repo_candidates = [os.path.abspath(os.path.join(BASE_DIR, "..")), BASE_DIR]
    for repo_dir in repo_candidates:
        if os.path.exists(os.path.join(repo_dir, ".git")):
            try:
                res = subprocess.run(
                    ["git", "rev-parse", "--short", "HEAD"],
                    cwd=repo_dir,
                    capture_output=True,
                    text=True,
                    timeout=0.5,
                )
                if res.returncode == 0:
                    git_commit = res.stdout.strip()
                    break
            except Exception:
                pass

    meta: Dict[str, Any] = {}
    cfg_candidates = [
        os.path.join(BASE_DIR, "..", "brain", "vehicle_config.json"),
        os.path.join(BASE_DIR, "brain", "vehicle_config.json"),
    ]
    for cfg_path in cfg_candidates:
        if os.path.isfile(cfg_path):
            try:
                with open(cfg_path, "r", encoding="utf-8") as f:
                    meta["vehicle_config"] = json.load(f)
                break
            except Exception:
                pass

    # Best-effort capture of planner_config scalar constants
    try:
        for p in (os.path.abspath(os.path.join(BASE_DIR, "..")), BASE_DIR):
            if os.path.isdir(os.path.join(p, "brain")) and p not in sys.path:
                sys.path.insert(0, p)
        import importlib
        pcfg = importlib.import_module("brain.planner_config")
        planner_consts: Dict[str, Any] = {}
        for name in dir(pcfg):
            if not name.isupper() or name.startswith("_"):
                continue
            val = getattr(pcfg, name)
            if isinstance(val, (bool, int, str)):
                planner_consts[name] = val
            elif isinstance(val, float) and math.isfinite(val):
                planner_consts[name] = val
        if planner_consts:
            meta["planner_config"] = planner_consts
    except Exception:
        pass

    try:
        config_json = json.dumps(meta)
    except Exception:
        config_json = "{}"
    return git_commit, config_json


_LOG_ERR_RE = re.compile(
    r"(\[(?:ERR|ERROR|WARN|FAIL)\]|"
    r"\b(?:FAIL|FAILED|FAILURE|FAULT|FATAL|CRITICAL|FAILSAFE|DISCONN\w*|TIMEOUT|LINK\s+LOST)\b|"
    r"\b(?:ERR(?:OR)?|OVERRUNS?)\s*[:=]\s*[1-9]\d*)"
)


def _classify_log_severity(msg: str) -> str:
    """Classifies an MCU/system log line without false-matching 'Yaw Error' or 'Err: 0'."""
    upper_msg = msg.upper()
    if "⚠" in msg or "🛑" in msg or _LOG_ERR_RE.search(upper_msg):
        return "ERROR"
    return "INFO"


def _decode_anomaly_flags(flags: int) -> List[str]:
    """Converts AnomalyFlag bitmask into human-readable labels."""
    labels: List[str] = []
    if flags & AnomalyFlag.SLIP:
        labels.append("SLIP/TRACTION")
    if flags & AnomalyFlag.EMERGENCY:
        labels.append("EMERGENCY/RECOVERY")
    if flags & AnomalyFlag.PROXIMITY:
        labels.append("NEAR_COLLISION")
    if flags & AnomalyFlag.LATENCY_SPIKE:
        labels.append("LATENCY_SPIKE")
    if flags & AnomalyFlag.STEER_JERK:
        labels.append("STEER_JERK")
    if flags & AnomalyFlag.SHIELD:
        labels.append("SHIELD_EVADE")
    if flags & AnomalyFlag.WRONG_WAY:
        labels.append("WRONG_WAY")
    if flags & AnomalyFlag.LINK_DROP:
        labels.append("LINK_DROP")
    return labels


def add_label(
    db_path: str,
    start_frame: int,
    end_frame: int,
    tag: str,
    note: str = "",
    author: str = "ai",
) -> int:
    """Writes a frame-range label into a v1 or v2 SQLite recording."""
    if not os.path.exists(db_path):
        raise FileNotFoundError(f"Flight database not found: {db_path}")
    lo, hi = (int(start_frame), int(end_frame))
    if lo > hi:
        lo, hi = hi, lo
    conn = sqlite3.connect(db_path)
    try:
        cur = conn.cursor()
        cur.execute("""
            CREATE TABLE IF NOT EXISTS labels (
                label_id INTEGER PRIMARY KEY AUTOINCREMENT,
                start_frame INTEGER NOT NULL,
                end_frame INTEGER NOT NULL,
                tag TEXT NOT NULL,
                note TEXT DEFAULT '',
                author TEXT DEFAULT 'user',
                created_at TEXT NOT NULL
            );
        """)
        cur.execute("CREATE INDEX IF NOT EXISTS idx_labels_range ON labels(start_frame, end_frame);")
        created_at = datetime.datetime.now().isoformat()
        cur.execute(
            """
            INSERT INTO labels (start_frame, end_frame, tag, note, author, created_at)
            VALUES (?, ?, ?, ?, ?, ?);
            """,
            (lo, hi, str(tag), str(note), str(author), created_at),
        )
        conn.commit()
        return int(cur.lastrowid or 0)
    finally:
        conn.close()


class TelemetryFlightRecorder:
    """
    High-performance asynchronous telemetry recorder.
    Writes full vehicle state and perception grids to an indexed SQLite file.
    """

    def __init__(
        self,
        output_dir: str = DEFAULT_DATA_DIR,
        session_name: Optional[str] = None,
        max_driving_seconds: float = 1800.0,       # 30-minute running buffer of active driving
        prune_interval_driving_s: float = 30.0,    # Lazy prune check every 30s of driving
        min_free_space_bytes: int = 5 * 1024 * 1024 * 1024,  # 5 GB safety reserve
        batch_size: int = 6,
        commit_interval_s: float = 0.25,
    ):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.max_driving_seconds = max_driving_seconds
        self.prune_interval_driving_s = prune_interval_driving_s
        self.min_free_space_bytes = min_free_space_bytes
        self.batch_size = batch_size
        self.commit_interval_s = commit_interval_s

        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        if session_name:
            self.session_id = f"{session_name}_{timestamp_str}"
        else:
            self.session_id = f"session_{timestamp_str}"

        self.db_path = os.path.join(self.output_dir, f"{self.session_id}.db")

        # Telemetry ingestion queue (bounded to prevent runaway RAM)
        self.queue: queue.Queue = queue.Queue(maxsize=1000)
        self.running = False
        self.worker_thread: Optional[threading.Thread] = None

        # Driving buffer state
        self.total_driving_seconds = 0.0
        self.last_sim_time = 0.0
        self.last_prune_driving_s = 0.0
        self.pruned_frames_count = 0
        self.min_frame_id = 1

        # Statistics
        self.total_frames_recorded = 0
        self.total_bytes_written = 0
        self.dropped_frames = 0
        self.last_steer_deg = 0.0

        # State tracking for compact events table
        self._prev_mode: Optional[str] = None
        self._prev_shield_active: int = 0
        self._prev_wrong_way: int = 0
        self._prev_on_ramp: int = 0
        self._prev_link_state: Optional[Tuple[bool, bool, bool, bool, bool]] = None
        self._seen_mcu_logs: deque = deque(maxlen=200)

        # Capture provenance & config once at session start
        self.git_commit, self.config_json = _collect_session_metadata()

        # Auto-clean old sessions to prevent disk bloat
        self._cleanup_old_session_files()

        # Initialize SQLite database
        self._init_db()

    def _init_db(self):
        """Creates tables, pragmas and indexes in SQLite."""
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()

        # Crash-safe write pragmas for SD card (WAL + FULL synchronous fsync + incremental vacuum)
        cur.execute("PRAGMA journal_mode = WAL;")
        cur.execute("PRAGMA synchronous = FULL;")
        cur.execute("PRAGMA temp_store = MEMORY;")
        cur.execute("PRAGMA cache_size = -64000;")  # 64 MB cache
        cur.execute("PRAGMA auto_vacuum = INCREMENTAL;")

        # Sessions table (v2 adds schema_version, git_commit, config_json)
        cur.execute("""
            CREATE TABLE IF NOT EXISTS session_info (
                session_id TEXT PRIMARY KEY,
                created_at TEXT,
                start_timestamp REAL,
                end_timestamp REAL,
                total_frames INTEGER,
                driving_seconds REAL,
                notes TEXT,
                schema_version INTEGER DEFAULT 2,
                git_commit TEXT DEFAULT '',
                config_json TEXT DEFAULT '{}'
            );
        """)

        # Frames table: structured scalar fields for lightning-fast queries + compressed payload
        cur.execute("""
            CREATE TABLE IF NOT EXISTS frames (
                frame_id INTEGER PRIMARY KEY,
                timestamp REAL,
                sim_time REAL,
                driving_time REAL,
                is_driving INTEGER,
                mode TEXT,
                v_cmd REAL,
                v_real REAL,
                steer_cmd_deg REAL,
                steer_actual_deg REAL,
                fl_rpm_cmd REAL,
                fr_rpm_cmd REAL,
                rl_rpm_cmd REAL,
                rr_rpm_cmd REAL,
                fl_rpm_act REAL,
                fr_rpm_act REAL,
                rl_rpm_act REAL,
                rr_rpm_act REAL,
                yaw_deg REAL,
                pitch_deg REAL,
                roll_deg REAL,
                gyro_z REAL,
                accel_fwd REAL,
                accel_lat REAL,
                slip_state TEXT,
                slip_ratio REAL,
                plan_ms REAL,
                map_ms REAL,
                osqp_iters INTEGER,
                f_tof_min REAL,
                anomaly_flags INTEGER,
                payload_compressed BLOB,
                v_tgt REAL,
                step_ms REAL,
                scout_ms REAL,
                servo_pulse_us INTEGER,
                status TEXT,
                shield_active INTEGER,
                is_on_ramp INTEGER,
                is_wrong_way INTEGER
            );
        """)

        # Events table: state transitions and deduplicated MCU logs
        cur.execute("""
            CREATE TABLE IF NOT EXISTS events (
                event_id INTEGER PRIMARY KEY AUTOINCREMENT,
                frame_id INTEGER,
                timestamp REAL,
                sim_time REAL,
                event_type TEXT,
                severity TEXT,
                label TEXT,
                details_json TEXT
            );
        """)

        # Labels table: human and AI annotations
        cur.execute("""
            CREATE TABLE IF NOT EXISTS labels (
                label_id INTEGER PRIMARY KEY AUTOINCREMENT,
                start_frame INTEGER NOT NULL,
                end_frame INTEGER NOT NULL,
                tag TEXT NOT NULL,
                note TEXT DEFAULT '',
                author TEXT DEFAULT 'user',
                created_at TEXT NOT NULL
            );
        """)

        # Indexes for instant frame-by-frame scrubbing, anomaly review, and driving buffer pruning
        cur.execute("CREATE INDEX IF NOT EXISTS idx_frames_ts ON frames(timestamp);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_frames_driving ON frames(driving_time);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_frames_anomaly ON frames(anomaly_flags);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_frames_mode ON frames(mode);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_events_frame ON events(frame_id);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_events_type ON events(event_type);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_labels_range ON labels(start_frame, end_frame);")

        # Insert session info record
        cur.execute("""
            INSERT OR REPLACE INTO session_info (
                session_id, created_at, start_timestamp, total_frames, driving_seconds,
                notes, schema_version, git_commit, config_json
            )
            VALUES (?, ?, ?, 0, 0.0, 'Folkrace Autonomous Flight Log', 2, ?, ?);
        """, (
            self.session_id,
            datetime.datetime.now().isoformat(),
            time.time(),
            self.git_commit,
            self.config_json,
        ))

        conn.commit()
        conn.close()

    def start(self):
        """Starts the background disk writer thread."""
        if not self.running:
            self.running = True
            self.worker_thread = threading.Thread(target=self._writer_loop, name="FlightRecorderWriter", daemon=True)
            self.worker_thread.start()
            print(f"[FlightRecorder] Started recording to: {self.db_path}")

    def stop(self):
        """Stops recording and flushes pending frames to disk."""
        if self.running:
            self.running = False
            if self.worker_thread:
                self.worker_thread.join(timeout=3.0)

            # Update session info
            try:
                conn = sqlite3.connect(self.db_path)
                cur = conn.cursor()
                cur.execute("""
                    UPDATE session_info
                    SET end_timestamp = ?, total_frames = ?, driving_seconds = ?
                    WHERE session_id = ?;
                """, (time.time(), self.total_frames_recorded, round(self.total_driving_seconds, 2), self.session_id))
                conn.commit()
                conn.close()
            except Exception as e:
                print(f"[FlightRecorder] Error updating session footer: {e}")

            print(f"[FlightRecorder] Stopped. Total frames saved: {self.total_frames_recorded} (Dropped: {self.dropped_frames})")

    def record_frame(self, telemetry: Dict[str, Any], raw_bytes: Optional[bytes] = None) -> bool:
        """
        Non-blocking ingestion of a telemetry frame.
        Guaranteed zero-wait for the caller.
        """
        if not self.running:
            return False

        try:
            self.queue.put_nowait((telemetry, raw_bytes))
            return True
        except queue.Full:
            self.dropped_frames += 1
            return False

    def detect_anomalies(self, telemetry: Dict[str, Any]) -> int:
        """Evaluates telemetry frame and flags potential unexpected robot behavior."""
        flags = AnomalyFlag.NONE

        # 1. Traction / Slip
        slip_str = str(telemetry.get("slip_state") or "").upper()
        slip_ratio = float(telemetry.get("slip_ratio") or 0.0)
        if any(x in slip_str for x in ("OVERSTEER", "UNDERSTEER", "SPIN")) or abs(slip_ratio) > 0.35:
            flags |= AnomalyFlag.SLIP

        # 2. Emergency stop or recovery state
        mode_str = str(telemetry.get("mode") or "").upper()
        if any(x in mode_str for x in ("PIVOT", "REVERSE", "DECEL", "STUCK", "EMERGENCY")):
            flags |= AnomalyFlag.EMERGENCY

        # 3. Dangerous close contact (< 0.12m) using f_tof or tof_ranges_raw fallback
        f_tof = _extract_f_tof_min(telemetry)
        if (
            mode_str not in ("STOPPED", "STANDBY", "OFFLINE")
            and telemetry.get("mcu_tof_connected", True) is not False
            and 0.001 < f_tof < 0.12
        ):
            flags |= AnomalyFlag.PROXIMITY

        # 4. Planner latency spike (> 16.6ms)
        plan_ms_val = telemetry.get("plan_ms")
        step_ms_val = telemetry.get("step_ms")
        plan_ms = float(0.0 if plan_ms_val is None else plan_ms_val)
        step_ms = float(0.0 if step_ms_val is None else step_ms_val)
        if plan_ms > 16.6 or step_ms > 16.6:
            flags |= AnomalyFlag.LATENCY_SPIKE

        # 5. Severe steering jerk
        steer_val = telemetry.get("steer_cmd_deg")
        if steer_val is None:
            steer_val = telemetry.get("steer_deg")
        steer_deg = float(0.0 if steer_val is None else steer_val)
        if abs(steer_deg - self.last_steer_deg) > 25.0:
            flags |= AnomalyFlag.STEER_JERK
        self.last_steer_deg = steer_deg

        # 6. Safety shield intervention
        shield = telemetry.get("shield")
        if isinstance(shield, dict) and (shield.get("active") or shield.get("reverse_suggested")):
            flags |= AnomalyFlag.SHIELD

        # 7. Wrong-way watchdog
        if telemetry.get("is_wrong_way"):
            flags |= AnomalyFlag.WRONG_WAY

        # 8. MCU / sensor link drop while planner/driving active
        is_active_run = bool(telemetry.get("planner_running")) or (
            mode_str not in ("STOPPED", "STANDBY", "OFFLINE", "IDLE", "WARMUP", "PAUSED")
        )
        if is_active_run:
            flags_dict = telemetry.get("flags") if isinstance(telemetry.get("flags"), dict) else {}
            if (
                telemetry.get("mcu_main_connected") is False
                or telemetry.get("mcu_tof_connected") is False
                or flags_dict.get("stm32_front") is False
                or flags_dict.get("stm32_rear") is False
                or flags_dict.get("imu_online") is False
            ):
                flags |= AnomalyFlag.LINK_DROP

        return flags

    def _writer_loop(self):
        """Asynchronous disk writer worker running on dedicated core."""
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        # Ensure every WAL commit fsyncs to SD card so sudden power loss loses at most one small batch
        cur.execute("PRAGMA synchronous = FULL;")

        batch = []
        last_commit_time = time.time()

        while self.running or not self.queue.empty():
            try:
                # Wait for next frame with short timeout for batch commits
                telemetry = self.queue.get(timeout=0.05)
                batch.append(telemetry)
            except queue.Empty:
                pass

            now = time.time()
            if len(batch) >= self.batch_size or (batch and (now - last_commit_time) >= self.commit_interval_s):
                self._flush_batch(cur, conn, batch)
                batch.clear()
                last_commit_time = now

                # Disk safety check every 500 frames
                if self.total_frames_recorded % 500 == 0:
                    self._check_disk_space()

        # Final flush
        if batch:
            self._flush_batch(cur, conn, batch)

        conn.close()

    def _collect_frame_events(
        self,
        frame_id: int,
        ts: float,
        sim_time: float,
        mode: str,
        telem: Dict[str, Any],
        shield_dict: Dict[str, Any],
        shield_active: int,
        is_on_ramp: int,
        is_wrong_way: int,
    ) -> List[Tuple[int, float, float, str, str, str, str]]:
        """Collects state-transition and deduplicated MCU log rows for the events table."""
        ev_rows: List[Tuple[int, float, float, str, str, str, str]] = []

        if self._prev_mode is not None and mode != self._prev_mode:
            ev_rows.append((
                frame_id, ts, sim_time, "MODE_CHANGE", "INFO",
                f"{self._prev_mode} -> {mode}",
                json.dumps({"from": self._prev_mode, "to": mode}),
            ))
        self._prev_mode = mode

        if shield_active != self._prev_shield_active:
            chosen = shield_dict.get("chosen_name") or ""
            ev_rows.append((
                frame_id, ts, sim_time, "SHIELD",
                "WARN" if shield_active else "INFO",
                f"SHIELD_ON:{chosen}" if shield_active else "SHIELD_OFF",
                json.dumps({
                    "active": bool(shield_active),
                    "chosen_name": chosen,
                    "reverse_suggested": bool(shield_dict.get("reverse_suggested")),
                }),
            ))
        self._prev_shield_active = shield_active

        if is_wrong_way != self._prev_wrong_way:
            ev_rows.append((
                frame_id, ts, sim_time, "WRONG_WAY",
                "CRITICAL" if is_wrong_way else "INFO",
                "WRONG_WAY_ON" if is_wrong_way else "WRONG_WAY_OFF",
                "{}",
            ))
        self._prev_wrong_way = is_wrong_way

        if is_on_ramp != self._prev_on_ramp:
            ev_rows.append((
                frame_id, ts, sim_time, "RAMP", "INFO",
                "RAMP_ON" if is_on_ramp else "RAMP_OFF",
                json.dumps({"pitch_deg": float(telem.get("pitch_deg") or 0.0)}),
            ))
        self._prev_on_ramp = is_on_ramp

        flags_dict = telem.get("flags") if isinstance(telem.get("flags"), dict) else {}
        link_state = (
            telem.get("mcu_main_connected") is not False,
            telem.get("mcu_tof_connected") is not False,
            flags_dict.get("stm32_front") is not False,
            flags_dict.get("stm32_rear") is not False,
            flags_dict.get("imu_online") is not False,
        )
        if self._prev_link_state is None:
            if not all(link_state):
                ev_rows.append((
                    frame_id, ts, sim_time, "LINK", "CRITICAL", "LINK_DEGRADED",
                    json.dumps({
                        "mcu_main": link_state[0],
                        "mcu_tof": link_state[1],
                        "stm32_front": link_state[2],
                        "stm32_rear": link_state[3],
                        "imu_online": link_state[4],
                    }),
                ))
        elif link_state != self._prev_link_state:
            all_ok = all(link_state)
            ev_rows.append((
                frame_id, ts, sim_time, "LINK",
                "INFO" if all_ok else "CRITICAL",
                "LINK_RESTORED" if all_ok else "LINK_DROP",
                json.dumps({
                    "mcu_main": link_state[0],
                    "mcu_tof": link_state[1],
                    "stm32_front": link_state[2],
                    "stm32_rear": link_state[3],
                    "imu_online": link_state[4],
                }),
            ))
        self._prev_link_state = link_state

        raw_logs: List[str] = []
        mcu_log = telem.get("mcu_log")
        if isinstance(mcu_log, list):
            for entry in mcu_log:
                if entry is not None:
                    s = str(entry).strip()
                    if s:
                        raw_logs.append(s)
        mcu_msg = telem.get("mcu_msg")
        if mcu_msg is not None:
            s_msg = str(mcu_msg).strip()
            if s_msg and s_msg not in raw_logs:
                raw_logs.append(s_msg)

        for msg in raw_logs:
            if msg in self._seen_mcu_logs:
                continue
            self._seen_mcu_logs.append(msg)
            sev = _classify_log_severity(msg)
            ev_rows.append((
                frame_id, ts, sim_time, "MCU_LOG", sev, msg[:160],
                json.dumps({"msg": msg}),
            ))
        return ev_rows

    def _flush_batch(self, cur: sqlite3.Cursor, conn: sqlite3.Connection, batch: List[Any]):
        """Flushes a batch of telemetry frames and events in a single transaction."""
        if not batch:
            return

        rows = []
        event_rows: List[Tuple[int, float, float, str, str, str, str]] = []
        for item in batch:
            if isinstance(item, tuple):
                telem, raw_b = item
            else:
                telem, raw_b = item, None
            frame_id = self.total_frames_recorded + 1
            ts = float(telem.get("timestamp") or time.time())
            sim_time = float(telem.get("sim_time") or 0.0)
            mode = str(telem.get("mode") or "DRIVE")

            v_cmd = float(telem.get("v_cmd") or 0.0)
            v_real_raw = telem.get("v_real")
            if v_real_raw is None:
                v_real_raw = telem.get("v_est")
            v_real = float(0.0 if v_real_raw is None else v_real_raw)

            # Track driving duration (only accumulates when vehicle is actively moving)
            dt = 1.0 / 60.0
            if self.last_sim_time > 0 and sim_time > self.last_sim_time:
                dt = min(0.1, sim_time - self.last_sim_time)
            self.last_sim_time = sim_time

            is_driving = 1 if ((abs(v_cmd) > 0.05 or abs(v_real) > 0.05) and (mode not in ("WARMUP", "IDLE", "PAUSED"))) else 0
            if is_driving:
                self.total_driving_seconds += dt

            steer_cmd_raw = telem.get("steer_cmd_deg")
            if steer_cmd_raw is None:
                steer_cmd_raw = telem.get("steer_deg")
            steer_cmd_deg = float(0.0 if steer_cmd_raw is None else steer_cmd_raw)
            steer_act_raw = telem.get("steer_actual_deg")
            steer_act_deg = float(steer_cmd_deg if steer_act_raw is None else steer_act_raw)

            # Wheel RPMs (commanded vs actual)
            rpms_cmd = [float(x or 0.0) for x in (telem.get("rpms_cmd") or [0.0, 0.0, 0.0, 0.0])]
            rpms_act = [float(x or 0.0) for x in (telem.get("rpms_act") or [0.0, 0.0, 0.0, 0.0])]
            fl_cmd, fr_cmd, rl_cmd, rr_cmd = (rpms_cmd + [0.0]*4)[:4]
            fl_act, fr_act, rl_act, rr_act = (rpms_act + [0.0]*4)[:4]

            # IMU
            yaw_raw = telem.get("yaw_deg")
            if yaw_raw is None:
                yaw_raw = telem.get("yaw_fused")
            yaw_deg = float(0.0 if yaw_raw is None else yaw_raw)
            pitch_deg = float(telem.get("pitch_deg") or 0.0)
            roll_deg = float(telem.get("roll_deg") or 0.0)
            gyro_z = float(telem.get("gyro_z") or 0.0)
            accel_fwd = float(telem.get("accel_fwd") or 0.0)
            accel_lat = float(telem.get("accel_lat") or 0.0)

            slip_state = str(telem.get("slip_state") or "NORMAL")
            slip_ratio = float(telem.get("slip_ratio") or 0.0)

            plan_ms = float(telem.get("plan_ms") or 0.0)
            map_ms = float(telem.get("map_ms") or 0.0)
            iters_raw = telem.get("iters_prim")
            if iters_raw is None:
                iters_raw = telem.get("osqp_iters")
            osqp_iters = int(0 if iters_raw is None else iters_raw)
            f_tof_min = _extract_f_tof_min(telem)

            # v2 scalar promotions
            v_tgt_raw = telem.get("v_tgt")
            v_tgt = float(v_cmd if v_tgt_raw is None else v_tgt_raw)
            step_ms = float(telem.get("step_ms") or 0.0)
            scout_ms = float(telem.get("scout_ms") or 0.0)
            servo_pulse_us = int(telem.get("servo_pulse_us") or 1510)
            status_str = str(telem.get("status") or "")
            shield_dict = telem.get("shield") if isinstance(telem.get("shield"), dict) else {}
            shield_active = 1 if (shield_dict.get("active") or shield_dict.get("reverse_suggested")) else 0
            is_on_ramp = 1 if telem.get("is_on_ramp") else 0
            is_wrong_way = 1 if telem.get("is_wrong_way") else 0

            anomaly_flags = self.detect_anomalies(telem)

            # Fast zlib-compressed JSON payload of full frame (trajectories, costmap, gaps)
            if raw_b is not None:
                raw_bytes = raw_b
            else:
                raw_bytes = json.dumps(telem).encode("utf-8")
            compressed_payload = zlib.compress(raw_bytes, level=1)

            rows.append((
                frame_id, ts, sim_time, self.total_driving_seconds, is_driving, mode,
                v_cmd, v_real, steer_cmd_deg, steer_act_deg,
                fl_cmd, fr_cmd, rl_cmd, rr_cmd,
                fl_act, fr_act, rl_act, rr_act,
                yaw_deg, pitch_deg, roll_deg, gyro_z, accel_fwd, accel_lat,
                slip_state, slip_ratio, plan_ms, map_ms, osqp_iters, f_tof_min,
                anomaly_flags, compressed_payload,
                v_tgt, step_ms, scout_ms, servo_pulse_us, status_str,
                shield_active, is_on_ramp, is_wrong_way,
            ))
            event_rows.extend(
                self._collect_frame_events(
                    frame_id, ts, sim_time, mode, telem, shield_dict,
                    shield_active, is_on_ramp, is_wrong_way,
                )
            )
            self.total_frames_recorded += 1
            self.total_bytes_written += len(compressed_payload)

        try:
            cur.executemany("""
                INSERT INTO frames (
                    frame_id, timestamp, sim_time, driving_time, is_driving, mode,
                    v_cmd, v_real, steer_cmd_deg, steer_actual_deg,
                    fl_rpm_cmd, fr_rpm_cmd, rl_rpm_cmd, rr_rpm_cmd,
                    fl_rpm_act, fr_rpm_act, rl_rpm_act, rr_rpm_act,
                    yaw_deg, pitch_deg, roll_deg, gyro_z, accel_fwd, accel_lat,
                    slip_state, slip_ratio, plan_ms, map_ms, osqp_iters, f_tof_min,
                    anomaly_flags, payload_compressed,
                    v_tgt, step_ms, scout_ms, servo_pulse_us, status,
                    shield_active, is_on_ramp, is_wrong_way
                ) VALUES (
                    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
                    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
                    ?, ?, ?, ?, ?, ?, ?, ?
                );
            """, rows)
            if event_rows:
                cur.executemany("""
                    INSERT INTO events (
                        frame_id, timestamp, sim_time, event_type, severity, label, details_json
                    ) VALUES (?, ?, ?, ?, ?, ?, ?);
                """, event_rows)
            conn.commit()

            # Lazy auto-deletion check: prunes oldest frames to maintain 30-min driving buffer
            if (self.total_driving_seconds - self.last_prune_driving_s) >= self.prune_interval_driving_s:
                self.last_prune_driving_s = self.total_driving_seconds
                self._prune_old_driving_frames(cur, conn)

        except Exception as e:
            print(f"[FlightRecorder] Database batch insert error: {e}")

    def _prune_old_driving_frames(self, cur: sqlite3.Cursor, conn: sqlite3.Connection):
        """
        Ultra-lightweight auto-deletion:
        Keeps strictly the last max_driving_seconds (e.g. 30 min) of active driving.
        Runs lazily in bulk via indexed primary key range delete.
        """
        if self.total_driving_seconds <= self.max_driving_seconds:
            return

        cutoff_driving_s = self.total_driving_seconds - self.max_driving_seconds
        try:
            cur.execute("""
                SELECT frame_id FROM frames
                WHERE is_driving = 1 AND driving_time <= ?
                ORDER BY driving_time DESC
                LIMIT 1;
            """, (cutoff_driving_s,))
            row = cur.fetchone()

            if row and row[0]:
                cutoff_frame_id = row[0]
                cur.execute("DELETE FROM frames WHERE frame_id < ?;", (cutoff_frame_id,))
                pruned = cur.rowcount
                cur.execute("DELETE FROM events WHERE frame_id < ?;", (cutoff_frame_id,))
                conn.commit()
                if pruned > 0:
                    self.pruned_frames_count += pruned
                    self.min_frame_id = cutoff_frame_id
        except Exception as e:
            print(f"[FlightRecorder] Prune error: {e}")

    def _cleanup_old_session_files(self, max_total_dir_size_bytes: int = 10 * 1024 * 1024 * 1024):
        """Deletes oldest session files if total folder size exceeds quota (10GB)."""
        try:
            files = []
            total_size = 0
            if not os.path.exists(self.output_dir):
                return
            for f in os.listdir(self.output_dir):
                if f.endswith(".db"):
                    p = os.path.join(self.output_dir, f)
                    try:
                        sz = os.path.getsize(p)
                        mtime = os.path.getmtime(p)
                        files.append((mtime, sz, p))
                        total_size += sz
                    except Exception:
                        pass

            if total_size > max_total_dir_size_bytes:
                files.sort(key=lambda x: x[0])
                for mtime, sz, p in files:
                    if total_size <= max_total_dir_size_bytes * 0.8:
                        break
                    try:
                        os.remove(p)
                        for ext in ("-wal", "-shm"):
                            if os.path.exists(p + ext):
                                os.remove(p + ext)
                        total_size -= sz
                        print(f"[FlightRecorder] Auto-deleted old session log: {os.path.basename(p)}")
                    except Exception:
                        pass
        except Exception:
            pass

    def _check_disk_space(self):
        """Verifies free storage on the SD card."""
        try:
            total, used, free = shutil.disk_usage(self.output_dir)
            if free < self.min_free_space_bytes:
                print(f"[FlightRecorder WARNING] Low SD card storage! Free: {free / (1024**3):.2f} GB")
        except Exception:
            pass


# ==============================================================================
# LOG READER & SCRUBBER UTILITIES
# ==============================================================================

class TelemetryLogReader:
    """Provides high-speed random access and playback of recorded flight logs."""

    def __init__(self, db_path: str):
        self.db_path = db_path
        if not os.path.exists(db_path):
            raise FileNotFoundError(f"Flight database not found: {db_path}")
        self._local = threading.local()

    def _get_conn(self) -> sqlite3.Connection:
        """Returns a thread-local read-only SQLite connection for fast 60 Hz playback."""
        conn = getattr(self._local, "conn", None)
        if conn is None:
            conn = _open_sqlite_ro(self.db_path)
            self._local.conn = conn
        return conn

    def close(self):
        conn = getattr(self._local, "conn", None)
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
            self._local.conn = None

    def get_frame_bounds(self) -> tuple:
        """Returns (min_frame_id, max_frame_id, total_frames) from the database."""
        conn = self._get_conn()
        cur = conn.cursor()
        cur.execute("SELECT MIN(frame_id), MAX(frame_id), COUNT(*) FROM frames;")
        row = cur.fetchone()
        if not row or row[2] == 0:
            return (1, 0, 0)
        return (int(row[0] or 1), int(row[1] or 0), int(row[2] or 0))

    def get_session_info(self) -> Dict[str, Any]:
        """Returns full session_info metadata (backward-compatible with v1 databases)."""
        conn = _open_sqlite_ro(self.db_path)
        try:
            cur = conn.cursor()
            cols = [r[1] for r in cur.execute("PRAGMA table_info(session_info);").fetchall()]
            if not cols:
                return {"schema_version": 1, "git_commit": "", "config": {}}
            cur.execute(f"SELECT {', '.join(cols)} FROM session_info LIMIT 1;")
            row = cur.fetchone()
            if not row:
                return {"schema_version": 1, "git_commit": "", "config": {}}
            info = dict(zip(cols, row))
            info.setdefault("schema_version", 1)
            info.setdefault("git_commit", "")
            raw_cfg = info.get("config_json") or "{}"
            try:
                info["config"] = json.loads(raw_cfg) if isinstance(raw_cfg, str) else (raw_cfg or {})
            except Exception:
                info["config"] = {}
            return info
        finally:
            conn.close()

    def get_summary(self) -> Dict[str, Any]:
        """Returns session metadata, total frames, duration, and anomaly count."""
        conn = _open_sqlite_ro(self.db_path)
        cur = conn.cursor()

        cols = [r[1] for r in cur.execute("PRAGMA table_info(session_info);").fetchall()]
        sess_row = None
        if cols:
            cur.execute(f"SELECT {', '.join(cols)} FROM session_info LIMIT 1;")
            r = cur.fetchone()
            if r:
                sess_row = dict(zip(cols, r))

        cur.execute("SELECT COUNT(*), MIN(timestamp), MAX(timestamp), MIN(sim_time), MAX(sim_time) FROM frames;")
        count_row = cur.fetchone()

        cur.execute("SELECT COUNT(*), MIN(timestamp), MAX(timestamp), MIN(sim_time), MAX(sim_time), MAX(driving_time) - MIN(driving_time) FROM frames WHERE is_driving = 1;")
        driving_row = cur.fetchone()

        cur.execute("SELECT COUNT(*) FROM frames WHERE anomaly_flags > 0;")
        anom_row = cur.fetchone()

        conn.close()

        total_frames = count_row[0] if count_row else 0
        t_start = count_row[1] if count_row and count_row[1] else 0.0
        t_end = count_row[2] if count_row and count_row[2] else 0.0
        duration_s = max(0.0, t_end - t_start)
        driving_s = round(float(driving_row[5]), 1) if driving_row and driving_row[5] else 0.0

        return {
            "session_id": (sess_row or {}).get("session_id", "unknown"),
            "created_at": (sess_row or {}).get("created_at", ""),
            "schema_version": int((sess_row or {}).get("schema_version") or 1),
            "git_commit": (sess_row or {}).get("git_commit", "") or "",
            "total_frames": total_frames,
            "duration_s": round(duration_s, 2),
            "sim_duration_s": round((count_row[4] - count_row[3]) if count_row and count_row[4] else 0.0, 2),
            "driving_seconds": driving_s,
            "driving_minutes": round(driving_s / 60.0, 1),
            "anomalies_count": anom_row[0] if anom_row else 0,
            "file_size_mb": round(os.path.getsize(self.db_path) / (1024 * 1024), 2),
        }

    def get_frame(self, frame_id: int) -> Optional[Dict[str, Any]]:
        """Instant O(1) query for full uncompressed frame data."""
        conn = self._get_conn()
        cur = conn.cursor()
        cur.execute("SELECT payload_compressed FROM frames WHERE frame_id = ? LIMIT 1;", (frame_id,))
        row = cur.fetchone()

        if row and row[0]:
            try:
                raw_json = zlib.decompress(row[0]).decode("utf-8")
                data = json.loads(raw_json)
                data["_frame_id"] = frame_id
                return data
            except Exception:
                pass
        return None

    def get_anomalies(self, limit: int = 500) -> List[Dict[str, Any]]:
        """Returns indexed list of all frames flagged with anomalies (bounded to recent)."""
        conn = _open_sqlite_ro(self.db_path)
        cur = conn.cursor()
        cur.execute("""
            SELECT frame_id, timestamp, sim_time, mode, slip_state, f_tof_min, plan_ms, anomaly_flags
            FROM frames
            WHERE anomaly_flags > 0
            ORDER BY frame_id DESC
            LIMIT ?;
        """, (limit,))
        rows = cur.fetchall()
        conn.close()
        rows.reverse()

        results = []
        for r in rows:
            flags = int(r[7] or 0)
            results.append({
                "frame_id": r[0],
                "timestamp": round(r[1], 3),
                "sim_time": round(r[2], 3),
                "mode": r[3],
                "slip_state": r[4],
                "f_tof_min": round(r[5], 2),
                "plan_ms": round(r[6], 2),
                "flags": flags,
                "reasons": _decode_anomaly_flags(flags),
            })
        return results

    def get_events(self, limit: int = 500) -> List[Dict[str, Any]]:
        """Returns recorded events from v2 events table (empty list on v1 DBs)."""
        conn = _open_sqlite_ro(self.db_path)
        try:
            cur = conn.cursor()
            exists = cur.execute(
                "SELECT 1 FROM sqlite_master WHERE type='table' AND name='events' LIMIT 1;"
            ).fetchone()
            if not exists:
                return []
            cur.execute("""
                SELECT event_id, frame_id, timestamp, sim_time, event_type, severity, label, details_json
                FROM events
                ORDER BY event_id ASC
                LIMIT ?;
            """, (limit,))
            rows = cur.fetchall()
            out: List[Dict[str, Any]] = []
            for r in rows:
                try:
                    details = json.loads(r[7]) if r[7] else {}
                except Exception:
                    details = {}
                out.append({
                    "event_id": r[0],
                    "frame_id": r[1],
                    "timestamp": r[2],
                    "sim_time": r[3],
                    "event_type": r[4],
                    "severity": r[5],
                    "label": r[6],
                    "details": details,
                })
            return out
        finally:
            conn.close()

    def get_labels(self) -> List[Dict[str, Any]]:
        """Returns human/AI frame-range labels (empty list if labels table absent)."""
        conn = _open_sqlite_ro(self.db_path)
        try:
            cur = conn.cursor()
            exists = cur.execute(
                "SELECT 1 FROM sqlite_master WHERE type='table' AND name='labels' LIMIT 1;"
            ).fetchone()
            if not exists:
                return []
            cur.execute("""
                SELECT label_id, start_frame, end_frame, tag, note, author, created_at
                FROM labels
                ORDER BY start_frame ASC, label_id ASC;
            """)
            return [
                {
                    "label_id": r[0],
                    "start_frame": r[1],
                    "end_frame": r[2],
                    "tag": r[3],
                    "note": r[4],
                    "author": r[5],
                    "created_at": r[6],
                }
                for r in cur.fetchall()
            ]
        finally:
            conn.close()

    def get_range_scalars(self, start_frame: int = 1, limit: int = 500) -> List[Dict[str, Any]]:
        """Lightweight query for timeline scrubber plots without decompressing full payloads."""
        conn = _open_sqlite_ro(self.db_path)
        cur = conn.cursor()
        cur.execute("""
            SELECT frame_id, timestamp, sim_time, mode, v_cmd, v_real, steer_cmd_deg, steer_actual_deg,
                   yaw_deg, gyro_z, slip_state, slip_ratio, plan_ms, anomaly_flags
            FROM frames
            WHERE frame_id >= ?
            ORDER BY frame_id ASC
            LIMIT ?;
        """, (start_frame, limit))
        rows = cur.fetchall()
        conn.close()

        return [
            {
                "frame_id": r[0],
                "timestamp": round(r[1], 3),
                "sim_time": round(r[2], 3),
                "mode": r[3],
                "v_cmd": round(r[4], 2),
                "v_real": round(r[5], 2),
                "steer_cmd_deg": round(r[6], 1),
                "steer_actual_deg": round(r[7], 1),
                "yaw_deg": round(r[8], 1),
                "gyro_z": round(r[9], 2),
                "slip_state": r[10],
                "slip_ratio": round(r[11], 2),
                "plan_ms": round(r[12], 2),
                "anomaly_flags": r[13],
            }
            for r in rows
        ]


if __name__ == "__main__":
    scratch_dir = Path("scratch") / "test_flight_recorder"
    scratch_dir.mkdir(parents=True, exist_ok=True)

    print("Testing TelemetryFlightRecorder v2 & Active Driving Auto-Deletion...")
    rec = TelemetryFlightRecorder(output_dir=str(scratch_dir), max_driving_seconds=5.0)
    rec.start()

    # Feed 100 frames (some idle, some driving, with new v2 flags & tof_ranges_raw fallback)
    for i in range(100):
        is_driving = i > 20
        v = 2.0 if is_driving else 0.0
        frame = {
            "timestamp": time.time(),
            "sim_time": i * 0.016,
            "mode": "CRUISE" if is_driving else "IDLE",
            "status": "OPTIMAL_OSQP_MPC" if is_driving else "STOPPED",
            "v_cmd": v,
            "v_tgt": v,
            "v_real": v,
            "steer_cmd_deg": 5.0,
            "steer_actual_deg": 4.8,
            "yaw_deg": 12.0,
            "gyro_z": 0.1,
            "slip_state": "NORMAL" if i != 50 else "OVERSTEER",
            "slip_ratio": 0.02 if i != 50 else 0.45,
            "tof_ranges_raw": [0.8] * 16 if i != 60 else [0.09] + [0.8] * 15,
            "plan_ms": 4.5,
            "step_ms": 6.2,
            "scout_ms": 0.4,
            "servo_pulse_us": 1510,
            "rpms_cmd": [1000, 1000, 1000, 1000],
            "rpms_act": [995, 1005, 998, 1002],
            "shield": {"active": i == 70, "chosen_name": "k1_left" if i == 70 else None, "reverse_suggested": False},
            "is_wrong_way": (i == 75),
            "is_on_ramp": (i >= 80),
            "mcu_main_connected": True,
            "mcu_tof_connected": (i != 85),
            "mcu_msg": "Test MCU boot message" if i < 10 else "Speed ramp engaged",
            "mcu_log": ["Test MCU boot message"] if i < 10 else ["Test MCU boot message", "Speed ramp engaged"],
        }
        rec.record_frame(frame)
        time.sleep(0.005)

    rec.stop()

    reader = TelemetryLogReader(rec.db_path)
    summary = reader.get_summary()
    print("Session Summary:", summary)
    assert summary["total_frames"] == 100, f"Expected 100 frames, got {summary['total_frames']}"
    assert summary["schema_version"] == 2, f"Expected schema_version 2, got {summary['schema_version']}"

    sess_info = reader.get_session_info()
    assert sess_info.get("schema_version") == 2
    assert isinstance(sess_info.get("config"), dict)

    # Verify frame query (i=50 is frame_id 51 in 1-based indexing)
    f51 = reader.get_frame(51)
    assert f51 is not None, "Frame 51 should exist"
    assert f51["slip_state"] == "OVERSTEER", f"Slip state mismatch, got {f51['slip_state']}"

    # Verify f_tof_min fallback + new anomaly flags (i=60 proximity, i=70 shield, i=75 wrong-way, i=85 link_drop)
    anomalies = reader.get_anomalies()
    print(f"Detected {len(anomalies)} anomalies.")
    all_reasons = {r for a in anomalies for r in a["reasons"]}
    assert "SLIP/TRACTION" in all_reasons
    assert "NEAR_COLLISION" in all_reasons
    assert "SHIELD_EVADE" in all_reasons
    assert "WRONG_WAY" in all_reasons
    assert "LINK_DROP" in all_reasons

    # Verify events table (mode changes, shield, wrong_way, ramp, link, deduplicated MCU logs)
    events = reader.get_events()
    ev_types = {e["event_type"] for e in events}
    assert "MODE_CHANGE" in ev_types
    assert "SHIELD" in ev_types
    assert "WRONG_WAY" in ev_types
    assert "MCU_LOG" in ev_types
    mcu_events = [e for e in events if e["event_type"] == "MCU_LOG"]
    assert len(mcu_events) == 2, f"Expected 2 deduplicated MCU log events, got {len(mcu_events)}"

    # Verify labels read/write
    lbl_id = add_label(rec.db_path, 70, 76, "test_evade", note="unit test label", author="test")
    assert lbl_id >= 1
    labels = reader.get_labels()
    assert len(labels) == 1 and labels[0]["tag"] == "test_evade"

    reader.close()
    shutil.rmtree(scratch_dir, ignore_errors=True)
    print("FlightRecorder v2 tests passed successfully!")


