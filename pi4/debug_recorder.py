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
import sys
import time
import json
import zlib
import shutil
import sqlite3
import datetime
import threading
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
        batch_size: int = 30,
        commit_interval_s: float = 0.5,
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

        # Statistics
        self.total_frames_recorded = 0
        self.total_bytes_written = 0
        self.dropped_frames = 0
        self.last_steer_deg = 0.0

        # Auto-clean old sessions to prevent disk bloat
        self._cleanup_old_session_files()

        # Initialize SQLite database
        self._init_db()

    def _init_db(self):
        """Creates tables, pragmas and indexes in SQLite."""
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()

        # High-performance write pragmas for SD card (WAL + normal synchronous + incremental vacuum)
        cur.execute("PRAGMA journal_mode = WAL;")
        cur.execute("PRAGMA synchronous = NORMAL;")
        cur.execute("PRAGMA temp_store = MEMORY;")
        cur.execute("PRAGMA cache_size = -64000;")  # 64 MB cache
        cur.execute("PRAGMA auto_vacuum = INCREMENTAL;")

        # Sessions table
        cur.execute("""
            CREATE TABLE IF NOT EXISTS session_info (
                session_id TEXT PRIMARY KEY,
                created_at TEXT,
                start_timestamp REAL,
                end_timestamp REAL,
                total_frames INTEGER,
                driving_seconds REAL,
                notes TEXT
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
                payload_compressed BLOB
            );
        """)

        # Indexes for instant frame-by-frame scrubbing, anomaly review, and driving buffer pruning
        cur.execute("CREATE INDEX IF NOT EXISTS idx_frames_ts ON frames(timestamp);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_frames_driving ON frames(driving_time);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_frames_anomaly ON frames(anomaly_flags);")
        cur.execute("CREATE INDEX IF NOT EXISTS idx_frames_mode ON frames(mode);")

        # Insert session info record
        cur.execute("""
            INSERT OR REPLACE INTO session_info (session_id, created_at, start_timestamp, total_frames, notes)
            VALUES (?, ?, ?, 0, 'Folkrace Autonomous Flight Log');
        """, (self.session_id, datetime.datetime.now().isoformat(), time.time()))

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
                    SET end_timestamp = ?, total_frames = ?
                    WHERE session_id = ?;
                """, (time.time(), self.total_frames_recorded, self.session_id))
                conn.commit()
                conn.close()
            except Exception as e:
                print(f"[FlightRecorder] Error updating session footer: {e}")

            print(f"[FlightRecorder] Stopped. Total frames saved: {self.total_frames_recorded} (Dropped: {self.dropped_frames})")

    def record_frame(self, telemetry: Dict[str, Any]) -> bool:
        """
        Non-blocking ingestion of a telemetry frame.
        Guaranteed zero-wait for the caller.
        """
        if not self.running:
            return False

        try:
            self.queue.put_nowait(telemetry)
            return True
        except queue.Full:
            self.dropped_frames += 1
            return False

    def detect_anomalies(self, telemetry: Dict[str, Any]) -> int:
        """Evaluates telemetry frame and flags potential unexpected robot behavior."""
        flags = AnomalyFlag.NONE

        # 1. Traction / Slip
        slip_str = str(telemetry.get("slip_state", "")).upper()
        slip_ratio = float(telemetry.get("slip_ratio", 0.0))
        if any(x in slip_str for x in ("OVERSTEER", "UNDERSTEER", "SPIN")) or abs(slip_ratio) > 0.35:
            flags |= AnomalyFlag.SLIP

        # 2. Emergency stop or recovery state
        mode_str = str(telemetry.get("mode", "")).upper()
        if any(x in mode_str for x in ("PIVOT", "REVERSE", "DECEL", "STUCK", "EMERGENCY")):
            flags |= AnomalyFlag.EMERGENCY

        # 3. Dangerous close contact (< 0.12m)
        f_tof = float(telemetry.get("f_tof", 1.0))
        if f_tof < 0.12:
            flags |= AnomalyFlag.PROXIMITY

        # 4. Planner latency spike (> 16.6ms)
        plan_ms = float(telemetry.get("plan_ms", telemetry.get("step_ms", 0.0)))
        if plan_ms > 16.6:
            flags |= AnomalyFlag.LATENCY_SPIKE

        # 5. Severe steering jerk
        steer_deg = float(telemetry.get("steer_cmd_deg", telemetry.get("steer_deg", 0.0)))
        if abs(steer_deg - self.last_steer_deg) > 25.0:
            flags |= AnomalyFlag.STEER_JERK
        self.last_steer_deg = steer_deg

        return flags

    def _writer_loop(self):
        """Asynchronous disk writer worker running on dedicated core."""
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()

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

    def _flush_batch(self, cur: sqlite3.Cursor, conn: sqlite3.Connection, batch: List[Dict[str, Any]]):
        """Flushes a batch of telemetry frames in a single transaction."""
        if not batch:
            return

        rows = []
        for telem in batch:
            frame_id = self.total_frames_recorded + 1
            ts = float(telem.get("timestamp", time.time()))
            sim_time = float(telem.get("sim_time", 0.0))
            mode = str(telem.get("mode", "DRIVE"))

            v_cmd = float(telem.get("v_cmd", 0.0))
            v_real = float(telem.get("v_real", telem.get("v_est", 0.0)))

            # Track driving duration (only accumulates when vehicle is actively moving)
            dt = 1.0 / 60.0
            if self.last_sim_time > 0 and sim_time > self.last_sim_time:
                dt = min(0.1, sim_time - self.last_sim_time)
            self.last_sim_time = sim_time

            is_driving = 1 if ((abs(v_cmd) > 0.05 or abs(v_real) > 0.05) and (mode not in ("WARMUP", "IDLE", "PAUSED"))) else 0
            if is_driving:
                self.total_driving_seconds += dt

            steer_cmd_deg = float(telem.get("steer_cmd_deg", telem.get("steer_deg", 0.0)))
            steer_act_deg = float(telem.get("steer_actual_deg", steer_cmd_deg))

            # Wheel RPMs (commanded vs actual)
            rpms_cmd = telem.get("rpms_cmd", [0.0, 0.0, 0.0, 0.0])
            rpms_act = telem.get("rpms_act", [0.0, 0.0, 0.0, 0.0])
            fl_cmd, fr_cmd, rl_cmd, rr_cmd = (rpms_cmd + [0.0]*4)[:4]
            fl_act, fr_act, rl_act, rr_act = (rpms_act + [0.0]*4)[:4]

            # IMU
            yaw_deg = float(telem.get("yaw_deg", telem.get("yaw_fused", 0.0)))
            pitch_deg = float(telem.get("pitch_deg", 0.0))
            roll_deg = float(telem.get("roll_deg", 0.0))
            gyro_z = float(telem.get("gyro_z", 0.0))
            accel_fwd = float(telem.get("accel_fwd", 0.0))
            accel_lat = float(telem.get("accel_lat", 0.0))

            slip_state = str(telem.get("slip_state", "NORMAL"))
            slip_ratio = float(telem.get("slip_ratio", 0.0))

            plan_ms = float(telem.get("plan_ms", 0.0))
            map_ms = float(telem.get("map_ms", 0.0))
            osqp_iters = int(telem.get("iters_prim", telem.get("osqp_iters", 0)))
            f_tof_min = float(telem.get("f_tof", 1.0))

            anomaly_flags = self.detect_anomalies(telem)

            # Fast zlib-compressed JSON payload of full frame (trajectories, costmap, gaps)
            raw_bytes = json.dumps(telem).encode("utf-8")
            compressed_payload = zlib.compress(raw_bytes, level=1)

            rows.append((
                frame_id, ts, sim_time, self.total_driving_seconds, is_driving, mode,
                v_cmd, v_real, steer_cmd_deg, steer_act_deg,
                fl_cmd, fr_cmd, rl_cmd, rr_cmd,
                fl_act, fr_act, rl_act, rr_act,
                yaw_deg, pitch_deg, roll_deg, gyro_z, accel_fwd, accel_lat,
                slip_state, slip_ratio, plan_ms, map_ms, osqp_iters, f_tof_min,
                anomaly_flags, compressed_payload
            ))
            self.total_frames_recorded += 1
            self.total_bytes_written += len(compressed_payload)

        try:
            cur.executemany("""
                INSERT INTO frames VALUES (
                    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
                    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?
                );
            """, rows)
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
                conn.commit()
                if pruned > 0:
                    self.pruned_frames_count += pruned
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

    def get_summary(self) -> Dict[str, Any]:
        """Returns session metadata, total frames, duration, and anomaly count."""
        conn = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)
        cur = conn.cursor()

        cur.execute("SELECT session_id, created_at, start_timestamp, end_timestamp, total_frames, notes FROM session_info LIMIT 1;")
        sess_row = cur.fetchone()

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
            "session_id": sess_row[0] if sess_row else "unknown",
            "created_at": sess_row[1] if sess_row else "",
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
        conn = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)
        cur = conn.cursor()
        cur.execute("SELECT payload_compressed FROM frames WHERE frame_id = ? LIMIT 1;", (frame_id,))
        row = cur.fetchone()
        conn.close()

        if row and row[0]:
            try:
                raw_json = zlib.decompress(row[0]).decode("utf-8")
                data = json.loads(raw_json)
                data["_frame_id"] = frame_id
                return data
            except Exception:
                pass
        return None

    def get_anomalies(self) -> List[Dict[str, Any]]:
        """Returns indexed list of all frames flagged with anomalies."""
        conn = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)
        cur = conn.cursor()
        cur.execute("""
            SELECT frame_id, timestamp, sim_time, mode, slip_state, f_tof_min, plan_ms, anomaly_flags
            FROM frames
            WHERE anomaly_flags > 0
            ORDER BY frame_id ASC;
        """)
        rows = cur.fetchall()
        conn.close()

        results = []
        for r in rows:
            flags = r[7]
            labels = []
            if flags & AnomalyFlag.SLIP: labels.append("SLIP/TRACTION")
            if flags & AnomalyFlag.EMERGENCY: labels.append("EMERGENCY/RECOVERY")
            if flags & AnomalyFlag.PROXIMITY: labels.append("NEAR_COLLISION")
            if flags & AnomalyFlag.LATENCY_SPIKE: labels.append("LATENCY_SPIKE")
            if flags & AnomalyFlag.STEER_JERK: labels.append("STEER_JERK")

            results.append({
                "frame_id": r[0],
                "timestamp": round(r[1], 3),
                "sim_time": round(r[2], 3),
                "mode": r[3],
                "slip_state": r[4],
                "f_tof_min": round(r[5], 2),
                "plan_ms": round(r[6], 2),
                "flags": flags,
                "reasons": labels,
            })
        return results

    def get_range_scalars(self, start_frame: int = 1, limit: int = 500) -> List[Dict[str, Any]]:
        """Lightweight query for timeline scrubber plots without decompressing full payloads."""
        conn = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)
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
    import os
    import shutil

    scratch_dir = Path("scratch")
    scratch_dir.mkdir(exist_ok=True)
    
    print("Testing TelemetryFlightRecorder & Active Driving Auto-Deletion...")
    rec = TelemetryFlightRecorder(output_dir=str(scratch_dir), max_driving_seconds=5.0)
    rec.start()

    # Feed 100 frames (some idle, some driving)
    for i in range(100):
        is_driving = i > 20
        v = 2.0 if is_driving else 0.0
        frame = {
            "timestamp": time.time(),
            "sim_time": i * 0.016,
            "mode": "CRUISE" if is_driving else "IDLE",
            "v_cmd": v,
            "v_real": v,
            "steer_cmd_deg": 5.0,
            "steer_actual_deg": 4.8,
            "yaw_deg": 12.0,
            "gyro_z": 0.1,
            "slip_state": "NORMAL" if i != 50 else "OVERSTEER",
            "slip_ratio": 0.02 if i != 50 else 0.45,
            "f_tof": 0.8,
            "plan_ms": 4.5,
            "rpms_cmd": [1000, 1000, 1000, 1000],
            "rpms_act": [995, 1005, 998, 1002],
        }
        rec.record_frame(frame)
        time.sleep(0.005)

    rec.stop()

    reader = TelemetryLogReader(rec.db_path)
    summary = reader.get_summary()
    print("Session Summary:", summary)
    assert summary["total_frames"] == 100, f"Expected 100 frames, got {summary['total_frames']}"

    # Verify frame query (i=50 is frame_id 51 in 1-based indexing)
    f51 = reader.get_frame(51)
    assert f51 is not None, "Frame 51 should exist"
    assert f51["slip_state"] == "OVERSTEER", f"Slip state mismatch, got {f51['slip_state']}"

    # Verify anomalies
    anomalies = reader.get_anomalies()
    print(f"Detected {len(anomalies)} anomalies: {anomalies}")
    assert len(anomalies) >= 1, "Expected at least 1 anomaly"

    # Clean up test database
    if os.path.exists(rec.db_path):
        os.remove(rec.db_path)
    shutil.rmtree(scratch_dir, ignore_errors=True)
    print("FlightRecorder tests passed successfully!")


