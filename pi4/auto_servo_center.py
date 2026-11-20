#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: AUTOMATIC SERVO CENTER CALIBRATION
================================================================================
  Closed-loop servo neutral trim detector:
    1. Runs pre-flight checks (planner idle, MCU/IMU/STM32 online, fresh data,
       no fake data, and sufficient front ToF clearance).
    2. Disables ESP32 GYRO countersteer and Rear TV during the straight-line test
       (restored unconditionally in finally/stop) and locks steering center.
    3. Measures stationary IMU gyro_z bias with movement & noise checks.
    4. Drives forward and backward at half-MU acceleration & low cruise speed,
       actively braking to a verified standstill after each leg and matching the
       return distance to the exact forward distance (including brake rollout).
    5. Rejects IMU bump outliers with MAD filtering and cancels residual gyro
       bias via exact unequal-speed forward/backward curvature averaging:
         kappa_err = (omega_fwd - omega_bwd) / (v_fwd + v_bwd)
    6. Uses a safeguarded secant / bracketed Illinois root-finder (no double
       integrator) to converge center_us within tolerance (<= 0.8 deg/m).
    7. Persists converged center to servo_trim.json and ESP32 NVS (TRIM_SAVE).
================================================================================
"""

import math
import os
import statistics
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(CURR_DIR)
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

try:
    from core.vehicle_params import (
        GRAVITY_G,
        MAX_STEERING_RAD,
        SERVO_PULSE_MAX_US,
        SERVO_PULSE_MIN_US,
        SERVO_PULSE_NEUTRAL_US,
        TIRE_MU,
        WHEELBASE,
    )
except Exception:
    TIRE_MU = 0.80
    GRAVITY_G = 9.80665
    WHEELBASE = 0.1325
    MAX_STEERING_RAD = 0.562285
    SERVO_PULSE_NEUTRAL_US = 1510.0
    SERVO_PULSE_MIN_US = 1239.0
    SERVO_PULSE_MAX_US = 1781.0


BASE_SERVO_NEUTRAL_US = float(SERVO_PULSE_NEUTRAL_US)
US_PER_RAD = (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / (2.0 * MAX_STEERING_RAD)
VALID_TOF_STATUSES = {5, 6, 9}


def _median(vals: List[float]) -> float:
    return float(statistics.median(vals))


class _TelemetryTracker:
    """Deduplicates telemetry frames and extracts wrap-safe elapsed dt."""

    def __init__(
        self,
        get_telemetry: Callable[[], Dict[str, Any]],
        stale_timeout_s: float,
        now_fn: Callable[[], float] = time.monotonic,
    ):
        self._get_telemetry = get_telemetry
        self._stale_timeout_s = stale_timeout_s
        self._now = now_fn
        self._prev_key: Any = None
        self._has_key = False
        self._prev_mcu_us: Optional[int] = None
        self._last_fresh_wall_t = self._now()

    @staticmethod
    def _packet_key(pkt: Dict[str, Any]) -> Any:
        if pkt.get("seq") is not None or pkt.get("mcu_timestamp_us") is not None:
            return (pkt.get("seq"), pkt.get("mcu_timestamp_us"))
        if pkt.get("timestamp") is not None:
            return ("ts", pkt.get("timestamp"))
        return None

    def poll(self, now: Optional[float] = None) -> Tuple[Dict[str, Any], bool, float, bool]:
        """Returns (pkt, is_fresh, dt_s, is_stale)."""
        t_now = self._now() if now is None else now
        pkt = self._get_telemetry() or {}
        key = self._packet_key(pkt)

        if key is None:
            dt = max(0.002, min(0.10, t_now - self._last_fresh_wall_t))
            self._last_fresh_wall_t = t_now
            return pkt, True, dt, False

        if not self._has_key or key != self._prev_key:
            dt = max(0.002, min(0.10, t_now - self._last_fresh_wall_t))
            curr_mcu = pkt.get("mcu_timestamp_us")
            if self._has_key and curr_mcu is not None and self._prev_mcu_us is not None:
                delta_us = (int(curr_mcu) - int(self._prev_mcu_us)) & 0xFFFFFFFF
                dt_mcu = delta_us * 1e-6
                if 0.002 <= dt_mcu <= 0.10:
                    dt = dt_mcu
            self._prev_key = key
            self._has_key = True
            self._prev_mcu_us = int(curr_mcu) if curr_mcu is not None else None
            self._last_fresh_wall_t = t_now
            return pkt, True, dt, False

        is_stale = (t_now - self._last_fresh_wall_t) > self._stale_timeout_s
        return pkt, False, 0.0, is_stale


class AutoServoCenterCalibrator:
    """Runs the forward/backward automatic servo centering calibration loop."""

    DRIVE_DISTANCE_M = 1.00
    CRUISE_SPEED_MPS = 0.60
    HALF_MU_ACCEL_MPS2 = 0.5 * TIRE_MU * GRAVITY_G
    RAMP_ACCEL_MPS2 = min(1.50, HALF_MU_ACCEL_MPS2)
    BRAKE_DECEL_MPS2 = 1.40

    BIAS_SAMPLE_SEC = 0.50
    BIAS_POLL_SEC = 0.015
    SETTLE_STOP_SEC = 0.20
    STANDSTILL_SPEED_MPS = 0.025
    STANDSTILL_HOLD_SEC = 0.20
    BRAKE_TIMEOUT_SEC = 1.5
    BRAKE_FALLBACK_SEC = 0.25
    POST_TRIM_WAIT_SEC = 0.12
    CONTROL_DT_SEC = 0.02
    LEG_TIMEOUT_SEC = 6.0
    STALL_TIMEOUT_SEC = 1.5
    TELEM_STALE_SEC = 0.35

    MIN_STEADY_SPEED_MPS = 0.25
    CONVERGE_DRIFT_DEG_PER_M = 0.8
    MAX_ITERATIONS = 6

    MAX_STEP_US = 80.0
    MIN_CENTER_US = max(1100.0, SERVO_PULSE_MIN_US - 50.0)
    MAX_CENTER_US = min(1900.0, SERVO_PULSE_MAX_US + 50.0)
    MIN_FRONT_TOF_M = 0.25

    def __init__(
        self,
        get_telemetry: Callable[[], Dict[str, Any]],
        send_cmd: Callable[[bytes], bool],
        get_center_us: Callable[[], float],
        save_center_us: Callable[[float], float],
    ):
        self._get_telemetry = get_telemetry
        self._send_cmd = send_cmd
        self._get_center_us = get_center_us
        self._save_center_us = save_center_us

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._tof_required: bool = False

        self._status: Dict[str, Any] = {
            "running": False,
            "phase": "IDLE",
            "iteration": 0,
            "max_iterations": self.MAX_ITERATIONS,
            "distance_m": 0.0,
            "target_distance_m": self.DRIVE_DISTANCE_M,
            "cruise_speed_mps": self.CRUISE_SPEED_MPS,
            "tolerance_deg_per_m": self.CONVERGE_DRIFT_DEG_PER_M,
            "gyro_offset_rad_s": 0.0,
            "gyro_bias_rad_s": 0.0,
            "resid_bias_rad_s": 0.0,
            "drift_deg_per_m": 0.0,
            "fwd_drift_deg_per_m": 0.0,
            "bwd_drift_deg_per_m": 0.0,
            "servo_center_us": float(get_center_us()),
            "last_step_us": 0.0,
            "message": "Ready",
            "history": [],
        }

    def is_running(self) -> bool:
        with self._lock:
            return bool(self._status["running"])

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            out = dict(self._status)
            out["history"] = list(self._status["history"])
            return out

    def _now(self) -> float:
        return time.monotonic()

    def _sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def _update_status(self, **kwargs: Any) -> None:
        with self._lock:
            self._status.update(kwargs)

    @staticmethod
    def _read_speed_est(pkt: Dict[str, Any]) -> float:
        v_raw = pkt.get("v_est")
        if v_raw is None:
            v_raw = pkt.get("v_real")
        return float(v_raw or 0.0)

    def _min_front_tof_m(self, pkt: Dict[str, Any]) -> Optional[float]:
        if not pkt.get("mcu_tof_connected"):
            return None
        tof = pkt.get("tof_ranges") or pkt.get("tof_ranges_raw") or []
        if len(tof) < 12:
            return None
        statuses = pkt.get("tof_target0_status")
        front_vals: List[float] = []
        for idx in (0, 1, 2, 3, 8, 9, 10, 11):
            val = tof[idx]
            if val is None:
                continue
            fval = float(val)
            if 0.035 < fval < 4.5:
                front_vals.append(fval)
            elif (
                0.02 < fval <= 0.035
                and isinstance(statuses, (list, tuple))
                and idx < len(statuses)
                and int(statuses[idx]) in VALID_TOF_STATUSES
            ):
                front_vals.append(fval)
        return min(front_vals) if front_vals else None

    def _check_front_obstacle(self, pkt: Dict[str, Any], speed_mps: float) -> Optional[float]:
        min_front = self._min_front_tof_m(pkt)
        if min_front is None:
            return None
        stop_dist = max(
            self.MIN_FRONT_TOF_M,
            (speed_mps * speed_mps) / (2.0 * self.BRAKE_DECEL_MPS2) + 0.18,
        )
        return min_front if min_front < stop_dist else None

    def _preflight_check(self, pkt: Dict[str, Any], drive_dist_m: float) -> Optional[str]:
        if pkt.get("planner_running"):
            return "Stop Path Planner before running Auto Servo Center"
        if pkt.get("mode") == "ESTOP":
            return "Emergency stop active — clear ESTOP or RESET before Auto Servo Center"

        pkt_age = pkt.get("_pkt_age_s")
        if pkt_age is not None and float(pkt_age) > 0.5:
            return "MCU telemetry offline or stale — connect ESP32 Main before starting"

        if "mcu_main_connected" in pkt and pkt.get("mcu_main_connected") is False:
            return "ESP32 Main offline — check USB connection"

        flags = pkt.get("flags") or {}
        if isinstance(flags, dict):
            if flags.get("fake_data"):
                return "Disable Fake Data (Bench) mode before running Auto Servo Center"
            if "imu_online" in flags or "imu" in flags:
                imu_ok = flags.get("imu_online") if "imu_online" in flags else flags.get("imu")
                if not imu_ok:
                    return "BNO086 IMU offline — cannot measure yaw drift"
            if "stm32_front" in flags or "stm32_rear" in flags:
                if not flags.get("stm32_front") and not flags.get("stm32_rear"):
                    return "STM32 motor controllers offline — cannot drive wheels"

        min_front = self._min_front_tof_m(pkt)
        if min_front is not None:
            req_clearance = max(self.MIN_FRONT_TOF_M, min(drive_dist_m + 0.15, 2.20))
            if min_front < req_clearance:
                return f"Front obstacle too close ({min_front:.2f}m < {req_clearance:.2f}m required)"
        return None

    def start(
        self,
        max_iterations: Optional[int] = None,
        cruise_speed: Optional[float] = None,
        drive_distance_m: Optional[float] = None,
        tolerance_deg_per_m: Optional[float] = None,
        trim_offset_us: Optional[float] = None,
    ) -> Dict[str, Any]:
        with self._lock:
            if self._status["running"]:
                return {"started": False, "error": "Auto servo centering already running"}

            max_iters = int(max_iterations) if max_iterations is not None else self.MAX_ITERATIONS
            max_iters = max(2, min(12, max_iters))

            speed = float(cruise_speed) if cruise_speed is not None else self.CRUISE_SPEED_MPS
            speed = max(0.35, min(1.00, speed))

            drive_dist = float(drive_distance_m) if drive_distance_m is not None else self.DRIVE_DISTANCE_M
            drive_dist = max(0.50, min(2.00, drive_dist))

            tol_deg_m = float(tolerance_deg_per_m) if tolerance_deg_per_m is not None else self.CONVERGE_DRIFT_DEG_PER_M
            tol_deg_m = max(0.3, min(3.0, tol_deg_m))

            trim_off = float(trim_offset_us) if trim_offset_us is not None else 0.0
            trim_off = max(-60.0, min(60.0, trim_off))

            telem = self._get_telemetry() or {}
            err = self._preflight_check(telem, drive_dist)
            if err:
                return {"started": False, "error": err}

            self._tof_required = bool(telem.get("mcu_tof_connected"))
            self._stop_event.clear()
            base_center = float(self._get_center_us())
            initial_center = round(max(self.MIN_CENTER_US, min(self.MAX_CENTER_US, base_center + trim_off)), 1)

            self._status.update(
                {
                    "running": True,
                    "phase": "STARTING",
                    "iteration": 0,
                    "max_iterations": max_iters,
                    "distance_m": 0.0,
                    "target_distance_m": round(drive_dist, 2),
                    "cruise_speed_mps": round(speed, 2),
                    "tolerance_deg_per_m": round(tol_deg_m, 2),
                    "gyro_offset_rad_s": 0.0,
                    "gyro_bias_rad_s": 0.0,
                    "resid_bias_rad_s": 0.0,
                    "drift_deg_per_m": 0.0,
                    "fwd_drift_deg_per_m": 0.0,
                    "bwd_drift_deg_per_m": 0.0,
                    "servo_center_us": initial_center,
                    "last_step_us": 0.0,
                    "message": "Starting auto servo center...",
                    "history": [],
                }
            )
            self._thread = threading.Thread(
                target=self._run_worker,
                args=(initial_center, max_iters, speed, drive_dist, tol_deg_m),
                daemon=True,
                name="AutoServoCenter",
            )
            self._thread.start()
            return {"started": True, "servo_center_us": initial_center}

    def stop(self, reason: str = "Stopped by user") -> None:
        self._stop_event.set()
        if self.is_running():
            self._update_status(phase="ABORTED", message=reason)
        self._send_cmd(b"V 0.0\n")
        self._send_cmd(b"GYRO 1\n")
        self._send_cmd(b"TV 1\n")
        self._send_cmd(b"RESET\n")

    def _apply_trim(self, center_us: float, lock_steer: bool = False) -> float:
        clamped = round(max(self.MIN_CENTER_US, min(self.MAX_CENTER_US, center_us)), 1)
        trim_offset = max(-500.0, min(500.0, clamped - BASE_SERVO_NEUTRAL_US))
        if lock_steer:
            self._send_cmd(b"A 90.0\n")
        self._send_cmd(f"TRIM {trim_offset:.1f}\n".encode("utf-8"))
        return clamped

    def _measure_static_gyro_bias(self) -> Dict[str, Any]:
        self._update_status(phase="BIAS_CAL", message="Measuring stationary IMU gyro bias...")
        tracker = _TelemetryTracker(self._get_telemetry, self.TELEM_STALE_SEC, self._now)
        samples: List[float] = []
        t_end = self._now() + self.BIAS_SAMPLE_SEC

        while self._now() < t_end and not self._stop_event.is_set():
            pkt, is_fresh, _, is_stale = tracker.poll()
            if pkt.get("mode") == "ESTOP":
                return {"ok": False, "error": "Emergency stop active"}
            if is_stale:
                return {"ok": False, "error": "Telemetry stalled during gyro bias calibration"}
            if abs(self._read_speed_est(pkt)) > 0.05:
                return {"ok": False, "error": "Vehicle moving during stationary gyro bias calibration"}
            if is_fresh:
                samples.append(float(pkt.get("gyro_z") or 0.0))
            self._sleep(self.BIAS_POLL_SEC)

        if self._stop_event.is_set():
            return {"ok": False, "error": "Stopped by user"}
        if len(samples) < 4:
            return {"ok": False, "error": "Insufficient fresh IMU samples during gyro bias calibration"}

        ordered = sorted(samples)
        if len(ordered) >= 8:
            trim_n = max(1, len(ordered) // 10)
            trimmed = ordered[trim_n:-trim_n]
        else:
            trimmed = ordered

        mean_val = sum(trimmed) / len(trimmed)
        if len(trimmed) >= 2:
            var = sum((x - mean_val) ** 2 for x in trimmed) / len(trimmed)
            if math.sqrt(var) > 0.08:
                return {"ok": False, "error": "IMU gyro noisy or bumped during stationary bias calibration"}

        return {"ok": True, "bias": mean_val}

    def _active_brake_stop(self, direction: int, phase_name: str, base_dist_m: float, target_dist_m: float) -> Dict[str, Any]:
        self._update_status(phase=phase_name, message="Active braking to standstill...")
        self._send_cmd(b"BRAKE\n")

        tracker = _TelemetryTracker(self._get_telemetry, self.TELEM_STALE_SEC, self._now)
        brake_dist_m = 0.0
        t_start = self._now()
        last_brake_tx = t_start
        standstill_since: Optional[float] = None
        sent_v0_fallback = False

        while not self._stop_event.is_set():
            now = self._now()
            pkt, is_fresh, dt, is_stale = tracker.poll(now)
            if pkt.get("mode") == "ESTOP":
                return {"ok": False, "error": "Emergency stop active"}
            if is_stale:
                return {"ok": False, "error": f"Telemetry stalled during {phase_name}"}

            if (now - t_start) > self.BRAKE_FALLBACK_SEC and not sent_v0_fallback:
                v_cmd_pkt = pkt.get("v_cmd")
                if v_cmd_pkt is not None and abs(float(v_cmd_pkt)) > 0.02:
                    self._send_cmd(b"V 0.0\n")
                    sent_v0_fallback = True

            if not sent_v0_fallback and (now - last_brake_tx) >= 0.10:
                self._send_cmd(b"BRAKE\n")
                last_brake_tx = now

            v_est = self._read_speed_est(pkt)
            if is_fresh:
                v_signed = direction * v_est
                brake_dist_m += max(0.0, v_signed) * dt
                self._update_status(distance_m=round(base_dist_m + brake_dist_m, 3))

                if abs(v_est) <= self.STANDSTILL_SPEED_MPS:
                    if standstill_since is None:
                        standstill_since = now
                    elif (now - standstill_since) >= self.STANDSTILL_HOLD_SEC:
                        return {"ok": True, "brake_dist_m": brake_dist_m}
                else:
                    standstill_since = None

            if (now - t_start) > self.BRAKE_TIMEOUT_SEC:
                self._send_cmd(b"V 0.0\n")
                return {
                    "ok": False,
                    "error": f"{phase_name} failed to reach standstill within {self.BRAKE_TIMEOUT_SEC:.1f}s (v={v_est:.2f} m/s)",
                }

            self._sleep(self.CONTROL_DT_SEC)

        return {"ok": False, "error": "Stopped by user"}

    @staticmethod
    def _filter_steady_samples(samples: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
        """Applies MAD outlier rejection on gyro_z samples and returns (omega_avg, v_avg, dt_sum)."""
        omegas = [s[0] for s in samples]
        med = _median(omegas)
        abs_devs = [abs(w - med) for w in omegas]
        mad = _median(abs_devs)
        thresh = max(0.08, 3.5 * 1.4826 * mad)
        inliers = [s for s in samples if abs(s[0] - med) <= thresh]
        if not inliers:
            inliers = samples
        dt_sum = sum(s[2] for s in inliers)
        if dt_sum <= 0.0:
            return 0.0, 0.0, 0.0
        omega_avg = sum(s[0] * s[2] for s in inliers) / dt_sum
        v_avg = sum(s[1] * s[2] for s in inliers) / dt_sum
        return omega_avg, v_avg, dt_sum

    def _drive_leg(self, direction: int, speed_mps: float, target_dist_m: float, gyro_bias: float) -> Dict[str, Any]:
        """
        Drives forward (direction=+1) or backward (direction=-1) with trapezoidal
        half-MU ramp, active braking, MAD outlier rejection, and return-distance tracking.
        """
        phase = "FORWARD" if direction > 0 else "BACKWARD"
        self._update_status(
            phase=phase,
            distance_m=0.0,
            target_distance_m=round(target_dist_m, 2),
            message=f"Driving {'forward' if direction > 0 else 'backward'} {target_dist_m:.2f}m @ {speed_mps:.2f} m/s...",
        )

        tracker = _TelemetryTracker(self._get_telemetry, self.TELEM_STALE_SEC, self._now)
        dist_m = 0.0
        v_cmd = 0.0
        t_start = self._now()
        t_prev_cmd = t_start
        last_motion_t = t_start
        steady_samples: List[Tuple[float, float, float]] = []

        min_steady_v = max(self.MIN_STEADY_SPEED_MPS, 0.78 * speed_mps)
        steady_dist_min = min(0.20, 0.22 * target_dist_m)
        steady_rem_min = min(0.18, 0.20 * target_dist_m)

        while not self._stop_event.is_set():
            now = self._now()
            cmd_dt = max(0.002, min(0.10, now - t_prev_cmd))
            t_prev_cmd = now

            pkt, is_fresh, dt, is_stale = tracker.poll(now)
            if pkt.get("mode") == "ESTOP":
                return {"ok": False, "error": "Emergency stop active"}
            if is_stale:
                return {"ok": False, "error": f"Telemetry stalled during {phase}"}

            if direction > 0:
                if self._tof_required and (not pkt.get("mcu_tof_connected") or pkt.get("tof_front_ok") is False):
                    self._send_cmd(b"BRAKE\n")
                    return {"ok": False, "error": "Front ToF link lost during forward drive"}
                obs_dist = self._check_front_obstacle(pkt, max(abs(v_cmd), abs(self._read_speed_est(pkt))))
                if obs_dist is not None:
                    self._send_cmd(b"BRAKE\n")
                    return {"ok": False, "error": f"Obstacle detected ahead ({obs_dist:.2f}m)"}

            v_est = self._read_speed_est(pkt)
            v_signed = direction * v_est
            gyro_z = float(pkt.get("gyro_z") or 0.0) - gyro_bias
            slip_ratio = float(pkt.get("slip_ratio") or 0.0)

            if is_fresh:
                dist_m += max(0.0, v_signed) * dt
                self._update_status(distance_m=round(dist_m, 3))

            if abs(v_est) >= 0.05:
                last_motion_t = now
            elif (now - t_start) > 0.6 and (now - last_motion_t) > self.STALL_TIMEOUT_SEC:
                return {"ok": False, "error": "No wheel motion detected (check motors/battery/encoders)"}

            rem_m = max(0.0, target_dist_m - dist_m)
            v_brake_cap = math.sqrt(max(0.0, 2.0 * self.BRAKE_DECEL_MPS2 * rem_m))
            if dist_m >= target_dist_m or rem_m <= 0.025 or v_brake_cap <= 0.08:
                break
            if (now - t_start) > self.LEG_TIMEOUT_SEC:
                return {"ok": False, "error": f"{phase} leg timed out at {dist_m:.2f}m"}

            v_target = direction * min(speed_mps, max(0.12, v_brake_cap))
            is_accel = (abs(v_target) > abs(v_cmd)) and (v_target * v_cmd >= 0.0)
            max_dv = (self.RAMP_ACCEL_MPS2 if is_accel else self.BRAKE_DECEL_MPS2) * cmd_dt
            if v_target > v_cmd:
                v_cmd = min(v_target, v_cmd + max_dv)
            else:
                v_cmd = max(v_target, v_cmd - max_dv)

            self._send_cmd(f"V {v_cmd:.2f}\n".encode("utf-8"))

            if (
                is_fresh
                and v_signed >= min_steady_v
                and abs(v_signed - speed_mps) <= 0.20
                and dist_m >= steady_dist_min
                and rem_m >= steady_rem_min
                and slip_ratio <= 0.35
            ):
                steady_samples.append((gyro_z, v_signed, dt))

            self._sleep(self.CONTROL_DT_SEC)

        if self._stop_event.is_set():
            return {"ok": False, "error": "Stopped by user"}

        brake_phase = "BRAKE_FWD" if direction > 0 else "BRAKE_BWD"
        brake_res = self._active_brake_stop(direction, brake_phase, dist_m, target_dist_m)
        if not brake_res["ok"]:
            return brake_res

        total_dist_m = dist_m + float(brake_res["brake_dist_m"])
        if len(steady_samples) < 5:
            return {"ok": False, "error": f"Insufficient steady-state samples during {phase}"}

        omega_avg, v_avg, steady_dt_sum = self._filter_steady_samples(steady_samples)
        if steady_dt_sum < 0.20 or v_avg < self.MIN_STEADY_SPEED_MPS:
            return {"ok": False, "error": f"Insufficient steady-state cruise duration during {phase}"}

        return {
            "ok": True,
            "omega_avg": omega_avg,
            "v_avg": v_avg,
            "dist_m": dist_m,
            "total_dist_m": total_dist_m,
        }

    @classmethod
    def _compute_next_step_us(
        cls,
        center_us: float,
        kappa_err: float,
        prev_u: Optional[float],
        prev_k: Optional[float],
        u_pos: Optional[float],
        k_pos: Optional[float],
        u_neg: Optional[float],
        k_neg: Optional[float],
        w_pos: float,
        w_neg: float,
    ) -> Tuple[float, str]:
        """Safeguarded Illinois bracketed / secant / kinematic step (u_pos < u_neg)."""
        s_nom = -1.0 / (WHEELBASE * US_PER_RAD)

        if (
            u_pos is not None
            and u_neg is not None
            and k_pos is not None
            and k_neg is not None
            and u_pos < u_neg
            and k_pos > 0.0
            and k_neg < 0.0
        ):
            kp = w_pos * k_pos
            kn = w_neg * k_neg
            frac = kp / max(1e-9, kp - kn)
            frac = max(0.15, min(0.85, frac))
            u_next = u_pos + frac * (u_neg - u_pos)
            step_us = u_next - center_us
            method = "bracket"
        elif prev_u is not None and prev_k is not None and abs(center_us - prev_u) >= 3.0:
            slope = (kappa_err - prev_k) / (center_us - prev_u)
            if slope >= -0.20 * abs(s_nom):
                step_us = 0.85 * math.atan(kappa_err * WHEELBASE) * US_PER_RAD
                method = "kinematic"
            else:
                clamped_slope = max(-2.5 * abs(s_nom), min(-0.35 * abs(s_nom), slope))
                step_us = -0.85 * (kappa_err / clamped_slope)
                method = "secant"
        else:
            step_us = 0.85 * math.atan(kappa_err * WHEELBASE) * US_PER_RAD
            method = "kinematic"

        step_us = max(-cls.MAX_STEP_US, min(cls.MAX_STEP_US, step_us))
        if abs(step_us) < 0.5:
            step_us = 0.5 if kappa_err > 0.0 else -0.5
        return step_us, method

    def _run_worker(
        self,
        initial_center_us: float,
        max_iters: int,
        cruise_speed: float,
        drive_distance_m: float,
        tol_deg_per_m: float,
    ) -> None:
        center_us = initial_center_us
        history: List[Dict[str, Any]] = []

        prev_u: Optional[float] = None
        prev_k: Optional[float] = None
        u_pos: Optional[float] = None
        k_pos: Optional[float] = None
        u_neg: Optional[float] = None
        k_neg: Optional[float] = None
        w_pos = 1.0
        w_neg = 1.0
        last_side: Optional[str] = None
        same_side_count = 0

        try:
            self._send_cmd(b"MOTORS\n")
            self._send_cmd(b"GYRO 0\n")
            self._send_cmd(b"TV 0\n")
            center_us = self._apply_trim(center_us, lock_steer=True)
            self._sleep(self.POST_TRIM_WAIT_SEC)

            bias_res = self._measure_static_gyro_bias()
            if not bias_res["ok"]:
                if not self._stop_event.is_set():
                    self._update_status(phase="ERROR", message=bias_res["error"])
                return
            gyro_bias = float(bias_res["bias"])
            self._update_status(gyro_bias_rad_s=round(gyro_bias, 4))

            converged = False
            for it in range(1, max_iters + 1):
                if self._stop_event.is_set():
                    break

                self._update_status(iteration=it, servo_center_us=center_us)

                fwd = self._drive_leg(+1, cruise_speed, drive_distance_m, gyro_bias)
                if not fwd["ok"]:
                    if not self._stop_event.is_set():
                        self._update_status(phase="ERROR", message=fwd["error"])
                    return

                bwd_target_dist = max(0.35, min(drive_distance_m * 1.35, float(fwd["total_dist_m"])))
                bwd = self._drive_leg(-1, cruise_speed, bwd_target_dist, gyro_bias)
                if not bwd["ok"]:
                    if not self._stop_event.is_set():
                        self._update_status(phase="ERROR", message=bwd["error"])
                    return

                v_fwd = max(0.20, float(fwd["v_avg"]))
                v_bwd = max(0.20, float(bwd["v_avg"]))
                omega_fwd = float(fwd["omega_avg"])
                omega_bwd = float(bwd["omega_avg"])

                # Exact unequal-speed gyro-bias cancellation:
                # Forward (+v_fwd):  omega_fwd = +kappa_err * v_fwd + b_resid
                # Backward (-v_bwd): omega_bwd = -kappa_err * v_bwd + b_resid
                kappa_err = (omega_fwd - omega_bwd) / (v_fwd + v_bwd)
                b_resid = (v_bwd * omega_fwd + v_fwd * omega_bwd) / (v_fwd + v_bwd)

                drift_fwd_deg_per_m = math.degrees(omega_fwd / v_fwd)
                drift_bwd_deg_per_m = math.degrees(-omega_bwd / v_bwd)
                drift_deg_per_m = math.degrees(kappa_err)
                gyro_equiv = kappa_err * cruise_speed

                entry = {
                    "iter": it,
                    "omega_fwd": round(omega_fwd, 4),
                    "omega_bwd": round(omega_bwd, 4),
                    "v_fwd": round(v_fwd, 3),
                    "v_bwd": round(v_bwd, 3),
                    "fwd_drift_deg_per_m": round(drift_fwd_deg_per_m, 2),
                    "bwd_drift_deg_per_m": round(drift_bwd_deg_per_m, 2),
                    "drift_deg_per_m": round(drift_deg_per_m, 2),
                    "resid_bias_rad_s": round(b_resid, 4),
                    "center_us": round(center_us, 1),
                    "step_us": 0.0,
                    "method": "converged",
                }

                if abs(drift_deg_per_m) <= tol_deg_per_m:
                    history.append(entry)
                    saved = self._save_center_us(center_us)
                    self._apply_trim(saved)
                    self._sleep(0.05)
                    self._send_cmd(b"TRIM_SAVE\n")
                    self._sleep(0.05)
                    self._send_cmd(b"TRIM_GET\n")
                    self._update_status(
                        phase="CONVERGED",
                        gyro_offset_rad_s=round(gyro_equiv, 4),
                        resid_bias_rad_s=round(b_resid, 4),
                        drift_deg_per_m=round(drift_deg_per_m, 2),
                        fwd_drift_deg_per_m=round(drift_fwd_deg_per_m, 2),
                        bwd_drift_deg_per_m=round(drift_bwd_deg_per_m, 2),
                        servo_center_us=saved,
                        last_step_us=0.0,
                        history=history,
                        message=f"Converged in {it} tries! Center = {saved:.1f} us (drift {drift_deg_per_m:+.2f} deg/m)",
                    )
                    converged = True
                    break

                # Update root-finding bracket before computing step
                if kappa_err > 0.0:
                    u_pos, k_pos = center_us, kappa_err
                    if u_neg is not None and u_pos >= u_neg:
                        u_neg, k_neg = None, None
                        w_pos, w_neg = 1.0, 1.0
                        last_side, same_side_count = None, 0
                    else:
                        same_side_count = (same_side_count + 1) if last_side == "pos" else 1
                        last_side = "pos"
                        w_pos = 1.0
                        if same_side_count >= 2:
                            w_neg *= 0.5
                else:
                    u_neg, k_neg = center_us, kappa_err
                    if u_pos is not None and u_pos >= u_neg:
                        u_pos, k_pos = None, None
                        w_pos, w_neg = 1.0, 1.0
                        last_side, same_side_count = None, 0
                    else:
                        same_side_count = (same_side_count + 1) if last_side == "neg" else 1
                        last_side = "neg"
                        w_neg = 1.0
                        if same_side_count >= 2:
                            w_pos *= 0.5

                step_us, method = self._compute_next_step_us(
                    center_us=center_us,
                    kappa_err=kappa_err,
                    prev_u=prev_u,
                    prev_k=prev_k,
                    u_pos=u_pos,
                    k_pos=k_pos,
                    u_neg=u_neg,
                    k_neg=k_neg,
                    w_pos=w_pos,
                    w_neg=w_neg,
                )

                next_center = round(max(self.MIN_CENTER_US, min(self.MAX_CENTER_US, center_us + step_us)), 1)
                if abs(next_center - center_us) < 0.05:
                    entry["method"] = "limit"
                    history.append(entry)
                    self._update_status(
                        phase="ERROR",
                        history=history,
                        message=f"Servo trim reached hard limit ({center_us:.1f} us) before converging",
                    )
                    return

                prev_u = center_us
                prev_k = kappa_err
                applied_step = round(next_center - center_us, 1)
                entry["step_us"] = applied_step
                entry["method"] = method
                history.append(entry)

                center_us = self._apply_trim(next_center)
                self._update_status(
                    phase="ADJUSTING",
                    gyro_offset_rad_s=round(gyro_equiv, 4),
                    resid_bias_rad_s=round(b_resid, 4),
                    drift_deg_per_m=round(drift_deg_per_m, 2),
                    fwd_drift_deg_per_m=round(drift_fwd_deg_per_m, 2),
                    bwd_drift_deg_per_m=round(drift_bwd_deg_per_m, 2),
                    servo_center_us=center_us,
                    last_step_us=applied_step,
                    history=history,
                    message=(
                        f"Iter {it}/{max_iters} ({method}): drift {drift_deg_per_m:+.2f} deg/m "
                        f"-> trim {applied_step:+.1f} us (center {center_us:.1f} us)"
                    ),
                )
                self._sleep(self.POST_TRIM_WAIT_SEC)

            if not converged and not self._stop_event.is_set() and history:
                best = min(history, key=lambda h: abs(h["drift_deg_per_m"]))
                init_drift = abs(history[0]["drift_deg_per_m"])
                improved = abs(best["drift_deg_per_m"]) < init_drift
                within_soft_band = abs(best["drift_deg_per_m"]) <= (3.0 * tol_deg_per_m)

                if improved or within_soft_band:
                    saved = self._save_center_us(best["center_us"])
                    self._apply_trim(saved)
                    self._sleep(0.05)
                    self._send_cmd(b"TRIM_SAVE\n")
                    self._sleep(0.05)
                    self._send_cmd(b"TRIM_GET\n")
                    msg = (
                        f"Completed {max_iters} tries without meeting +/-{tol_deg_per_m:.1f} deg/m. "
                        f"Saved best = {saved:.1f} us (drift {best['drift_deg_per_m']:+.2f} deg/m)"
                    )
                    final_center = saved
                else:
                    self._apply_trim(initial_center_us)
                    final_center = initial_center_us
                    msg = (
                        f"Completed {max_iters} tries without improvement (best drift "
                        f"{best['drift_deg_per_m']:+.2f} deg/m). Restored initial {initial_center_us:.1f} us"
                    )

                self._update_status(
                    phase="DONE_BEST",
                    drift_deg_per_m=best["drift_deg_per_m"],
                    fwd_drift_deg_per_m=best["fwd_drift_deg_per_m"],
                    bwd_drift_deg_per_m=best["bwd_drift_deg_per_m"],
                    servo_center_us=final_center,
                    history=history,
                    message=msg,
                )
        except Exception as exc:
            self._update_status(phase="ERROR", message=f"Auto-center error: {exc}")
        finally:
            try:
                if not self._stop_event.is_set() and abs(self._read_speed_est(self._get_telemetry() or {})) > 0.05:
                    self._send_cmd(b"BRAKE\n")
                    last_brake_tx = self._now()
                    t_wait = last_brake_tx + 0.35
                    while self._now() < t_wait and not self._stop_event.is_set():
                        if abs(self._read_speed_est(self._get_telemetry() or {})) <= 0.05:
                            break
                        now_w = self._now()
                        if (now_w - last_brake_tx) >= 0.10:
                            self._send_cmd(b"BRAKE\n")
                            last_brake_tx = now_w
                        self._sleep(0.02)
            except Exception:
                pass
            self._send_cmd(b"V 0.0\n")
            self._send_cmd(b"GYRO 1\n")
            self._send_cmd(b"TV 1\n")
            self._send_cmd(b"RESET\n")
            self._update_status(running=False)
