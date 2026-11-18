#!/usr/bin/env python3
"""
================================================================================
  FOLKRACE PI 4: AUTOMATIC SERVO CENTER CALIBRATION
================================================================================
  Closed-loop servo neutral trim detector:
    1. Measures stationary IMU gyro_z bias.
    2. Drives forward 1.0 m slowly (half-MU acceleration & 0.60 m/s cruise so
       tires never slip and wheel encoders remain accurate), averaging gyro_z.
    3. Drives backward 1.0 m slowly back to start, averaging gyro_z.
    4. Cancels residual gyro bias by combining forward & backward curvature:
         kappa_err = 0.5 * ((omega_fwd / |v_fwd|) + (-omega_bwd / |v_bwd|))
       and converts to steering angle error:
         delta_err = atan(kappa_err * WHEELBASE)
    5. Applies kinematic feedforward + simple PID + zero-crossing damping to
       adjust servo_center_us until heading drift is within margin (<= 1.0 deg/m).
    6. Persists converged center to servo_trim.json and ESP32 NVS (TRIM_SAVE).
================================================================================
"""

import math
import os
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional

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
    TIRE_MU = 0.75
    GRAVITY_G = 9.80665
    WHEELBASE = 0.1325
    MAX_STEERING_RAD = 0.562285
    SERVO_PULSE_NEUTRAL_US = 1510.0
    SERVO_PULSE_MIN_US = 1239.0
    SERVO_PULSE_MAX_US = 1781.0


BASE_SERVO_NEUTRAL_US = float(SERVO_PULSE_NEUTRAL_US)
US_PER_RAD = (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / (2.0 * MAX_STEERING_RAD)


class AutoServoCenterCalibrator:
    """Runs the 1m forward / 1m backward automatic servo centering loop."""

    DRIVE_DISTANCE_M = 1.00
    CRUISE_SPEED_MPS = 0.60
    HALF_MU_ACCEL_MPS2 = 0.5 * TIRE_MU * GRAVITY_G
    RAMP_ACCEL_MPS2 = min(1.80, HALF_MU_ACCEL_MPS2)

    BIAS_SAMPLE_SEC = 0.40
    SETTLE_STOP_SEC = 0.40
    CONTROL_DT_SEC = 0.03
    LEG_TIMEOUT_SEC = 5.5
    STALL_TIMEOUT_SEC = 1.5

    MIN_STEADY_SPEED_MPS = 0.25
    STEADY_DIST_MIN_M = 0.15
    STEADY_DIST_MAX_M = 0.90

    CONVERGE_DRIFT_DEG_PER_M = 1.0
    MAX_ITERATIONS = 8

    # Simple PID gains (applied to kinematic delta_err_rad * US_PER_RAD)
    KP = 0.75
    KI = 0.20
    KD = 0.10
    INTEGRAL_CLAMP_US = 15.0
    MAX_STEP_US = 100.0
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

        self._status: Dict[str, Any] = {
            "running": False,
            "phase": "IDLE",
            "iteration": 0,
            "max_iterations": self.MAX_ITERATIONS,
            "distance_m": 0.0,
            "target_distance_m": self.DRIVE_DISTANCE_M,
            "gyro_offset_rad_s": 0.0,
            "drift_deg_per_m": 0.0,
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

    def _update_status(self, **kwargs: Any) -> None:
        with self._lock:
            self._status.update(kwargs)

    def start(self, max_iterations: Optional[int] = None, cruise_speed: Optional[float] = None) -> Dict[str, Any]:
        with self._lock:
            if self._status["running"]:
                return {"started": False, "error": "Auto servo centering already running"}

            telem = self._get_telemetry()
            if telem.get("planner_running"):
                return {"started": False, "error": "Stop Path Planner before running Auto Servo Center"}

            self._stop_event.clear()
            max_iters = int(max_iterations) if max_iterations else self.MAX_ITERATIONS
            speed = float(cruise_speed) if cruise_speed else self.CRUISE_SPEED_MPS
            speed = max(0.35, min(1.00, speed))

            center_us = float(self._get_center_us())
            self._status.update(
                {
                    "running": True,
                    "phase": "STARTING",
                    "iteration": 0,
                    "max_iterations": max_iters,
                    "distance_m": 0.0,
                    "target_distance_m": self.DRIVE_DISTANCE_M,
                    "gyro_offset_rad_s": 0.0,
                    "drift_deg_per_m": 0.0,
                    "servo_center_us": center_us,
                    "last_step_us": 0.0,
                    "message": "Starting auto servo center...",
                    "history": [],
                }
            )
            self._thread = threading.Thread(
                target=self._run_worker,
                args=(max_iters, speed),
                daemon=True,
                name="AutoServoCenter",
            )
            self._thread.start()
            return {"started": True, "servo_center_us": center_us}

    def stop(self, reason: str = "Stopped by user") -> None:
        self._stop_event.set()
        if self.is_running():
            self._update_status(phase="ABORTED", message=reason)
        self._send_cmd(b"V 0.0\n")
        self._send_cmd(b"RESET\n")

    def _apply_trim(self, center_us: float) -> float:
        clamped = round(max(self.MIN_CENTER_US, min(self.MAX_CENTER_US, center_us)), 1)
        trim_offset = max(-500.0, min(500.0, clamped - BASE_SERVO_NEUTRAL_US))
        self._send_cmd(b"A 90.0\n")
        self._send_cmd(f"TRIM {trim_offset:.1f}\n".encode("utf-8"))
        return clamped

    def _measure_static_gyro_bias(self) -> float:
        self._update_status(phase="BIAS_CAL", message="Measuring stationary IMU gyro bias...")
        samples: List[float] = []
        t_end = time.monotonic() + self.BIAS_SAMPLE_SEC
        while time.monotonic() < t_end and not self._stop_event.is_set():
            pkt = self._get_telemetry()
            samples.append(float(pkt.get("gyro_z") or 0.0))
            time.sleep(0.02)
        if not samples:
            return 0.0
        return sum(samples) / len(samples)

    def _check_front_obstacle(self, pkt: Dict[str, Any]) -> bool:
        if not pkt.get("mcu_tof_connected"):
            return False
        tof = pkt.get("tof_ranges") or []
        if len(tof) < 12:
            return False
        # Front-left center (0..3) and front-right center (8..11)
        front_vals = [float(v) for v in (list(tof[0:4]) + list(tof[8:12])) if v is not None and 0.02 < float(v) < 5.0]
        return bool(front_vals and min(front_vals) < self.MIN_FRONT_TOF_M)

    def _drive_leg(self, direction: int, speed_mps: float, gyro_bias: float) -> Dict[str, Any]:
        """
        Drives 1.0 m forward (direction=+1) or backward (direction=-1) with smooth
        half-MU acceleration ramp and returns averaged steady-state gyro_z and speed.
        """
        phase = "FORWARD" if direction > 0 else "BACKWARD"
        self._update_status(
            phase=phase,
            distance_m=0.0,
            message=f"Driving {'forward' if direction > 0 else 'backward'} {self.DRIVE_DISTANCE_M:.1f}m @ {speed_mps:.2f} m/s...",
        )

        dist_m = 0.0
        v_cmd = 0.0
        t_start = time.monotonic()
        t_prev = t_start
        last_motion_t = t_start

        gyro_weighted_sum = 0.0
        speed_weighted_sum = 0.0
        steady_dt_sum = 0.0

        while not self._stop_event.is_set():
            now = time.monotonic()
            dt = max(0.005, min(0.10, now - t_prev))
            t_prev = now

            pkt = self._get_telemetry()
            if pkt.get("mode") == "ESTOP":
                return {"ok": False, "error": "Emergency stop active"}

            if direction > 0 and self._check_front_obstacle(pkt):
                return {"ok": False, "error": f"Obstacle detected ahead (<{self.MIN_FRONT_TOF_M:.2f}m)"}

            v_est = float(pkt.get("v_est") or 0.0)
            gyro_z = float(pkt.get("gyro_z") or 0.0) - gyro_bias

            dist_m += abs(v_est) * dt
            self._update_status(distance_m=round(dist_m, 3))

            if abs(v_est) >= 0.05:
                last_motion_t = now
            elif (now - t_start) > 0.6 and (now - last_motion_t) > self.STALL_TIMEOUT_SEC:
                return {"ok": False, "error": "No wheel motion detected (check motors/battery/encoders)"}

            if dist_m >= self.DRIVE_DISTANCE_M:
                break
            if (now - t_start) > self.LEG_TIMEOUT_SEC:
                return {"ok": False, "error": f"{phase} leg timed out at {dist_m:.2f}m"}

            # Trapezoidal half-MU velocity profile: gentle ramp up & ramp down near 1.0m
            rem_m = max(0.0, self.DRIVE_DISTANCE_M - dist_m)
            v_brake_cap = math.sqrt(max(0.04, 2.0 * self.RAMP_ACCEL_MPS2 * rem_m))
            v_target = direction * min(speed_mps, max(0.22, v_brake_cap))

            max_dv = self.RAMP_ACCEL_MPS2 * dt
            if v_target > v_cmd:
                v_cmd = min(v_target, v_cmd + max_dv)
            else:
                v_cmd = max(v_target, v_cmd - max_dv)

            self._send_cmd(f"V {v_cmd:.2f}\n".encode("utf-8"))

            # Accumulate gyro only during steady-state cruise away from launch/stop transients
            if (
                abs(v_est) >= self.MIN_STEADY_SPEED_MPS
                and self.STEADY_DIST_MIN_M <= dist_m <= self.STEADY_DIST_MAX_M
            ):
                gyro_weighted_sum += gyro_z * dt
                speed_weighted_sum += abs(v_est) * dt
                steady_dt_sum += dt

            time.sleep(self.CONTROL_DT_SEC)

        # Smooth stop
        self._send_cmd(b"V 0.0\n")
        if self._stop_event.is_set():
            return {"ok": False, "error": "Stopped by user"}

        if steady_dt_sum <= 0.05:
            return {"ok": False, "error": f"Insufficient steady-state samples during {phase}"}

        return {
            "ok": True,
            "omega_avg": gyro_weighted_sum / steady_dt_sum,
            "v_avg": speed_weighted_sum / steady_dt_sum,
            "dist_m": dist_m,
        }

    def _settle_stop(self, phase_name: str) -> bool:
        self._update_status(phase=phase_name, message="Settling to a complete stop...")
        t_end = time.monotonic() + self.SETTLE_STOP_SEC
        while time.monotonic() < t_end:
            if self._stop_event.is_set():
                return False
            time.sleep(0.02)
        return True

    def _run_worker(self, max_iters: int, cruise_speed: float) -> None:
        center_us = float(self._get_center_us())
        integral_us = 0.0
        prev_err_rad: Optional[float] = None
        history: List[Dict[str, Any]] = []

        try:
            self._send_cmd(b"MOTORS\n")
            center_us = self._apply_trim(center_us)
            time.sleep(0.15)

            gyro_bias = self._measure_static_gyro_bias()
            if self._stop_event.is_set():
                return

            converged = False
            for it in range(1, max_iters + 1):
                if self._stop_event.is_set():
                    break

                self._update_status(iteration=it, servo_center_us=center_us)

                fwd = self._drive_leg(+1, cruise_speed, gyro_bias)
                if not fwd["ok"]:
                    self._update_status(phase="ERROR", message=fwd["error"])
                    return

                if not self._settle_stop("SETTLE_FWD"):
                    return

                bwd = self._drive_leg(-1, cruise_speed, gyro_bias)
                if not bwd["ok"]:
                    self._update_status(phase="ERROR", message=bwd["error"])
                    return

                if not self._settle_stop("SETTLE_BWD"):
                    return

                # Forward & backward curvature averaging cancels residual gyro bias:
                # Forward (+v):  omega_fwd = +(v/L)*tan(delta_err) + bias
                # Backward (-v): omega_bwd = -(v/L)*tan(delta_err) + bias
                kappa_fwd = fwd["omega_avg"] / max(0.20, fwd["v_avg"])
                kappa_bwd = -bwd["omega_avg"] / max(0.20, bwd["v_avg"])
                kappa_err = 0.5 * (kappa_fwd + kappa_bwd)

                delta_err_rad = math.atan(kappa_err * WHEELBASE)
                drift_deg_per_m = math.degrees(kappa_err * 1.0)
                gyro_equiv = kappa_err * cruise_speed

                entry = {
                    "iter": it,
                    "omega_fwd": round(fwd["omega_avg"], 4),
                    "omega_bwd": round(bwd["omega_avg"], 4),
                    "drift_deg_per_m": round(drift_deg_per_m, 2),
                    "center_us": round(center_us, 1),
                    "step_us": 0.0,
                }

                # Check convergence margin
                if abs(drift_deg_per_m) <= self.CONVERGE_DRIFT_DEG_PER_M:
                    history.append(entry)
                    saved = self._save_center_us(center_us)
                    self._apply_trim(saved)
                    self._send_cmd(b"TRIM_SAVE\n")
                    self._update_status(
                        phase="CONVERGED",
                        gyro_offset_rad_s=round(gyro_equiv, 4),
                        drift_deg_per_m=round(drift_deg_per_m, 2),
                        servo_center_us=saved,
                        last_step_us=0.0,
                        history=history,
                        message=f"Converged in {it} tries! Center = {saved:.1f} us (drift {drift_deg_per_m:+.2f} deg/m)",
                    )
                    converged = True
                    break

                # Simple PID + zero-crossing damping on kinematic pulse error
                # Positive delta_err_rad (drifts left going fwd) -> increase pulse_us (steer right)
                err_us = delta_err_rad * US_PER_RAD
                p_us = self.KP * err_us
                integral_us = max(-self.INTEGRAL_CLAMP_US, min(self.INTEGRAL_CLAMP_US, integral_us + self.KI * err_us))
                d_us = self.KD * ((delta_err_rad - prev_err_rad) * US_PER_RAD) if prev_err_rad is not None else 0.0

                step_us = p_us + integral_us + d_us
                # Zero-crossing damping prevents bouncing around center
                if prev_err_rad is not None and (delta_err_rad * prev_err_rad) < 0.0:
                    step_us *= 0.65

                step_us = max(-self.MAX_STEP_US, min(self.MAX_STEP_US, step_us))
                # Ensure at least 0.5 us step when outside margin so we don't stall on tiny Ki
                if abs(step_us) < 0.5:
                    step_us = 0.5 if err_us > 0 else -0.5

                prev_err_rad = delta_err_rad
                entry["step_us"] = round(step_us, 1)
                history.append(entry)

                center_us = self._apply_trim(center_us + step_us)
                self._update_status(
                    phase="ADJUSTING",
                    gyro_offset_rad_s=round(gyro_equiv, 4),
                    drift_deg_per_m=round(drift_deg_per_m, 2),
                    servo_center_us=center_us,
                    last_step_us=round(step_us, 1),
                    history=history,
                    message=f"Iter {it}/{max_iters}: drift {drift_deg_per_m:+.2f} deg/m -> trim {step_us:+.1f} us (center {center_us:.1f} us)",
                )
                time.sleep(0.15)

            if not converged and not self._stop_event.is_set() and history:
                # Pick best-observed center from history
                best = min(history, key=lambda h: abs(h["drift_deg_per_m"]))
                saved = self._save_center_us(best["center_us"])
                self._apply_trim(saved)
                self._send_cmd(b"TRIM_SAVE\n")
                self._update_status(
                    phase="CONVERGED",
                    servo_center_us=saved,
                    history=history,
                    message=f"Completed {max_iters} tries. Best center = {saved:.1f} us (drift {best['drift_deg_per_m']:+.2f} deg/m)",
                )
        except Exception as exc:
            self._update_status(phase="ERROR", message=f"Auto-center error: {exc}")
        finally:
            self._send_cmd(b"V 0.0\n")
            self._send_cmd(b"RESET\n")
            self._update_status(running=False)
