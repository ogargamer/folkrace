"""Unit and closed-loop simulation tests for pi4/auto_servo_center.py."""

import math
import os
import sys
import threading
import time
import unittest
from typing import Any, Callable, Dict, List, Optional

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PI4_DIR = os.path.join(ROOT_DIR, "pi4")
for path_dir in (ROOT_DIR, PI4_DIR):
    if path_dir not in sys.path:
        sys.path.insert(0, path_dir)

from auto_servo_center import BASE_SERVO_NEUTRAL_US, US_PER_RAD, AutoServoCenterCalibrator  # noqa: E402
from core.vehicle_params import WHEELBASE  # noqa: E402


class SimulatedVehicle:
    """Deterministic virtual-clock simulated ESP32 + chassis kinematics + IMU + ToF."""

    def __init__(
        self,
        true_center_us: float = 1540.0,
        steer_gain: float = 1.0,
        static_bias: float = 0.02,
        drive_bias: float = 0.02,
        fwd_speed_scale: float = 1.0,
        bwd_speed_scale: float = 1.0,
        brake_decel_mps2: float = 1.8,
        tof_dist_m: float = 3.0,
        spike_hook: Optional[Callable[[int, str, float], float]] = None,
        step_hook: Optional[Callable[["SimulatedVehicle"], None]] = None,
    ):
        self.true_center_us = true_center_us
        self.steer_gain = steer_gain
        self.static_bias = static_bias
        self.drive_bias = drive_bias
        self.fwd_speed_scale = fwd_speed_scale
        self.bwd_speed_scale = bwd_speed_scale
        self.brake_decel_mps2 = brake_decel_mps2
        self.tof_dist_m = tof_dist_m
        self.spike_hook = spike_hook
        self.step_hook = step_hook

        self.lock = threading.Lock()
        self.commands: List[str] = []
        self.saved_centers: List[float] = []

        self.sim_time_s = 0.0
        self.trim_us = 0.0
        self.v_cmd = 0.0
        self.v_est = 0.0
        self.mode = "IDLE"
        self.braking = False
        self.gyro_assist = True
        self.tv_enabled = True
        self.seq = 1
        self.mcu_us = 100_000
        self.tof_ranges_raw = [tof_dist_m] * 16
        self.tof_target0_status = [5] * 16
        self.leg_sample_idx = 0
        self.fwd_travel_m = 0.0
        self.bwd_travel_m = 0.0
        self.last_leg_dir = 0
        self.override_pkt: Dict[str, Any] = {}

    def stop_sim(self) -> None:
        pass

    def advance(self, dt_s: float) -> None:
        sub_steps = max(1, int(math.ceil(dt_s / 0.010)))
        sub_dt = dt_s / sub_steps
        for _ in range(sub_steps):
            with self.lock:
                self.sim_time_s += sub_dt
                if self.braking:
                    dv = self.brake_decel_mps2 * sub_dt
                    if abs(self.v_est) <= dv:
                        self.v_est = 0.0
                    else:
                        self.v_est -= math.copysign(dv, self.v_est)
                else:
                    scale = self.fwd_speed_scale if self.v_cmd >= 0 else self.bwd_speed_scale
                    target_v = self.v_cmd * scale
                    dv = 2.5 * sub_dt
                    if target_v > self.v_est:
                        self.v_est = min(target_v, self.v_est + dv)
                    else:
                        self.v_est = max(target_v, self.v_est - dv)

                if self.last_leg_dir > 0 and self.v_est > 0.0:
                    self.fwd_travel_m += self.v_est * sub_dt
                elif self.last_leg_dir < 0 and self.v_est < 0.0:
                    self.bwd_travel_m += abs(self.v_est) * sub_dt

                self.seq += 1
                self.mcu_us = (self.mcu_us + int(round(sub_dt * 1_000_000))) & 0xFFFFFFFF
                self.leg_sample_idx += 1

            if self.step_hook is not None:
                self.step_hook(self)

    def send_cmd(self, raw: bytes) -> bool:
        cmd = raw.decode("utf-8", errors="ignore").strip()
        with self.lock:
            self.commands.append(cmd)
            upper = cmd.upper()
            if upper.startswith("TRIM ") and upper != "TRIM_SAVE" and upper != "TRIM_GET":
                self.trim_us = float(cmd.split()[1])
            elif upper.startswith("V "):
                self.v_cmd = float(cmd.split()[1])
                self.braking = False
                self.mode = "DRIVE" if abs(self.v_cmd) > 0.001 else "IDLE"
                if self.v_cmd > 0.05 and self.last_leg_dir != 1:
                    self.last_leg_dir = 1
                    self.leg_sample_idx = 0
                elif self.v_cmd < -0.05 and self.last_leg_dir != -1:
                    self.last_leg_dir = -1
                    self.leg_sample_idx = 0
            elif upper == "BRAKE":
                self.v_cmd = 0.0
                self.braking = True
                self.mode = "DRIVE"
            elif upper == "RESET":
                self.v_cmd = 0.0
                self.braking = False
                self.mode = "IDLE"
                self.last_leg_dir = 0
            elif upper == "GYRO 0":
                self.gyro_assist = False
            elif upper == "GYRO 1":
                self.gyro_assist = True
            elif upper == "TV 0":
                self.tv_enabled = False
            elif upper == "TV 1":
                self.tv_enabled = True
        return True

    def get_center_us(self) -> float:
        with self.lock:
            return BASE_SERVO_NEUTRAL_US + self.trim_us

    def save_center_us(self, center_us: float) -> float:
        val = round(float(center_us), 1)
        with self.lock:
            self.saved_centers.append(val)
            self.trim_us = val - BASE_SERVO_NEUTRAL_US
        return val

    def get_telemetry(self) -> Dict[str, Any]:
        with self.lock:
            applied_center = BASE_SERVO_NEUTRAL_US + self.trim_us
            steer_err_rad = self.steer_gain * (self.true_center_us - applied_center) / US_PER_RAD
            is_moving = abs(self.v_est) > 0.02
            bias = self.drive_bias if is_moving else self.static_bias
            gyro_z = (self.v_est * math.tan(steer_err_rad)) / WHEELBASE + bias

            if self.spike_hook is not None and is_moving:
                leg_name = "FWD" if self.v_est > 0 else "BWD"
                gyro_z = self.spike_hook(self.leg_sample_idx, leg_name, gyro_z)

            pkt: Dict[str, Any] = {
                "seq": self.seq,
                "mcu_timestamp_us": self.mcu_us,
                "_pkt_age_s": 0.01,
                "planner_running": False,
                "mcu_main_connected": True,
                "mcu_tof_connected": True,
                "mode": self.mode,
                "v_cmd": round(self.v_cmd, 3),
                "v_est": round(self.v_est, 4),
                "gyro_z": gyro_z,
                "slip_ratio": 0.02,
                "tof_ranges_raw": list(self.tof_ranges_raw),
                "tof_target0_status": list(self.tof_target0_status),
                "flags": {
                    "fake_data": False,
                    "imu_online": True,
                    "stm32_front": True,
                    "stm32_rear": True,
                },
            }
            pkt.update(self.override_pkt)
            return pkt


class VirtualClockCalibrator(AutoServoCenterCalibrator):
    """Calibrator wired to SimulatedVehicle virtual clock (runs in milliseconds)."""

    def __init__(self, sim: SimulatedVehicle):
        self._sim = sim
        super().__init__(
            get_telemetry=sim.get_telemetry,
            send_cmd=sim.send_cmd,
            get_center_us=sim.get_center_us,
            save_center_us=sim.save_center_us,
        )

    def _now(self) -> float:
        with self._sim.lock:
            return self._sim.sim_time_s

    def _sleep(self, seconds: float) -> None:
        self._sim.advance(max(0.001, seconds))


def _wait_done(cal: AutoServoCenterCalibrator, timeout_s: float = 2.0) -> Dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        st = cal.get_status()
        if not st["running"]:
            return st
        time.sleep(0.001)
    raise TimeoutError("Calibrator did not finish in time")


class TestAutoServoCenter(unittest.TestCase):
    def _make_cal(self, sim: SimulatedVehicle, cls=VirtualClockCalibrator) -> AutoServoCenterCalibrator:
        return cls(sim)

    def test_nominal_convergence_from_30us(self) -> None:
        sim = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US + 30.0)
        cal = self._make_cal(sim)
        res = cal.start(max_iterations=6, tolerance_deg_per_m=0.8)
        self.assertTrue(res["started"], res)

        st = _wait_done(cal)
        self.assertEqual(st["phase"], "CONVERGED", st)
        self.assertLessEqual(len(st["history"]), 4)
        self.assertAlmostEqual(st["servo_center_us"], BASE_SERVO_NEUTRAL_US + 30.0, delta=1.5)
        self.assertLessEqual(abs(st["drift_deg_per_m"]), 0.8)

        for entry in st["history"]:
            if entry["step_us"] != 0.0:
                self.assertGreater(entry["step_us"] * entry["drift_deg_per_m"], 0.0)

    def test_gain_mismatch_overshoot_converges_via_bracket(self) -> None:
        sim = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US - 32.0, steer_gain=1.6)
        cal = self._make_cal(sim)
        res = cal.start(max_iterations=6, tolerance_deg_per_m=0.8)
        self.assertTrue(res["started"], res)

        st = _wait_done(cal)
        self.assertEqual(st["phase"], "CONVERGED", st)
        self.assertLessEqual(len(st["history"]), 4)
        self.assertAlmostEqual(st["servo_center_us"], BASE_SERVO_NEUTRAL_US - 32.0, delta=1.2)
        methods = [h["method"] for h in st["history"]]
        self.assertTrue("bracket" in methods or "secant" in methods, methods)

    def test_asymmetric_speeds_and_residual_bias_cancellation(self) -> None:
        sim = SimulatedVehicle(
            true_center_us=BASE_SERVO_NEUTRAL_US + 24.0,
            static_bias=0.01,
            drive_bias=0.07,
            fwd_speed_scale=1.0,
            bwd_speed_scale=0.80,
        )
        cal = self._make_cal(sim)
        res = cal.start(max_iterations=6, tolerance_deg_per_m=0.8)
        self.assertTrue(res["started"], res)

        st = _wait_done(cal)
        self.assertEqual(st["phase"], "CONVERGED", st)
        self.assertAlmostEqual(st["servo_center_us"], BASE_SERVO_NEUTRAL_US + 24.0, delta=1.5)
        self.assertAlmostEqual(st["resid_bias_rad_s"], 0.06, delta=0.015)

    def test_mad_outlier_rejection_with_gyro_spikes(self) -> None:
        def inject_spikes(idx: int, _leg: str, gz: float) -> float:
            if idx % 29 == 17:
                return gz + 2.8
            if idx % 37 == 23:
                return gz - 3.2
            return gz

        sim = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US + 18.0, spike_hook=inject_spikes)
        cal = self._make_cal(sim)
        res = cal.start(max_iterations=6, tolerance_deg_per_m=0.8)
        self.assertTrue(res["started"], res)

        st = _wait_done(cal)
        self.assertEqual(st["phase"], "CONVERGED", st)
        self.assertAlmostEqual(st["servo_center_us"], BASE_SERVO_NEUTRAL_US + 18.0, delta=1.5)

    def test_command_protocol_and_abort_restores_assists(self) -> None:
        sim = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US + 15.0)
        cal = self._make_cal(sim)
        res = cal.start(max_iterations=6)
        self.assertTrue(res["started"], res)
        st = _wait_done(cal)
        self.assertEqual(st["phase"], "CONVERGED", st)

        cmds = sim.commands
        self.assertIn("MOTORS", cmds)
        self.assertIn("GYRO 0", cmds)
        self.assertIn("TV 0", cmds)
        self.assertIn("BRAKE", cmds)
        self.assertIn("TRIM_SAVE", cmds)
        self.assertIn("TRIM_GET", cmds)
        self.assertEqual(cmds.count("A 90.0"), 1)
        self.assertIn("GYRO 1", cmds)
        self.assertIn("TV 1", cmds)
        self.assertIn("RESET", cmds)
        self.assertTrue(sim.gyro_assist)
        self.assertTrue(sim.tv_enabled)

        # User abort mid-run via deterministic step hook
        sim2 = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US + 40.0)
        cal2 = self._make_cal(sim2)

        def abort_mid_leg(s: SimulatedVehicle) -> None:
            if s.fwd_travel_m >= 0.40:
                s.step_hook = None
                cal2.stop("User cancelled")

        sim2.step_hook = abort_mid_leg
        self.assertTrue(cal2.start()["started"])
        st2 = _wait_done(cal2)
        self.assertEqual(st2["phase"], "ABORTED", st2)
        self.assertIn("GYRO 1", sim2.commands)
        self.assertIn("TV 1", sim2.commands)
        self.assertIn("RESET", sim2.commands)
        self.assertTrue(sim2.gyro_assist)
        self.assertTrue(sim2.tv_enabled)

    def test_preflight_and_tof_ranges_raw_obstacle_guard(self) -> None:
        bad_cases = [
            {"planner_running": True},
            {"flags": {"fake_data": True, "imu_online": True, "stm32_front": True, "stm32_rear": True}},
            {"flags": {"fake_data": False, "imu_online": False, "stm32_front": True, "stm32_rear": True}},
            {"_pkt_age_s": 0.9},
            {"mcu_main_connected": False},
            {"tof_ranges_raw": [0.80] * 16},
        ]
        for override in bad_cases:
            sim = SimulatedVehicle()
            with sim.lock:
                sim.override_pkt = dict(override)
            cal = self._make_cal(sim)
            res = cal.start(drive_distance_m=1.0)
            self.assertFalse(res["started"], f"Expected preflight fail for {override}")
            self.assertTrue(res.get("error"))

        # Mid-leg obstacle via tof_ranges_raw only (no tof_ranges key)
        sim_obs = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US + 20.0, tof_dist_m=3.0)

        def inject_obstacle(s: SimulatedVehicle) -> None:
            if s.fwd_travel_m >= 0.35:
                s.step_hook = None
                with s.lock:
                    s.tof_ranges_raw[2] = 0.20

        sim_obs.step_hook = inject_obstacle
        cal_obs = self._make_cal(sim_obs)
        self.assertTrue(cal_obs.start(drive_distance_m=1.0)["started"])
        st_obs = _wait_done(cal_obs)
        self.assertEqual(st_obs["phase"], "ERROR", st_obs)
        self.assertIn("Obstacle", st_obs["message"])
        self.assertIn("BRAKE", sim_obs.commands)
        self.assertIn("GYRO 1", sim_obs.commands)
        self.assertIn("TV 1", sim_obs.commands)
        self.assertIn("RESET", sim_obs.commands)

    def test_backward_leg_matches_forward_total_distance(self) -> None:
        sim = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US, brake_decel_mps2=0.9)
        cal = self._make_cal(sim)
        res = cal.start(drive_distance_m=0.80, cruise_speed=0.75)
        self.assertTrue(res["started"], res)
        st = _wait_done(cal)
        self.assertEqual(st["phase"], "CONVERGED", st)
        self.assertAlmostEqual(sim.bwd_travel_m, sim.fwd_travel_m, delta=0.05)

    def test_max_iterations_done_best_and_no_worse_overwrite(self) -> None:
        class TinyStepCal(VirtualClockCalibrator):
            MAX_STEP_US = 8.0

        sim_imp = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US + 45.0)
        cal_imp = self._make_cal(sim_imp, cls=TinyStepCal)
        self.assertTrue(cal_imp.start(max_iterations=2, tolerance_deg_per_m=0.8)["started"])
        st_imp = _wait_done(cal_imp)
        self.assertEqual(st_imp["phase"], "DONE_BEST", st_imp)
        self.assertEqual(len(sim_imp.saved_centers), 1)
        self.assertGreater(sim_imp.saved_centers[0], BASE_SERVO_NEUTRAL_US)

        class WrongSignCal(VirtualClockCalibrator):
            @classmethod
            def _compute_next_step_us(cls, **kwargs):
                return 40.0, "forced_bad"

        sim_worse = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US - 35.0)
        cal_worse = self._make_cal(sim_worse, cls=WrongSignCal)
        self.assertTrue(cal_worse.start(max_iterations=2, tolerance_deg_per_m=0.5)["started"])
        st_worse = _wait_done(cal_worse)
        self.assertEqual(st_worse["phase"], "DONE_BEST", st_worse)
        self.assertEqual(len(sim_worse.saved_centers), 0)
        self.assertNotIn("TRIM_SAVE", sim_worse.commands)
        self.assertAlmostEqual(st_worse["servo_center_us"], BASE_SERVO_NEUTRAL_US, delta=0.1)

    def test_mid_leg_tof_loss_aborts_and_restores_assists(self) -> None:
        sim = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US + 20.0, tof_dist_m=3.0)

        def drop_tof(s: SimulatedVehicle) -> None:
            if s.fwd_travel_m >= 0.30:
                s.step_hook = None
                with s.lock:
                    s.override_pkt["mcu_tof_connected"] = False

        sim.step_hook = drop_tof
        cal = self._make_cal(sim)
        self.assertTrue(cal.start(drive_distance_m=1.0)["started"])
        st = _wait_done(cal)
        self.assertEqual(st["phase"], "ERROR", st)
        self.assertIn("ToF", st["message"])
        self.assertIn("BRAKE", sim.commands)
        self.assertIn("GYRO 1", sim.commands)
        self.assertIn("TV 1", sim.commands)
        self.assertIn("RESET", sim.commands)
        self.assertTrue(sim.gyro_assist)
        self.assertTrue(sim.tv_enabled)

    def test_active_brake_sends_periodic_keepalive(self) -> None:
        sim = SimulatedVehicle(true_center_us=BASE_SERVO_NEUTRAL_US, brake_decel_mps2=1.0)
        cal = self._make_cal(sim)
        sim.v_cmd = 0.80
        sim.v_est = 0.80
        res = cal._active_brake_stop(+1, "BRAKE_FWD", 0.5, 1.0)
        self.assertTrue(res["ok"], res)
        brake_count = sum(1 for c in sim.commands if c == "BRAKE")
        self.assertGreaterEqual(brake_count, 3, sim.commands)


if __name__ == "__main__":
    unittest.main()
