"""
================================================================================
  UNIFIED FOLKRACE VEHICLE SIMULATION HARNESS
================================================================================
  Hardware abstraction, sensor/actuator discovery, 120 Hz/60 Hz execution
  pipeline, and telemetry generation for Webots autonomous racing.
================================================================================
"""

import os
import sys
import math
import time
import json
import zlib
import base64
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)

for p in [CURR_DIR, CONTROLLERS_DIR, MY_PROJECT_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from electronic_diff_4wd import ElectronicDifferential4WD
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from imu_encoder_fusion import IMUEncoderFusion
from tof_gap_scout import ToFGapScout
from planner_option2_sl_mpc import CurvilinearSLMPC
from stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState
from planner_config import (
    TIME_STEP_MS,
    DT,
    SIM_HZ,
    PLANNER_HZ,
    DT_PLANNER,
    PLANNER_DECIMATION,
    TOF_TIME_STEP_MS,
    TOF_SENSOR_NAMES,
    TOF_MAX_RANGE_M,
    TOF_MIN_RANGE_M,
    ENCODER_NAMES,
    WHEEL_RADIUS,
    V_MIN_CRUISE,
    V_MAX_CRUISE,
    MAX_STEERING_RAD,
    WHEELBASE,
    TRACK_REAR,
    TRACK_FRONT,
    TRACK_KINGPIN,
    MOTOR_DIR,
    V_MAX_WHEEL,
    GRID_X_MIN,
    GRID_X_MAX,
    GRID_Y_MIN,
    GRID_Y_MAX,
    GRID_RES,
    R_INFLATION_M,
    RECOVERY_REVERSE_SPEED_MPS,
    RECOVERY_STEER_ANGLE_RAD,
    RECOVERY_FRONT_BLOCKED_M,
    RECOVERY_CLEARANCE_EXIT_M,
    RECOVERY_ARC_LOOKAHEAD_M,
    ACCEL_FWD_MAX,
    BRAKE_DECEL_MAX,
    SERVO_MAX_SPEED_RAD_S,
    STRAIGHT_LOCK_DIST_M,
    STRAIGHT_LOCK_SPEED_MPS,
    EMERGENCY_BACKUP_DIST_M,
)


def to_py(obj: Any) -> Any:
    """Recursively converts NumPy numeric types to standard Python types for JSON."""
    if isinstance(obj, (np.floating, np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, (np.integer, np.int32, np.int64)):
        return int(obj)
    elif isinstance(obj, (list, tuple)):
        return [to_py(x) for x in obj]
    elif isinstance(obj, dict):
        return {k: to_py(v) for k, v in obj.items()}
    return obj


def read_tof_multi_ray(dev: Any, noise_pct: float = 0.02) -> float:
    """Reads distance from Webots Lidar or DistanceSensor node."""
    val = TOF_MAX_RANGE_M
    if dev is None:
        return val
    if hasattr(dev, "getRangeImage"):
        r_img = dev.getRangeImage()
        if r_img:
            valid_r = [v for v in r_img if not math.isinf(v) and not math.isnan(v) and v <= TOF_MAX_RANGE_M]
            if valid_r:
                val = float(min(valid_r))
    elif hasattr(dev, "getValue"):
        raw = dev.getValue()
        if not math.isinf(raw) and not math.isnan(raw):
            val = float(raw)
    val = max(TOF_MIN_RANGE_M, min(TOF_MAX_RANGE_M, val))
    return val


read_tof_device = read_tof_multi_ray


class FolkraceVehicle:
    """
    Encapsulates all actuators, sensors, and the full multi-rate control
    pipeline (120 Hz physics / 60 Hz SL-MPC) for the Folkrace vehicle.
    """

    def __init__(self, robot: Any, is_supervisor: bool = False, genes: Optional[Dict[str, Any]] = None):
        self.robot = robot
        self.is_supervisor = is_supervisor
        self.robot_node = robot.getSelf() if (is_supervisor and hasattr(robot, "getSelf")) else None

        # 1. Discover Actuators
        self.motors = {
            "FL": robot.getDevice("motor FL"),
            "FR": robot.getDevice("motor FR"),
            "RL": robot.getDevice("motor RL"),
            "RR": robot.getDevice("motor RR"),
        }
        for m in self.motors.values():
            if m:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)

        self.servos = {
            "left": robot.getDevice("servo left"),
            "right": robot.getDevice("servo right"),
        }
        for s in self.servos.values():
            if s:
                s.setPosition(0.0)

        # 2. Discover Sensors
        self.tofs = {}
        self.tof_devices = []
        for name in TOF_SENSOR_NAMES:
            dev = robot.getDevice(name)
            if dev is None:
                short_name = name.replace("tof_front_", "tof_")
                dev = robot.getDevice(short_name)
            if dev is not None:
                dev.enable(TOF_TIME_STEP_MS)
                self.tofs[name] = dev
                self.tof_devices.append(dev)
            else:
                self.tofs[name] = None
                self.tof_devices.append(None)

        self.imu_dev = (
            robot.getDevice("bno08x_imu")
            or robot.getDevice("imu")
            or robot.getDevice("inertial unit")
        )
        if self.imu_dev:
            self.imu_dev.enable(TIME_STEP_MS)

        self.gyro = (
            robot.getDevice("gyro")
            or robot.getDevice("bno08x_gyro")
        )
        if self.gyro:
            self.gyro.enable(TIME_STEP_MS)

        self.accel = (
            robot.getDevice("accelerometer")
            or robot.getDevice("bno08x_accel")
        )
        if self.accel:
            self.accel.enable(TIME_STEP_MS)

        self.encoders = {}
        for wheel, dev_name in ENCODER_NAMES.items():
            enc = robot.getDevice(dev_name)
            if enc:
                enc.enable(TIME_STEP_MS)
                self.encoders[wheel] = enc

        # 3. Core Vehicle Modules
        self.fusion = IMUEncoderFusion(wheel_radius=WHEEL_RADIUS, dt=DT)
        self.fusion.attach_devices(self.encoders, imu_dev=self.imu_dev, gyro_dev=self.gyro, accel_dev=self.accel)

        self.costmap = CppSmartHybridCostmap(
            grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
            resolution=GRID_RES,
            r_inflation_m=R_INFLATION_M,
        )

        self.gap_scout = ToFGapScout()
        self.planner = CurvilinearSLMPC(costmap_engine=self.costmap)
        self.planner.costmap = self.costmap
        self.ediff = ElectronicDifferential4WD(
            wheelbase=WHEELBASE,
            track_rear=TRACK_REAR,
            track_front=TRACK_FRONT,
            track_kingpin=TRACK_KINGPIN,
            wheel_radius=WHEEL_RADIUS,
            max_steer=MAX_STEERING_RAD,
            motor_dir=MOTOR_DIR,
            v_max=V_MAX_WHEEL,
        )
        self.recovery_manager = StuckRecoveryManager(
            reverse_speed_m_s=RECOVERY_REVERSE_SPEED_MPS,
            steer_angle_rad=RECOVERY_STEER_ANGLE_RAD,
            clearance_exit_m=RECOVERY_CLEARANCE_EXIT_M,
            servo_slew_rad_s=SERVO_MAX_SPEED_RAD_S,
            brake_decel_max=BRAKE_DECEL_MAX,
            accel_fwd_max=ACCEL_FWD_MAX,
            arc_lookahead_m=RECOVERY_ARC_LOOKAHEAD_M,
        )

        # Default Tuning Parameters
        self.mu_val = 0.817
        self.r_infl = R_INFLATION_M
        self.target_lookahead_m = 1.35
        self.aggr_dist_far = 1.70
        self.aggr_dist_close = 1.00
        self.aggr_min_scale = 0.30
        self.steer_deadband = 0.002
        self.straight_lock_dist = 1.70
        self.straight_lock_speed = 5.50
        self.v_cruise_min = V_MIN_CRUISE
        self.v_cruise_max = V_MAX_CRUISE

        # Dynamic State Tracking
        self.step_counter = 0
        self.active_steer = 0.0
        self.cached_target_steer = 0.0
        self.cached_target_speed = self.v_cruise_min
        self.rate_limited_speed = 0.0
        self.was_recovering = False
        self.mode_desc = "OPTION_2_SL_MPC"
        self.cached_plan_ms = 0.0
        self.cached_map_ms = 0.0
        self.x_est = 0.0
        self.y_est = 0.0
        self.yaw_est = 0.0
        self.stall_counter = 0
        self.last_gap_info = (0.0, 1.0, 0.50, 0.0)
        self.last_plan_res = {}
        self.last_tofs = {name: TOF_MAX_RANGE_M for name in TOF_SENSOR_NAMES}

        # Apply genes if provided
        if genes:
            self.configure(genes)

    def configure(self, genes: Dict[str, Any]) -> None:
        """Dynamically applies genome parameters to all vehicle subsystems."""
        if not genes:
            return

        self.mu_val = float(genes.get("mu_friction", 0.817))
        self.r_infl = float(genes.get("r_inflation_m", R_INFLATION_M))
        self.target_lookahead_m = float(genes.get("target_lookahead_m", 1.35))
        self.aggr_dist_far = float(genes.get("aggr_dist_far_m", 1.70))
        self.aggr_dist_close = float(genes.get("aggr_dist_close_m", 1.00))
        self.aggr_min_scale = float(genes.get("aggr_min_scale", 0.30))
        self.steer_deadband = float(genes.get("steer_deadband_rad", 0.002))
        self.straight_lock_dist = float(genes.get("straight_lock_dist", 1.70))
        self.straight_lock_speed = float(genes.get("straight_lock_speed", 5.50))
        self.v_cruise_min = float(genes.get("v_cruise_min", V_MIN_CRUISE))
        self.v_cruise_max = float(genes.get("v_cruise_max", V_MAX_CRUISE))
        self.cached_target_speed = self.v_cruise_min

        self.costmap = CppSmartHybridCostmap(
            grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
            resolution=GRID_RES,
            r_inflation_m=self.r_infl,
        )

        self.gap_scout = ToFGapScout(
            w_gap_distance=float(genes.get("w_gap_distance", 3.50)),
            w_gap_width=float(genes.get("w_gap_width", 2.00)),
            w_align_steer=float(genes.get("w_gap_steer", 0.80)),
            w_align_straight=float(genes.get("w_gap_straight", 0.40)),
            w_side_clearance=float(genes.get("w_side_clearance", 2.80)),
            gap_dist_exp_k=float(genes.get("gap_dist_exp_k", 0.60)),
            gap_progress_margin_m=float(genes.get("gap_progress_margin_m", 0.15)),
            alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
            ray_origin_offset_m=float(genes.get("ray_origin_offset_m", 0.070)),
        )

        self.planner = CurvilinearSLMPC(
            costmap_engine=self.costmap,
            horizon_N=int(genes.get("horizon_N", 30)),
            dt=float(genes.get("dt", 0.016667)),
            q_progress=float(genes.get("q_progress", 8.00)),
            q_cte=float(genes.get("q_cte", 18.00)),
            q_heading=float(genes.get("q_heading", 8.00)),
            r_steer=float(genes.get("r_steer", 5.00)),
            r_accel=float(genes.get("r_accel", 2.00)),
            r_steer_rate=float(genes.get("r_steer_rate", 10.00)),
            mu_friction=self.mu_val,
            v_corner_factor=float(genes.get("v_corner_factor", 0.95)),
            pure_pursuit_blend_gain=float(genes.get("pp_blend_gain", 0.35)),
            lookahead_blend_m=float(genes.get("lookahead_blend_m", 0.45)),
            v_cruise_max=self.v_cruise_max,
            v_cruise_min=self.v_cruise_min,
            straight_lock_dist_m=self.straight_lock_dist,
            straight_lock_speed_mps=self.straight_lock_speed,
            alt_switch_hysteresis=float(genes.get("alt_switch_hysteresis", 15.00)),
            alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
        )
        self.planner.costmap = self.costmap

        self.recovery_manager = StuckRecoveryManager(
            reverse_speed_m_s=float(genes.get("recovery_reverse_speed", 1.30)),
            steer_angle_rad=float(genes.get("recovery_steer_angle_rad", 0.4538)),
            clearance_exit_m=float(genes.get("recovery_clearance_exit_m", 0.28)),
            servo_slew_rad_s=SERVO_MAX_SPEED_RAD_S,
            brake_decel_max=BRAKE_DECEL_MAX,
            accel_fwd_max=ACCEL_FWD_MAX,
            arc_lookahead_m=float(genes.get("recovery_arc_lookahead_m", RECOVERY_ARC_LOOKAHEAD_M)),
        )

    def reset(self, start_pos: Optional[List[float]] = None, start_rot: Optional[List[float]] = None) -> None:
        """Resets vehicle physics state, odometry, costmap, and controller memories."""
        if self.is_supervisor and self.robot_node:
            trans_field = self.robot_node.getField("translation")
            rot_field = self.robot_node.getField("rotation")
            if start_pos and trans_field:
                trans_field.setSFVec3f([float(start_pos[0]), float(start_pos[1]), float(start_pos[2])])
            if start_rot and rot_field:
                rot_field.setSFRotation([float(start_rot[0]), float(start_rot[1]), float(start_rot[2]), float(start_rot[3])])
            self.robot_node.resetPhysics()

        for m in self.motors.values():
            if m:
                m.setVelocity(0.0)
        for s in self.servos.values():
            if s:
                s.setPosition(0.0)

        self.fusion.reset()
        self.costmap.clear()
        if hasattr(self, "planner") and hasattr(self.planner, "reset_active_target"):
            self.planner.reset_active_target()
        self.step_counter = 0
        self.active_steer = 0.0
        self.cached_target_steer = 0.0
        self.cached_target_speed = self.v_cruise_min
        self.rate_limited_speed = 0.0
        self.was_recovering = False
        self.mode_desc = "OPTION_2_SL_MPC"
        self.stall_counter = 0
        self.last_plan_res = {}
        self.x_est = float(start_pos[0]) if start_pos else 0.0
        self.y_est = float(start_pos[1]) if start_pos else 0.0
        self.yaw_est = 0.0

    def read_tof_sensors(self) -> Dict[str, float]:
        """Reads all 16 ToF range sensors and returns a dict mapping sensor name -> range in meters."""
        cached_tofs: Dict[str, float] = {}
        for name in TOF_SENSOR_NAMES:
            dev = self.tofs.get(name)
            if dev is not None:
                cached_tofs[name] = read_tof_device(dev)
            else:
                cached_tofs[name] = TOF_MAX_RANGE_M
        self.last_tofs = cached_tofs
        return cached_tofs

    def get_ground_truth_pose(self) -> Tuple[float, float, float]:
        """Returns (x, y, yaw_rad) ground-truth if supervisor available, else odometry."""
        if self.is_supervisor and self.robot_node:
            try:
                trans = self.robot_node.getField("translation")
                rot = self.robot_node.getField("rotation")
                if trans and rot:
                    pos = trans.getSFVec3f()
                    r = rot.getSFRotation()
                    yaw = float(r[3]) * (1.0 if r[2] >= 0.0 else -1.0)
                    return float(pos[0]), float(pos[1]), yaw
            except Exception:
                pass
        return float(self.x_est), float(self.y_est), float(self.yaw_est)

    def get_ground_truth_velocity(self) -> float:
        """Returns real linear velocity from supervisor if available, else fused estimate."""
        if self.is_supervisor and self.robot_node:
            try:
                vel = self.robot_node.getVelocity()
                return float(math.hypot(vel[0], vel[1]))
            except Exception:
                pass
        return float(self.rate_limited_speed)

    def step(self, dt: float = DT, full_telemetry: bool = True) -> Tuple[Dict[str, Any], bool]:
        """
        Executes one 120 Hz dynamics step with 60 Hz planner decimation.
        Returns: (telemetry_dict, planner_executed_boolean)
        """
        self.step_counter += 1
        sim_time = self.step_counter * dt

        # Initial physics warmup pause (0.50 s)
        if sim_time < 0.50:
            for m in self.motors.values():
                if m:
                    m.setVelocity(0.0)
            for s in self.servos.values():
                if s:
                    s.setPosition(0.0)
            if self.step_counter % PLANNER_DECIMATION == 0:
                tof_readings = self.read_tof_sensors()
            else:
                tof_readings = self.last_tofs
            gx, gy, gyaw = self.get_ground_truth_pose()
            telemetry = {
                "time_step": self.step_counter,
                "mode": "WARMUP",
                "planner": "OPTION2_SL_MPC",
                "status": "WARMUP",
                "v_est": 0.0,
                "v_fused": 0.0,
                "v_real": 0.0,
                "yaw_fused": 0.0,
                "v_tgt": 0.0,
                "v_cmd": 0.0,
                "steer_deg": 0.0,
                "steer_rad": 0.0,
                "pose": [float(gx), float(gy), float(gyaw)],
                "gap": [0.0, 1.0, 0.5],
                "targets": [],
                "plan_ms": 0.0,
                "map_ms": 0.0,
                "surviving": 0,
                "candidates": 0,
                "tof": {k: round(float(v), 3) for k, v in tof_readings.items()},
                "tof_ranges": [round(float(tof_readings.get(k, TOF_MAX_RANGE_M)), 3) for k in TOF_SENSOR_NAMES],
                "best_line": [],
                "candidate_lines": [],
                "grid_shape": [1, 1],
                "grid_bounds": [float(GRID_X_MIN), float(GRID_X_MAX), float(GRID_Y_MIN), float(GRID_Y_MAX)],
                "grid_data": "",
                "hazard_data": "",
                "recovery_state": "NORMAL",
                "plan": {},
            }
            return telemetry, False

        planner_ran = False
        tof_readings = self.last_tofs

        # 60 Hz Control Tick (Decimation = 2)
        if self.step_counter % PLANNER_DECIMATION == 0:
            planner_ran = True

            # Read ToF Sensor Array (16 channels at 60 Hz planner rate)
            tof_readings = self.read_tof_sensors()

            # Sensor fusion at 60 Hz (Zero Cheating)
            v_fused, yaw_fused, yaw_rate = self.fusion.update(dt=DT_PLANNER)
            self.v_est = max(0.0, float(v_fused)) if not (math.isnan(v_fused) or math.isinf(v_fused)) else 0.0
            self.cached_yaw_rate = float(yaw_rate) if not (math.isnan(yaw_rate) or math.isinf(yaw_rate)) else 0.0
            self.yaw_est = float(yaw_fused) if not (math.isnan(yaw_fused) or math.isinf(yaw_fused)) else 0.0

            self.x_est += self.v_est * math.cos(self.yaw_est) * (dt * PLANNER_DECIMATION)
            self.y_est += self.v_est * math.sin(self.yaw_est) * (dt * PLANNER_DECIMATION)

            prob_map, hazard_mask, map_ms = self.costmap.process_tof_frame(tof_readings)
            self.cached_map_ms = map_ms

            # Scout Forward Gaps
            xg, yg, width, theta_g = self.gap_scout.scout_gap(
                tof_readings=tof_readings,
                hazard_mask=hazard_mask,
                costmap_obj=self.costmap,
                target_lookahead_m=self.target_lookahead_m,
                steer_current=self.active_steer,
                prob_map=prob_map,
            )
            self.last_gap_info = (xg, yg, width, theta_g)

            # Curvilinear SL-MPC Trajectory Optimization
            t_plan_0 = time.perf_counter()
            plan_res = self.planner.plan(
                xg=xg,
                yg=yg,
                theta_g=theta_g,
                aperture_width=width,
                v_current=self.v_est,
                steer_current=self.active_steer,
                yaw_rate_current=self.cached_yaw_rate,
                extra_targets=getattr(self.gap_scout, "last_targets", None),
            )
            self.cached_plan_ms = (time.perf_counter() - t_plan_0) * 1000.0
            self.last_plan_res = plan_res

            raw_target_steer = float(plan_res.get("cmd_steer_rad", 0.0))
            raw_target_speed = float(plan_res.get("cmd_speed_mps", self.v_cruise_min))

            # Recovery Manager at 60 Hz (Kinematic Escape Arc Costmap Evaluation)
            is_recovering, rec_v, rec_steer = self.recovery_manager.update(
                dt=DT_PLANNER,
                current_v=self.v_est,
                commanded_v=raw_target_speed,
                cached_tofs=tof_readings,
                heading_err=float(theta_g),
                active_steer=self.active_steer,
                hazard_mask=hazard_mask,
                costmap_obj=self.costmap,
            )

            if is_recovering:
                target_speed = rec_v
                target_steer = rec_steer
                self.active_steer = rec_steer
                self.mode_desc = f"RECOVERY_{getattr(self.recovery_manager.state, 'name', self.recovery_manager.state)}"
                self.was_recovering = True
                if hasattr(self.planner, "reset_active_target"):
                    self.planner.reset_active_target()
            else:
                if self.was_recovering:
                    self.was_recovering = False
                    self.active_steer = 0.0

                # Fast Straight-Line Lock (unguarded by turn intent; engages purely on forward clearance)
                d_fl_lock = tof_readings.get("tof_front_left_4.1deg", tof_readings.get("tof_fl_3", 0.0))
                d_fr_lock = tof_readings.get("tof_front_right_4.1deg", tof_readings.get("tof_fr_3", 0.0))
                is_straight_locked = (
                    d_fl_lock >= self.straight_lock_dist and
                    d_fr_lock >= self.straight_lock_dist
                )

                if is_straight_locked:
                    target_speed = self.straight_lock_speed
                    target_steer = 0.0
                    self.active_steer = 0.0
                    self.mode_desc = "STRAIGHT_LOCK"
                else:
                    target_speed = raw_target_speed
                    target_steer = raw_target_steer
                    steer_delta = target_steer - self.active_steer

                    # Progressive distance & small-offset steering aggressiveness
                    cached_d_straight = min(
                        tof_readings.get("tof_front_left_4.1deg", tof_readings.get("tof_fl_3", TOF_MAX_RANGE_M)),
                        tof_readings.get("tof_front_right_4.1deg", tof_readings.get("tof_fr_3", TOF_MAX_RANGE_M)),
                    )
                    if cached_d_straight >= self.aggr_dist_far:
                        dist_scale = self.aggr_min_scale
                    elif cached_d_straight <= self.aggr_dist_close:
                        dist_scale = 1.0
                    else:
                        frac = (self.aggr_dist_far - cached_d_straight) / max(0.001, (self.aggr_dist_far - self.aggr_dist_close))
                        dist_scale = float(self.aggr_min_scale + (frac ** 1.5) * (1.0 - self.aggr_min_scale))

                    err_mag = abs(steer_delta)
                    if err_mag <= self.steer_deadband:
                        err_scale = 0.90
                    elif err_mag >= 0.05:
                        err_scale = 1.0
                    else:
                        err_frac = (err_mag - self.steer_deadband) / max(0.001, (0.05 - self.steer_deadband))
                        err_scale = float(0.90 + err_frac * (1.0 - 0.90))

                    steer_aggressiveness = float(max(0.85, min(1.0, dist_scale * err_scale)))
                    self.active_steer = self.active_steer + steer_aggressiveness * steer_delta
                    self.mode_desc = "OPTION_2_SL_MPC"

            self.cached_target_speed = target_speed
            self.cached_target_steer = self.active_steer

        # 4. Friction Rate Limiter for speed command (120 Hz)
        speed_err = self.cached_target_speed - self.rate_limited_speed
        if (self.rate_limited_speed >= 0.0 and speed_err > 0.0) or (self.rate_limited_speed <= 0.0 and speed_err < 0.0):
            max_dv = (0.85 * self.mu_val * 9.81) * dt
        else:
            max_dv = (1.00 * self.mu_val * 9.81) * dt

        if abs(speed_err) <= max_dv:
            self.rate_limited_speed = self.cached_target_speed
        else:
            self.rate_limited_speed += math.copysign(max_dv, speed_err)

        live_gyro_z = float(self.gyro.getValues()[2]) if self.gyro else self.cached_yaw_rate
        diff_res = self.ediff.update(
            target_v=self.rate_limited_speed,
            target_steer_rad=self.cached_target_steer,
            gyro_z_rad_s=live_gyro_z,
        )

        # 5. Apply Actuator Commands
        if self.servos.get("left"):
            self.servos["left"].setPosition(diff_res["steer_left"])
        if self.servos.get("right"):
            self.servos["right"].setPosition(diff_res["steer_right"])

        if self.motors.get("FL"):
            self.motors["FL"].setVelocity(diff_res["omega_FL"])
        if self.motors.get("FR"):
            self.motors["FR"].setVelocity(diff_res["omega_FR"])
        if self.motors.get("RL"):
            self.motors["RL"].setVelocity(diff_res["omega_RL"])
        if self.motors.get("RR"):
            self.motors["RR"].setVelocity(diff_res["omega_RR"])

        # 6. Assemble Telemetry Payload (Full for GUI / Lean for headless GA)
        gx, gy, gyaw = self.get_ground_truth_pose()
        v_real = self.get_ground_truth_velocity()

        if not full_telemetry:
            telemetry = {
                "time_step": self.step_counter,
                "mode": self.mode_desc,
                "planner": "OPTION2_SL_MPC",
                "status": self.last_plan_res.get("status", "OK"),
                "v_est": round(float(self.v_est), 3),
                "v_fused": round(float(self.v_est), 3),
                "v_real": round(float(v_real), 3),
                "yaw_fused": round(float(self.yaw_est), 4),
                "v_tgt": round(float(self.cached_target_speed), 3),
                "v_cmd": round(float(self.cached_target_speed), 3),
                "steer_deg": round(math.degrees(float(self.cached_target_steer)), 2),
                "steer_rad": round(float(self.cached_target_steer), 4),
                "pose": [float(gx), float(gy), float(gyaw)],
                "plan_ms": round(float(self.cached_plan_ms), 2),
                "map_ms": round(float(self.cached_map_ms), 2),
                "recovery_state": str(getattr(self.recovery_manager.state, "name", self.recovery_manager.state)),
            }
            return telemetry, planner_ran

        prob_down = (self.costmap.prob_map[::2, ::2] * 255.0).astype(np.uint8)
        hazard_down = self.costmap.hazard_mask[::2, ::2].astype(np.uint8)
        grid_compressed = base64.b64encode(zlib.compress(prob_down.tobytes())).decode("ascii")
        hazard_compressed = base64.b64encode(zlib.compress(hazard_down.tobytes())).decode("ascii")

        b_line = to_py(self.last_plan_res.get("best_line", []))
        c_lines = to_py(self.last_plan_res.get("candidate_lines", []))[:20]
        xg, yg, width, theta_g = self.last_gap_info
        g_gap = [round(float(xg), 3), round(float(yg), 3), round(float(width), 3)]
        g_targets = [
            [round(float(tx), 3), round(float(ty), 3), round(float(ap), 3)]
            for (tx, ty, ap) in (getattr(self.gap_scout, "last_targets", []) or [])
        ]

        telemetry = {
            "time_step": self.step_counter,
            "mode": self.mode_desc,
            "planner": "OPTION2_SL_MPC",
            "status": self.last_plan_res.get("status", "OK"),
            "v_est": round(float(self.v_est), 3),
            "v_fused": round(float(self.v_est), 3),
            "v_real": round(float(v_real), 3),
            "yaw_fused": round(float(self.yaw_est), 4),
            "v_tgt": round(float(self.cached_target_speed), 3),
            "v_cmd": round(float(self.cached_target_speed), 3),
            "steer_deg": round(math.degrees(float(self.cached_target_steer)), 2),
            "steer_rad": round(float(self.cached_target_steer), 4),
            "pose": [float(gx), float(gy), float(gyaw)],
            "gap": g_gap,
            "targets": g_targets,
            "plan_ms": round(float(self.cached_plan_ms), 2),
            "map_ms": round(float(self.cached_map_ms), 2),
            "surviving": int(self.last_plan_res.get("surviving_count", 0)),
            "candidates": int(self.last_plan_res.get("candidate_count", len(c_lines))),
            "tof": {k: round(float(v), 3) for k, v in tof_readings.items()},
            "tof_ranges": [round(float(tof_readings.get(k, TOF_MAX_RANGE_M)), 3) for k in TOF_SENSOR_NAMES],
            "best_line": b_line,
            "candidate_lines": c_lines,
            "grid_shape": [int(s) for s in prob_down.shape],
            "grid_bounds": [float(GRID_X_MIN), float(GRID_X_MAX), float(GRID_Y_MIN), float(GRID_Y_MAX)],
            "grid_data": grid_compressed,
            "hazard_data": hazard_compressed,
            "recovery_state": str(getattr(self.recovery_manager.state, "name", self.recovery_manager.state)),
            "plan": self.last_plan_res,
        }

        return telemetry, planner_ran
