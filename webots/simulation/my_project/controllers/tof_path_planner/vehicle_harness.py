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
import copy
import collections
import gc
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

_gc_stats = {"step_gc_ms": 0.0, "last_gen": -1}
_gc_t0 = 0.0

def _folkrace_gc_cb(phase, info):
    global _gc_t0
    if phase == "start":
        _gc_t0 = time.perf_counter()
    elif phase == "stop":
        dt_ms = (time.perf_counter() - _gc_t0) * 1000.0
        _gc_stats["step_gc_ms"] += dt_ms
        _gc_stats["last_gen"] = info.get("generation", -1)

if not any(getattr(cb, "__name__", "") == "_folkrace_gc_cb" for cb in gc.callbacks):
    gc.callbacks.append(_folkrace_gc_cb)
gc.set_threshold(50000, 20, 20)

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)

for p in [CURR_DIR, CONTROLLERS_DIR, MY_PROJECT_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from electronic_diff_4wd import ElectronicDifferential4WD
from traction_drift_controller import TractionDriftController
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from imu_encoder_fusion import IMUEncoderFusion
from tof_gap_scout import ToFGapScout
from planner_sl_mpc import CurvilinearSLMPC
from stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState
from anti_wrong_way_watchdog import AntiWrongWayWatchdog
from dynamic_obstacle_tracker import DynamicObstacleTracker
from planner_config import (
    TIME_STEP_MS,
    DT,
    SIM_HZ,
    PLANNER_HZ,
    DT_PLANNER,
    PLANNER_DECIMATION,
    TOF_TIME_STEP_MS,
    TOF_LATENCY_STEPS,
    ACTUATOR_LATENCY_STEPS,
    TOTAL_LATENCY_STEPS,
    TOF_SENSOR_NAMES,
    TOF_MAX_RANGE_M,
    TOF_MIN_RANGE_M,
    ENCODER_NAMES,
    WHEEL_RADIUS,
    V_MIN_CRUISE,
    V_MAX_CRUISE,
    MAX_SERVO_RAD,
    MAX_ACKERMANN_RAD,
    MAX_STEERING_RAD,
    ACKERMANN_PERCENT,
    WHEELBASE,
    TRACK_REAR,
    TRACK_FRONT,
    TRACK_KINGPIN,
    REAR_AXLE_OFFSET_M,
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
    RECOVERY_ANY_TOF_EXIT_M,
    RECOVERY_ARC_LOOKAHEAD_M,
    ACCEL_FWD_MAX,
    BRAKE_DECEL_MAX,
    SERVO_MAX_SPEED_RAD_S,
    STRAIGHT_LOCK_DIST_M,
    STRAIGHT_LOCK_SPEED_MPS,
    FRONT_LATERAL_RESERVE_MIN,
    ENABLE_TRACTION_DRIFT_CONTROL,
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


def wrap_to_pi(angle: float) -> float:
    """Wraps angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class LatencyQueue:
    """
    Deterministic FIFO delay line guaranteeing exactly `steps` delay.
    At step k, update(x_k) returns x_{k - steps}.
    """

    def __init__(self, steps: int, default_val: Any):
        self.steps = max(0, int(steps))
        self.queue = collections.deque([copy.deepcopy(default_val) for _ in range(self.steps)])

    def update(self, new_val: Any) -> Any:
        """Atomic push new value and pop delayed value."""
        if self.steps <= 0:
            return new_val
        self.queue.append(new_val)
        return self.queue.popleft()

    def peek(self) -> Any:
        """Peeks at the delayed element without evicting."""
        return self.queue[0] if self.queue else None

    def push(self, new_val: Any) -> Any:
        """Alias for update() to support push-pop paradigms."""
        return self.update(new_val)

    def reset(self, default_val: Any) -> None:
        self.queue = collections.deque([copy.deepcopy(default_val) for _ in range(self.steps)])


class FolkraceVehicle:
    """
    Encapsulates all actuators, sensors, and the full multi-rate control
    pipeline (120 Hz physics / 60 Hz SL-MPC) for the Folkrace vehicle.
    """

    def __init__(self, robot: Any, is_supervisor: bool = False, genes: Optional[Dict[str, Any]] = None):
        self.robot = robot
        self.is_supervisor = is_supervisor
        self.robot_node = robot.getSelf() if (is_supervisor and hasattr(robot, "getSelf")) else None
        self._trans_field = self.robot_node.getField("translation") if (self.is_supervisor and self.robot_node) else None
        self._rot_field = self.robot_node.getField("rotation") if (self.is_supervisor and self.robot_node) else None
        self._step_gt_pose = None
        self._step_gt_vel = None
        self._step_rear_flow = None

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

        # Discover wheel endpoint Solid nodes for synchronized physics resets
        self.wheel_nodes = []
        if self.is_supervisor and self.robot_node:
            try:
                children = self.robot_node.getField("children")
                if children:
                    for i in range(children.getCount()):
                        child = children.getMFNode(i)
                        if child and child.getTypeName() in ("HingeJoint", "Hinge2Joint"):
                            ep = child.getField("endPoint")
                            if ep:
                                ep_node = ep.getSFNode()
                                if ep_node:
                                    self.wheel_nodes.append(ep_node)
            except Exception:
                pass

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
        self.fusion = IMUEncoderFusion(
            wheel_radius=WHEEL_RADIUS,
            track_rear=TRACK_REAR,
            track_front=TRACK_FRONT,
            wheelbase=WHEELBASE,
            dt=DT,
        )
        self.fusion.attach_devices(self.encoders, imu_dev=self.imu_dev, gyro_dev=self.gyro, accel_dev=self.accel)

        self.costmap = CppSmartHybridCostmap(
            grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
            resolution=GRID_RES,
            r_inflation_m=R_INFLATION_M,
        )

        # Dynamic moving obstacle tracker (NOTE: Disabled because it is not working properly, WIP)
        self.obstacle_tracker = DynamicObstacleTracker(
            grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
            resolution=GRID_RES,
            enabled=False,
        )
        self.last_dynamic_tracks = []
        self.last_obs_telem = {}
        self.last_enlarged_hazard = None

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
            max_servo_angle=MAX_SERVO_RAD,
            ackermann_percent=ACKERMANN_PERCENT,
            motor_dir=MOTOR_DIR,
            v_max=V_MAX_WHEEL,
        )
        self.recovery_manager = StuckRecoveryManager(
            reverse_speed_m_s=RECOVERY_REVERSE_SPEED_MPS,
            forward_speed_m_s=V_MIN_CRUISE,
            steer_angle_rad=RECOVERY_STEER_ANGLE_RAD,
            clearance_exit_m=RECOVERY_CLEARANCE_EXIT_M,
            any_tof_exit_m=RECOVERY_ANY_TOF_EXIT_M,
            servo_slew_rad_s=SERVO_MAX_SPEED_RAD_S,
            brake_decel_max=BRAKE_DECEL_MAX,
            accel_fwd_max=ACCEL_FWD_MAX,
            arc_lookahead_m=RECOVERY_ARC_LOOKAHEAD_M,
        )
        self.drift_controller = TractionDriftController(
            wheelbase=WHEELBASE,
            track_rear=TRACK_REAR,
            max_steer=MAX_STEERING_RAD,
            enabled=ENABLE_TRACTION_DRIFT_CONTROL,
        )

        # Default Tuning Parameters
        self.base_mu = 0.817
        self.mu_val = self.base_mu
        self.r_infl = R_INFLATION_M
        self.target_lookahead_m = 1.35
        self.aggr_dist_far = 1.70
        self.aggr_dist_close = 1.00
        self.aggr_min_scale = 0.30
        self.steer_deadband = 0.002
        self.straight_lock_dist = 1.80
        self.straight_lock_speed = 5.00
        self.v_cruise_min = V_MIN_CRUISE
        self.v_cruise_max = V_MAX_CRUISE

        # Dynamic State Tracking
        self.step_counter = 0
        self.active_steer = 0.0
        self.cached_target_steer = 0.0
        self.cached_target_speed = self.v_cruise_min
        self.rate_limited_speed = 0.0
        self.was_recovering = False
        self.mode_desc = "SL_MPC"
        self.cached_plan_ms = 0.0
        self.cached_plan_cpu_ms = 0.0
        self.cached_map_ms = 0.0
        self.cached_scout_ms = 0.0
        self.cached_obs_ms = 0.0
        self.cached_opt_ms = 0.0
        self.cached_opt_prim_ms = 0.0
        self.cached_opt_alt_ms = 0.0
        self.cached_opt_shield_ms = 0.0
        self.cached_osqp_iters = 0
        self.cached_osqp_iters_alt = 0
        self.cached_telem_ms = 0.0
        self.cached_step_ms = 0.0
        self.cached_step_cpu_ms = 0.0
        self.cached_gc_ms = 0.0
        self.v_est = 0.0
        self.cached_yaw_rate = 0.0
        self.x_est = 0.0
        self.y_est = 0.0
        self.yaw_est = 0.0
        self.stall_counter = 0
        self.last_gap_info = (0.0, 1.0, 0.50, 0.0)
        self.last_plan_res = {}
        self.last_tofs = {name: TOF_MAX_RANGE_M for name in TOF_SENSOR_NAMES}
        self.cached_f_tof = TOF_MAX_RANGE_M

        # Hardware Latency Buffers (2-step ToF delay + 1-step actuator lag = 3 steps total)
        initial_tofs = {name: TOF_MAX_RANGE_M for name in TOF_SENSOR_NAMES}
        self.tof_queue = LatencyQueue(steps=TOF_LATENCY_STEPS, default_val=initial_tofs)
        start_x = float(self.x_est)
        start_y = float(self.y_est)
        start_yaw = float(self.yaw_est)
        self.pose_history = collections.deque(
            [(start_x, start_y, start_yaw) for _ in range(TOF_LATENCY_STEPS + 1)],
            maxlen=TOF_LATENCY_STEPS + 1,
        )
        self.proj_anchor = (0.0, 0.0, 0.0)
        self.proj_v = 0.0
        self.proj_steer = 0.0

        # Anti-Wrong-Way Watchdog
        self.watchdog = AntiWrongWayWatchdog()
        self.last_aww_status = None

        # Apply genes if provided
        if genes:
            self.configure(genes)

    def configure(self, genes: Dict[str, Any]) -> None:
        """Dynamically applies genome parameters to all vehicle subsystems."""
        if not genes:
            return

        self.base_mu = float(genes.get("mu_friction", 0.817))
        self.mu_val = self.base_mu
        self.r_infl = float(genes.get("r_inflation_m", R_INFLATION_M))
        self.target_lookahead_m = float(genes.get("target_lookahead_m", 1.35))
        self.aggr_dist_far = float(genes.get("aggr_dist_far_m", 1.70))
        self.aggr_dist_close = float(genes.get("aggr_dist_close_m", 1.00))
        self.aggr_min_scale = float(genes.get("aggr_min_scale", 0.30))
        self.steer_deadband = float(genes.get("steer_deadband_rad", 0.002))
        self.straight_lock_dist = float(genes.get("straight_lock_dist", 1.80))
        self.straight_lock_speed = float(genes.get("straight_lock_speed", 5.00))
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
            gap_dist_thresh_m=float(genes.get("gap_dist_thresh_m", 1.00 + genes.get("gap_progress_margin_m", 0.0) if "gap_progress_margin_m" in genes else 0.95)),
            alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
            scout_obs_width_m=float(genes.get("scout_obs_width_m", 0.22)),
            scout_block_dist_m=float(genes.get("scout_block_dist_m", genes.get("target_lookahead_m", 1.72))),
            scout_straight_pen_k=float(genes.get("scout_straight_pen_k", 2.50)),
            scout_straight_exp_k=float(genes.get("scout_straight_exp_k", 1.50)),
            ray_origin_offset_m=float(genes.get("ray_origin_offset_m", 0.070)),
            w_gap_clear=float(genes.get("w_gap_clear", genes.get("alt_lookahead_depth_weight", 3.00))),
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
            v_cruise_max=self.v_cruise_max,
            v_cruise_min=self.v_cruise_min,
            straight_lock_dist_m=self.straight_lock_dist,
            straight_lock_speed_mps=self.straight_lock_speed,
            alt_switch_hysteresis=float(genes.get("alt_switch_hysteresis", 15.00)),
            alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
            w_gap_clear=float(genes.get("w_gap_clear", genes.get("alt_lookahead_depth_weight", 5.00))),
        )
        self.planner.costmap = self.costmap

        rec_steer = float(genes.get("recovery_steer_angle_rad", RECOVERY_STEER_ANGLE_RAD))
        # Compute maximum traction-stable reverse velocity governed by tire friction:
        # v_rev = sqrt(mu * g * (WHEELBASE / tan(steer_angle)))
        rec_speed = float(
            math.sqrt(max(0.01, self.mu_val * 9.81 * (WHEELBASE / max(0.01, math.tan(abs(rec_steer))))))
        )
        if "recovery_reverse_speed" in genes:
            rec_speed = float(genes["recovery_reverse_speed"])

        self.recovery_manager = StuckRecoveryManager(
            reverse_speed_m_s=rec_speed,
            steer_angle_rad=rec_steer,
            clearance_exit_m=float(genes.get("recovery_clearance_exit_m", RECOVERY_CLEARANCE_EXIT_M)),
            any_tof_exit_m=float(genes.get("recovery_any_tof_exit_m", RECOVERY_ANY_TOF_EXIT_M)),
            servo_slew_rad_s=SERVO_MAX_SPEED_RAD_S,
            brake_decel_max=BRAKE_DECEL_MAX,
            accel_fwd_max=ACCEL_FWD_MAX,
            arc_lookahead_m=float(genes.get("recovery_arc_lookahead_m", RECOVERY_ARC_LOOKAHEAD_M)),
        )

        # Dynamic Obstacle Tracker Configuration
        if hasattr(self, "obstacle_tracker") and self.obstacle_tracker is not None:
            self.obstacle_tracker.configure(
                prob_threshold=genes.get("obs_prob_threshold"),
                max_wall_dim_cells=genes.get("obs_max_wall_dim_cells"),
                min_area_cells=genes.get("obs_min_area_cells"),
                gating_radius_m=genes.get("obs_gating_radius_m"),
                persistence_frames=genes.get("obs_persistence_frames"),
                dynamic_threshold_mps=genes.get("obs_dynamic_threshold_mps"),
                dyn_confirm_frames=genes.get("obs_dyn_confirm_frames"),
                lead_time_s=genes.get("obs_lead_time_s"),
                max_lead_m=genes.get("obs_max_lead_m"),
                tail_clear_m=genes.get("obs_tail_clear_m"),
                max_missed_frames=genes.get("obs_max_missed_frames"),
                alpha_ab=genes.get("obs_alpha_ab"),
                beta_ab=genes.get("obs_beta_ab"),
                vel_decay=genes.get("obs_vel_decay"),
            )

    def reset(self, start_pos: Optional[List[float]] = None, start_rot: Optional[List[float]] = None) -> None:
        """Resets vehicle physics state, odometry, costmap, and controller memories."""
        self._step_gt_pose = None
        self._step_gt_vel = None
        self._step_rear_flow = None
        if self.is_supervisor and self.robot_node:
            trans_field = self._trans_field if self._trans_field is not None else self.robot_node.getField("translation")
            rot_field = self._rot_field if self._rot_field is not None else self.robot_node.getField("rotation")
            if start_pos and trans_field:
                trans_field.setSFVec3f([float(start_pos[0]), float(start_pos[1]), float(start_pos[2])])
            if start_rot and rot_field:
                rot_field.setSFRotation([float(start_rot[0]), float(start_rot[1]), float(start_rot[2]), float(start_rot[3])])
            try:
                self.robot_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass
            self.robot_node.resetPhysics()

            # Synchronously zero velocities and reset physics on all wheel endpoint solids
            for wn in self.wheel_nodes:
                try:
                    wn.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    wn.resetPhysics()
                except Exception:
                    pass

        for m in self.motors.values():
            if m:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
        for s in self.servos.values():
            if s:
                s.setPosition(0.0)

        self.fusion.reset()
        self.costmap.clear()
        self.watchdog.reset()
        self.drift_controller.reset()
        self.last_aww_status = None
        if hasattr(self, "planner") and hasattr(self.planner, "reset_active_target"):
            self.planner.reset_active_target()
        self.step_counter = 0
        self.active_steer = 0.0
        self.cached_target_steer = 0.0
        self.cached_target_speed = self.v_cruise_min
        self.rate_limited_speed = 0.0
        self.was_recovering = False
        self.mode_desc = "SL_MPC"
        self.stall_counter = 0
        self.v_est = 0.0
        self.cached_yaw_rate = 0.0
        self.x_est = float(start_pos[0]) if start_pos else 0.0
        self.y_est = float(start_pos[1]) if start_pos else 0.0
        self.yaw_est = 0.0
        self.obstacle_tracker.reset()
        self.last_dynamic_tracks = []
        self.last_obs_telem = {}
        self.last_enlarged_hazard = None

        # Reset latency queues & pose history
        initial_tofs = {name: TOF_MAX_RANGE_M for name in TOF_SENSOR_NAMES}
        self.tof_queue.reset(initial_tofs)
        sx = float(self.x_est)
        sy = float(self.y_est)
        syaw = float(self.yaw_est)
        self.pose_history = collections.deque(
            [(sx, sy, syaw) for _ in range(TOF_LATENCY_STEPS + 1)],
            maxlen=TOF_LATENCY_STEPS + 1,
        )
        self.proj_anchor = (0.0, 0.0, 0.0)
        self.proj_v = 0.0
        self.proj_steer = 0.0

    def read_tof_sensors(self) -> Dict[str, float]:
        """Reads all 16 ToF range sensors and returns a dict mapping sensor name -> range in meters."""
        cached_tofs: Dict[str, float] = {}
        noise_pct = getattr(self, "tof_noise_pct", float(os.environ.get("TOF_NOISE_PCT", 0.0)))
        for name in TOF_SENSOR_NAMES:
            dev = self.tofs.get(name)
            if dev is not None:
                val = read_tof_device(dev)
                if noise_pct > 0.0 and val < (TOF_MAX_RANGE_M - 0.01):
                    noise = float(np.random.uniform(-noise_pct, noise_pct)) * val
                    val = float(np.clip(val + noise, 0.02, TOF_MAX_RANGE_M))
                cached_tofs[name] = val
            else:
                cached_tofs[name] = TOF_MAX_RANGE_M
        self.raw_tofs = cached_tofs
        return cached_tofs

    def get_ground_truth_pose(self) -> Tuple[float, float, float]:
        """Returns (x, y, yaw_rad) ground-truth if supervisor available, else odometry."""
        if self._step_gt_pose is not None:
            return self._step_gt_pose
        if self.is_supervisor and self.robot_node:
            try:
                trans = self._trans_field if self._trans_field is not None else self.robot_node.getField("translation")
                rot = self._rot_field if self._rot_field is not None else self.robot_node.getField("rotation")
                if trans and rot:
                    pos = trans.getSFVec3f()
                    r = rot.getSFRotation()
                    yaw = float(r[3]) * (1.0 if r[2] >= 0.0 else -1.0)
                    res = (float(pos[0]), float(pos[1]), yaw)
                    self._step_gt_pose = res
                    return res
            except Exception:
                pass
        res = (float(self.x_est), float(self.y_est), float(self.yaw_est))
        self._step_gt_pose = res
        return res

    def get_ground_truth_velocity(self) -> float:
        """Returns real signed longitudinal velocity from supervisor if available, else fused estimate."""
        if self._step_gt_vel is not None:
            return self._step_gt_vel
        if self.is_supervisor and self.robot_node:
            try:
                vel = self.robot_node.getVelocity()
                rot = self._rot_field if self._rot_field is not None else self.robot_node.getField("rotation")
                if rot:
                    r = rot.getSFRotation()
                    yaw = float(r[3]) * (1.0 if r[2] >= 0.0 else -1.0)
                    # Vehicle forward unit vector in Webots world frame: (-sin(yaw), cos(yaw))
                    v_long = vel[0] * (-math.sin(yaw)) + vel[1] * math.cos(yaw)
                    self._step_gt_vel = float(v_long)
                    return self._step_gt_vel
                self._step_gt_vel = float(math.hypot(vel[0], vel[1]))
                return self._step_gt_vel
            except Exception:
                pass
        self._step_gt_vel = float(self.v_est)
        return self._step_gt_vel

    def get_ground_truth_rear_axle_flow(self) -> Dict[str, float]:
        """
        Calculates ground-truth optical flow velocity at the center of the rear axle.
        Matches what an ideal downward-facing optical flow sensor mounted at (0, -0.070, 0)
        in the vehicle body frame measures.

        Returns:
            dict containing:
                - 'v_forward': forward longitudinal speed in m/s
                - 'v_lateral': lateral drift / slide speed in m/s (+ = rightward slip)
                - 'v_mag': resultant ground speed magnitude in m/s
                - 'slip_angle_rad': tire slip angle beta = atan2(v_lateral, v_forward) in radians
                - 'yaw_rate_rad_s': angular rate around Z axis in rad/s
        """
        if self._step_rear_flow is not None:
            return self._step_rear_flow
        if self.is_supervisor and self.robot_node:
            try:
                vel = self.robot_node.getVelocity()  # [vx, vy, vz, wx, wy, wz]
                rot = self._rot_field if self._rot_field is not None else self.robot_node.getField("rotation")
                if rot and vel:
                    r = rot.getSFRotation()
                    yaw = float(r[3]) * (1.0 if r[2] >= 0.0 else -1.0)
                    vx_w, vy_w = float(vel[0]), float(vel[1])
                    wz = float(vel[5])

                    # Local body frame velocities at chassis origin:
                    v_fwd_origin = vx_w * (-math.sin(yaw)) + vy_w * math.cos(yaw)
                    v_lat_origin = vx_w * math.cos(yaw) + vy_w * math.sin(yaw)

                    # Kinematic rigid-body offset to rear axle center (r_y = REAR_AXLE_OFFSET_M = -0.070 m):
                    # v_rear = v_origin + omega x r -> v_lat_rear = v_lat_origin - wz * r_y
                    v_fwd_rear = float(v_fwd_origin)
                    v_lat_rear = float(v_lat_origin - wz * REAR_AXLE_OFFSET_M)
                    v_mag = float(math.hypot(v_fwd_rear, v_lat_rear))
                    slip_angle = float(math.atan2(v_lat_rear, v_fwd_rear)) if abs(v_fwd_rear) > 1e-4 else 0.0

                    res = {
                        "v_forward": round(v_fwd_rear, 4),
                        "v_lateral": round(v_lat_rear, 4),
                        "v_mag": round(v_mag, 4),
                        "slip_angle_rad": round(slip_angle, 4),
                        "yaw_rate_rad_s": round(wz, 4),
                    }
                    self._step_rear_flow = res
                    return res
            except Exception:
                pass

        v_fwd = float(self.v_est)
        wz = float(self.cached_yaw_rate)
        res = {
            "v_forward": round(v_fwd, 4),
            "v_lateral": 0.0,
            "v_mag": round(abs(v_fwd), 4),
            "slip_angle_rad": 0.0,
            "yaw_rate_rad_s": round(wz, 4),
        }
        self._step_rear_flow = res
        return res

    def step(
        self,
        dt: float = DT,
        full_telemetry: bool = True,
        override_speed: Optional[float] = None,
        override_steer: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], bool]:
        """
        Executes one 120 Hz dynamics step with 60 Hz planner decimation.
        Returns: (telemetry_dict, planner_executed_boolean)
        """
        t_step_start = time.perf_counter()
        t_step_cpu_start = time.process_time()
        _gc_stats["step_gc_ms"] = 0.0
        self.step_counter += 1
        self._step_gt_pose = None
        self._step_gt_vel = None
        self._step_rear_flow = None
        sim_time = self.step_counter * dt

        # Initial physics warmup pause (0.50 s)
        if sim_time < 0.50:
            for m in self.motors.values():
                if m:
                    m.setVelocity(0.0)
            for s in self.servos.values():
                if s:
                    s.setPosition(0.0)
            raw_tofs = self.read_tof_sensors()
            tof_readings = self.tof_queue.update(raw_tofs)
            self.last_tofs = tof_readings
            self.pose_history.append((self.x_est, self.y_est, self.yaw_est))
            gx, gy, gyaw = self.get_ground_truth_pose()
            rear_flow = self.get_ground_truth_rear_axle_flow()
            telemetry = {
                "time_step": self.step_counter,
                "sim_time": round(float(sim_time), 3),
                "mode": "WARMUP",
                "planner": "SL_MPC",
                "status": "WARMUP",
                "v_est": 0.0,
                "v_fused": 0.0,
                "v_real": 0.0,
                "flow_rear_axle": rear_flow,
                "yaw_fused": 0.0,
                "v_plan": 0.0,
                "v_tgt": 0.0,
                "v_cmd": 0.0,
                "steer_plan_deg": 0.0,
                "steer_cmd_deg": 0.0,
                "steer_deg": 0.0,
                "steer_rad": 0.0,
                "steer_actual_deg": 0.0,
                "pose": [float(gx), float(gy), float(gyaw)],
                "proj_anchor": [0.0, 0.0, 0.0],
                "proj_v": 0.0,
                "proj_steer_deg": 0.0,
                "gap": [0.0, 1.0, 0.5],
                "targets": [],
                "plan_ms": 0.0,
                "map_ms": 0.0,
                "scout_ms": 0.0,
                "obs_ms": 0.0,
                "opt_ms": 0.0,
                "opt_prim_ms": 0.0,
                "opt_alt_ms": 0.0,
                "opt_shield_ms": 0.0,
                "telem_ms": 0.0,
                "step_ms": 0.0,
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
                "dynamic_obstacles": [],
                "obs_telem": {},
                "escape_state": "NORMAL",
                "f_tof": round(float(min(tof_readings.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M), tof_readings.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M))), 2),
                "plan": {},
            }
            return telemetry, False

        # 120 Hz IMU + 4-Wheel Encoder Sensor Fusion & Odometry Dead Reckoning
        active_steer = float(getattr(self.ediff, "current_steer_rad", self.active_steer))
        v_fused, yaw_rate, yaw_fused = self.fusion.update(dt=dt, steer_rad=active_steer)
        self.v_est = float(v_fused) if not (math.isnan(v_fused) or math.isinf(v_fused)) else 0.0
        self.cached_yaw_rate = float(yaw_rate) if not (math.isnan(yaw_rate) or math.isinf(yaw_rate)) else 0.0
        self.yaw_est = float(yaw_fused) if not (math.isnan(yaw_fused) or math.isinf(yaw_fused)) else 0.0

        # Odometry Dead Reckoning in world frame (Robot body: +Y is Forward, +X is Right)
        # Use mid-point heading for exact second-order curvature accuracy during sharp cornering
        yaw_mid = self.yaw_est + 0.5 * self.cached_yaw_rate * dt
        self.x_est += -self.v_est * math.sin(yaw_mid) * dt
        self.y_est +=  self.v_est * math.cos(yaw_mid) * dt

        # Record current fused pose in dead-reckoning history buffer for SE(2) projection
        self.pose_history.append((self.x_est, self.y_est, self.yaw_est))

        planner_ran = False
        tof_readings = self.last_tofs

        # Closed-loop Control Tick (125 Hz with Decimation = 1)
        if self.step_counter % PLANNER_DECIMATION == 0:
            planner_ran = True
            raw_tofs = self.read_tof_sensors()
            tof_readings = self.tof_queue.update(raw_tofs)
            self.last_tofs = tof_readings

        f_tof = min(
            tof_readings.get("tof_front_left_4.1deg", tof_readings.get("tof_fl_3", TOF_MAX_RANGE_M)),
            tof_readings.get("tof_front_right_4.1deg", tof_readings.get("tof_fr_3", TOF_MAX_RANGE_M)),
        )
        self.cached_f_tof = f_tof

        if self.step_counter % PLANNER_DECIMATION == 0:
            prob_map, hazard_mask, map_ms = self.costmap.process_tof_frame(tof_readings)
            self.cached_map_ms = map_ms

            # Dynamic Moving Obstacle Detection, Tracking & Vector Enlargement
            # NOTE: Disabled because it is not working properly, WIP.
            t_obs_0 = time.perf_counter()
            enlarged_hazard_mask, dyn_tracks, obs_telem = self.obstacle_tracker.process_frame(
                prob_map=prob_map,
                robot_pose=(self.x_est, self.y_est, self.yaw_est),
                now_s=sim_time,
                base_hazard_mask=hazard_mask,
                source_grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
                v_ego=self.v_est,
                yaw_rate=self.cached_yaw_rate,
                tof_readings=tof_readings,
            )
            self.cached_obs_ms = (time.perf_counter() - t_obs_0) * 1000.0
            self.last_dynamic_tracks = dyn_tracks
            self.last_confirmed_tracks = getattr(self.obstacle_tracker, "last_confirmed_tracks", dyn_tracks)
            self.last_obs_telem = obs_telem
            self.last_enlarged_hazard = enlarged_hazard_mask

            # ------------------------------------------------------------------
            # 3-STEP SE(2) FORWARD LOOKAHEAD PROJECTION
            # Cancels 2-step ToF integration delay + 1-step actuator execution lag
            # ------------------------------------------------------------------
            # 1. Past Motion (step k-2 -> step k, 16 ms elapsed in F_sens)
            x_past, y_past, yaw_past = self.pose_history[0]
            x_curr, y_curr, yaw_curr = self.pose_history[-1]

            delta_yaw_past = wrap_to_pi(yaw_curr - yaw_past)
            dx_w = x_curr - x_past
            dy_w = y_curr - y_past
            cos_p = math.cos(yaw_past)
            sin_p = math.sin(yaw_past)
            dx_past = cos_p * dx_w + sin_p * dy_w
            dy_past = -sin_p * dx_w + cos_p * dy_w

            # 2. Future Motion (step k -> step k+1, 8 ms future via bicycle model)
            v_k = self.v_est if self.recovery_manager.state != StuckRecoveryState.NORMAL_DRIVE else max(0.0, self.v_est)
            dt_act = ACTUATOR_LATENCY_STEPS * dt
            current_physical_steer = float(self.ediff.current_steer_rad)
            delta_yaw_future = (v_k / WHEELBASE) * math.tan(current_physical_steer) * dt_act
            yaw_future_mid = 0.5 * delta_yaw_future
            dx_future = -v_k * math.sin(yaw_future_mid) * dt_act
            dy_future =  v_k * math.cos(yaw_future_mid) * dt_act

            # 3. Composite 3-step projected anchor pose in F_sens
            delta_yaw_3 = wrap_to_pi(delta_yaw_past + delta_yaw_future)
            cos_dp = math.cos(delta_yaw_past)
            sin_dp = math.sin(delta_yaw_past)
            dx_3 = dx_past + (cos_dp * dx_future - sin_dp * dy_future)
            dy_3 = dy_past + (sin_dp * dx_future + cos_dp * dy_future)

            # 4. Projected actuator state (speed v_3 and steering servo angle delta_3)
            v_3 = v_k
            steer_cmd = float(self.cached_target_steer)
            max_steer_delta = SERVO_MAX_SPEED_RAD_S * dt_act
            delta_steer = max(-max_steer_delta, min(max_steer_delta, steer_cmd - current_physical_steer))
            steer_3 = max(-MAX_STEERING_RAD, min(MAX_STEERING_RAD, current_physical_steer + delta_steer))

            self.proj_anchor = (dx_3, dy_3, delta_yaw_3)
            self.proj_v = v_3
            self.proj_steer = steer_3

            # Scout Forward Gaps using projected anchor pose in F_sens
            t_scout_0 = time.perf_counter()
            xg, yg, width, theta_g = self.gap_scout.scout_gap(
                tof_readings=tof_readings,
                hazard_mask=enlarged_hazard_mask,
                costmap_obj=self.costmap,
                target_lookahead_m=self.target_lookahead_m,
                steer_current=steer_3,
                prob_map=prob_map,
                anchor_x=dx_3,
                anchor_y=dy_3,
                anchor_yaw=delta_yaw_3,
            )
            self.cached_scout_ms = (time.perf_counter() - t_scout_0) * 1000.0
            self.last_gap_info = (xg, yg, width, theta_g)

            # Curvilinear SL-MPC Trajectory Optimization anchored at projected state
            t_plan_0 = time.perf_counter()
            plan_res = self.planner.plan(
                xg=xg,
                yg=yg,
                theta_g=theta_g,
                aperture_width=width,
                v_current=v_3,
                steer_current=steer_3,
                yaw_rate_current=self.cached_yaw_rate,
                extra_targets=getattr(self.gap_scout, "last_targets", None),
                x0=dx_3,
                y0=dy_3,
                psi0=delta_yaw_3,
            )
            self.cached_plan_ms = (time.perf_counter() - t_plan_0) * 1000.0
            self.cached_plan_cpu_ms = plan_res.get("opt_cpu_ms", 0.0)
            self.cached_opt_ms = plan_res.get("opt_ms", self.cached_plan_ms)
            self.cached_opt_prim_ms = plan_res.get("opt_prim_ms", 0.0)
            self.cached_opt_alt_ms = plan_res.get("opt_alt_ms", 0.0)
            self.cached_opt_shield_ms = plan_res.get("opt_shield_ms", 0.0)
            self.cached_osqp_iters = plan_res.get("osqp_iters", 0)
            self.cached_osqp_iters_alt = plan_res.get("osqp_iters_alt", 0)
            self.last_plan_res = plan_res

            raw_target_steer = float(plan_res.get("cmd_steer_rad", 0.0))
            raw_target_speed = float(plan_res.get("cmd_speed_mps", self.v_cruise_min))

            min_fwd_dist = min(
                tof_readings.get("tof_front_left_4.1deg", 1.0),
                tof_readings.get("tof_front_right_4.1deg", 1.0),
            ) if tof_readings else 1.0

            # Candidate Gap Scout Targets for Early Exit Evaluation (Only when front is open!)
            candidate_gaps = []
            if min_fwd_dist >= 0.50:
                for tgt in (getattr(self.gap_scout, "last_targets", []) or []):
                    if isinstance(tgt, (list, tuple)) and len(tgt) >= 2:
                        candidate_gaps.append((float(tgt[0]), float(tgt[1])))

            # Anti-Wrong-Way Watchdog (only if enabled)
            if getattr(self.watchdog, "mode", "DISABLED") != "DISABLED":
                gx_curr, gy_curr, gyaw_curr = self.get_ground_truth_pose()
                aww_status = self.watchdog.update(
                    x=gx_curr,
                    y=gy_curr,
                    yaw=gyaw_curr,
                    v_est=self.v_est,
                    yaw_rate=self.cached_yaw_rate,
                    cached_tofs=tof_readings,
                    now_s=sim_time,
                    hazard_mask=enlarged_hazard_mask,
                    state=self.recovery_manager.state,
                    candidate_gaps=candidate_gaps,
                )
                self.last_aww_status = aww_status

                # Early Gap Exit: abort U-turn ONLY if in CORRECTIVE_180 with open front (never cancel PIVOT_REVERSE!)
                if (
                    aww_status.early_exit_found
                    and self.recovery_manager.state == StuckRecoveryState.CORRECTIVE_180
                    and getattr(self.recovery_manager, "corrective_substate", "") == "FORWARD"
                    and min_fwd_dist >= 0.80
                ):
                    self.recovery_manager.finish_recovery_early()

                # Trigger CORRECTIVE_180 if in NORMAL_DRIVE and wrong-way is confirmed.
                # Chooses to do the 180 towards the more open side (costmap escape arcs + ToFs).
                # Supported by reverse logic if front clearance is tight (< 0.22m) or obstacle hit.
                if (
                    aww_status.is_wrong_way
                    and self.recovery_manager.state == StuckRecoveryState.NORMAL_DRIVE
                ):
                    tgt_yaw = aww_status.target_recovery_yaw if aww_status.target_recovery_yaw is not None else wrap_to_pi(gyaw_curr + math.pi)
                    yaw_diff = wrap_to_pi(tgt_yaw - gyaw_curr)
                    arc_cl = self.recovery_manager.evaluate_costmap_escape_arcs(hazard_mask, self.costmap)
                    is_l_pin, is_r_pin = self.recovery_manager.evaluate_pinned_state(hazard_mask, self.costmap, tof_readings)
                    open_side = self.recovery_manager.resolve_open_side(
                        arc_clearance=arc_cl,
                        cached_tofs=tof_readings,
                        heading_err=yaw_diff,
                        is_left_pinned=is_l_pin,
                        is_right_pinned=is_r_pin,
                    )
                    min_fwd_clearance = min_fwd_dist
                    self.recovery_manager.trigger_corrective_180(
                        target_yaw=tgt_yaw,
                        steer_sign=open_side,
                        current_yaw=gyaw_curr,
                        arc_clearance=arc_cl,
                        cached_tofs=tof_readings,
                        front_clearance_m=min_fwd_clearance,
                    )
            else:
                gx_curr, gy_curr, gyaw_curr = self.get_ground_truth_pose()
                self.last_aww_status = None

            # Shield Reverse or Physical Wall Contact / Stall Trigger:
            # 1) All 5-step evasion arcs blocked (reverse_suggested == True)
            # 2) Physical front bumper wall collision (min_fwd_dist <= 0.05m)
            # 3) Front obstructed (< 0.20m) while stalled against obstacle (v_est < 0.35 m/s while commanding forward)
            _shield_cur = plan_res.get("shield", {})
            is_stall = (min_fwd_dist < 0.20 and self.v_est < 0.35 and raw_target_speed > 0.5)
            is_wall_contact = (min_fwd_dist <= 0.05)
            shield_reverse = bool(_shield_cur.get("reverse_suggested", False))
            should_reverse = (shield_reverse or is_wall_contact or is_stall)

            if (
                should_reverse
                and self.recovery_manager.state == StuckRecoveryState.NORMAL_DRIVE
            ):
                _arc_cl = self.recovery_manager.evaluate_costmap_escape_arcs(hazard_mask, self.costmap)
                _is_l_pin, _is_r_pin = self.recovery_manager.evaluate_pinned_state(hazard_mask, self.costmap, tof_readings)
                _side = self.recovery_manager.resolve_escape_steer_dir(
                    arc_clearance=_arc_cl, heading_err=float(theta_g),
                    is_left_pinned=_is_l_pin, is_right_pinned=_is_r_pin,
                )
                _target_bc_yaw = self.last_aww_status.target_recovery_yaw if self.last_aww_status else None
                self.recovery_manager.trigger_emergency_recovery(
                    heading_err=float(theta_g),
                    steer_dir=_side,
                    current_v=self.v_est,
                    current_steer=current_physical_steer,
                    instant_reverse=True,
                    target_breadcrumb_yaw=_target_bc_yaw,
                    current_yaw=gyaw_curr,
                    front_clearance_m=min_fwd_dist,
                    hazard_mask=hazard_mask,
                    costmap_obj=self.costmap,
                    cached_tofs=tof_readings,
                    is_left_pinned=_is_l_pin,
                    is_right_pinned=_is_r_pin,
                )

            # Recovery Manager at 60 Hz (Kinematic Escape Arc Costmap Evaluation & CORRECTIVE_180)
            target_breadcrumb_yaw = self.last_aww_status.target_recovery_yaw if self.last_aww_status else None
            is_early_exit = bool(
                self.last_aww_status
                and self.last_aww_status.early_exit_found
                and self.recovery_manager.state == StuckRecoveryState.CORRECTIVE_180
                and min_fwd_dist >= 0.80
            )
            is_recovering, rec_v, rec_steer = self.recovery_manager.update(
                dt=DT_PLANNER,
                current_v=self.v_est,
                commanded_v=raw_target_speed,
                cached_tofs=tof_readings,
                heading_err=float(theta_g),
                active_steer=current_physical_steer,
                hazard_mask=hazard_mask,
                costmap_obj=self.costmap,
                current_yaw=gyaw_curr,
                target_breadcrumb_yaw=target_breadcrumb_yaw,
                early_exit_available=is_early_exit,
            )

            if override_speed is not None:
                target_speed = float(override_speed)
                target_steer = float(override_steer) if override_steer is not None else 0.0
                self.active_steer = target_steer
                self.mode_desc = "OVERRIDE_TEST"
            elif is_recovering:
                if not self.was_recovering:
                    self.rate_limited_speed = min(self.rate_limited_speed, self.v_est)
                target_speed = rec_v
                target_steer = rec_steer
                self.active_steer = rec_steer
                rec_state = getattr(self.recovery_manager.state, 'name', self.recovery_manager.state)
                if rec_state == "CORRECTIVE_180":
                    sub = getattr(self.recovery_manager, "corrective_substate", "FORWARD")
                    self.mode_desc = f"PIVOT_180_{sub}"
                elif rec_state == "PIVOT_REVERSE":
                    self.mode_desc = "PIVOT_REVERSE"
                elif rec_state == "DECEL_AND_CENTER":
                    self.mode_desc = "DECEL_CENTER"
                else:
                    self.mode_desc = f"ESCAPE_{rec_state}"
                self.was_recovering = True
                if hasattr(self.planner, "reset_active_target"):
                    self.planner.reset_active_target()
            else:
                if self.was_recovering:
                    self.was_recovering = False
                    self.active_steer = current_physical_steer
                    self.rate_limited_speed = self.v_est

                # Fast Straight-Line Lock (only engage when forward is clear AND chassis is not sliding sideways)
                d_fl_lock = tof_readings.get("tof_front_left_4.1deg", tof_readings.get("tof_fl_3", 0.0))
                d_fr_lock = tof_readings.get("tof_front_right_4.1deg", tof_readings.get("tof_fr_3", 0.0))
                is_straight_locked = (
                    d_fl_lock >= 1.79 and
                    d_fr_lock >= 1.79
                )

                if is_straight_locked:
                    target_speed = self.straight_lock_speed
                    target_steer = 0.0
                    self.active_steer = 0.0
                    self.mode_desc = "STRAIGHT_LOCK"
                else:
                    target_speed = raw_target_speed
                    target_steer = raw_target_steer

                    is_shield_active = bool(_shield_cur.get("active", False))
                    if is_shield_active:
                        # Direct emergency evasive steering without progressive rate lag or deadbands
                        self.active_steer = max(
                            -MAX_STEERING_RAD,
                            min(MAX_STEERING_RAD, target_steer),
                        )
                        c_name = _shield_cur.get("chosen_name", "ACTIVE").replace("_", " ").upper()
                        self.mode_desc = f"SHIELD_{c_name}"
                    else:
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
                        self.active_steer = max(
                            -MAX_STEERING_RAD,
                            min(MAX_STEERING_RAD, self.active_steer + steer_aggressiveness * steer_delta),
                        )
                        self.mode_desc = "SL_MPC"

            self.cached_target_speed = target_speed
            self.cached_target_steer = self.active_steer

        # 4. Drift, Traction & Stability Control Execution
        rear_flow = self.get_ground_truth_rear_axle_flow()
        live_gyro_z = float(self.gyro.getValues()[2]) if self.gyro else self.cached_yaw_rate

        drift_res = self.drift_controller.update(
            v_forward=float(rear_flow["v_forward"]),
            beta_rear=float(rear_flow["slip_angle_rad"]),
            gyro_z=live_gyro_z,
            target_steer_rad=self.cached_target_steer,
            base_mu=self.base_mu,
            dt=dt,
        )

        # Couple Dynamic MU to Upstream SL-MPC Planner & Low-Level Harness
        self.mu_val = float(drift_res["effective_mu"])
        if hasattr(self.planner, "set_dynamic_mu"):
            self.planner.set_dynamic_mu(self.mu_val)

        # 5. Friction Rate Limiter Aware of Fused Speed with Front Lateral Grip Reservation
        if override_speed is not None:
            self.rate_limited_speed = float(override_speed)
        else:
            # Synchronize rate limiter with actual fused speed (self.v_est) to prevent
            # driving forward into obstacles or dragging brakes after dynamic speed loss
            target_v = self.cached_target_speed
            v_curr = self.v_est

            # When requesting deceleration or reverse, command must not stay higher than actual speed
            if target_v <= v_curr:
                if self.rate_limited_speed > v_curr:
                    self.rate_limited_speed = v_curr
            # When requesting acceleration forward, command must not stay lower than actual speed
            elif target_v >= v_curr:
                if self.rate_limited_speed < v_curr:
                    self.rate_limited_speed = v_curr

            speed_err = target_v - self.rate_limited_speed
            is_accel = (self.rate_limited_speed >= 0.0 and speed_err > 0.0) or (self.rate_limited_speed <= 0.0 and speed_err < 0.0)
            if is_accel:
                max_dv = (0.85 * self.mu_val * 9.81) * dt
            else:
                # Friction circle reserve on front steering axle:
                # Caps forward braking decel during high steering angles to prevent front tire wash-out
                steer_mag = abs(float(drift_res["steer_cmd"]))
                lateral_fraction = min(0.80, abs(math.sin(steer_mag)))
                max_longitudinal_fraction = math.sqrt(max(FRONT_LATERAL_RESERVE_MIN ** 2, 1.0 - lateral_fraction ** 2))
                max_dv = (1.00 * self.mu_val * 9.81 * max_longitudinal_fraction) * dt

            if abs(speed_err) <= max_dv:
                self.rate_limited_speed = target_v
            else:
                self.rate_limited_speed += math.copysign(max_dv, speed_err)

            # Traction slip envelope: Cap commanded wheel velocity relative to fused ground speed
            # to strictly prevent runaway wheelspin if tires break traction or car is pinned against obstacle.
            max_slip_lead = max(0.40, 0.20 * abs(v_curr))
            if self.rate_limited_speed > 0.0:
                self.rate_limited_speed = min(self.rate_limited_speed, max(0.0, v_curr) + max_slip_lead)
            elif self.rate_limited_speed < 0.0:
                self.rate_limited_speed = max(self.rate_limited_speed, min(0.0, v_curr) - max_slip_lead)

        # 6. Electronic Differential & Ground-Roll Clamped Actuation
        is_drift_active = (drift_res.get("mode", "GRIP") != "GRIP") or bool(drift_res.get("is_stabilizing", False))
        diff_res = self.ediff.update(
            target_v=self.rate_limited_speed,
            target_steer_rad=drift_res["steer_cmd"],
            gyro_z_rad_s=live_gyro_z,
            dt=dt,
            rear_alpha=drift_res["rear_alpha"],
            v_ground_RL=drift_res["v_ground_RL"],
            v_ground_RR=drift_res["v_ground_RR"],
            is_drifting=is_drift_active,
        )

        # 7. Apply Actuator Commands
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

        # 8. Assemble Telemetry Payload (Full for GUI / Lean for headless GA)
        gx, gy, gyaw = self.get_ground_truth_pose()
        v_real = self.get_ground_truth_velocity()

        aww_detailed = self.watchdog.get_detailed_telemetry(
            x=gx,
            y=gy,
            yaw=gyaw,
            v_est=self.v_est,
            yaw_rate=self.cached_yaw_rate,
            now_s=sim_time,
            hazard_mask=self.last_enlarged_hazard,
        )
        aww_detailed["corrective_substate"] = getattr(self.recovery_manager, "corrective_substate", "FORWARD")
        # Compose combined operational mode string (e.g. "SL_MPC", "SL_MPC_DRIFT")
        drift_mode = drift_res.get("mode", "GRIP")
        active_mode = f"{self.mode_desc}_{drift_mode}" if drift_mode not in ("GRIP", "DISABLED") else self.mode_desc

        v_plan = round(float(self.cached_target_speed), 3)
        v_cmd_act = round(float(self.rate_limited_speed), 3)
        steer_plan = round(math.degrees(float(self.cached_target_steer)), 2)
        steer_cmd_act = round(math.degrees(float(drift_res["steer_cmd"])), 2)

        if not full_telemetry:
            telemetry = {
                "time_step": self.step_counter,
                "sim_time": round(float(sim_time), 3),
                "mode": active_mode,
                "planner": "SL_MPC",
                "status": self.last_plan_res.get("status", "OK"),
                "v_est": round(float(self.v_est), 3),
                "v_fused": round(float(self.v_est), 3),
                "v_real": round(float(v_real), 3),
                "flow_rear_axle": rear_flow,
                "yaw_fused": round(float(self.yaw_est), 4),
                "v_plan": v_plan,
                "v_tgt": v_plan,
                "v_cmd": v_cmd_act,
                "steer_plan_deg": steer_plan,
                "steer_cmd_deg": steer_cmd_act,
                "steer_deg": steer_cmd_act,
                "steer_rad": round(float(drift_res["steer_cmd"]), 4),
                "steer_actual_deg": round(math.degrees(float(getattr(self.ediff, "current_steer_rad", self.active_steer))), 2),
                "pose": [float(gx), float(gy), float(gyaw)],
                "proj_anchor": [round(float(self.proj_anchor[0]), 4), round(float(self.proj_anchor[1]), 4), round(float(self.proj_anchor[2]), 4)],
                "proj_v": round(float(self.proj_v), 3),
                "proj_steer_deg": round(math.degrees(float(self.proj_steer)), 2),
                "plan_ms": round(float(self.cached_plan_ms), 2),
                "plan_cpu_ms": round(float(self.cached_plan_cpu_ms), 2),
                "map_ms": round(float(self.cached_map_ms), 2),
                "map_perf": getattr(self.costmap, "last_perf", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
                "scout_ms": round(float(self.cached_scout_ms), 2),
                "obs_ms": round(float(self.cached_obs_ms), 2),
                "opt_ms": round(float(self.cached_opt_ms), 2),
                "opt_prim_ms": round(float(self.cached_opt_prim_ms), 2),
                "opt_alt_ms": round(float(self.cached_opt_alt_ms), 2),
                "opt_shield_ms": round(float(self.cached_opt_shield_ms), 2),
                "iters_prim": int(self.cached_osqp_iters),
                "iters_alt": int(self.cached_osqp_iters_alt),
                "telem_ms": 0.0,
                "step_ms": round((time.perf_counter() - t_step_start) * 1000.0, 2),
                "step_cpu_ms": round((time.process_time() - t_step_cpu_start) * 1000.0, 2),
                "gc_ms": round(float(_gc_stats["step_gc_ms"]), 2),
                "escape_state": str(getattr(self.recovery_manager.state, "name", self.recovery_manager.state)),
                "watchdog_mode": str(getattr(self.watchdog.mode, "name", self.watchdog.mode)),
                "f_tof": round(float(self.cached_f_tof), 2),
                "aww": aww_detailed,
                "drift": drift_res,
                "rear_alpha": drift_res["rear_alpha"],
                "effective_mu": drift_res["effective_mu"],
            }
            return telemetry, planner_ran

        t_telem_0 = time.perf_counter()

        # 2x2 Max Pooling for Costmap Telemetry:
        # Prevents small 1-2 cell obstacle hits from being skipped/aliased during decimation,
        # ensuring the red obstacle core is always rendered within its C-space halo.
        pm = self.costmap.prob_map
        h_even = (pm.shape[0] // 2) * 2
        w_even = (pm.shape[1] // 2) * 2
        pm_even = pm[:h_even, :w_even]
        prob_max = np.maximum.reduce([
            pm_even[0::2, 0::2],
            pm_even[0::2, 1::2],
            pm_even[1::2, 0::2],
            pm_even[1::2, 1::2]
        ])
        prob_down = (prob_max * 255.0).astype(np.uint8)

        active_hazard = self.last_enlarged_hazard if self.last_enlarged_hazard is not None else self.costmap.hazard_mask
        haz_even = active_hazard[:h_even, :w_even]
        haz_max = np.maximum.reduce([
            haz_even[0::2, 0::2],
            haz_even[0::2, 1::2],
            haz_even[1::2, 0::2],
            haz_even[1::2, 1::2]
        ])
        hazard_down = (haz_max > 0).astype(np.uint8)

        grid_compressed = base64.b64encode(zlib.compress(prob_down.tobytes(), 1)).decode("ascii")
        hazard_compressed = base64.b64encode(zlib.compress(hazard_down.tobytes(), 1)).decode("ascii")

        b_line = self.last_plan_res.get("best_line", [])
        c_lines = self.last_plan_res.get("candidate_lines", [])[:20]
        xg, yg, width, theta_g = self.last_gap_info
        g_gap = [round(float(xg), 3), round(float(yg), 3), round(float(width), 3)]
        g_targets = [
            [round(float(tgt[0]), 3), round(float(tgt[1]), 3), round(float(tgt[2]), 3)]
            for tgt in (getattr(self.gap_scout, "last_targets", []) or [])
            if isinstance(tgt, (list, tuple)) and len(tgt) >= 3
        ]

        telem_ms = (time.perf_counter() - t_telem_0) * 1000.0
        self.cached_telem_ms = telem_ms
        step_ms = (time.perf_counter() - t_step_start) * 1000.0
        step_cpu_ms = (time.process_time() - t_step_cpu_start) * 1000.0
        gc_ms = _gc_stats["step_gc_ms"]
        self.cached_step_ms = step_ms
        self.cached_step_cpu_ms = step_cpu_ms
        self.cached_gc_ms = gc_ms

        telemetry = {
            "time_step": self.step_counter,
            "sim_time": round(float(sim_time), 3),
            "mode": active_mode,
            "planner": "SL_MPC",
            "status": self.last_plan_res.get("status", "OK"),
            "v_est": round(float(self.v_est), 3),
            "v_fused": round(float(self.v_est), 3),
            "v_real": round(float(v_real), 3),
            "flow_rear_axle": rear_flow,
            "drift": drift_res,
            "rear_alpha": drift_res["rear_alpha"],
            "effective_mu": drift_res["effective_mu"],
            "yaw_fused": round(float(self.yaw_est), 4),
            "v_plan": v_plan,
            "v_tgt": v_plan,
            "v_cmd": v_cmd_act,
            "steer_plan_deg": steer_plan,
            "steer_cmd_deg": steer_cmd_act,
            "steer_deg": steer_cmd_act,
            "steer_rad": round(float(drift_res["steer_cmd"]), 4),
            "steer_actual_deg": round(math.degrees(float(getattr(self.ediff, "current_steer_rad", self.active_steer))), 2),
            "pose": [float(gx), float(gy), float(gyaw)],
            "proj_anchor": [round(float(self.proj_anchor[0]), 4), round(float(self.proj_anchor[1]), 4), round(float(self.proj_anchor[2]), 4)],
            "proj_v": round(float(self.proj_v), 3),
            "proj_steer_deg": round(math.degrees(float(self.proj_steer)), 2),
            "f_tof": round(float(min(tof_readings.get("tof_front_left_4.1deg", 1.0), tof_readings.get("tof_front_right_4.1deg", 1.0))), 3) if tof_readings else 1.0,
            "gap": g_gap,
            "targets": g_targets,
            "plan_ms": round(float(self.cached_plan_ms), 2),
            "plan_cpu_ms": round(float(self.cached_plan_cpu_ms), 2),
            "map_ms": round(float(self.cached_map_ms), 2),
            "map_perf": getattr(self.costmap, "last_perf", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            "scout_ms": round(float(self.cached_scout_ms), 2),
            "obs_ms": round(float(self.cached_obs_ms), 2),
            "opt_ms": round(float(self.cached_opt_ms), 2),
            "opt_prim_ms": round(float(self.cached_opt_prim_ms), 2),
            "opt_alt_ms": round(float(self.cached_opt_alt_ms), 2),
            "opt_shield_ms": round(float(self.cached_opt_shield_ms), 2),
            "iters_prim": int(self.cached_osqp_iters),
            "iters_alt": int(self.cached_osqp_iters_alt),
            "telem_ms": round(float(telem_ms), 2),
            "step_ms": round(float(step_ms), 2),
            "step_cpu_ms": round(float(step_cpu_ms), 2),
            "gc_ms": round(float(gc_ms), 2),
            "surviving": int(self.last_plan_res.get("surviving_count", 0)),
            "candidates": int(self.last_plan_res.get("candidate_count", len(c_lines))),
            "tof": {k: round(float(v), 3) for k, v in tof_readings.items()},
            "tof_ranges": [round(float(tof_readings.get(k, TOF_MAX_RANGE_M)), 3) for k in TOF_SENSOR_NAMES],
            "best_line": b_line,
            "candidate_lines": c_lines,
            "guide_line": self.last_plan_res.get("guide_line", []),
            "corridor_left": self.last_plan_res.get("corridor_left", []),
            "corridor_right": self.last_plan_res.get("corridor_right", []),
            "shield": self.last_plan_res.get("shield", {}),
            "f_tof": round(float(self.cached_f_tof), 2),
            "grid_shape": [int(s) for s in prob_down.shape],
            "grid_bounds": [float(GRID_X_MIN), float(GRID_X_MAX), float(GRID_Y_MIN), float(GRID_Y_MAX)],
            "grid_data": grid_compressed,
            "hazard_data": hazard_compressed,
            "dynamic_obstacles": [
                {
                    "id": t.track_id,
                    "x_r": round(float(t.x_r), 3),
                    "y_r": round(float(t.y_r), 3),
                    "v_mag": round(float(t.v_mag), 3),
                    "v_rx": round(float(t.v_rx), 3),
                    "v_ry": round(float(t.v_ry), 3),
                    "v_longitudinal": round(float(getattr(t, "v_longitudinal", 0.0)), 3),
                    "state": str(getattr(t, "state", "STATIC")),
                    "is_dynamic": bool(t.is_dynamic),
                }
                for t in getattr(self, "last_confirmed_tracks", getattr(self, "last_dynamic_tracks", []))
            ],
            "obs_telem": getattr(self, "last_obs_telem", {}),
            "escape_state": str(getattr(self.recovery_manager.state, "name", self.recovery_manager.state)),
            "watchdog_mode": str(getattr(self.watchdog.mode, "name", self.watchdog.mode)),
            "aww": aww_detailed,
            "plan": self.last_plan_res,
        }

        return telemetry, planner_ran

    def step_planner(
        self,
        sim_time: float = 0.0,
        override_speed: Optional[float] = None,
        override_steer: Optional[float] = None,
    ) -> Tuple[Dict[str, Any], bool]:
        """Convenience test wrapper for FolkraceVehicle.step()."""
        return self.step(
            dt=DT,
            full_telemetry=True,
            override_speed=override_speed,
            override_steer=override_steer,
        )
