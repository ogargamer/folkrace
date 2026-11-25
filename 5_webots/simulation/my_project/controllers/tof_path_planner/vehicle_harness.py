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
REPO_ROOT = os.path.abspath(os.path.join(CURR_DIR, "..", "..", "..", "..", ".."))

for p in [CURR_DIR, CONTROLLERS_DIR, MY_PROJECT_DIR, REPO_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from brain import AutonomousBrain
from electronic_diff_4wd import ElectronicDifferential4WD
from traction_drift_controller import TractionDriftController
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from imu_encoder_fusion import IMUEncoderFusion
from tof_gap_scout import ToFGapScout
from planner_sl_mpc import CurvilinearSLMPC
from stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState
from anti_wrong_way_watchdog import AntiWrongWayWatchdog
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
    RECOVERY_CLEARANCE_EXIT_M,
    RECOVERY_ANY_TOF_EXIT_M,
    RECOVERY_ARC_LOOKAHEAD_M,
    ACCEL_FWD_MAX,
    BRAKE_DECEL_MAX,
    ACCEL_FWD_RATIO,
    BRAKE_DECEL_RATIO,
    GRAVITY_G,
    SERVO_MAX_SPEED_RAD_S,
    ENABLE_OPTICAL_FLOW_SENSOR,
    ENABLE_TRACTION_DRIFT_CONTROL,
    ENABLE_RAMP_DETECTION,
    RAMP_SPEED_LIMIT_MPS,
    RAMP_ENTER_PITCH_DEG,
    RAMP_ENTER_TILT_DEG,
    RAMP_EXIT_PITCH_DEG,
    RAMP_EXIT_TILT_DEG,
    RAMP_HOLD_TIME_S,
)
from ramp_detector import RampDetector


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
            best = TOF_MAX_RANGE_M
            for v in r_img:
                if 0.0 < v < best:
                    best = v
            val = float(best)
    elif hasattr(dev, "getValue"):
        raw = dev.getValue()
        if not math.isinf(raw) and not math.isnan(raw):
            val = float(raw)
    val = max(TOF_MIN_RANGE_M, min(TOF_MAX_RANGE_M, val))
    return val


read_tof_device = read_tof_multi_ray

# 45-degree 4-zone VL53L8CX bank mapping (Webots getRangeImage orders rays left-to-right: +FOV/2 -> -FOV/2)
TOF_BANK_LAYOUT: Tuple[Tuple[str, Tuple[str, ...]], ...] = (
    ("tof_front_left_inner",  ("tof_front_left_4.1deg",  "tof_front_left_15.4deg", "tof_front_left_26.6deg", "tof_front_left_37.9deg")),
    ("tof_front_left_outer",  ("tof_front_left_49.1deg", "tof_front_left_60.4deg", "tof_front_left_71.6deg", "tof_front_left_82.9deg")),
    ("tof_front_right_inner", ("tof_front_right_37.9deg", "tof_front_right_26.6deg", "tof_front_right_15.4deg", "tof_front_right_4.1deg")),
    ("tof_front_right_outer", ("tof_front_right_82.9deg", "tof_front_right_71.6deg", "tof_front_right_60.4deg", "tof_front_right_49.1deg")),
)


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

    def __init__(
        self,
        robot: Any,
        is_supervisor: bool = False,
        genes: Optional[Dict[str, Any]] = None,
        planner_decimation: int = PLANNER_DECIMATION,
        planner_phase: int = 0,
        defer_tof_enable: bool = False,
        disable_alt_solver: bool = False,
    ):
        self.robot = robot
        self.is_supervisor = is_supervisor
        self.planner_decimation = max(1, int(planner_decimation))
        self.planner_phase = int(planner_phase) % self.planner_decimation
        self._global_step_counter = self.planner_phase
        self._tof_step_ms_int = max(1, int(round(TIME_STEP_MS)) * self.planner_decimation)
        self._tofs_enabled = False
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
        self.motor_max_vel: Dict[str, float] = {}
        for k, m in self.motors.items():
            if m:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
                try:
                    self.motor_max_vel[k] = float(m.getMaxVelocity())
                except Exception:
                    self.motor_max_vel[k] = 139.64

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
        self.tof_banks: List[Tuple[Any, Tuple[str, ...]]] = []
        step_ms_int = int(round(TIME_STEP_MS))

        bank_devs = [(bank_name, robot.getDevice(bank_name), zone_names) for bank_name, zone_names in TOF_BANK_LAYOUT]
        if any(dev is not None for _, dev, _ in bank_devs):
            for bank_name, dev, zone_names in bank_devs:
                self.tof_banks.append((dev, zone_names))
                self.tofs[bank_name] = dev
                self.tof_devices.append(dev)
        else:
            for name in TOF_SENSOR_NAMES:
                dev = robot.getDevice(name)
                if dev is None:
                    short_name = name.replace("tof_front_", "tof_")
                    dev = robot.getDevice(short_name)
                if dev is not None:
                    self.tofs[name] = dev
                    self.tof_devices.append(dev)
                else:
                    self.tofs[name] = None
                    self.tof_devices.append(None)

        if not defer_tof_enable:
            self.enable_tof_sensors()

        self.imu_dev = (
            robot.getDevice("bno08x_imu")
            or robot.getDevice("imu")
            or robot.getDevice("inertial unit")
        )
        if self.imu_dev:
            self.imu_dev.enable(step_ms_int)

        self.gyro = (
            robot.getDevice("gyro")
            or robot.getDevice("bno08x_gyro")
        )
        if self.gyro:
            self.gyro.enable(step_ms_int)

        self.accel = (
            robot.getDevice("accelerometer")
            or robot.getDevice("bno08x_accel")
        )
        if self.accel:
            self.accel.enable(step_ms_int)

        self.encoders = {}
        for wheel, dev_name in ENCODER_NAMES.items():
            enc = robot.getDevice(dev_name)
            if enc:
                enc.enable(step_ms_int)
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

        # 3. Core Perception & Autonomous Trajectory Planner (Shared Brain)
        self.brain = AutonomousBrain(genes=genes, disable_alt_solver=disable_alt_solver)

        # Expose sub-modules for backwards compatibility & telemetry
        self.costmap = self.brain.costmap
        self.gap_scout = self.brain.gap_scout
        self.planner = self.brain.planner
        self.recovery_manager = self.brain.recovery_manager
        self.watchdog = self.brain.watchdog
        self.pose_tracker = self.brain.pose_tracker

        self.ediff = ElectronicDifferential4WD(
            wheelbase=WHEELBASE,
            track_rear=TRACK_REAR,
            track_front=TRACK_FRONT,
            track_kingpin=TRACK_KINGPIN,
            wheel_radius=WHEEL_RADIUS,
            max_steer=MAX_STEERING_RAD,
            max_servo_angle=MAX_SERVO_RAD,
            ackermann_percent=ACKERMANN_PERCENT,
            # Webots only_front_tofs.proto wheels rotate around +X (axis 1 0 0),
            # where negative omega drives +Y forward (matching imu_encoder_fusion.py).
            motor_dir=-1.0,
            v_max=V_MAX_WHEEL,
        )
        self.drift_controller = TractionDriftController(
            wheelbase=WHEELBASE,
            track_rear=TRACK_REAR,
            max_steer=MAX_STEERING_RAD,
            enabled=ENABLE_TRACTION_DRIFT_CONTROL,
        )
        self.ramp_detector = RampDetector(
            enabled=ENABLE_RAMP_DETECTION,
            speed_limit_mps=RAMP_SPEED_LIMIT_MPS,
            enter_pitch_deg=RAMP_ENTER_PITCH_DEG,
            enter_tilt_deg=RAMP_ENTER_TILT_DEG,
            exit_pitch_deg=RAMP_EXIT_PITCH_DEG,
            exit_tilt_deg=RAMP_EXIT_TILT_DEG,
            hold_time_s=RAMP_HOLD_TIME_S,
        )

        # Default Tuning Parameters (inherited directly from AutonomousBrain / best_planner_genome.json)
        self.base_mu = float(self.brain.base_mu)
        self.mu_val = float(self.brain.mu_val)
        self.r_infl = float(self.brain.r_infl)
        self.target_lookahead_m = float(self.brain.target_lookahead_m)
        self.aggr_dist_far = float(self.brain.aggr_dist_far)
        self.aggr_dist_close = float(self.brain.aggr_dist_close)
        self.aggr_min_scale = float(self.brain.aggr_min_scale)
        self.steer_deadband = float(self.brain.steer_deadband)
        self.v_cruise_min = float(self.brain.v_cruise_min)
        self.v_cruise_max = float(self.brain.v_cruise_max)
        self._last_front_vtrq_avg = 0.0
        self._last_rear_vtrq_avg = 0.0

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

        # Hardware Latency Buffers (1-step 60 Hz ToF delay = 16.67 ms + 1-step 120 Hz actuator lag = 8.33 ms)
        initial_tofs = {name: TOF_MAX_RANGE_M for name in TOF_SENSOR_NAMES}
        tof_queue_steps = max(1, int(round(TOF_LATENCY_STEPS / max(1, self.planner_decimation))))
        self.tof_queue = LatencyQueue(steps=tof_queue_steps, default_val=initial_tofs)
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

        self.last_aww_status = None

        # Apply genes if provided
        if genes:
            self.configure(genes)

    def configure(self, genes: Dict[str, Any]) -> None:
        """Dynamically applies genome parameters to all vehicle subsystems."""
        if not genes:
            return

        self.brain.configure(genes)

        self.costmap = self.brain.costmap
        self.gap_scout = self.brain.gap_scout
        self.planner = self.brain.planner
        self.recovery_manager = self.brain.recovery_manager
        self.watchdog = self.brain.watchdog
        self.pose_tracker = self.brain.pose_tracker

        self.base_mu = self.brain.base_mu
        self.mu_val = self.brain.mu_val
        self.r_infl = self.brain.r_infl
        self.target_lookahead_m = self.brain.target_lookahead_m
        self.aggr_dist_far = self.brain.aggr_dist_far
        self.aggr_dist_close = self.brain.aggr_dist_close
        self.aggr_min_scale = self.brain.aggr_min_scale
        self.steer_deadband = self.brain.steer_deadband
        self.v_cruise_min = self.brain.v_cruise_min
        self.v_cruise_max = self.brain.v_cruise_max
        self.cached_target_speed = self.v_cruise_min

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

        self.motor_max_vel: Dict[str, float] = {}
        for k, m in self.motors.items():
            if m:
                m.setPosition(float("inf"))
                m.setVelocity(0.0)
                try:
                    self.motor_max_vel[k] = float(m.getMaxVelocity())
                except Exception:
                    self.motor_max_vel[k] = 139.64
        for s in self.servos.values():
            if s:
                s.setPosition(0.0)

        self.fusion.reset()
        start_x = float(start_pos[0]) if start_pos else 0.0
        start_y = float(start_pos[1]) if start_pos else 0.0
        self.brain.reset(start_pose=(start_x, start_y, 0.0))
        self.costmap = self.brain.costmap
        self.gap_scout = self.brain.gap_scout
        self.planner = self.brain.planner
        self.recovery_manager = self.brain.recovery_manager
        self.watchdog = self.brain.watchdog
        self.pose_tracker = self.brain.pose_tracker

        self.ediff.reset()
        self.drift_controller.reset()
        self.ramp_detector.reset()
        self.last_aww_status = None
        self.last_tofs = {name: TOF_MAX_RANGE_M for name in TOF_SENSOR_NAMES}
        self.cached_f_tof = TOF_MAX_RANGE_M
        self.last_gap_info = (0.0, 1.0, 0.50, 0.0)
        self.last_plan_res = {}
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
        self._last_front_vtrq_avg = 0.0
        self._last_rear_vtrq_avg = 0.0

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

    def enable_tof_sensors(self) -> None:
        """Enables ToF Lidar/DistanceSensor devices at the configured decimated period."""
        for dev in self.tof_devices:
            if dev is not None:
                dev.enable(self._tof_step_ms_int)
        self._tofs_enabled = True

    def read_tof_sensors(self) -> Dict[str, float]:
        """Reads all 16 ToF range zones (from 4-bank Lidars or 16 legacy devices) and returns a dict."""
        cached_tofs: Dict[str, float] = {}
        noise_pct = getattr(self, "tof_noise_pct", float(os.environ.get("TOF_NOISE_PCT", 0.0)))
        if self.tof_banks:
            for dev, zone_names in self.tof_banks:
                n_zones = len(zone_names)
                r_img = dev.getRangeImage() if (dev is not None and hasattr(dev, "getRangeImage")) else None
                if r_img and len(r_img) >= n_zones:
                    rays_per_zone = len(r_img) // n_zones
                    for z_idx, name in enumerate(zone_names):
                        start = z_idx * rays_per_zone
                        end = (z_idx + 1) * rays_per_zone if z_idx < n_zones - 1 else len(r_img)
                        best = TOF_MAX_RANGE_M
                        for v in r_img[start:end]:
                            if 0.0 < v < best:
                                best = v
                        val = max(TOF_MIN_RANGE_M, min(TOF_MAX_RANGE_M, float(best)))
                        if noise_pct > 0.0 and val < (TOF_MAX_RANGE_M - 0.01):
                            noise = float(np.random.uniform(-noise_pct, noise_pct)) * val
                            val = float(np.clip(val + noise, 0.02, TOF_MAX_RANGE_M))
                        cached_tofs[name] = val
                else:
                    for name in zone_names:
                        cached_tofs[name] = TOF_MAX_RANGE_M
        else:
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
        if ENABLE_OPTICAL_FLOW_SENSOR and self.is_supervisor and self.robot_node:
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
        build_telemetry: bool = True,
    ) -> Tuple[Dict[str, Any], bool]:
        """
        Executes one 120 Hz dynamics step with 60 Hz planner decimation.
        Returns: (telemetry_dict, planner_executed_boolean)
        """
        t_step_start = time.perf_counter()
        t_step_cpu_start = time.process_time()
        _gc_stats["step_gc_ms"] = 0.0
        self.step_counter += 1
        self._global_step_counter += 1
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
            if not build_telemetry:
                return {}, False
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
                "escape_state": "NORMAL",
                "f_tof": round(float(min(tof_readings.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M), tof_readings.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M))), 2),
                "plan": {},
            }
            return telemetry, False

        # 120 Hz IMU + 4-Wheel Encoder Sensor Fusion & Odometry Dead Reckoning
        active_steer = float(getattr(self.ediff, "current_steer_rad", self.active_steer))
        v_fused, yaw_rate, yaw_fused = self.fusion.update(
            dt=dt,
            steer_rad=active_steer,
            front_vtrq_avg=self._last_front_vtrq_avg,
            rear_vtrq_avg=self._last_rear_vtrq_avg,
        )
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
        is_planner_tick = ((self._global_step_counter - self.planner_phase - 2) % self.planner_decimation == 0)

        # Closed-loop Control Tick (60 Hz Pi/ToF with configurable decimation on 120 Hz ESP/physics)
        if is_planner_tick:
            planner_ran = True
            raw_tofs = self.read_tof_sensors()
            tof_readings = self.tof_queue.update(raw_tofs)
            self.last_tofs = tof_readings

        f_tof = min(
            tof_readings.get("tof_front_left_4.1deg", tof_readings.get("tof_fl_3", TOF_MAX_RANGE_M)),
            tof_readings.get("tof_front_right_4.1deg", tof_readings.get("tof_fr_3", TOF_MAX_RANGE_M)),
        )
        self.cached_f_tof = f_tof

        if is_planner_tick:
            current_physical_steer = float(getattr(self.ediff, "current_steer_rad", self.active_steer))
            target_speed, target_steer, brain_telem = self.brain.step(
                tof_readings=tof_readings,
                v_est=self.v_est,
                steer_actual_rad=current_physical_steer,
                gyro_z_rad_s=self.cached_yaw_rate,
                dt=dt * self.planner_decimation,
                sim_time_s=sim_time,
                ground_truth_pose=None,  # Zero ground truth: test watchdog on dead-reckoned odometry matching Pi 4
                full_telemetry=full_telemetry,
                pitch_rad=self.fusion.pitch,
                roll_rad=self.fusion.roll,
                imu_yaw_rad=(self.fusion.yaw if self.imu_dev is not None else None),
            )

            self.was_recovering = (self.recovery_manager.state != StuckRecoveryState.NORMAL_DRIVE)

            if override_speed is not None:
                target_speed = float(override_speed)
                target_steer = float(override_steer) if override_steer is not None else 0.0
                self.active_steer = target_steer
                self.cached_target_speed = target_speed
                self.cached_target_steer = target_steer
                self.mode_desc = "OVERRIDE_TEST"
            else:
                self.active_steer = target_steer
                self.cached_target_speed = target_speed
                self.cached_target_steer = target_steer
                self.mode_desc = brain_telem.get("mode", "SL_MPC")

            self.cached_plan_ms = brain_telem.get("plan_ms", 0.0)
            self.cached_plan_cpu_ms = self.brain.last_plan_res.get("opt_cpu_ms", 0.0)
            self.cached_map_ms = brain_telem.get("map_ms", 0.0)
            self.cached_scout_ms = brain_telem.get("scout_ms", 0.0)
            self.cached_opt_ms = self.brain.last_plan_res.get("opt_ms", self.cached_plan_ms)
            self.cached_opt_prim_ms = brain_telem.get("opt_prim_ms", 0.0)
            self.cached_opt_alt_ms = brain_telem.get("opt_alt_ms", 0.0)
            self.cached_opt_shield_ms = self.brain.last_plan_res.get("opt_shield_ms", 0.0)
            self.cached_osqp_iters = self.brain.last_plan_res.get("osqp_iters", 0)
            self.cached_osqp_iters_alt = self.brain.last_plan_res.get("osqp_iters_alt", 0)
            self.last_plan_res = self.brain.last_plan_res
            self.last_gap_info = self.brain.last_gap_info
            self.last_aww_status = self.brain.last_aww_status
            self.proj_anchor = self.brain.pose_tracker.proj_anchor
            self.proj_v = self.brain.pose_tracker.proj_v
            self.proj_steer = self.brain.pose_tracker.proj_steer

        # 4. Drift, Traction & Stability Control Execution
        if ENABLE_OPTICAL_FLOW_SENSOR or self.drift_controller.enabled or full_telemetry:
            rear_flow = self.get_ground_truth_rear_axle_flow()
            v_fwd_flow = float(rear_flow["v_forward"])
            beta_flow = float(rear_flow["slip_angle_rad"])
        else:
            rear_flow = None
            v_fwd_flow = self.v_est
            beta_flow = 0.0
        live_gyro_z = float(self.gyro.getValues()[2]) if self.gyro else self.cached_yaw_rate

        drift_res = self.drift_controller.update(
            v_forward=v_fwd_flow,
            beta_rear=beta_flow,
            gyro_z=live_gyro_z,
            target_steer_rad=self.cached_target_steer,
            base_mu=self.base_mu,
            dt=dt,
        )

        # Couple Dynamic MU to Upstream SL-MPC Planner & Low-Level Harness
        new_mu = float(drift_res["effective_mu"])
        if new_mu != self.mu_val:
            self.mu_val = new_mu
            if hasattr(self.planner, "set_dynamic_mu"):
                self.planner.set_dynamic_mu(self.mu_val)
            if hasattr(self.brain, "set_dynamic_mu"):
                self.brain.set_dynamic_mu(self.mu_val)

        # 4b. Ramp Detection & Speed Governance
        ramp_res = self.ramp_detector.update(
            pitch_rad=self.fusion.pitch,
            roll_rad=self.fusion.roll,
            dt=dt,
        )

        # Combined Steering Command (Planner + Drift Countersteer)
        effective_steer_cmd = max(
            -MAX_STEERING_RAD,
            min(MAX_STEERING_RAD, drift_res["steer_cmd"]),
        )

        # Rear Drive Alpha from Drift Traction Controller
        combined_alpha = drift_res["rear_alpha"]

        # 5-6. ESP32-Matched Option A1 Speed Envelope, Ramp Ceiling & 4WD Electronic Differential
        bypass_speed_envelope = (override_speed is not None)
        if override_speed is not None:
            cmd_target_v = float(override_speed)
        else:
            cmd_target_v = float(self.cached_target_speed)
            # Hardware Ramp Speed Governor (mirrors ESP32 main.cpp pitch >= 8 deg + brain hysteresis)
            if ramp_res["is_on_ramp"] or abs(self.fusion.pitch_deg) >= RAMP_ENTER_PITCH_DEG:
                ramp_cap = float(ramp_res["speed_limit"])
                if cmd_target_v > 0.0:
                    cmd_target_v = min(cmd_target_v, ramp_cap)
                elif cmd_target_v < 0.0:
                    cmd_target_v = max(cmd_target_v, -ramp_cap)

        is_drift_active = (drift_res.get("mode", "GRIP") != "GRIP") or bool(drift_res.get("is_stabilizing", False))
        diff_res = self.ediff.update(
            target_v=cmd_target_v,
            target_steer_rad=effective_steer_cmd,
            gyro_z_rad_s=live_gyro_z,
            dt=dt,
            rear_alpha=combined_alpha,
            v_ground_RL=drift_res["v_ground_RL"],
            v_ground_RR=drift_res["v_ground_RR"],
            is_drifting=is_drift_active,
            mu=self.mu_val,
            v_curr=self.v_est,
            bypass_speed_envelope=bypass_speed_envelope,
        )
        self.rate_limited_speed = float(diff_res["rate_limited_v"])
        self._last_front_vtrq_avg = 0.5 * (float(diff_res["vtrq_ff_FL"]) + float(diff_res["vtrq_ff_FR"]))
        self._last_rear_vtrq_avg = 0.5 * (float(diff_res["vtrq_ff_RL"]) + float(diff_res["vtrq_ff_RR"]))

        # 7. Apply Actuator Commands
        if self.servos.get("left"):
            self.servos["left"].setPosition(diff_res["steer_left"])
        if self.servos.get("right"):
            self.servos["right"].setPosition(diff_res["steer_right"])

        for k, omega_key in (("FL", "omega_FL"), ("FR", "omega_FR"), ("RL", "omega_RL"), ("RR", "omega_RR")):
            m = self.motors.get(k)
            if m:
                lim = self.motor_max_vel.get(k, 139.64)
                w_cmd = max(-lim, min(lim, float(diff_res[omega_key])))
                m.setVelocity(w_cmd)


        # 8. Assemble Telemetry Payload (Full for GUI / Lean for headless GA)
        if not build_telemetry:
            return {}, planner_ran
        if full_telemetry:
            gx, gy, gyaw = self.get_ground_truth_pose()
            v_real = self.get_ground_truth_velocity()

            aww_detailed = self.watchdog.get_detailed_telemetry(
                x=self.pose_tracker.x,
                y=self.pose_tracker.y,
                yaw=self.pose_tracker.yaw,
                v_est=self.v_est,
                yaw_rate=self.cached_yaw_rate,
                now_s=sim_time,
                hazard_mask=self.costmap.hazard_mask,
                costmap_obj=self.costmap,
                unknown_mask=self.costmap.unknown_mask,
            )
            aww_detailed["corrective_substate"] = getattr(self.recovery_manager, "corrective_substate", "FORWARD")
        else:
            gx, gy, gyaw = self.x_est, self.y_est, self.yaw_est
            v_real = self.v_est
            aww_detailed = None
        # Compose combined operational mode string (e.g. "SL_MPC", "SL_MPC_RAMP", "SL_MPC_DRIFT")
        drift_mode = drift_res.get("mode", "GRIP")
        if ramp_res["is_on_ramp"]:
            active_mode = f"{self.mode_desc}_RAMP"
        elif drift_mode not in ("GRIP", "DISABLED"):
            active_mode = f"{self.mode_desc}_{drift_mode}"
        else:
            active_mode = self.mode_desc

        v_plan = round(float(self.cached_target_speed), 3)
        v_cmd_act = round(float(self.rate_limited_speed), 3)
        steer_plan = round(math.degrees(float(self.cached_target_steer)), 2)
        steer_cmd_act = round(math.degrees(float(effective_steer_cmd)), 2)

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
                "roll_deg": round(float(self.fusion.roll_deg), 2),
                "pitch_deg": round(float(self.fusion.pitch_deg), 2),
                "roll_rate": round(float(self.fusion.roll_rate), 3),
                "v_plan": v_plan,
                "v_tgt": v_plan,
                "v_cmd": v_cmd_act,
                "steer_plan_deg": steer_plan,
                "steer_cmd_deg": steer_cmd_act,
                "steer_deg": steer_cmd_act,
                "steer_rad": round(float(effective_steer_cmd), 4),
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
                "recovery_state": str(self.recovery_manager.state),
                "is_wrong_way": bool(self.last_aww_status.is_wrong_way) if self.last_aww_status else False,
                "watchdog_mode": str(getattr(self.watchdog.mode, "name", self.watchdog.mode)),
                "f_tof": round(float(self.cached_f_tof), 2),
                "aww": aww_detailed,
                "drift": drift_res,
                "ramp": ramp_res,
                "on_ramp": bool(ramp_res["is_on_ramp"]),
                "ramp_speed_limit": ramp_res["speed_limit"],
                "arm_active": False,
                "tilt_detected": bool(ramp_res["is_on_ramp"]),
                "v_tilt_limit": ramp_res["speed_limit"] if ramp_res["is_on_ramp"] else None,
                "rear_alpha": combined_alpha,
                "effective_mu": drift_res["effective_mu"],
                "slip_state": diff_res["slip_state"],
                "yaw_error": round(float(diff_res["yaw_error"]), 3),
            }
            return telemetry, planner_ran

        t_telem_0 = time.perf_counter()

        # Full-fidelity 1cm costmap for laptop telemetry (no max-pooling distortion or pinched gaps)
        prob_u8 = (np.clip(self.costmap.prob_map, 0.0, 1.0) * 255.0).astype(np.uint8)
        hazard_u8 = (self.costmap.hazard_mask > 0).astype(np.uint8)

        grid_compressed = base64.b64encode(zlib.compress(prob_u8.tobytes(), 1)).decode("ascii")
        hazard_compressed = base64.b64encode(zlib.compress(hazard_u8.tobytes(), 1)).decode("ascii")

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
            "ramp": ramp_res,
            "on_ramp": bool(ramp_res["is_on_ramp"]),
            "ramp_speed_limit": ramp_res["speed_limit"],
            "arm_active": False,
            "tilt_detected": bool(ramp_res["is_on_ramp"]),
            "v_tilt_limit": ramp_res["speed_limit"] if ramp_res["is_on_ramp"] else None,
            "rear_alpha": combined_alpha,
            "effective_mu": drift_res["effective_mu"],
            "slip_state": diff_res["slip_state"],
            "yaw_error": round(float(diff_res["yaw_error"]), 3),
            "yaw_fused": round(float(self.yaw_est), 4),
            "roll_deg": round(float(self.fusion.roll_deg), 2),
            "pitch_deg": round(float(self.fusion.pitch_deg), 2),
            "roll_rate": round(float(self.fusion.roll_rate), 3),
            "v_plan": v_plan,
            "v_tgt": v_plan,
            "v_cmd": v_cmd_act,
            "steer_plan_deg": steer_plan,
            "steer_cmd_deg": steer_cmd_act,
            "steer_deg": steer_cmd_act,
            "steer_rad": round(float(effective_steer_cmd), 4),
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
            "grid_shape": [int(s) for s in prob_u8.shape],
            "grid_bounds": [float(GRID_X_MIN), float(GRID_X_MAX), float(GRID_Y_MIN), float(GRID_Y_MAX)],
            "grid_data": grid_compressed,
            "hazard_data": hazard_compressed,
            "escape_state": str(getattr(self.recovery_manager.state, "name", self.recovery_manager.state)),
            "recovery_state": str(self.recovery_manager.state),
            "is_wrong_way": bool(self.last_aww_status.is_wrong_way) if self.last_aww_status else False,
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
