"""
================================================================================
  FOLKRACE AUTONOMOUS BRAIN: MASTER COORDINATOR (FRAMEWORK-AGNOSTIC)
================================================================================
  Single authoritative autonomous perception and trajectory optimization brain.
  Executed identically in:
    1. Webots Simulation (120 Hz / 60 Hz harness)
    2. Raspberry Pi 4 Real-World Runner (Core 1 dedicated 60 Hz loop)

  Encapsulates:
    - 30 mm minimum floor ToF perception pre-processor
    - C++ Smart Hybrid Costmap (native AVX2 / ARM64 .so)
    - ToF Gap Scout & Aperture Evaluator
    - Curvilinear Slack-Constrained SL-MPC (Native OSQP solver)
    - SE(2) Forward Lookahead Anchor Pose Projector (cancels ToF & actuator lag)
    - Stuck Recovery State Machine & 5-Step Costmap Escape Arc Evaluator
    - Anti-Wrong-Way Watchdog Sentry & Early Gap Exit Evaluator
================================================================================
"""

import os
import sys
import time
import math
from typing import Dict, List, Optional, Tuple, Any, Union
import numpy as np

# Ensure brain directory is on sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

from pose_tracker import PoseTracker, wrap_to_pi
from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
from tof_gap_scout import ToFGapScout
from planner_sl_mpc import CurvilinearSLMPC
from stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState
from anti_wrong_way_watchdog import AntiWrongWayWatchdog
from ramp_detector import RampDetector

try:
    from .planner_config import (
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        R_INFLATION_M,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
        MAX_STEERING_RAD,
        SERVO_MAX_SPEED_RAD_S,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_ARC_LOOKAHEAD_M,
        BRAKE_DECEL_MAX,
        ACCEL_FWD_MAX,
        ACCEL_FWD_RATIO,
        BRAKE_DECEL_RATIO,
        GRAVITY_G,
        MPC_MU_FRICTION,
        MPC_CENTER_BIAS,
        TOF_MAX_RANGE_M,
        WHEELBASE,
    )
except ImportError:
    from planner_config import (
        GRID_X_MIN,
        GRID_X_MAX,
        GRID_Y_MIN,
        GRID_Y_MAX,
        GRID_RES,
        R_INFLATION_M,
        V_MIN_CRUISE,
        V_MAX_CRUISE,
        MAX_STEERING_RAD,
        SERVO_MAX_SPEED_RAD_S,
        RECOVERY_REVERSE_SPEED_MPS,
        RECOVERY_STEER_ANGLE_RAD,
        RECOVERY_CLEARANCE_EXIT_M,
        RECOVERY_ANY_TOF_EXIT_M,
        RECOVERY_ARC_LOOKAHEAD_M,
        BRAKE_DECEL_MAX,
        ACCEL_FWD_MAX,
        ACCEL_FWD_RATIO,
        BRAKE_DECEL_RATIO,
        GRAVITY_G,
        MPC_MU_FRICTION,
        MPC_CENTER_BIAS,
        TOF_MAX_RANGE_M,
        WHEELBASE,
    )

MIN_TOF_PHYSICAL_M = 0.030  # 30 mm minimum floor matching physical VL53L8CX blind zone


class AutonomousBrain:
    """
    Unified, framework-agnostic Autonomous Driving Brain for Folkrace.
    Consumes clean physical sensor readings and produces motion targets.
    """

    def __init__(self, genes: Optional[Dict[str, Any]] = None):
        # 1. Perception & Navigation Engines
        self.r_infl = R_INFLATION_M
        self.costmap = CppSmartHybridCostmap(
            grid_bounds=(GRID_X_MIN, GRID_X_MAX, GRID_Y_MIN, GRID_Y_MAX),
            resolution=GRID_RES,
            r_inflation_m=self.r_infl,
        )
        self.gap_scout = ToFGapScout()
        self.planner = CurvilinearSLMPC(costmap_engine=self.costmap)
        self.planner.costmap = self.costmap

        self.pose_tracker = PoseTracker()

        # 2. Safety State Machines
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
        self.watchdog = AntiWrongWayWatchdog()
        self.ramp_detector = RampDetector()

        # 3. Dynamic Tuning Parameters
        self.base_mu = MPC_MU_FRICTION
        self.mu_val = self.base_mu
        self.target_lookahead_m = 2.40
        self.aggr_dist_far = 2.40
        self.aggr_dist_close = 1.00
        self.aggr_min_scale = 0.30
        self.steer_deadband = 0.002
        self.v_cruise_min = V_MIN_CRUISE
        self.v_cruise_max = V_MAX_CRUISE

        # 4. Internal State Memories
        self.step_counter = 0
        self.active_steer = 0.0
        self.cached_target_steer = 0.0
        self.cached_target_speed = 0.0
        self.rate_limited_speed = 0.0
        self.was_recovering = False
        self.mode_desc = "SL_MPC"
        self.last_aww_status = None
        self.last_gap_info = (0.0, 1.0, 0.50, 0.0)
        self.last_plan_res: Dict[str, Any] = {}
        self.cached_f_tof = TOF_MAX_RANGE_M
        self._clean_tofs: Dict[str, float] = {name: TOF_MAX_RANGE_M for name in SENSOR_NAMES}

        if genes:
            self.configure(genes)
        else:
            try:
                from .planner_config import load_best_genome_genes
            except ImportError:
                from planner_config import load_best_genome_genes
            best_genes, best_path = load_best_genome_genes()
            if best_genes:
                print(f"[AutonomousBrain] Auto-loaded best genome from {best_path}", flush=True)
                self.configure(best_genes)

    def configure(self, genes: Dict[str, Any]) -> None:
        """Applies dynamic tuning parameters or genome weights."""
        if not genes:
            return

        self.base_mu = float(genes.get("mu_friction", MPC_MU_FRICTION))
        self.mu_val = self.base_mu
        self.r_infl = float(genes.get("r_inflation_m", R_INFLATION_M))
        self.target_lookahead_m = float(genes.get("target_lookahead_m", 2.40))
        self.aggr_dist_far = float(genes.get("aggr_dist_far_m", 2.40))
        self.aggr_dist_close = float(genes.get("aggr_dist_close_m", 1.00))
        self.aggr_min_scale = float(genes.get("aggr_min_scale", 0.30))
        self.steer_deadband = float(genes.get("steer_deadband_rad", 0.002))
        self.v_cruise_min = float(genes.get("v_cruise_min", V_MIN_CRUISE))
        self.v_cruise_max = float(genes.get("v_cruise_max", V_MAX_CRUISE))

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
            gap_dist_thresh_m=float(genes.get("gap_dist_thresh_m", 0.95)),
            alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
            scout_obs_width_m=float(genes.get("scout_obs_width_m", 0.22)),
            scout_block_dist_m=float(genes.get("scout_block_dist_m", self.target_lookahead_m)),
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
            alt_switch_hysteresis=float(genes.get("alt_switch_hysteresis", 15.00)),
            alt_min_separation_m=float(genes.get("alt_min_separation_m", 0.25)),
            alt_aperture_bias=float(genes.get("alt_aperture_bias", 1.80)),
            w_gap_clear=float(genes.get("w_gap_clear", 5.00)),
            center_bias=float(genes.get("center_bias", MPC_CENTER_BIAS)),
            track_width_cap_m=float(genes.get("track_width_cap_m", 1.25)),
            corner_outer_bias=float(genes.get("corner_outer_bias", 0.0)),
            inner_corner_margin_m=float(genes.get("inner_corner_margin_m", 0.0)),
            blind_turn_kappa=float(genes.get("blind_turn_kappa", 0.0)),
            brake_decel_scale=float(genes.get("brake_decel_scale", 1.0)),
            narrow_corridor_v_gain=float(genes.get("narrow_corridor_v_gain", 0.0)),
            use_corridor_flow=int(genes.get("use_corridor_flow", 0)),
        )
        self.planner.costmap = self.costmap

        rec_steer = min(MAX_STEERING_RAD, float(genes.get("recovery_steer_angle_rad", RECOVERY_STEER_ANGLE_RAD)))
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
            mu_friction=self.mu_val,
        )

    def set_dynamic_mu(self, new_mu: float) -> None:
        """Dynamically updates the effective tire-road friction coefficient."""
        self.mu_val = float(new_mu)
        if hasattr(self.planner, "set_dynamic_mu"):
            self.planner.set_dynamic_mu(new_mu)
        if hasattr(self.recovery_manager, "mu_friction"):
            self.recovery_manager.mu_friction = float(new_mu)

    def reset(self, start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        """Resets all internal filters, costmap, pose tracker, and recovery state machines."""
        self.costmap.clear()
        self.pose_tracker.reset(start_pose[0], start_pose[1], start_pose[2])
        self.recovery_manager.reset()
        self.watchdog.reset()
        self.ramp_detector.reset()
        self.last_aww_status = None
        if hasattr(self.planner, "reset_active_target"):
            self.planner.reset_active_target()
        self.step_counter = 0
        self.active_steer = 0.0
        self.cached_target_steer = 0.0
        self.cached_target_speed = 0.0
        self.rate_limited_speed = 0.0
        self.was_recovering = False
        self.mode_desc = "SL_MPC"
        self.cached_f_tof = TOF_MAX_RANGE_M

    def step(
        self,
        tof_readings: Union[Dict[str, float], List[float]],
        v_est: float,
        steer_actual_rad: float,
        gyro_z_rad_s: float,
        dt: float = 0.016667,
        sim_time_s: float = 0.0,
        ground_truth_pose: Optional[Tuple[float, float, float]] = None,
        full_telemetry: bool = True,
        pitch_rad: float = 0.0,
        roll_rad: float = 0.0,
    ) -> Tuple[float, float, Dict[str, Any]]:
        """
        Executes one full autonomous perception and trajectory optimization step (typically @ 60 Hz).

        Args:
            tof_readings: 16-channel ToF ranges (either as dict name->meters or list of 16 floats).
            v_est: Current fused forward velocity estimate (m/s).
            steer_actual_rad: Current active steering angle of the front wheels (radians).
            gyro_z_rad_s: Current body yaw rate around Z-axis (rad/s).
            dt: Timestep duration in seconds (default 1/60s = 0.016667s).
            sim_time_s: Current wall or simulation timestamp in seconds.
            ground_truth_pose: Optional (x, y, yaw) from simulator supervisor for global tracking.
            full_telemetry: If False, skips constructing high-allocation trajectory arrays (lean fast-path).

        Returns:
            Tuple[float, float, Dict[str, Any]]:
              - cmd_speed_mps: Commanded forward linear velocity (m/s).
              - cmd_steer_rad: Commanded virtual center steering angle (radians).
              - telemetry: Comprehensive or lean telemetry dictionary for logging/GUI.
        """
        t_start = time.perf_counter()
        self.step_counter += 1

        # 1. Perception Pre-Processing & 30mm Blind-Zone Clamping (In-Place Mutation)
        # Note: values <= 0.0 represent offline/missing sensors and are preserved as 0.0
        # so the costmap leaves their rays as unknown space rather than a 30mm obstacle.
        clean_tofs = self._clean_tofs
        if isinstance(tof_readings, (list, tuple)):
            for i, name in enumerate(SENSOR_NAMES):
                val = float(tof_readings[i]) if i < len(tof_readings) else TOF_MAX_RANGE_M
                if val <= 0.0 or math.isnan(val):
                    clean_tofs[name] = 0.0
                else:
                    clean_tofs[name] = max(MIN_TOF_PHYSICAL_M, min(TOF_MAX_RANGE_M, val))
        else:
            for name in SENSOR_NAMES:
                val = float(tof_readings.get(name, TOF_MAX_RANGE_M))
                if val <= 0.0 or math.isnan(val):
                    clean_tofs[name] = 0.0
                else:
                    clean_tofs[name] = max(MIN_TOF_PHYSICAL_M, min(TOF_MAX_RANGE_M, val))

        f_tof = min(
            clean_tofs.get("tof_front_left_4.1deg", TOF_MAX_RANGE_M),
            clean_tofs.get("tof_front_right_4.1deg", TOF_MAX_RANGE_M),
        )
        self.cached_f_tof = f_tof

        # 2. Dead-Reckoning Odometry Update
        self.pose_tracker.update_odometry(v_est=v_est, yaw_rate=gyro_z_rad_s, dt=dt)
        nav_pose = ground_truth_pose if ground_truth_pose is not None else (self.pose_tracker.x, self.pose_tracker.y, self.pose_tracker.yaw)

        # 3. C++ Smart Hybrid Costmap Processing
        t_map_0 = time.perf_counter()
        prob_map, hazard_mask, map_ms = self.costmap.process_tof_frame(clean_tofs)
        t_map_ms = (time.perf_counter() - t_map_0) * 1000.0

        # 4. 3-Step Composite SE(2) Forward Lookahead Anchor Projection
        is_reversing = (self.recovery_manager.state != StuckRecoveryState.NORMAL_DRIVE)
        (dx_3, dy_3, delta_yaw_3), v_3, steer_3 = self.pose_tracker.compute_projected_anchor(
            v_current=v_est,
            steer_current=steer_actual_rad,
            steer_target=self.cached_target_steer,
            dt=dt,
            is_reversing=is_reversing,
        )

        # 5. Gap Scouting from Projected Anchor
        t_scout_0 = time.perf_counter()
        xg, yg, width, theta_g = self.gap_scout.scout_gap(
            tof_readings=clean_tofs,
            hazard_mask=hazard_mask,
            costmap_obj=self.costmap,
            target_lookahead_m=self.target_lookahead_m,
            steer_current=steer_3,
            prob_map=prob_map,
            anchor_x=dx_3,
            anchor_y=dy_3,
            anchor_yaw=delta_yaw_3,
        )
        t_scout_ms = (time.perf_counter() - t_scout_0) * 1000.0
        self.last_gap_info = (xg, yg, width, theta_g)

        # 6. Curvilinear SL-MPC Trajectory Optimization
        t_plan_0 = time.perf_counter()
        plan_res = self.planner.plan(
            xg=xg,
            yg=yg,
            theta_g=theta_g,
            aperture_width=width,
            v_current=v_3,
            steer_current=steer_3,
            yaw_rate_current=gyro_z_rad_s,
            extra_targets=getattr(self.gap_scout, "last_targets", None),
            x0=dx_3,
            y0=dy_3,
            psi0=delta_yaw_3,
            pitch_rad=pitch_rad,
        )
        t_plan_ms = (time.perf_counter() - t_plan_0) * 1000.0
        self.last_plan_res = plan_res

        raw_target_steer = float(plan_res.get("cmd_steer_rad", 0.0))
        raw_target_speed = float(plan_res.get("cmd_speed_mps", self.v_cruise_min))

        min_fwd_dist = min(
            clean_tofs.get("tof_front_left_4.1deg", 1.0),
            clean_tofs.get("tof_front_right_4.1deg", 1.0),
        )

        # 7. Candidate Gap Scout Targets for Early Exit
        candidate_gaps = []
        if min_fwd_dist >= 0.50:
            for tgt in (getattr(self.gap_scout, "last_targets", []) or []):
                if isinstance(tgt, (list, tuple)) and len(tgt) >= 2:
                    candidate_gaps.append((float(tgt[0]), float(tgt[1])))

        # 8. Anti-Wrong-Way Watchdog
        if getattr(self.watchdog, "mode", "DISABLED") != "DISABLED":
            aww_status = self.watchdog.update(
                x=nav_pose[0],
                y=nav_pose[1],
                yaw=nav_pose[2],
                v_est=v_est,
                yaw_rate=gyro_z_rad_s,
                cached_tofs=clean_tofs,
                now_s=sim_time_s,
                hazard_mask=hazard_mask,
                state=self.recovery_manager.state,
                candidate_gaps=candidate_gaps,
            )
            self.last_aww_status = aww_status

            if (
                aww_status.early_exit_found
                and self.recovery_manager.state == StuckRecoveryState.CORRECTIVE_180
                and getattr(self.recovery_manager, "corrective_substate", "") == "FORWARD"
                and min_fwd_dist >= 0.80
            ):
                self.recovery_manager.finish_recovery_early()

            if aww_status.is_wrong_way and self.recovery_manager.state == StuckRecoveryState.NORMAL_DRIVE:
                tgt_yaw = aww_status.target_recovery_yaw if aww_status.target_recovery_yaw is not None else wrap_to_pi(nav_pose[2] + math.pi)
                yaw_diff = wrap_to_pi(tgt_yaw - nav_pose[2])
                arc_cl = self.recovery_manager.evaluate_costmap_escape_arcs(hazard_mask, self.costmap)
                is_l_pin, is_r_pin = self.recovery_manager.evaluate_pinned_state(hazard_mask, self.costmap, clean_tofs)
                open_side = self.recovery_manager.resolve_open_side(
                    arc_clearance=arc_cl,
                    cached_tofs=clean_tofs,
                    heading_err=yaw_diff,
                    is_left_pinned=is_l_pin,
                    is_right_pinned=is_r_pin,
                )
                self.recovery_manager.trigger_corrective_180(
                    target_yaw=tgt_yaw,
                    steer_sign=open_side,
                    current_yaw=nav_pose[2],
                    arc_clearance=arc_cl,
                    cached_tofs=clean_tofs,
                    front_clearance_m=min_fwd_dist,
                )
        else:
            self.last_aww_status = None

        # 9. Stuck Detection & Shield Reverse Evaluation
        _shield_cur = plan_res.get("shield", {})
        is_stall = (min_fwd_dist < 0.20 and v_est < 0.35 and raw_target_speed > 0.5)
        is_wall_contact = (min_fwd_dist <= 0.05)
        shield_reverse = bool(_shield_cur.get("reverse_suggested", False))

        is_aww_wrong_way = bool(self.last_aww_status and self.last_aww_status.is_wrong_way)
        target_breadcrumb_yaw = (
            self.last_aww_status.target_recovery_yaw
            if (self.last_aww_status and self.last_aww_status.target_recovery_yaw is not None)
            else self.watchdog.get_reverse_breadcrumb_yaw(nav_pose[0], nav_pose[1], nav_pose[2])
        )

        if (shield_reverse or is_wall_contact or is_stall) and self.recovery_manager.state == StuckRecoveryState.NORMAL_DRIVE:
            _arc_cl = self.recovery_manager.evaluate_costmap_escape_arcs(hazard_mask, self.costmap)
            _is_l_pin, _is_r_pin = self.recovery_manager.evaluate_pinned_state(hazard_mask, self.costmap, clean_tofs)
            _side = self.recovery_manager.resolve_escape_steer_dir(
                arc_clearance=_arc_cl, heading_err=float(theta_g),
                is_left_pinned=_is_l_pin, is_right_pinned=_is_r_pin,
            )
            self.recovery_manager.trigger_emergency_recovery(
                heading_err=float(theta_g),
                steer_dir=_side,
                current_v=v_est,
                current_steer=steer_actual_rad,
                instant_reverse=True,
                target_breadcrumb_yaw=target_breadcrumb_yaw,
                current_yaw=nav_pose[2],
                front_clearance_m=min_fwd_dist,
                hazard_mask=hazard_mask,
                costmap_obj=self.costmap,
                cached_tofs=clean_tofs,
                is_left_pinned=_is_l_pin,
                is_right_pinned=_is_r_pin,
                is_wrong_way=is_aww_wrong_way,
            )

        # 10. Execute Recovery State Machine
        is_early_exit = bool(
            self.last_aww_status
            and self.last_aww_status.early_exit_found
            and self.recovery_manager.state == StuckRecoveryState.CORRECTIVE_180
            and min_fwd_dist >= 0.80
        )
        is_recovering, rec_v, rec_steer = self.recovery_manager.update(
            dt=dt,
            current_v=v_est,
            commanded_v=raw_target_speed,
            cached_tofs=clean_tofs,
            heading_err=float(theta_g),
            active_steer=steer_actual_rad,
            hazard_mask=hazard_mask,
            costmap_obj=self.costmap,
            current_yaw=nav_pose[2],
            target_breadcrumb_yaw=target_breadcrumb_yaw,
            early_exit_available=is_early_exit,
            is_wrong_way=is_aww_wrong_way,
        )

        if is_recovering:
            target_speed = rec_v
            target_steer = rec_steer
            self.active_steer = rec_steer
            rec_state = getattr(self.recovery_manager.state, 'name', str(self.recovery_manager.state))
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
                self.active_steer = steer_actual_rad

            target_speed = raw_target_speed
            target_steer = raw_target_steer
            is_shield_active = bool(_shield_cur.get("active", False))

            if is_shield_active:
                self.active_steer = max(-MAX_STEERING_RAD, min(MAX_STEERING_RAD, target_steer))
                c_name = _shield_cur.get("chosen_name", "ACTIVE").replace("_", " ").upper()
                self.mode_desc = f"SHIELD_{c_name}"
            else:
                steer_delta = target_steer - self.active_steer
                # Progressive distance & offset aggressiveness
                if min_fwd_dist >= self.aggr_dist_far:
                    dist_scale = self.aggr_min_scale
                elif min_fwd_dist <= self.aggr_dist_close:
                    dist_scale = 1.0
                else:
                    frac = (self.aggr_dist_far - min_fwd_dist) / max(0.001, (self.aggr_dist_far - self.aggr_dist_close))
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

        # 10b. Ramp Detection & Speed Governance (Unified Hardware & Sim Safety)
        ramp_res = self.ramp_detector.update(pitch_rad=pitch_rad, roll_rad=roll_rad, dt=dt)
        if ramp_res["is_on_ramp"]:
            ramp_cap = float(ramp_res["speed_limit"])
            if target_speed > 0.0:
                target_speed = min(target_speed, ramp_cap)
            elif target_speed < 0.0:
                target_speed = max(target_speed, -ramp_cap)
            if "RAMP" not in self.mode_desc:
                self.mode_desc = f"RAMP_{self.mode_desc}"

        # 10c. Unified Dynamic Speed Rate Limiter (Smooth takeoff, no 0 -> 400 RPM step shock)
        if is_recovering:
            # During reverse recovery, target_speed is negative (e.g. -1.2 m/s)
            target_v = target_speed
            d_limit = min(BRAKE_DECEL_MAX, BRAKE_DECEL_RATIO * self.mu_val * GRAVITY_G)
            speed_err = target_v - self.rate_limited_speed
            max_dv = d_limit * dt
            if abs(speed_err) <= max_dv:
                self.rate_limited_speed = target_v
            else:
                self.rate_limited_speed += math.copysign(max_dv, speed_err)
        else:
            target_v = target_speed
            v_curr = v_est

            # Loose 60 Hz guardrail clamp; 400 Hz ESP32 electronic_diff governs physical traction ramp
            a_limit = 2.2 * min(ACCEL_FWD_MAX, ACCEL_FWD_RATIO * self.mu_val * GRAVITY_G)
            d_limit = 1.5 * min(BRAKE_DECEL_MAX, BRAKE_DECEL_RATIO * self.mu_val * GRAVITY_G)

            # Instant crash/stop resync: wide threshold so ESP32 400 Hz ramp lead never triggers false resyncs
            if self.rate_limited_speed >= 0.0:
                if v_curr < (self.rate_limited_speed - 0.85):
                    self.rate_limited_speed = max(0.0, v_curr)
            else:
                if v_curr > (self.rate_limited_speed + 0.85):
                    self.rate_limited_speed = min(0.0, v_curr)

            speed_err = target_v - self.rate_limited_speed
            is_accel = (self.rate_limited_speed >= 0.0 and speed_err > 0.0) or (self.rate_limited_speed <= 0.0 and speed_err < 0.0)
            max_dv = (a_limit if is_accel else d_limit) * dt

            if abs(speed_err) <= max_dv:
                self.rate_limited_speed = target_v
            else:
                self.rate_limited_speed += math.copysign(max_dv, speed_err)

        target_speed = self.rate_limited_speed
        self.cached_target_speed = target_speed
        self.cached_target_steer = self.active_steer
        t_total_ms = (time.perf_counter() - t_start) * 1000.0

        # 11. Compile Telemetry Dictionary
        if full_telemetry:
            if self.last_aww_status is not None and getattr(self.watchdog, "mode", "DISABLED") != "DISABLED":
                trail_pts = [
                    [round(float(p.x), 3), round(float(p.y), 3)]
                    for p in list(self.watchdog.breadcrumb_watchdog.breadcrumbs)[::2]
                ][-60:]
                sightline_rays = [
                    {
                        "xv": round(float(r["xv"]), 3),
                        "yv": round(float(r["yv"]), 3),
                        "clear": bool(r["clear"]),
                        "opposing": bool(r["opposing"]),
                        "heading_diff_deg": round(float(r["heading_diff_deg"]), 1),
                    }
                    for r in (self.last_aww_status.sightline_rays or [])
                ]
                raw_tgt_yaw = self.last_aww_status.target_recovery_yaw
                if raw_tgt_yaw is None:
                    raw_tgt_yaw = getattr(self.recovery_manager, "target_corrective_yaw", None)
                aww_target_yaw = round(float(raw_tgt_yaw), 3) if raw_tgt_yaw is not None else 0.0
            else:
                trail_pts = []
                sightline_rays = []
                aww_target_yaw = 0.0

            telemetry = {
                "time_step": self.step_counter,
                "sim_time": round(float(sim_time_s), 3),
                "mode": self.mode_desc,
                "planner": "SL_MPC",
                "status": plan_res.get("status", "OK"),
                "v_cmd": round(float(target_speed), 2),
                "v_tgt": round(float(raw_target_speed), 2),
                "v_est": round(float(v_est), 2),
                "steer_cmd_deg": round(math.degrees(self.active_steer), 1),
                "steer_deg": round(math.degrees(self.active_steer), 1),
                "steer_actual_deg": round(math.degrees(steer_actual_rad), 1),
                "pose": [round(float(nav_pose[0]), 3), round(float(nav_pose[1]), 3), round(float(nav_pose[2]), 3)],
                "proj_anchor": [round(float(dx_3), 3), round(float(dy_3), 3), round(float(delta_yaw_3), 3)],
                "proj_v": round(float(v_3), 2),
                "proj_steer_deg": round(math.degrees(steer_3), 1),
                "f_tof": round(float(f_tof), 3),
                "tof": {k: round(float(v), 3) for k, v in clean_tofs.items()},
                "tof_ranges": [round(float(clean_tofs.get(k, TOF_MAX_RANGE_M)), 3) for k in SENSOR_NAMES],
                "gap": [round(float(xg), 3), round(float(yg), 3), round(float(width), 3)],
                "targets": getattr(self.gap_scout, "last_targets", []),
                "best_line": plan_res.get("best_line", []),
                "candidate_lines": plan_res.get("candidate_lines", []),
                "guide_line": plan_res.get("guide_line", []),
                "corridor_left": plan_res.get("corridor_left", []),
                "corridor_right": plan_res.get("corridor_right", []),
                "shield": plan_res.get("shield", {}),
                "plan_ms": round(float(t_plan_ms), 2),
                "map_ms": round(float(t_map_ms), 2),
                "scout_ms": round(float(t_scout_ms), 2),
                "opt_prim_ms": round(float(plan_res.get("opt_prim_ms", 0.0)), 2),
                "opt_alt_ms": round(float(plan_res.get("opt_alt_ms", 0.0)), 2),
                "osqp_iters": int(plan_res.get("osqp_iters", 0)),
                "osqp_iters_alt": int(plan_res.get("osqp_iters_alt", 0)),
                "iters_prim": int(plan_res.get("osqp_iters", 0)),
                "iters_alt": int(plan_res.get("osqp_iters_alt", 0)),
                "status_prim": plan_res.get("status_prim"),
                "status_alt": plan_res.get("status_alt"),
                "cost_primary": round(float(plan_res["cost_primary"]), 3) if plan_res.get("cost_primary") is not None else None,
                "cost_alt": round(float(plan_res["cost_alt"]), 3) if plan_res.get("cost_alt") is not None else None,
                "depth_credit": round(float(plan_res.get("depth_credit", 0.0)), 3),
                "winner_id": int(plan_res.get("winner_id", 0)),
                "active_target": [round(float(v), 3) for v in self.planner._active_target] if getattr(self.planner, "_active_target", None) is not None else None,
                "step_ms": round(float(t_total_ms), 2),
                "recovery_state": str(self.recovery_manager.state),
                "is_wrong_way": bool(self.last_aww_status.is_wrong_way) if self.last_aww_status else False,
                "target_yaw": aww_target_yaw,
                "trail": trail_pts,
                "sightlines": sightline_rays,
                "is_on_ramp": bool(ramp_res["is_on_ramp"]),
                "ramp_speed_limit": float(ramp_res["speed_limit"]),
                "pitch_deg": float(ramp_res["pitch_deg"]),
                "roll_deg": float(ramp_res["roll_deg"]),
            }
        else:
            telemetry = {
                "time_step": self.step_counter,
                "sim_time": round(float(sim_time_s), 3),
                "mode": self.mode_desc,
                "planner": "SL_MPC",
                "status": plan_res.get("status", "OK"),
                "v_cmd": round(float(target_speed), 2),
                "v_tgt": round(float(raw_target_speed), 2),
                "v_est": round(float(v_est), 2),
                "steer_cmd_deg": round(math.degrees(self.active_steer), 1),
                "steer_deg": round(math.degrees(self.active_steer), 1),
                "steer_actual_deg": round(math.degrees(steer_actual_rad), 1),
                "f_tof": round(float(f_tof), 3),
                "plan_ms": round(float(t_plan_ms), 2),
                "map_ms": round(float(t_map_ms), 2),
                "scout_ms": round(float(t_scout_ms), 2),
                "opt_prim_ms": round(float(plan_res.get("opt_prim_ms", 0.0)), 2),
                "opt_alt_ms": round(float(plan_res.get("opt_alt_ms", 0.0)), 2),
                "osqp_iters": int(plan_res.get("osqp_iters", 0)),
                "osqp_iters_alt": int(plan_res.get("osqp_iters_alt", 0)),
                "iters_prim": int(plan_res.get("osqp_iters", 0)),
                "iters_alt": int(plan_res.get("osqp_iters_alt", 0)),
                "status_prim": plan_res.get("status_prim"),
                "status_alt": plan_res.get("status_alt"),
                "cost_primary": round(float(plan_res["cost_primary"]), 3) if plan_res.get("cost_primary") is not None else None,
                "cost_alt": round(float(plan_res["cost_alt"]), 3) if plan_res.get("cost_alt") is not None else None,
                "depth_credit": round(float(plan_res.get("depth_credit", 0.0)), 3),
                "winner_id": int(plan_res.get("winner_id", 0)),
                "active_target": [round(float(v), 3) for v in self.planner._active_target] if getattr(self.planner, "_active_target", None) is not None else None,
                "step_ms": round(float(t_total_ms), 2),
                "recovery_state": str(self.recovery_manager.state),
                "is_wrong_way": bool(self.last_aww_status.is_wrong_way) if self.last_aww_status else False,
                "is_on_ramp": bool(ramp_res["is_on_ramp"]),
                "ramp_speed_limit": float(ramp_res["speed_limit"]),
                "pitch_deg": float(ramp_res["pitch_deg"]),
                "roll_deg": float(ramp_res["roll_deg"]),
            }

        return (target_speed, self.active_steer, telemetry)
