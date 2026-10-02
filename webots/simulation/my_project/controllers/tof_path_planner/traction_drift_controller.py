"""
================================================================================
  TRACTION, DRIFT & STABILITY CONTROLLER (REAR OPTICAL FLOW + ACTIVE FRONT STEER)
================================================================================
  High-dynamic drift detection, active front countersteering (AFS), anti-snap
  Schmitt trigger, per-wheel ground-roll clamping, and dynamic friction scaling.
================================================================================
"""

import math
from typing import Dict, Any, Optional

try:
    from .planner_config import (
        WHEELBASE,
        TRACK_REAR,
        MAX_STEERING_RAD,
        ENABLE_TRACTION_DRIFT_CONTROL,
        V_ARM_DRIFT_MPS,
        V_DISARM_CRAWL_MPS,
        V_CRAWL_FLOOR_MPS,
        BETA_SCRUB_RAD,
        BETA_DRIFT_RAD,
        BETA_SPIN_RAD,
        K_BETA,
        K_YAW_RATE,
        K_YAW_DOT,
        K_RECOVERY_RELAX,
        ANTI_SNAP_HOLD_TIME_S,
        D_BETA_LP_ALPHA,
        ENABLE_DYNAMIC_MU_SCALING,
        ALPHA_SLEW_DROP_RATE,
        ALPHA_SLEW_RISE_RATE,
    )
except ImportError:
    from planner_config import (
        WHEELBASE,
        TRACK_REAR,
        MAX_STEERING_RAD,
        ENABLE_TRACTION_DRIFT_CONTROL,
        V_ARM_DRIFT_MPS,
        V_DISARM_CRAWL_MPS,
        V_CRAWL_FLOOR_MPS,
        BETA_SCRUB_RAD,
        BETA_DRIFT_RAD,
        BETA_SPIN_RAD,
        K_BETA,
        K_YAW_RATE,
        K_YAW_DOT,
        K_RECOVERY_RELAX,
        ANTI_SNAP_HOLD_TIME_S,
        D_BETA_LP_ALPHA,
        ENABLE_DYNAMIC_MU_SCALING,
        ALPHA_SLEW_DROP_RATE,
        ALPHA_SLEW_RISE_RATE,
    )


class TractionDriftController:
    """
    Closed-loop stability, countersteering and traction management using
    ground-truth / optical-flow rear axle kinematics and IMU yaw telemetry.
    """

    def __init__(
        self,
        wheelbase: float = WHEELBASE,
        track_rear: float = TRACK_REAR,
        max_steer: float = MAX_STEERING_RAD,
        v_arm_drift: float = V_ARM_DRIFT_MPS,
        v_disarm_crawl: float = V_DISARM_CRAWL_MPS,
        v_crawl_floor: float = V_CRAWL_FLOOR_MPS,
        beta_scrub: float = BETA_SCRUB_RAD,
        beta_drift: float = BETA_DRIFT_RAD,
        beta_spin: float = BETA_SPIN_RAD,
        k_beta: float = K_BETA,
        k_yaw_rate: float = K_YAW_RATE,
        k_yaw_dot: float = K_YAW_DOT,
        k_recovery_relax: float = K_RECOVERY_RELAX,
        anti_snap_hold_time: float = ANTI_SNAP_HOLD_TIME_S,
        d_beta_lp_alpha: float = D_BETA_LP_ALPHA,
        alpha_slew_drop: float = ALPHA_SLEW_DROP_RATE,
        alpha_slew_rise: float = ALPHA_SLEW_RISE_RATE,
        enable_dynamic_mu: bool = ENABLE_DYNAMIC_MU_SCALING,
        enabled: bool = ENABLE_TRACTION_DRIFT_CONTROL,
    ):
        self.wheelbase = wheelbase
        self.track_rear = track_rear
        self.max_steer = max_steer

        self.v_arm_drift = v_arm_drift
        self.v_disarm_crawl = v_disarm_crawl
        self.v_crawl_floor = v_crawl_floor

        self.beta_scrub = beta_scrub
        self.beta_drift = beta_drift
        self.beta_spin = beta_spin

        self.k_beta = k_beta
        self.k_yaw_rate = k_yaw_rate
        self.k_yaw_dot = k_yaw_dot
        self.k_recovery_relax = k_recovery_relax
        self.anti_snap_hold_time = anti_snap_hold_time
        self.d_beta_lp_alpha = d_beta_lp_alpha

        self.alpha_slew_drop_rate = alpha_slew_drop
        self.alpha_slew_rise_rate = alpha_slew_rise
        self.enable_dynamic_mu = enable_dynamic_mu
        self.enabled = enabled

        # Stateful variables
        self.drift_armed: bool = False
        self.prev_beta_rear: float = 0.0
        self.d_beta_filt: float = 0.0
        self.prev_gyro_z: float = 0.0
        self.yaw_dot_filt: float = 0.0
        self.recovery_hold_timer: float = 0.0
        self.current_alpha: float = 1.0

    def reset(self) -> None:
        """Resets all dynamic filters, latch states, and alpha slew."""
        self.drift_armed = False
        self.prev_beta_rear = 0.0
        self.d_beta_filt = 0.0
        self.prev_gyro_z = 0.0
        self.yaw_dot_filt = 0.0
        self.recovery_hold_timer = 0.0
        self.current_alpha = 1.0

    def update(
        self,
        v_forward: float,
        beta_rear: float,
        gyro_z: float,
        target_steer_rad: float,
        base_mu: float = 0.86,
        dt: float = 0.008,
    ) -> Dict[str, Any]:
        """
        Executes one control cycle for drift stability and traction allocation.

        Args:
            v_forward: Body forward velocity at rear axle (m/s)
            beta_rear: Rear tire sideslip angle atan2(v_lat, v_fwd) in radians
            gyro_z: Measured body yaw rate from IMU (rad/s)
            target_steer_rad: Desired virtual steering angle from trajectory planner (rad)
            base_mu: Nominal ground friction coefficient
            dt: Timestep duration in seconds (default 8 ms / 125 Hz)

        Returns:
            Dictionary containing commanded steer, alpha, ground speeds, and telemetry.
        """
        # Per-wheel local road surface contact velocities
        half_Wr_gyro = gyro_z * (self.track_rear * 0.5)
        v_ground_RL = v_forward - half_Wr_gyro
        v_ground_RR = v_forward + half_Wr_gyro

        abs_beta = abs(beta_rear)

        if not self.enabled:
            return {
                "drift_armed": False,
                "speed_fade": 0.0,
                "mode": "DISABLED",
                "beta_rear": beta_rear,
                "beta_rear_deg": math.degrees(beta_rear),
                "d_beta_filt": 0.0,
                "is_stabilizing": False,
                "anti_snap_timer": 0.0,
                "active_k_beta": self.k_beta,
                "delta_counter": 0.0,
                "delta_counter_deg": 0.0,
                "steer_cmd": target_steer_rad,
                "rear_alpha": 1.0,
                "target_alpha": 1.0,
                "v_ground_RL": v_ground_RL,
                "v_ground_RR": v_ground_RR,
                "effective_mu": base_mu,
                "slip_fraction": 0.0,
            }

        # 1. Dual-Stage Hysteretic Speed Gate
        if not self.drift_armed:
            if v_forward >= self.v_arm_drift:
                self.drift_armed = True
        else:
            # Disarm ONLY if speed drops below crawl speed AND tires have regained grip
            if v_forward < self.v_disarm_crawl and abs_beta < self.beta_scrub:
                self.drift_armed = False

        # Soft Speed Fade Ramp
        if self.drift_armed:
            if v_forward >= self.v_arm_drift:
                speed_fade = 1.0
            else:
                # Linear fade between crawl floor and arming threshold
                denom = max(1e-4, self.v_arm_drift - self.v_crawl_floor)
                speed_fade = max(0.0, min(1.0, (v_forward - self.v_crawl_floor) / denom))
        else:
            speed_fade = 0.0

        # 2. Optical Flow Slip Rate Low-Pass Filtering
        if dt > 1e-5:
            d_beta_raw = (beta_rear - self.prev_beta_rear) / dt
            d_yaw_raw = (gyro_z - self.prev_gyro_z) / dt
        else:
            d_beta_raw = 0.0
            d_yaw_raw = 0.0

        self.d_beta_filt = (1.0 - self.d_beta_lp_alpha) * self.d_beta_filt + self.d_beta_lp_alpha * d_beta_raw
        self.yaw_dot_filt = 0.50 * self.yaw_dot_filt + 0.50 * d_yaw_raw

        self.prev_beta_rear = beta_rear
        self.prev_gyro_z = gyro_z

        # 3. Stateful Anti-Snap Schmitt Trigger (80 ms Hold Latch)
        # Condition: beta_rear and d_beta_filt have opposing signs with magnitude > 0.05
        # (slip angle collapsing rapidly -> traction recovery imminent)
        if (beta_rear * self.d_beta_filt) < -0.05:
            self.recovery_hold_timer = self.anti_snap_hold_time
        else:
            self.recovery_hold_timer = max(0.0, self.recovery_hold_timer - dt)

        is_recovering = (self.recovery_hold_timer > 0.0)
        active_k_beta = (self.k_beta * self.k_recovery_relax) if is_recovering else self.k_beta

        # 4. Slip Severity & Rear Axle Drive Target Alpha
        if not self.drift_armed or abs_beta < self.beta_scrub:
            mode = "GRIP"
            target_alpha = 1.0
            slip_fraction = 0.0
        elif abs_beta < self.beta_drift:
            mode = "SCRUB"
            # Smooth transition from 1.0 to 0.50
            ratio = (abs_beta - self.beta_scrub) / max(1e-4, self.beta_drift - self.beta_scrub)
            target_alpha = 1.0 - 0.50 * ratio
            slip_fraction = (abs_beta - self.beta_scrub) / max(1e-4, self.beta_spin - self.beta_scrub)
        elif abs_beta < self.beta_spin:
            mode = "DRIFT"
            # Continuous decay from 0.50 down to 0.0
            ratio = (abs_beta - self.beta_drift) / max(1e-4, self.beta_spin - self.beta_drift)
            target_alpha = 0.50 - 0.50 * ratio
            slip_fraction = (abs_beta - self.beta_scrub) / max(1e-4, self.beta_spin - self.beta_scrub)
        else:
            mode = "SPIN"
            target_alpha = 0.0
            slip_fraction = 1.0

        slip_fraction = max(0.0, min(1.0, slip_fraction))

        # 5. Active Front Countersteering Calculation
        # Strictly zero intervention while tires maintain normal grip
        if mode == "GRIP" and not is_recovering:
            delta_counter = 0.0
            steer_cmd = target_steer_rad
        else:
            # Kinematic expected yaw rate from front steering angle:
            r_kin = (v_forward / self.wheelbase) * math.tan(target_steer_rad)
            yaw_err = gyro_z - r_kin

            # Countersteering angle (negative sign points wheels into the slide):
            # In a left turn (steer > 0, gyro_z > 0), rightward slip (beta > 0) or oversteer (yaw_err > 0)
            # requires rightward countersteer (steer delta < 0).
            delta_counter_raw = -(active_k_beta * beta_rear) - (self.k_yaw_rate * yaw_err) - (self.k_yaw_dot * self.yaw_dot_filt)
            delta_counter = speed_fade * delta_counter_raw
            steer_cmd = max(-self.max_steer, min(self.max_steer, target_steer_rad + delta_counter))

        # 6. Slew-Rate-Limited Rear Alpha (Asymmetric 4.0/s drop, 2.5/s rise)
        if target_alpha < self.current_alpha:
            max_drop = self.alpha_slew_drop_rate * dt
            self.current_alpha = max(target_alpha, self.current_alpha - max_drop)
        else:
            max_rise = self.alpha_slew_rise_rate * dt
            self.current_alpha = min(target_alpha, self.current_alpha + max_rise)

        # 7. Linear Dynamic MU Scaling
        if self.enable_dynamic_mu and self.drift_armed:
            effective_mu = base_mu * (1.0 - 0.15 * slip_fraction)
        else:
            effective_mu = base_mu

        return {
            "drift_armed": self.drift_armed,
            "speed_fade": round(speed_fade, 3),
            "mode": mode,
            "beta_rear": round(beta_rear, 4),
            "beta_rear_deg": round(math.degrees(beta_rear), 2),
            "d_beta_filt": round(self.d_beta_filt, 4),
            "is_stabilizing": is_recovering,
            "anti_snap_timer": round(self.recovery_hold_timer, 4),
            "active_k_beta": round(active_k_beta, 3),
            "delta_counter": round(delta_counter, 4),
            "delta_counter_deg": round(math.degrees(delta_counter), 2),
            "steer_cmd": steer_cmd,
            "rear_alpha": round(self.current_alpha, 4),
            "target_alpha": round(target_alpha, 4),
            "v_ground_RL": round(v_ground_RL, 4),
            "v_ground_RR": round(v_ground_RR, 4),
            "effective_mu": round(effective_mu, 4),
            "slip_fraction": round(slip_fraction, 4),
        }
