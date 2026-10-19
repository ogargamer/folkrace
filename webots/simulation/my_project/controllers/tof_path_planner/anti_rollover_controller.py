"""
================================================================================
  ACTIVE ANTI-ROLLOVER MITIGATION (ARM) CONTROLLER
================================================================================
  - 15° to 22° C¹-continuous cubic Hermite blending ramp (rejects ramps/curbs)
  - Proportional countersteering + derivative roll-rate landing damping
  - Dynamic tire mu coupling for friction-circle safe deceleration
  - Individual closed-loop FOC wheel speed clamping on lifted airborne wheels
  - 80 ms touchdown latch to prevent chatter and suspension snap-back
================================================================================
"""

import math
from typing import Dict, Any, Optional

try:
    from .planner_config import (
        MAX_STEERING_RAD,
        ENABLE_ANTI_ROLLOVER,
        ARM_TILT_WARN_RAD,
        ARM_TILT_CRIT_RAD,
        ARM_K_P_ROLL,
        ARM_K_D_ROLL,
        ARM_SETTLE_TIME_S,
        ARM_DECEL_FRACTION,
        ARM_MIN_MU_RATIO,
        ARM_NOMINAL_MU,
        ENABLE_TILT_SPEED_SCALING,
        TILT_SLOWDOWN_START_DEG,
        TILT_SLOWDOWN_END_DEG,
        TILT_MAX_SPEED_CAP,
        TILT_MIN_SPEED_CAP,
        TILT_EXP_FACTOR,
    )
except ImportError:
    from planner_config import (
        MAX_STEERING_RAD,
        ENABLE_ANTI_ROLLOVER,
        ARM_TILT_WARN_RAD,
        ARM_TILT_CRIT_RAD,
        ARM_K_P_ROLL,
        ARM_K_D_ROLL,
        ARM_SETTLE_TIME_S,
        ARM_DECEL_FRACTION,
        ARM_MIN_MU_RATIO,
        ARM_NOMINAL_MU,
        ENABLE_TILT_SPEED_SCALING,
        TILT_SLOWDOWN_START_DEG,
        TILT_SLOWDOWN_END_DEG,
        TILT_MAX_SPEED_CAP,
        TILT_MIN_SPEED_CAP,
        TILT_EXP_FACTOR,
    )


class AntiRolloverController:
    """
    Active roll-stability and dynamic rollover mitigation controller.
    """

    def __init__(
        self,
        max_steer: float = MAX_STEERING_RAD,
        tilt_warn_rad: float = ARM_TILT_WARN_RAD,
        tilt_crit_rad: float = ARM_TILT_CRIT_RAD,
        k_p_roll: float = ARM_K_P_ROLL,
        k_d_roll: float = ARM_K_D_ROLL,
        settle_time_s: float = ARM_SETTLE_TIME_S,
        decel_fraction: float = ARM_DECEL_FRACTION,
        min_mu_ratio: float = ARM_MIN_MU_RATIO,
        nominal_mu: float = ARM_NOMINAL_MU,
        tilt_slowdown_start_deg: float = TILT_SLOWDOWN_START_DEG,
        tilt_slowdown_end_deg: float = TILT_SLOWDOWN_END_DEG,
        tilt_max_speed_cap: float = TILT_MAX_SPEED_CAP,
        tilt_min_speed_cap: float = TILT_MIN_SPEED_CAP,
        tilt_exp_factor: float = TILT_EXP_FACTOR,
        enable_tilt_speed_scaling: bool = ENABLE_TILT_SPEED_SCALING,
        enabled: bool = ENABLE_ANTI_ROLLOVER,
    ):
        self.max_steer = max_steer
        self.tilt_warn_rad = tilt_warn_rad
        self.tilt_crit_rad = tilt_crit_rad
        self.k_p_roll = k_p_roll
        self.k_d_roll = k_d_roll
        self.settle_time_s = settle_time_s
        self.decel_fraction = decel_fraction
        self.min_mu_ratio = min_mu_ratio
        self.nominal_mu = nominal_mu
        self.tilt_slowdown_start_deg = tilt_slowdown_start_deg
        self.tilt_slowdown_end_deg = tilt_slowdown_end_deg
        self.tilt_max_speed_cap = tilt_max_speed_cap
        self.tilt_min_speed_cap = tilt_min_speed_cap
        self.tilt_exp_factor = tilt_exp_factor
        self.enable_tilt_speed_scaling = enable_tilt_speed_scaling
        self.enabled = enabled

        # Stateful latch & low-pass filters
        self.settle_timer: float = 0.0
        self.prev_w_tilt: float = 0.0
        self.is_mitigating: bool = False

    def reset(self) -> None:
        """Resets dynamic timers and mitigation state."""
        self.settle_timer = 0.0
        self.prev_w_tilt = 0.0
        self.is_mitigating = False

    def update(
        self,
        roll_rad: float,
        roll_rate: float,
        target_steer_rad: float,
        v_forward: float,
        mu: float = ARM_NOMINAL_MU,
        dt: float = 0.008,
        pitch_rad: float = 0.0,
    ) -> Dict[str, Any]:
        """
        Executes one anti-rollover evaluation cycle.

        Args:
            roll_rad: Vehicle lateral bank/roll angle in radians (+ = leaning right, - = leaning left)
            roll_rate: Vehicle roll angular velocity in rad/s from gyro
            target_steer_rad: Active front steering angle command from planner / drift controller
            v_forward: Current vehicle ground forward speed (m/s)
            mu: Live effective tire friction coefficient
            dt: Timestep duration (seconds)
            pitch_rad: Vehicle pitch angle in radians (transverse elevation)

        Returns:
            Dictionary containing blended steer command, countersteer delta, alpha,
            decel target, tilt detection flag, tilt speed limit, and individual FOC wheel clamping factors.
        """
        abs_roll = abs(roll_rad)
        sign_roll = math.copysign(1.0, roll_rad) if abs_roll > 1e-4 else 0.0
        tilt_mag_rad = math.hypot(roll_rad, pitch_rad)
        tilt_deg = math.degrees(tilt_mag_rad)

        # Dynamic Pre-Emptive Tilt Speed Scaling (detect > 4° up to 15° ARM threshold)
        is_tilt_detected = (tilt_deg >= self.tilt_slowdown_start_deg)
        if is_tilt_detected and self.enable_tilt_speed_scaling:
            delta_tilt = max(0.01, self.tilt_slowdown_end_deg - self.tilt_slowdown_start_deg)
            u_tilt = min(1.0, max(0.0, (tilt_deg - self.tilt_slowdown_start_deg) / delta_tilt))
            # Slightly exponential decay scale factor: exp(-k * u)
            scale_factor = math.exp(-self.tilt_exp_factor * u_tilt)
            v_tilt_limit = max(self.tilt_min_speed_cap, self.tilt_max_speed_cap * scale_factor)
        else:
            u_tilt = 0.0
            scale_factor = 1.0
            v_tilt_limit = None

        if not self.enabled:
            return {
                "is_active": False,
                "tilt_detected": False,
                "tilt_deg": round(tilt_deg, 2),
                "v_tilt_limit": None,
                "tilt_scale_factor": 1.0,
                "w_tilt": 0.0,
                "delta_counter": 0.0,
                "delta_counter_deg": 0.0,
                "steer_cmd": target_steer_rad,
                "alpha_rollover": 1.0,
                "v_arm_target": None,
                "lifted_side": "NONE",
                "lifted_alpha": 1.0,
                "roll_deg": round(math.degrees(roll_rad), 2),
                "roll_rate": round(roll_rate, 3),
            }

        # 1. Continuous C¹ Cubic Hermite Activation Blend (15° to 22°)
        if abs_roll >= self.tilt_warn_rad:
            self.settle_timer = self.settle_time_s
            u = min(1.0, (abs_roll - self.tilt_warn_rad) / max(1e-4, self.tilt_crit_rad - self.tilt_warn_rad))
            # Smoothstep: 3*u^2 - 2*u^3 (zero derivative at boundaries, zero harsh jerk)
            w_tilt = 3.0 * (u ** 2) - 2.0 * (u ** 3)
            self.is_mitigating = True
        elif self.settle_timer > 0.0:
            # Touchdown latch: smoothly decay remaining intervention over settle window
            self.settle_timer = max(0.0, self.settle_timer - dt)
            decay_ratio = self.settle_timer / max(1e-4, self.settle_time_s)
            w_tilt = self.prev_w_tilt * decay_ratio
            self.is_mitigating = (w_tilt > 0.01)
        else:
            w_tilt = 0.0
            self.is_mitigating = False

        self.prev_w_tilt = w_tilt

        # If below 15° and settled, return zero countersteer intervention but maintain tilt speed limit
        if w_tilt <= 0.0001 and abs_roll < self.tilt_warn_rad:
            return {
                "is_active": False,
                "tilt_detected": is_tilt_detected,
                "tilt_deg": round(tilt_deg, 2),
                "v_tilt_limit": round(v_tilt_limit, 4) if v_tilt_limit is not None else None,
                "tilt_scale_factor": round(scale_factor, 4),
                "w_tilt": 0.0,
                "delta_counter": 0.0,
                "delta_counter_deg": 0.0,
                "steer_cmd": target_steer_rad,
                "alpha_rollover": 1.0,
                "v_arm_target": None,
                "lifted_side": "NONE",
                "lifted_alpha": 1.0,
                "roll_deg": round(math.degrees(roll_rad), 2),
                "roll_rate": round(roll_rate, 3),
            }

        # 2. Dynamic Tire Mu-Aware Countersteering & Roll-Rate Damping
        # On slick surfaces (low mu), scale down countersteer gain so front tires don't wash out.
        # On high grip (high mu), countersteer asserts authority to cancel rollover moment.
        mu_clamped = max(0.10, mu)
        kp_eff = self.k_p_roll * min(1.0, max(self.min_mu_ratio, mu_clamped / self.nominal_mu))

        # Countersteer into the roll:
        # Lean right (roll > 0) -> steer right (+ delta) to unwind corner and push wheels under mass
        # Lean left (roll < 0)  -> steer left (- delta)
        roll_excess = max(0.0, abs_roll - self.tilt_warn_rad)
        # Roll rate damping: if car is rolling faster outward, add damping; if rolling back inward, soften
        roll_rate_term = sign_roll * roll_rate
        delta_counter_raw = sign_roll * (kp_eff * roll_excess + self.k_d_roll * roll_rate_term)

        # Apply smoothstep weight
        delta_counter = w_tilt * delta_counter_raw
        steer_cmd = max(-self.max_steer, min(self.max_steer, target_steer_rad + delta_counter))

        # 3. Dynamic Mu-Aware Deceleration & Drive Alpha Attenuation
        # Smoothly attenuate rear drive power to prevent high torque from exacerbating roll
        alpha_rollover = max(0.0, min(1.0, 1.0 - w_tilt))

        # Deceleration budget strictly bounded by available tire grip (mu * g)
        max_decel_mps2 = self.decel_fraction * mu_clamped * 9.81
        v_fwd_pos = max(0.0, v_forward)
        # Target velocity ramps downward under roll proportional to blend weight
        v_arm_target = max(0.0, v_fwd_pos - (w_tilt * max_decel_mps2 * dt))

        # 4. Individual Closed-Loop FOC Airborne Wheel Clamping
        # Identify which side is lifting off the ground:
        # roll > 0: leaning right -> left wheels (FL, RL) are airborne
        # roll < 0: leaning left  -> right wheels (FR, RR) are airborne
        if sign_roll > 0:
            lifted_side = "LEFT"
        elif sign_roll < 0:
            lifted_side = "RIGHT"
        else:
            lifted_side = "NONE"

        # Drive factor for lifted wheels (clamped towards 0 drive so they ground-roll match without overspeed)
        lifted_alpha = max(0.0, min(1.0, 1.0 - w_tilt))

        return {
            "is_active": True,
            "tilt_detected": is_tilt_detected,
            "tilt_deg": round(tilt_deg, 2),
            "v_tilt_limit": round(v_tilt_limit, 4) if v_tilt_limit is not None else None,
            "tilt_scale_factor": round(scale_factor, 4),
            "w_tilt": round(w_tilt, 4),
            "delta_counter": round(delta_counter, 4),
            "delta_counter_deg": round(math.degrees(delta_counter), 2),
            "steer_cmd": steer_cmd,
            "alpha_rollover": round(alpha_rollover, 4),
            "v_arm_target": round(v_arm_target, 4),
            "lifted_side": lifted_side,
            "lifted_alpha": round(lifted_alpha, 4),
            "roll_deg": round(math.degrees(roll_rad), 2),
            "roll_rate": round(roll_rate, 3),
        }
