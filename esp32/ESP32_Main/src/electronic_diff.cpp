#include "electronic_diff.h"

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

namespace {
constexpr float kGyroYawRefTauS = 0.030f;       // 30 ms 1st-order tire/chassis yaw reference lag
constexpr float kGyroYawDotTauS = 0.020f;       // 20 ms 1st-order derivative low-pass filter
constexpr float kGyroDeadbandRadS = 0.20f;      // IMU yaw-rate noise deadband
constexpr float kGyroKpSteer = 0.08f;           // Proportional gyro countersteer gain (rad / (rad/s))
constexpr float kGyroKdSteer = 0.008f;          // Derivative yaw damper gain (rad / (rad/s^2))
constexpr float kGyroFullSpeedMps = 0.60f;      // Speed where gyro assist reaches 100% authority
constexpr float kSlipHoldTimeS = 0.060f;        // 60 ms hysteresis hold latch for slip telemetry/interventions
constexpr float kDiffFadeRate = 12.0f;          // Fast fade-out of rear diff split on oversteer (1/s)
constexpr float kDiffRiseRate = 4.0f;           // Smooth recovery of rear diff split (1/s)
constexpr float kUndersteerSpeedScale = 0.75f;  // Speed reduction ratio during front-plow understeer
constexpr float kOversteerSpeedScale = 0.85f;   // Speed reduction ratio during rear-axle oversteer
constexpr float kSlipSpeedFloorMps = 0.50f;     // Minimum cornering speed floor during slip throttle cut
} // namespace

ElectronicDifferential4WD::ElectronicDifferential4WD(
    float wheelbase,
    float track_rear,
    float track_front,
    float track_kingpin,
    float wheel_radius,
    float max_steer,
    float max_servo_rate,
    float ackermann_pct,
    float motor_dir,
    float v_max,
    float slip_gate,
    float thresh_spin
) : L_(wheelbase),
    W_r_(track_rear),
    W_f_(track_front),
    W_k_(track_kingpin),
    R_w_(wheel_radius),
    max_steer_rad_(max_steer),
    max_servo_rate_(max_servo_rate),
    ackermann_percent_(ackermann_pct),
    motor_dir_(motor_dir),
    v_max_wheel_(v_max),
    v_slip_gate_(slip_gate),
    thresh_spin_(thresh_spin),
    current_steer_rad_(0.0f),
    driver_steer_rad_(0.0f),
    rate_limited_speed_(0.0f),
    omega_ref_(0.0f),
    prev_gyro_z_(0.0f),
    gyro_z_dot_filt_(0.0f),
    has_prev_gyro_(false),
    slip_state_(SLIP_NORMAL),
    slip_hold_timer_(0.0f),
    diff_split_gain_(1.0f) {}

void ElectronicDifferential4WD::reset() {
    current_steer_rad_ = 0.0f;
    driver_steer_rad_ = 0.0f;
    rate_limited_speed_ = 0.0f;
    omega_ref_ = 0.0f;
    prev_gyro_z_ = 0.0f;
    gyro_z_dot_filt_ = 0.0f;
    has_prev_gyro_ = false;
    slip_state_ = SLIP_NORMAL;
    slip_hold_timer_ = 0.0f;
    diff_split_gain_ = 1.0f;
}

ElectronicDiffOutputs IRAM_ATTR ElectronicDifferential4WD::update(
    float target_v,
    float target_steer_rad,
    float gyro_z_rad_s,
    float dt,
    float mu,
    float v_curr
) {
    ElectronicDiffOutputs out;
    const float safe_dt = (dt > 1e-5f) ? dt : CONTROL_LOOP_DT_S;
    const float safe_mu = fmaxf(0.10f, mu);

    // -------------------------------------------------------------------------
    // 1. Driver Steering Intent & Grip-Capped Lagged Yaw-Rate Reference
    // -------------------------------------------------------------------------
    float target_delta = fmaxf(-max_steer_rad_, fminf(target_steer_rad, max_steer_rad_));
    float max_d_steer = max_servo_rate_ * safe_dt;

    float driver_diff = target_delta - driver_steer_rad_;
    if (fabsf(driver_diff) <= max_d_steer) {
        driver_steer_rad_ = target_delta;
    } else {
        driver_steer_rad_ += copysignf(max_d_steer, driver_diff);
    }
    driver_steer_rad_ = fmaxf(-max_steer_rad_, fminf(driver_steer_rad_, max_steer_rad_));
    out.driver_steer_rad = driver_steer_rad_;
    out.driver_steer_deg = driver_steer_rad_ * (180.0f / PI_F);

    float omega_cmd_driver = (rate_limited_speed_ * tanf(driver_steer_rad_)) / L_;
    float omega_grip_max = (safe_mu * GRAVITY_G_CONST) / fmaxf(fabsf(rate_limited_speed_), 0.50f);
    float omega_ref_target = fmaxf(-omega_grip_max, fminf(omega_grip_max, omega_cmd_driver));
    float alpha_ref = safe_dt / (kGyroYawRefTauS + safe_dt);
    omega_ref_ += alpha_ref * (omega_ref_target - omega_ref_);
    out.omega_ref = omega_ref_;

    // -------------------------------------------------------------------------
    // 2. Yaw-Rate Derivative Filter & Continuous PD Gyro-Assisted Steering
    // -------------------------------------------------------------------------
    float d_gyro_raw = 0.0f;
    if (has_prev_gyro_) {
        d_gyro_raw = (gyro_z_rad_s - prev_gyro_z_) / safe_dt;
    }
    prev_gyro_z_ = gyro_z_rad_s;
    has_prev_gyro_ = true;

    float alpha_d = safe_dt / (kGyroYawDotTauS + safe_dt);
    gyro_z_dot_filt_ += alpha_d * (d_gyro_raw - gyro_z_dot_filt_);

    // Continuous excess yaw error: zero throughout normal turn-in / understeer
    // [0 .. omega_ref_] so gyro never adds extra front lock during front plow.
    float excess_yaw = 0.0f;
    if (omega_ref_ > 1e-4f) {
        if (gyro_z_rad_s > omega_ref_) {
            excess_yaw = gyro_z_rad_s - omega_ref_;
        } else if (gyro_z_rad_s < 0.0f) {
            excess_yaw = gyro_z_rad_s;
        }
    } else if (omega_ref_ < -1e-4f) {
        if (gyro_z_rad_s < omega_ref_) {
            excess_yaw = gyro_z_rad_s - omega_ref_;
        } else if (gyro_z_rad_s > 0.0f) {
            excess_yaw = gyro_z_rad_s;
        }
    } else {
        excess_yaw = gyro_z_rad_s;
    }

    float speed_abs = fabsf(rate_limited_speed_);
    float gyro_fade = 0.0f;
    if (speed_abs > v_slip_gate_) {
        gyro_fade = fminf(1.0f, (speed_abs - v_slip_gate_) / fmaxf(0.05f, kGyroFullSpeedMps - v_slip_gate_));
    }
    float speed_sign = (rate_limited_speed_ >= 0.0f) ? 1.0f : -1.0f;

    float delta_gyro = 0.0f;
    if (gyro_fade > 0.0f && fabsf(excess_yaw) > kGyroDeadbandRadS) {
        float excess_db = copysignf(fabsf(excess_yaw) - kGyroDeadbandRadS, excess_yaw);
        float d_weight = fminf(1.0f, fabsf(excess_db) / 0.25f);
        float pd_raw = -(kGyroKpSteer * excess_db) - (d_weight * kGyroKdSteer * gyro_z_dot_filt_);
        if (excess_yaw > 0.0f) {
            pd_raw = fminf(0.0f, pd_raw);
        } else {
            pd_raw = fmaxf(0.0f, pd_raw);
        }
        delta_gyro = gyro_fade * speed_sign * pd_raw;
    }
    out.delta_gyro_rad = delta_gyro;

    // -------------------------------------------------------------------------
    // 3. Steering Slew Rate Limiter (Physical Servo Dynamics @ 22.0 rad/s)
    // -------------------------------------------------------------------------
    float stabilized_target = fmaxf(-max_steer_rad_, fminf(target_delta + delta_gyro, max_steer_rad_));
    float steer_diff = stabilized_target - current_steer_rad_;

    if (fabsf(steer_diff) <= max_d_steer) {
        current_steer_rad_ = stabilized_target;
    } else {
        current_steer_rad_ += copysignf(max_d_steer, steer_diff);
    }
    current_steer_rad_ = fmaxf(-max_steer_rad_, fminf(current_steer_rad_, max_steer_rad_));

    out.steer_rad = current_steer_rad_;
    out.steer_deg = current_steer_rad_ * (180.0f / PI_F);

    // Single Servo PWM calculation (333 Hz Digital Servo on GPIO 21)
    float norm_steer = fmaxf(-1.0f, fminf(1.0f, current_steer_rad_ / fmaxf(1e-4f, max_steer_rad_)));
    if (norm_steer >= 0.0f) {
        out.servo_pulse_us = SERVO_PULSE_NEUTRAL_US - norm_steer * (SERVO_PULSE_NEUTRAL_US - SERVO_PULSE_MIN_US);
    } else {
        out.servo_pulse_us = SERVO_PULSE_NEUTRAL_US + (-norm_steer) * (SERVO_PULSE_MAX_US - SERVO_PULSE_NEUTRAL_US);
    }
    out.servo_pulse_us = fmaxf(SERVO_PULSE_MIN_US, fminf(SERVO_PULSE_MAX_US, out.servo_pulse_us));

    // -------------------------------------------------------------------------
    // 4. Slip-Coupled Speed Cap & Dynamic Traction Rate Limiter
    // -------------------------------------------------------------------------
    float effective_target_v = target_v;
    if (slip_state_ == SLIP_UNDERSTEER) {
        float tan_driver = fmaxf(0.05f, fabsf(tanf(driver_steer_rad_)));
        float v_corner_max = sqrtf((safe_mu * GRAVITY_G_CONST * L_) / tan_driver);
        float v_cap = fmaxf(kSlipSpeedFloorMps, fminf(v_corner_max, kUndersteerSpeedScale * fabsf(rate_limited_speed_)));
        if (effective_target_v > 0.0f) {
            effective_target_v = fminf(effective_target_v, v_cap);
        } else if (effective_target_v < 0.0f) {
            effective_target_v = fmaxf(effective_target_v, -v_cap);
        }
    } else if (slip_state_ == SLIP_OVERSTEER) {
        float v_cap = fmaxf(kSlipSpeedFloorMps, kOversteerSpeedScale * fabsf(rate_limited_speed_));
        if (effective_target_v > 0.0f) {
            effective_target_v = fminf(effective_target_v, v_cap);
        } else if (effective_target_v < 0.0f) {
            effective_target_v = fmaxf(effective_target_v, -v_cap);
        }
    }

    float steer_mag = fabsf(current_steer_rad_);
    float lateral_fraction = fminf(0.80f, fabsf(sinf(steer_mag)));
    const float front_lat_reserve_min = 0.20f;
    float max_longitudinal_fraction = sqrtf(fmaxf(front_lat_reserve_min * front_lat_reserve_min, 1.0f - lateral_fraction * lateral_fraction));

    float a_limit = fminf(ACCEL_FWD_MAX, ACCEL_FWD_RATIO * safe_mu * GRAVITY_G_CONST);
    float d_limit = fminf(BRAKE_DECEL_MAX, BRAKE_DECEL_RATIO * safe_mu * GRAVITY_G_CONST * max_longitudinal_fraction);

    // Dynamic Slip Envelope referencing physical latency (~24ms) & peak tire slip
    const float dt_lag = 0.024f;
    float slip_ratio_accel = fminf(0.30f, fmaxf(0.08f, 0.10f * (a_limit / GRAVITY_G_CONST)));
    float max_slip_lead = (a_limit * dt_lag) + (slip_ratio_accel * fabsf(v_curr));

    // Harmonized Instant Wall Hit / Collision Resync
    float collision_threshold = max_slip_lead + 0.15f;
    if (rate_limited_speed_ >= 0.0f) {
        if (v_curr < (rate_limited_speed_ - collision_threshold)) {
            rate_limited_speed_ = v_curr;
        }
    } else {
        if (v_curr > (rate_limited_speed_ + collision_threshold)) {
            rate_limited_speed_ = v_curr;
        }
    }

    if (effective_target_v <= v_curr) {
        if (rate_limited_speed_ > v_curr) {
            rate_limited_speed_ = v_curr;
        }
    } else if (effective_target_v >= v_curr) {
        if (rate_limited_speed_ < v_curr) {
            rate_limited_speed_ = v_curr;
        }
    }

    float speed_err = effective_target_v - rate_limited_speed_;
    bool is_accel = (rate_limited_speed_ >= 0.0f && speed_err > 0.0f) || (rate_limited_speed_ <= 0.0f && speed_err < 0.0f);
    float max_dv = (is_accel ? a_limit : d_limit) * safe_dt;

    if (fabsf(speed_err) <= max_dv) {
        rate_limited_speed_ = effective_target_v;
    } else {
        rate_limited_speed_ += copysignf(max_dv, speed_err);
    }

    // Dynamic Traction Slip Envelope Clamping
    if (rate_limited_speed_ > 0.0f) {
        rate_limited_speed_ = fminf(rate_limited_speed_, fmaxf(0.0f, v_curr) + max_slip_lead);
    } else if (rate_limited_speed_ < 0.0f) {
        rate_limited_speed_ = fmaxf(rate_limited_speed_, fminf(0.0f, v_curr) - max_slip_lead);
    }
    out.rate_limited_v = rate_limited_speed_;

    // -------------------------------------------------------------------------
    // 5. Kinematic Yaw Rate & 45% Geometric Ackermann Knuckle Headings
    // -------------------------------------------------------------------------
    float sin_d = sinf(current_steer_rad_);
    float cos_d = cosf(current_steer_rad_);
    float tan_d = tanf(current_steer_rad_);
    out.omega_cmd = (rate_limited_speed_ * tan_d) / L_;

    if (fabsf(tan_d) > 1e-4f) {
        float half_Wk_sin = (W_k_ * 0.5f) * sin_d;
        float L_cos = L_ * cos_d;
        float L_sin = L_ * sin_d;
        float steer_left_true = atan2f(L_sin, L_cos - half_Wk_sin);
        float steer_right_true = atan2f(L_sin, L_cos + half_Wk_sin);
        out.steer_left_rad = fmaxf(-VEHICLE_MAX_SERVO_RAD, fminf(VEHICLE_MAX_SERVO_RAD, current_steer_rad_ + ackermann_percent_ * (steer_left_true - current_steer_rad_)));
        out.steer_right_rad = fmaxf(-VEHICLE_MAX_SERVO_RAD, fminf(VEHICLE_MAX_SERVO_RAD, current_steer_rad_ + ackermann_percent_ * (steer_right_true - current_steer_rad_)));
    } else {
        out.steer_left_rad = current_steer_rad_;
        out.steer_right_rad = current_steer_rad_;
    }

    // -------------------------------------------------------------------------
    // 6. Standstill Guard
    // -------------------------------------------------------------------------
    if (fabsf(rate_limited_speed_) < 0.001f) {
        omega_ref_ = 0.0f;
        slip_state_ = SLIP_NORMAL;
        slip_hold_timer_ = 0.0f;
        diff_split_gain_ = fminf(1.0f, diff_split_gain_ + kDiffRiseRate * safe_dt);
        out.omega_ref = 0.0f;
        out.v_FL = out.v_FR = out.v_RL = out.v_RR = 0.0f;
        out.omega_FL = out.omega_FR = out.omega_RL = out.omega_RR = 0.0f;
        out.rpm_FL = out.rpm_FR = out.rpm_RL = out.rpm_RR = 0.0f;
        out.scale_factor = 1.0f;
        out.slip_state = SLIP_NORMAL;
        out.yaw_error = 0.0f;
        return out;
    }

    // -------------------------------------------------------------------------
    // 7. Grip-Capped, Hysteretic IMU Slip Classification
    // -------------------------------------------------------------------------
    out.yaw_error = gyro_z_rad_s - omega_ref_;
    VehicleSlipState raw_slip = SLIP_NORMAL;

    if (fabsf(rate_limited_speed_) >= v_slip_gate_) {
        float dyn_thresh = fmaxf(thresh_spin_, 0.35f * fabsf(omega_ref_));

        if (omega_ref_ > 0.05f && (gyro_z_rad_s - omega_ref_) > dyn_thresh) {
            raw_slip = SLIP_OVERSTEER;
        } else if (omega_ref_ < -0.05f && (omega_ref_ - gyro_z_rad_s) > dyn_thresh) {
            raw_slip = SLIP_OVERSTEER;
        } else if (fabsf(gyro_z_rad_s) > fmaxf(1.00f, thresh_spin_ * 2.0f) && (gyro_z_rad_s * omega_ref_ <= 0.0f)) {
            raw_slip = SLIP_OVERSTEER;
        } else if (fabsf(driver_steer_rad_) > 0.15f && fabsf(omega_ref_) > 1.00f &&
                   (gyro_z_rad_s * omega_ref_ >= 0.0f) && fabsf(gyro_z_rad_s) < (0.40f * fabsf(omega_ref_))) {
            raw_slip = SLIP_UNDERSTEER;
        }
    }

    if (fabsf(rate_limited_speed_) < v_slip_gate_) {
        slip_state_ = SLIP_NORMAL;
        slip_hold_timer_ = 0.0f;
    } else if (raw_slip != SLIP_NORMAL) {
        slip_state_ = raw_slip;
        slip_hold_timer_ = kSlipHoldTimeS;
    } else if (slip_hold_timer_ > 0.0f) {
        slip_hold_timer_ = fmaxf(0.0f, slip_hold_timer_ - safe_dt);
    } else {
        slip_state_ = SLIP_NORMAL;
    }
    out.slip_state = slip_state_;

    // Fade out rear differential speed split during oversteer so outer rear wheel
    // does not keep feeding yaw torque into the spin.
    float target_diff_gain = (slip_state_ == SLIP_OVERSTEER) ? 0.0f : 1.0f;
    if (target_diff_gain < diff_split_gain_) {
        diff_split_gain_ = fmaxf(target_diff_gain, diff_split_gain_ - kDiffFadeRate * safe_dt);
    } else {
        diff_split_gain_ = fminf(target_diff_gain, diff_split_gain_ + kDiffRiseRate * safe_dt);
    }

    // -------------------------------------------------------------------------
    // 8. 4-Wheel Speed Allocation (Forward & Reverse)
    // -------------------------------------------------------------------------
    float half_Wr_omega_rear = out.omega_cmd * (W_r_ * 0.5f) * diff_split_gain_;
    float half_Wf_omega = out.omega_cmd * (W_f_ * 0.5f);

    // Rear axle (differential split fades smoothly toward 0 on oversteer)
    out.v_RL = rate_limited_speed_ - half_Wr_omega_rear;
    out.v_RR = rate_limited_speed_ + half_Wr_omega_rear;

    // Front axle (vector projection along steered knuckle angles)
    float fl_x = rate_limited_speed_ - half_Wf_omega;
    float fr_x = rate_limited_speed_ + half_Wf_omega;
    float front_y = L_ * out.omega_cmd;

    out.v_FL = fl_x * cosf(out.steer_left_rad) + front_y * sinf(out.steer_left_rad);
    out.v_FR = fr_x * cosf(out.steer_right_rad) + front_y * sinf(out.steer_right_rad);

    // -------------------------------------------------------------------------
    // 9. Proportional Saturation Scaling (Preserve turning radius at limit)
    // -------------------------------------------------------------------------
    float v_peak = fmaxf(fmaxf(fabsf(out.v_FL), fabsf(out.v_FR)),
                         fmaxf(fabsf(out.v_RL), fabsf(out.v_RR)));
    out.scale_factor = 1.0f;
    if (v_peak > v_max_wheel_) {
        out.scale_factor = v_max_wheel_ / v_peak;
        out.v_FL *= out.scale_factor;
        out.v_FR *= out.scale_factor;
        out.v_RL *= out.scale_factor;
        out.v_RR *= out.scale_factor;
    }

    // -------------------------------------------------------------------------
    // 10. Convert Linear Velocities to Actuator Rotational Velocities & RPM
    // -------------------------------------------------------------------------
    float max_omega = (v_max_wheel_ / R_w_);
    out.omega_FL = fmaxf(-max_omega, fminf(max_omega, (out.v_FL / R_w_) * motor_dir_));
    out.omega_FR = fmaxf(-max_omega, fminf(max_omega, (out.v_FR / R_w_) * motor_dir_));
    out.omega_RL = fmaxf(-max_omega, fminf(max_omega, (out.v_RL / R_w_) * motor_dir_));
    out.omega_RR = fmaxf(-max_omega, fminf(max_omega, (out.v_RR / R_w_) * motor_dir_));

    const float rad_s_to_rpm = 60.0f / (2.0f * PI_F);
    out.rpm_FL = out.omega_FL * rad_s_to_rpm;
    out.rpm_FR = out.omega_FR * rad_s_to_rpm;
    out.rpm_RL = out.omega_RL * rad_s_to_rpm;
    out.rpm_RR = out.omega_RR * rad_s_to_rpm;

    return out;
}

