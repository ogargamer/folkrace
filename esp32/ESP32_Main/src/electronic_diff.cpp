#include "electronic_diff.h"

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

namespace {
constexpr float kGyroYawRefTauS = 0.055f;         // 55 ms chassis + servo yaw-rate buildup lag
constexpr float kGyroYawRefUnwindTauS = 0.020f;   // 20 ms fast unwind when straightening out
constexpr float kGyroYawDotTauS = 0.032f;         // 32 ms 1st-order derivative low-pass filter
constexpr float kGyroDeadbandRadS = 0.28f;        // IMU yaw-rate noise deadband
constexpr float kGyroKpSteer = 0.105f;            // Proportional gyro countersteer gain (rad / (rad/s))
constexpr float kGyroKdSteer = 0.004f;            // Derivative yaw damper gain (rad / (rad/s^2))
constexpr float kGyroKdRecoveryScale = 0.15f;     // Attenuate D-term during spin recovery so it won't unwind countersteer
constexpr float kGyroMaxRateRadS = 12.0f;         // Countersteer slew cap (rad/s) to reject landing/shock spikes
constexpr float kGyroFullSpeedMps = 0.60f;        // Speed where gyro assist reaches 100% authority
constexpr float kUndersteerConfirmTimeS = 0.065f; // Require 65 ms sustained plow before throttling
constexpr float kOversteerConfirmTimeS = 0.035f;  // Require 35 ms sustained breakaway before latching
constexpr float kSlipHoldTimeS = 0.035f;          // 35 ms hysteresis hold latch
constexpr float kDiffFadeRate = 12.0f;            // Fast fade-out of rear diff split on oversteer (1/s)
constexpr float kDiffRiseRate = 4.0f;             // Smooth recovery of rear diff split (1/s)
constexpr float kUndersteerSpeedScale = 0.85f;    // Fixed cap fraction of entry speed on sustained understeer
constexpr float kOversteerSpeedScale = 0.85f;     // 15% throttle lift on oversteer (fixed entry cap, no slam braking)
constexpr float kOversteerAccelScale = 0.25f;     // Cut forward acceleration torque during oversteer
constexpr float kOversteerDecelScale = 0.60f;     // Keep oversteer decel smooth to avoid rear weight transfer
constexpr float kUndersteerAccelScale = 0.30f;    // Cut forward acceleration torque during front plow
constexpr float kSlipSpeedFloorMps = 0.60f;       // Minimum cornering speed floor during slip throttle cap

// Proactive Traction Prevention Constants
constexpr float kProactiveCornerMuScale = 0.94f;  // Feedforward corner speed mu scale (~97% apex speed, ~34% combined-slip reserve)
constexpr float kSteerGripMargin = 1.25f;         // Extra grip margin on speed-based steer cap (allows grippy tires up to ~1.25x mu)
constexpr float kPeakFrontSlipRad = 0.16f;        // ~9.2 deg peak front slip angle allowance above kinematic Ackermann
constexpr float kSteerSlewVLowMps = 1.20f;        // Speed below which turn-in uses full servo slew rate
constexpr float kSteerSlewVHighMps = 3.80f;       // Speed where turn-in slew reaches minimum taper
constexpr float kSteerSlewMinScale = 0.45f;       // Minimum turn-in servo slew scale at high speed (unwind stays 1.0x)
constexpr float kAccelMinLongFraction = 0.30f;    // Minimum longitudinal accel fraction in hard cornering
constexpr float kBrakeMinLongFraction = 0.50f;    // Minimum longitudinal braking fraction in hard cornering
constexpr float kGeomSteerLatWeight = 0.35f;      // Baseline geometric steer utilization weight
constexpr float kEarlyUsYawRatioHigh = 0.72f;     // Yaw ratio where incipient understeer modulation starts
constexpr float kEarlyUsYawRatioLow = 0.45f;      // Yaw ratio where incipient understeer modulation saturates (before 0.40 latch)
constexpr float kEarlyUsFiltTauS = 0.030f;        // 30 ms LPF on incipient understeer & observed lateral accel
constexpr float kEarlyUsMinSpeedMps = 0.65f;      // Minimum speed to evaluate incipient front saturation
constexpr float kEarlyUsMinSteerRad = 0.12f;      // Minimum steering angle to evaluate incipient front saturation
constexpr float kEarlyUsMinOmegaRef = 1.00f;      // Minimum reference yaw rate for incipient front saturation
constexpr float kEarlyUsAccelCut = 0.55f;         // Max proactive accel cut (55%) at full incipient understeer
constexpr float kEarlyUsSpeedFloorFrac = 0.70f;   // Floor on observed-grip corner cap relative to nominal v_proactive
constexpr float kEarlyOsStartRadS = 0.35f;        // Excess yaw (with lead) where incipient oversteer starts
constexpr float kEarlyOsFullRadS = 0.90f;         // Excess yaw (with lead) where incipient oversteer reaches 1.0
constexpr float kEarlyOsYawDotLeadS = 0.025f;     // 25 ms yaw-acceleration lead for snap breakaway
constexpr float kEarlyOsRiseTauS = 0.007f;        // 7 ms rise LPF to reject single-tick kerb/chatter spikes
constexpr float kEarlyOsDecayTauS = 0.025f;       // 25 ms smooth decay when incipient oversteer subsides
constexpr float kEarlyOsAccelCut = 0.65f;         // Max proactive accel cut (65%) at full incipient oversteer
constexpr float kEarlyOsDecelCut = 0.35f;         // Max proactive decel softening (35%) to prevent rear lift-off snap
constexpr float kSlipRatioAccelMax = 0.15f;       // Peak tire longitudinal slip ratio cap (15%)
constexpr float kSlipRatioMinCorner = 0.04f;      // Tightened cornering longitudinal slip ratio floor (4%)
constexpr float kMinSlipLeadMps = 0.12f;          // Absolute slip lead floor (m/s) for crisp low-speed launches & exits
constexpr float kAbsMinSpeedMps = 0.30f;          // Minimum speed for ABS wheel-lock floor
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
    prev_delta_gyro_rad_(0.0f),
    rate_limited_speed_(0.0f),
    omega_ref_(0.0f),
    prev_gyro_z_(0.0f),
    gyro_z_dot_filt_(0.0f),
    has_prev_gyro_(false),
    slip_state_(SLIP_NORMAL),
    slip_hold_timer_(0.0f),
    understeer_confirm_timer_(0.0f),
    oversteer_confirm_timer_(0.0f),
    slip_entry_speed_(0.0f),
    diff_split_gain_(1.0f),
    early_us_filt_(0.0f),
    early_os_(0.0f),
    a_lat_obs_filt_(0.0f) {}

void ElectronicDifferential4WD::reset() {
    current_steer_rad_ = 0.0f;
    driver_steer_rad_ = 0.0f;
    prev_delta_gyro_rad_ = 0.0f;
    rate_limited_speed_ = 0.0f;
    omega_ref_ = 0.0f;
    prev_gyro_z_ = 0.0f;
    gyro_z_dot_filt_ = 0.0f;
    has_prev_gyro_ = false;
    slip_state_ = SLIP_NORMAL;
    slip_hold_timer_ = 0.0f;
    understeer_confirm_timer_ = 0.0f;
    oversteer_confirm_timer_ = 0.0f;
    slip_entry_speed_ = 0.0f;
    diff_split_gain_ = 1.0f;
    early_us_filt_ = 0.0f;
    early_os_ = 0.0f;
    a_lat_obs_filt_ = 0.0f;
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
    // 1. Driver Steering Intent (Speed-Capped & Asymmetric Slew) & Yaw Reference
    // -------------------------------------------------------------------------
    float target_delta = fmaxf(-max_steer_rad_, fminf(target_steer_rad, max_steer_rad_));
    float max_d_steer = max_servo_rate_ * safe_dt;

    float v_for_steer = fmaxf(fabsf(v_curr), fabsf(rate_limited_speed_));
    float steer_cap_rad = max_steer_rad_;
    if (v_for_steer > 0.50f) {
        float delta_grip = atanf((kSteerGripMargin * safe_mu * GRAVITY_G_CONST * L_) / (v_for_steer * v_for_steer));
        steer_cap_rad = fminf(max_steer_rad_, delta_grip + kPeakFrontSlipRad);
    }
    float capped_target_delta = fmaxf(-steer_cap_rad, fminf(target_delta, steer_cap_rad));

    bool is_steer_unwinding = (fabsf(capped_target_delta) < fabsf(driver_steer_rad_)) ||
                              (capped_target_delta * driver_steer_rad_ < 0.0f);
    float slew_scale = 1.0f;
    if (!is_steer_unwinding && v_for_steer > kSteerSlewVLowMps) {
        float t = fminf(1.0f, (v_for_steer - kSteerSlewVLowMps) / (kSteerSlewVHighMps - kSteerSlewVLowMps));
        slew_scale = 1.0f - t * (1.0f - kSteerSlewMinScale);
    }
    float max_d_driver = max_servo_rate_ * slew_scale * safe_dt;

    float driver_diff = capped_target_delta - driver_steer_rad_;
    if (fabsf(driver_diff) <= max_d_driver) {
        driver_steer_rad_ = capped_target_delta;
    } else {
        driver_steer_rad_ += copysignf(max_d_driver, driver_diff);
    }
    driver_steer_rad_ = fmaxf(-max_steer_rad_, fminf(driver_steer_rad_, max_steer_rad_));
    out.driver_steer_rad = driver_steer_rad_;
    out.driver_steer_deg = driver_steer_rad_ * (180.0f / PI_F);

    float v_for_ref = rate_limited_speed_;
    if (fabsf(v_curr) > fabsf(rate_limited_speed_) && (v_curr * rate_limited_speed_ >= 0.0f)) {
        v_for_ref = copysignf(fabsf(v_curr), rate_limited_speed_);
    }
    float omega_cmd_driver = (v_for_ref * tanf(driver_steer_rad_)) / L_;
    float omega_grip_max = (safe_mu * GRAVITY_G_CONST) / fmaxf(fabsf(v_for_ref), 0.50f);
    float omega_ref_target = fmaxf(-omega_grip_max, fminf(omega_grip_max, omega_cmd_driver));
    bool is_unwinding = (fabsf(omega_ref_target) < fabsf(omega_ref_)) || (omega_ref_target * omega_ref_ < 0.0f);
    float tau_ref = is_unwinding ? kGyroYawRefUnwindTauS : kGyroYawRefTauS;
    float alpha_ref = safe_dt / (tau_ref + safe_dt);
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

    float speed_abs = fabsf(v_for_ref);
    float gyro_fade = 0.0f;
    if (speed_abs > v_slip_gate_) {
        gyro_fade = fminf(1.0f, (speed_abs - v_slip_gate_) / fmaxf(0.05f, kGyroFullSpeedMps - v_slip_gate_));
    }
    float speed_sign = (v_for_ref >= 0.0f) ? 1.0f : -1.0f;

    float raw_delta_gyro = 0.0f;
    if (gyro_fade > 0.0f && fabsf(excess_yaw) > kGyroDeadbandRadS) {
        float excess_db = copysignf(fabsf(excess_yaw) - kGyroDeadbandRadS, excess_yaw);
        float d_weight = fminf(1.0f, fabsf(excess_db) / 0.25f);
        // Only apply full derivative damping while spin is diverging; attenuate during recovery
        // so D-term does not unwind countersteer before yaw rate returns to normal.
        float kd_scale = (excess_yaw * gyro_z_dot_filt_ >= 0.0f) ? 1.0f : kGyroKdRecoveryScale;
        float pd_raw = -(kGyroKpSteer * excess_db) - (d_weight * kd_scale * kGyroKdSteer * gyro_z_dot_filt_);
        if (excess_yaw > 0.0f) {
            pd_raw = fminf(0.0f, pd_raw);
        } else {
            pd_raw = fmaxf(0.0f, pd_raw);
        }
        raw_delta_gyro = gyro_fade * speed_sign * pd_raw;
    }

    float max_d_gyro = kGyroMaxRateRadS * safe_dt;
    float gyro_diff = raw_delta_gyro - prev_delta_gyro_rad_;
    if (fabsf(gyro_diff) <= max_d_gyro) {
        prev_delta_gyro_rad_ = raw_delta_gyro;
    } else {
        prev_delta_gyro_rad_ += copysignf(max_d_gyro, gyro_diff);
    }
    float delta_gyro = prev_delta_gyro_rad_;
    out.delta_gyro_rad = delta_gyro;

    // -------------------------------------------------------------------------
    // 2b. Model-Free Incipient Tire-Saturation Modulation (Early US / Early OS)
    // -------------------------------------------------------------------------
    float raw_early_os = 0.0f;
    if (fabsf(v_for_ref) >= v_slip_gate_ && fabsf(excess_yaw) > kGyroDeadbandRadS) {
        float yaw_dot_lead = (excess_yaw * gyro_z_dot_filt_ > 0.0f)
                                 ? (kEarlyOsYawDotLeadS * fabsf(gyro_z_dot_filt_))
                                 : 0.0f;
        float excess_lead = fabsf(excess_yaw) + yaw_dot_lead;
        raw_early_os = fmaxf(0.0f, fminf(1.0f, (excess_lead - kEarlyOsStartRadS) /
                                               (kEarlyOsFullRadS - kEarlyOsStartRadS)));
    }
    float tau_os = (raw_early_os >= early_os_) ? kEarlyOsRiseTauS : kEarlyOsDecayTauS;
    float alpha_os = safe_dt / (tau_os + safe_dt);
    early_os_ += alpha_os * (raw_early_os - early_os_);
    if (early_os_ < 0.005f) {
        early_os_ = 0.0f;
    }

    float a_lat_meas_now = fabsf(v_curr * gyro_z_rad_s);
    float alpha_us = safe_dt / (kEarlyUsFiltTauS + safe_dt);
    a_lat_obs_filt_ += alpha_us * (a_lat_meas_now - a_lat_obs_filt_);

    float raw_early_us = 0.0f;
    float a_lat_req_driver = fabsf(v_for_ref * omega_cmd_driver);
    if (fabsf(v_curr) >= kEarlyUsMinSpeedMps &&
        fabsf(driver_steer_rad_) > kEarlyUsMinSteerRad &&
        fabsf(omega_ref_) > kEarlyUsMinOmegaRef &&
        a_lat_req_driver > (0.35f * safe_mu * GRAVITY_G_CONST) &&
        (gyro_z_rad_s * omega_ref_ >= 0.0f)) {
        float yaw_ratio = fabsf(gyro_z_rad_s) / fmaxf(1e-4f, fabsf(omega_ref_));
        if (yaw_ratio < kEarlyUsYawRatioHigh) {
            raw_early_us = fmaxf(0.0f, fminf(1.0f, (kEarlyUsYawRatioHigh - yaw_ratio) /
                                                   (kEarlyUsYawRatioHigh - kEarlyUsYawRatioLow)));
        }
    }
    early_us_filt_ += alpha_us * (raw_early_us - early_us_filt_);
    if (early_us_filt_ < 0.005f) {
        early_us_filt_ = 0.0f;
    }

    // -------------------------------------------------------------------------
    // 3. Steering Slew Rate Limiter (Physical Servo Dynamics @ 22.0 rad/s)
    // -------------------------------------------------------------------------
    float stabilized_target = fmaxf(-max_steer_rad_, fminf(driver_steer_rad_ + delta_gyro, max_steer_rad_));
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
    // 4. Proactive + Slip-Coupled Speed Cap & Combined-Slip Friction Circle
    // -------------------------------------------------------------------------
    float effective_target_v = target_v;
    float steer_for_cap = fmaxf(fabsf(driver_steer_rad_), fabsf(capped_target_delta));
    float tan_steer_cap = fabsf(tanf(steer_for_cap));
    float v_cap_total = 1e6f;

    if (tan_steer_cap > 0.05f) {
        float v_proactive = sqrtf((kProactiveCornerMuScale * safe_mu * GRAVITY_G_CONST * L_) / tan_steer_cap);
        if (early_us_filt_ > 0.0f) {
            float v_floor = fmaxf(kSlipSpeedFloorMps, kEarlyUsSpeedFloorFrac * v_proactive);
            float v_obs_raw = sqrtf((fmaxf(0.50f, a_lat_obs_filt_) * L_) / tan_steer_cap);
            float v_obs_grip = fminf(v_proactive, fmaxf(v_floor, v_obs_raw));
            v_proactive += early_us_filt_ * (v_obs_grip - v_proactive);
        }
        v_cap_total = fmaxf(kSlipSpeedFloorMps, v_proactive);
    }

    if (slip_state_ == SLIP_UNDERSTEER) {
        float tan_driver = fmaxf(0.05f, fabsf(tanf(driver_steer_rad_)));
        float v_corner_max = sqrtf((safe_mu * GRAVITY_G_CONST * L_) / tan_driver);
        float v_us_cap = fmaxf(kSlipSpeedFloorMps, fminf(v_corner_max, kUndersteerSpeedScale * slip_entry_speed_));
        v_cap_total = fminf(v_cap_total, v_us_cap);
    } else if (slip_state_ == SLIP_OVERSTEER) {
        float v_os_cap = fmaxf(kSlipSpeedFloorMps, kOversteerSpeedScale * slip_entry_speed_);
        v_cap_total = fminf(v_cap_total, v_os_cap);
    }

    if (v_cap_total < 1e5f) {
        if (effective_target_v > 0.0f) {
            effective_target_v = fminf(effective_target_v, v_cap_total);
        } else if (effective_target_v < 0.0f) {
            effective_target_v = fmaxf(effective_target_v, -v_cap_total);
        }
    }

    // Combined-slip friction circle using real lateral utilization (measured & predicted)
    float v_ref_lat = fmaxf(fabsf(v_curr), fabsf(rate_limited_speed_));
    float a_lat_meas = fabsf(v_curr * gyro_z_rad_s);
    float a_lat_pred = fabsf(v_ref_lat * omega_cmd_driver);
    float a_lat_eff = fmaxf(a_lat_meas, a_lat_pred);
    float mu_g = safe_mu * GRAVITY_G_CONST;
    float lat_util_dyn = a_lat_eff / fmaxf(0.50f, mu_g);
    float lat_util_geom = kGeomSteerLatWeight * fabsf(sinf(current_steer_rad_));
    float lat_util = fminf(0.96f, fmaxf(lat_util_dyn, lat_util_geom));
    float raw_long_frac = sqrtf(fmaxf(0.0f, 1.0f - lat_util * lat_util));
    float accel_long_frac = fmaxf(kAccelMinLongFraction, raw_long_frac);
    float brake_long_frac = fmaxf(kBrakeMinLongFraction, raw_long_frac);

    float a_limit_base = fminf(ACCEL_FWD_MAX, ACCEL_FWD_RATIO * mu_g);
    float d_limit_base = fminf(BRAKE_DECEL_MAX, BRAKE_DECEL_RATIO * mu_g);
    float a_limit = a_limit_base * accel_long_frac;
    float d_limit = d_limit_base * brake_long_frac;

    float proactive_accel_scale = fminf(1.0f - kEarlyUsAccelCut * early_us_filt_,
                                        1.0f - kEarlyOsAccelCut * early_os_);
    float proactive_decel_scale = 1.0f - kEarlyOsDecelCut * early_os_;
    float reactive_accel_scale = 1.0f;
    float reactive_decel_scale = 1.0f;
    if (slip_state_ == SLIP_OVERSTEER) {
        reactive_accel_scale = kOversteerAccelScale;
        reactive_decel_scale = kOversteerDecelScale;
    } else if (slip_state_ == SLIP_UNDERSTEER) {
        reactive_accel_scale = kUndersteerAccelScale;
    }
    float effective_a_limit = a_limit * fminf(proactive_accel_scale, reactive_accel_scale);
    float effective_d_limit = d_limit * fminf(proactive_decel_scale, reactive_decel_scale);

    // Dynamic Slip Envelope referencing physical latency (~24ms) & peak tire slip
    const float dt_lag = 0.024f;
    float slip_ratio_base = fminf(kSlipRatioAccelMax, fmaxf(0.08f, 0.10f * (a_limit_base / GRAVITY_G_CONST)));
    float slip_ratio_accel = fmaxf(kSlipRatioMinCorner, slip_ratio_base * accel_long_frac);
    float slip_ratio_brake = fmaxf(kSlipRatioMinCorner, slip_ratio_base * brake_long_frac);
    float max_slip_lead = fmaxf(kMinSlipLeadMps, (effective_a_limit * dt_lag) + (slip_ratio_accel * fabsf(v_curr)));
    float max_brake_slip_lag = (effective_d_limit * dt_lag) + (slip_ratio_brake * fabsf(v_curr));
    float nominal_slip_lead = (a_limit_base * dt_lag) + (slip_ratio_base * fabsf(v_curr));

    // Harmonized Instant Wall Hit / Collision Resync
    float collision_threshold = nominal_slip_lead + 0.15f;
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
    float max_dv = (is_accel ? effective_a_limit : effective_d_limit) * safe_dt;

    if (fabsf(speed_err) <= max_dv) {
        rate_limited_speed_ = effective_target_v;
    } else {
        rate_limited_speed_ += copysignf(max_dv, speed_err);
    }

    // Dynamic Traction Slip Envelope Clamping (Accel TCS + Brake ABS Floor)
    if (rate_limited_speed_ > 0.0f) {
        rate_limited_speed_ = fminf(rate_limited_speed_, fmaxf(0.0f, v_curr) + max_slip_lead);
        if (v_curr > kAbsMinSpeedMps) {
            rate_limited_speed_ = fmaxf(rate_limited_speed_, v_curr - max_brake_slip_lag);
        }
    } else if (rate_limited_speed_ < 0.0f) {
        rate_limited_speed_ = fmaxf(rate_limited_speed_, fminf(0.0f, v_curr) - max_slip_lead);
        if (v_curr < -kAbsMinSpeedMps) {
            rate_limited_speed_ = fminf(rate_limited_speed_, v_curr + max_brake_slip_lag);
        }
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
        prev_delta_gyro_rad_ = 0.0f;
        slip_state_ = SLIP_NORMAL;
        slip_hold_timer_ = 0.0f;
        understeer_confirm_timer_ = 0.0f;
        oversteer_confirm_timer_ = 0.0f;
        slip_entry_speed_ = 0.0f;
        early_us_filt_ = 0.0f;
        early_os_ = 0.0f;
        a_lat_obs_filt_ = 0.0f;
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
    // 7. Grip-Capped, Confirmed & Hysteretic IMU Slip Classification
    // -------------------------------------------------------------------------
    out.yaw_error = gyro_z_rad_s - omega_ref_;
    VehicleSlipState raw_slip = SLIP_NORMAL;

    if (fabsf(v_for_ref) >= v_slip_gate_) {
        float dyn_thresh = fmaxf(fmaxf(thresh_spin_ * 2.2f, 0.95f), 0.40f * fabsf(omega_ref_));
        float a_lat_req = fabsf(v_for_ref * omega_cmd_driver);

        if (omega_ref_ > 0.10f && (gyro_z_rad_s - omega_ref_) > dyn_thresh) {
            raw_slip = SLIP_OVERSTEER;
        } else if (omega_ref_ < -0.10f && (omega_ref_ - gyro_z_rad_s) > dyn_thresh) {
            raw_slip = SLIP_OVERSTEER;
        } else if (fabsf(gyro_z_rad_s) > fmaxf(1.35f, thresh_spin_ * 3.0f) && (gyro_z_rad_s * omega_ref_ <= 0.0f)) {
            raw_slip = SLIP_OVERSTEER;
        } else if (fabsf(driver_steer_rad_) > 0.17f && fabsf(omega_ref_) > 1.25f &&
                   a_lat_req > (0.50f * safe_mu * GRAVITY_G_CONST) &&
                   (gyro_z_rad_s * omega_ref_ >= 0.0f) && fabsf(gyro_z_rad_s) < (0.40f * fabsf(omega_ref_))) {
            raw_slip = SLIP_UNDERSTEER;
        }
    }

    VehicleSlipState prev_slip_state = slip_state_;
    if (fabsf(v_for_ref) < v_slip_gate_) {
        slip_state_ = SLIP_NORMAL;
        slip_hold_timer_ = 0.0f;
        understeer_confirm_timer_ = 0.0f;
        oversteer_confirm_timer_ = 0.0f;
        slip_entry_speed_ = 0.0f;
    } else if (raw_slip == SLIP_OVERSTEER) {
        oversteer_confirm_timer_ += safe_dt;
        understeer_confirm_timer_ = 0.0f;
        if (oversteer_confirm_timer_ >= kOversteerConfirmTimeS) {
            slip_state_ = SLIP_OVERSTEER;
            slip_hold_timer_ = kSlipHoldTimeS;
        }
    } else if (raw_slip == SLIP_UNDERSTEER) {
        understeer_confirm_timer_ += safe_dt;
        oversteer_confirm_timer_ = 0.0f;
        if (understeer_confirm_timer_ >= kUndersteerConfirmTimeS) {
            slip_state_ = SLIP_UNDERSTEER;
            slip_hold_timer_ = kSlipHoldTimeS;
        }
    } else {
        understeer_confirm_timer_ = 0.0f;
        oversteer_confirm_timer_ = 0.0f;
        if (slip_hold_timer_ > 0.0f) {
            slip_hold_timer_ = fmaxf(0.0f, slip_hold_timer_ - safe_dt);
        } else {
            slip_state_ = SLIP_NORMAL;
            slip_entry_speed_ = 0.0f;
        }
    }

    if (slip_state_ != SLIP_NORMAL && prev_slip_state != slip_state_) {
        slip_entry_speed_ = fmaxf(fabsf(rate_limited_speed_), fabsf(v_curr));
    }
    out.slip_state = slip_state_;

    // Fade out rear differential speed split proactively on incipient oversteer
    // and completely on confirmed oversteer so outer rear wheel does not feed spin.
    float target_diff_gain = (slip_state_ == SLIP_OVERSTEER) ? 0.0f : fmaxf(0.0f, 1.0f - early_os_);
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

