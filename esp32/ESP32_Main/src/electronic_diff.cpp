#include "electronic_diff.h"

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

namespace {
// Yaw reference & gyro assist
constexpr float kGyroYawRefTauS = 0.055f;         // 55 ms chassis + servo yaw-rate buildup lag
constexpr float kGyroYawRefUnwindTauS = 0.020f;   // 20 ms fast unwind when straightening out
constexpr float kTvYawRefTauS = 0.012f;           // 12 ms short-lag uncapped yaw target for Rear TV
constexpr float kYawAccelLpfTauS = 0.00531f;      // ~30 Hz 1st-order LPF on clock-beat yaw accel
constexpr float kGyroDeadbandRadS = 0.28f;        // IMU yaw-rate noise deadband for servo countersteer
constexpr float kGyroKpSteer = 0.105f;            // Proportional gyro countersteer gain (rad / (rad/s))
constexpr float kGyroKdSteer = 0.004f;            // Derivative yaw damper gain (rad / (rad/s^2))
constexpr float kGyroKdRecoveryScale = 0.15f;     // Attenuate D-term during spin recovery so it won't unwind countersteer
constexpr float kGyroMaxRateRadS = 12.0f;         // Countersteer slew cap (rad/s) to reject landing/shock spikes
constexpr float kGyroFullSpeedMps = 0.60f;        // Speed where gyro assist reaches 100% authority

// Reactive slip telemetry latch timers (used for telemetry diagnostics)
constexpr float kUndersteerConfirmTimeS = 0.065f;
constexpr float kOversteerConfirmTimeS = 0.035f;
constexpr float kSlipHoldTimeS = 0.035f;

// Steering cap & asymmetric turn-in slew
constexpr float kSteerGripMargin = 1.25f;         // Extra grip margin on speed-based steer cap
constexpr float kPeakFrontSlipRad = 0.16f;        // ~9.2 deg peak front slip angle allowance
constexpr float kSteerSlewVLowMps = 1.20f;        // Speed below which turn-in uses full servo slew rate
constexpr float kSteerSlewVHighMps = 3.80f;       // Speed where turn-in slew reaches minimum taper
constexpr float kSteerSlewMinScale = 0.70f;       // Minimum turn-in servo slew scale at high speed (unwind stays 1.0x)

// Torque-envelope TCS, SVPWM headroom & Feedforward gains (GM3506 @ 12V, 5.6 Ohm)
constexpr float kVtrqFullV = 6.93f;               // Linear SVPWM phase-peak voltage headroom (12V / sqrt(3))
constexpr float kEmergencyCornerMu = 1.30f;       // High-mu emergency sanity ceiling (above Shore A22 grip)
constexpr float kEnvAccelMaxMps2 = 11.28f;        // ~1.15 g max longitudinal accel ramp
constexpr float kEnvDecelMaxMps2 = 11.28f;        // ~1.15 g max longitudinal braking ramp
constexpr float kFfAccelMaxMps2 = 7.50f;          // ~0.76 g accel FF scale ceiling (~4.58 V launch)
constexpr float kFfDecelMaxMps2 = 7.85f;          // ~0.80 g brake FF scale ceiling
constexpr float kFfTrackTauS = 0.12f;             // Velocity error -> longitudinal accel FF horizon
constexpr float kOverspeedFloorRadS = 18.0f;      // ~0.50 m/s safety overspeed floor above v_est
constexpr float kUnderspeedFloorRadS = 16.0f;     // ~0.44 m/s ABS underspeed floor below v_est
constexpr float kSlipCeilingRatio = 0.15f;        // 15% high-speed longitudinal slip allowance

#if TEMP_3WD_FR_DEAD
constexpr float kKaFront = 0.0f;                  // Zero front FF while FR is dead to prevent asymmetric torque steer
constexpr float kKaRear  = 0.72f;                 // Compensate rear FF in temporary 3WD mode
constexpr float kTemp3wdFlVtrqCap = 1.50f;        // Keep FL drive torque low until FR is restored
#else
constexpr float kKaFront = 0.61f;                 // 4WD launch FF gain (V / (m/s^2))
constexpr float kKaRear  = 0.61f;                 // 4WD launch FF gain (V / (m/s^2))
#endif

// Rear Torque Vectoring & Axle Envelope defaults
constexpr float kTvDeadbandRadS = 0.20f;          // +/- 0.20 rad/s deadband on yaw rate error
constexpr float kDefaultTvKpPsi = 4.0f;           // Rear TV proportional slip bias ((rad/s wheel) / (rad/s yaw))
constexpr float kDefaultTvKdPsi = 0.08f;          // Rear TV yaw acceleration damping ((rad/s wheel) / (rad/s^2))
constexpr float kDefaultTvMaxVff = 1.65f;         // Max rear TV feedforward torque voltage differential (V)
constexpr float kDefaultOversteerMuCap = 1.20f;   // Oversteer reference grip cap for Shore A22 silicone
constexpr float kTvSlipRatioMax = 0.10f;          // 10% wheel speed slip ceiling (8-12% Shore A22 peak)
constexpr float kTvSlipFloorRadS = 4.0f;          // 3-5 rad/s low-speed TV slip floor
constexpr float kFrontTurnMinFrac = 0.15f;        // Reserve ~85% front friction circle at full steering lock
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
    omega_tv_target_(0.0f),
    prev_gyro_z_(0.0f),
    gyro_z_dot_filt_(0.0f),
    prev_gyro_seq_(0),
    prev_gyro_ts_us_(0),
    has_prev_gyro_(false),
    slip_state_(SLIP_NORMAL),
    slip_hold_timer_(0.0f),
    understeer_confirm_timer_(0.0f),
    oversteer_confirm_timer_(0.0f),
    gyro_assist_enabled_(true),
    rear_tv_enabled_(true),
    front_env_enabled_(true),
    tv_kp_psi_(kDefaultTvKpPsi),
    tv_kd_psi_(kDefaultTvKdPsi),
    tv_max_vff_(kDefaultTvMaxVff),
    mu_os_cap_(kDefaultOversteerMuCap) {}

void ElectronicDifferential4WD::reset() {
    current_steer_rad_ = 0.0f;
    driver_steer_rad_ = 0.0f;
    prev_delta_gyro_rad_ = 0.0f;
    rate_limited_speed_ = 0.0f;
    omega_ref_ = 0.0f;
    omega_tv_target_ = 0.0f;
    prev_gyro_z_ = 0.0f;
    gyro_z_dot_filt_ = 0.0f;
    prev_gyro_seq_ = 0;
    prev_gyro_ts_us_ = 0;
    has_prev_gyro_ = false;
    slip_state_ = SLIP_NORMAL;
    slip_hold_timer_ = 0.0f;
    understeer_confirm_timer_ = 0.0f;
    oversteer_confirm_timer_ = 0.0f;
}

ElectronicDiffOutputs IRAM_ATTR ElectronicDifferential4WD::update(
    float target_v,
    float target_steer_rad,
    float gyro_z_rad_s,
    float dt,
    float mu,
    float v_curr,
    uint32_t gyro_seq,
    uint64_t gyro_ts_us
) {
    ElectronicDiffOutputs out;
    const float safe_dt = (dt > 1e-5f) ? dt : CONTROL_LOOP_DT_S;
    const float safe_mu = fmaxf(0.10f, mu);

    // -------------------------------------------------------------------------
    // 1. Driver Steering Intent (Speed-Capped & Asymmetric Slew)
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

    // -------------------------------------------------------------------------
    // 2. Split Yaw References (Option C2) & Clock-Beat Guarded Yaw Accel
    // -------------------------------------------------------------------------
    // Always reference pre-countersteer driver_steer_rad_ and fused ground velocity
    float v_yaw = (fabsf(v_curr) > 0.05f) ? v_curr : rate_limited_speed_;
    float omega_kin_driver = (v_yaw * tanf(driver_steer_rad_)) / L_;

    // 2a. Short-lag (12 ms) uncapped kinematic yaw target for Rear TV turn-in
    float alpha_tv_ref = safe_dt / (kTvYawRefTauS + safe_dt);
    omega_tv_target_ += alpha_tv_ref * (omega_kin_driver - omega_tv_target_);
    out.omega_tv_target = omega_tv_target_;

    // 2b. Grip-capped (mu_os_cap_ ~ 1.20) & lagged reference for oversteer & gyro assist
    float omega_grip_max = (mu_os_cap_ * GRAVITY_G_CONST) / fmaxf(fabsf(v_yaw), 0.50f);
    float omega_ref_target = fmaxf(-omega_grip_max, fminf(omega_grip_max, omega_kin_driver));
    bool is_unwinding = (fabsf(omega_ref_target) < fabsf(omega_ref_)) || (omega_ref_target * omega_ref_ < 0.0f);
    float tau_ref = is_unwinding ? kGyroYawRefUnwindTauS : kGyroYawRefTauS;
    float alpha_ref = safe_dt / (tau_ref + safe_dt);
    omega_ref_ += alpha_ref * (omega_ref_target - omega_ref_);
    out.omega_ref = omega_ref_;

    // 2c. Clock-beat guard: differentiate gyro_z only on fresh BNO086 report with true inter-packet dt
    if (gyro_seq != 0U) {
        if (gyro_seq != prev_gyro_seq_) {
            if (has_prev_gyro_ && gyro_ts_us > prev_gyro_ts_us_) {
                float dt_gyro = (float)(gyro_ts_us - prev_gyro_ts_us_) * 1e-6f;
                if (dt_gyro <= 0.050f) {
                    float safe_dt_gyro = fmaxf(0.0015f, dt_gyro);
                    float d_gyro_raw = (gyro_z_rad_s - prev_gyro_z_) / safe_dt_gyro;
                    float alpha_d = safe_dt_gyro / (kYawAccelLpfTauS + safe_dt_gyro);
                    gyro_z_dot_filt_ += alpha_d * (d_gyro_raw - gyro_z_dot_filt_);
                }
            }
            prev_gyro_z_ = gyro_z_rad_s;
            prev_gyro_seq_ = gyro_seq;
            prev_gyro_ts_us_ = gyro_ts_us;
            has_prev_gyro_ = true;
        }
    } else {
        if (has_prev_gyro_) {
            float d_gyro_raw = (gyro_z_rad_s - prev_gyro_z_) / safe_dt;
            float alpha_d = safe_dt / (kYawAccelLpfTauS + safe_dt);
            gyro_z_dot_filt_ += alpha_d * (d_gyro_raw - gyro_z_dot_filt_);
        }
        prev_gyro_z_ = gyro_z_rad_s;
        has_prev_gyro_ = true;
    }
    out.yaw_accel_filt = gyro_z_dot_filt_;

    // -------------------------------------------------------------------------
    // 3. Coordinated Continuous PD Gyro-Assisted Steering (Option B1)
    // -------------------------------------------------------------------------
    // Strictly zero throughout normal turn-in / understeer [0 .. omega_ref_]
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

    float speed_abs = fabsf(v_yaw);
    float gyro_fade = 0.0f;
    if (speed_abs > v_slip_gate_) {
        gyro_fade = fminf(1.0f, (speed_abs - v_slip_gate_) / fmaxf(0.05f, kGyroFullSpeedMps - v_slip_gate_));
    }
    float speed_sign = (v_yaw >= 0.0f) ? 1.0f : -1.0f;

    float raw_delta_gyro = 0.0f;
    if (gyro_assist_enabled_ && gyro_fade > 0.0f && fabsf(excess_yaw) > kGyroDeadbandRadS) {
        float excess_db = copysignf(fabsf(excess_yaw) - kGyroDeadbandRadS, excess_yaw);
        float d_weight = fminf(1.0f, fabsf(excess_db) / 0.25f);
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

    // Physical servo dynamics (22.0 rad/s)
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

    float norm_steer = fmaxf(-1.0f, fminf(1.0f, current_steer_rad_ / fmaxf(1e-4f, max_steer_rad_)));
    if (norm_steer >= 0.0f) {
        out.servo_pulse_us = SERVO_PULSE_NEUTRAL_US - norm_steer * (SERVO_PULSE_NEUTRAL_US - SERVO_PULSE_MIN_US);
    } else {
        out.servo_pulse_us = SERVO_PULSE_NEUTRAL_US + (-norm_steer) * (SERVO_PULSE_MAX_US - SERVO_PULSE_NEUTRAL_US);
    }
    out.servo_pulse_us = fmaxf(SERVO_PULSE_MIN_US, fminf(SERVO_PULSE_MAX_US, out.servo_pulse_us));

    // -------------------------------------------------------------------------
    // 4. Torque-Envelope Speed Target, Overspeed Ceiling & V_trq,ff (Option A1)
    // -------------------------------------------------------------------------
    float effective_target_v = target_v;
    // Emergency-only high-mu sanity ceiling (1.30 g); SL-MPC owns normal corner speeds
    float steer_for_cap = fmaxf(fabsf(driver_steer_rad_), fabsf(capped_target_delta));
    float tan_steer_cap = fabsf(tanf(steer_for_cap));
    if (tan_steer_cap > 0.05f) {
        float v_emerg = sqrtf((kEmergencyCornerMu * GRAVITY_G_CONST * L_) / tan_steer_cap);
        v_emerg = fmaxf(0.80f, v_emerg);
        if (effective_target_v > 0.0f) {
            effective_target_v = fminf(effective_target_v, v_emerg);
        } else if (effective_target_v < 0.0f) {
            effective_target_v = fmaxf(effective_target_v, -v_emerg);
        }
    }

    // Wide collision resync
    if (rate_limited_speed_ >= 0.0f) {
        if (v_curr < (rate_limited_speed_ - 0.85f)) {
            rate_limited_speed_ = v_curr;
        }
    } else {
        if (v_curr > (rate_limited_speed_ + 0.85f)) {
            rate_limited_speed_ = v_curr;
        }
    }

    if (effective_target_v <= v_curr) {
        if (rate_limited_speed_ > v_curr) {
            rate_limited_speed_ = v_curr;
        }
    } else {
        if (rate_limited_speed_ < v_curr) {
            rate_limited_speed_ = v_curr;
        }
    }

    float speed_err = effective_target_v - rate_limited_speed_;
    bool is_accel = (rate_limited_speed_ >= 0.0f && speed_err > 0.0f) || (rate_limited_speed_ <= 0.0f && speed_err < 0.0f);
    float max_dv = (is_accel ? kEnvAccelMaxMps2 : kEnvDecelMaxMps2) * safe_dt;
    if (fabsf(speed_err) <= max_dv) {
        rate_limited_speed_ = effective_target_v;
    } else {
        rate_limited_speed_ += copysignf(max_dv, speed_err);
    }

    // Reasonable overspeed / ABS underspeed envelope (~18 rad/s floor, 15% high-speed margin)
    float max_overspeed = fmaxf(kOverspeedFloorRadS * R_w_, kSlipCeilingRatio * fabsf(v_curr));
    float max_underspeed = fmaxf(kUnderspeedFloorRadS * R_w_, kSlipCeilingRatio * fabsf(v_curr));
    if (rate_limited_speed_ > 0.0f) {
        rate_limited_speed_ = fminf(rate_limited_speed_, fmaxf(0.0f, v_curr) + max_overspeed);
        if (v_curr > 0.30f) {
            rate_limited_speed_ = fmaxf(rate_limited_speed_, v_curr - max_underspeed);
        }
    } else if (rate_limited_speed_ < 0.0f) {
        rate_limited_speed_ = fmaxf(rate_limited_speed_, fminf(0.0f, v_curr) - max_overspeed);
        if (v_curr < -0.30f) {
            rate_limited_speed_ = fminf(rate_limited_speed_, v_curr + max_underspeed);
        }
    }
    out.rate_limited_v = rate_limited_speed_;

    // Longitudinal acceleration feedforward (V_trq,ff = Ka * a_x_cmd)
    float v_err_long = effective_target_v - v_curr;
    float a_x_cmd = 0.0f;
    if (fabsf(v_err_long) > 0.05f) {
        a_x_cmd = fmaxf(-kFfDecelMaxMps2, fminf(kFfAccelMaxMps2, v_err_long / kFfTrackTauS));
    }
    float v_ff_front_base = kKaFront * a_x_cmd;
    float v_ff_rear_base  = kKaRear  * a_x_cmd;

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
    if (fabsf(rate_limited_speed_) < 0.001f && fabsf(v_curr) < 0.05f) {
        omega_ref_ = 0.0f;
        omega_tv_target_ = 0.0f;
        prev_delta_gyro_rad_ = 0.0f;
        slip_state_ = SLIP_NORMAL;
        slip_hold_timer_ = 0.0f;
        understeer_confirm_timer_ = 0.0f;
        oversteer_confirm_timer_ = 0.0f;
        out.omega_ref = 0.0f;
        out.omega_tv_target = 0.0f;
        out.delta_omega_tv = 0.0f;
        out.delta_vtrq_tv = 0.0f;
        out.v_FL = out.v_FR = out.v_RL = out.v_RR = 0.0f;
        out.omega_FL = out.omega_FR = out.omega_RL = out.omega_RR = 0.0f;
        out.rpm_FL = out.rpm_FR = out.rpm_RL = out.rpm_RR = 0.0f;
        out.vtrq_min_FL = out.vtrq_max_FL = out.vtrq_ff_FL = 0.0f;
        out.vtrq_min_FR = out.vtrq_max_FR = out.vtrq_ff_FR = 0.0f;
        out.vtrq_min_RL = out.vtrq_max_RL = out.vtrq_ff_RL = 0.0f;
        out.vtrq_min_RR = out.vtrq_max_RR = out.vtrq_ff_RR = 0.0f;
        out.scale_factor = 1.0f;
        out.slip_state = SLIP_NORMAL;
        out.yaw_error = 0.0f;
        out.is_countersteering = false;
        return out;
    }

    // -------------------------------------------------------------------------
    // 7. Reverse Guard (Symmetric 4WD kinematic allocation, zero TV)
    // -------------------------------------------------------------------------
    if (effective_target_v < -0.05f || (rate_limited_speed_ < -0.05f && v_curr < 0.05f)) {
        out.yaw_error = gyro_z_rad_s - omega_ref_;
        out.slip_state = SLIP_NORMAL;
        out.is_countersteering = false;
        out.delta_omega_tv = 0.0f;
        out.delta_vtrq_tv = 0.0f;

        float half_Wr_omega = out.omega_cmd * (W_r_ * 0.5f);
        float half_Wf_omega = out.omega_cmd * (W_f_ * 0.5f);
        out.v_RL = rate_limited_speed_ - half_Wr_omega;
        out.v_RR = rate_limited_speed_ + half_Wr_omega;

        float fl_x = rate_limited_speed_ - half_Wf_omega;
        float fr_x = rate_limited_speed_ + half_Wf_omega;
        float front_y = L_ * out.omega_cmd;
        out.v_FL = fl_x * cosf(out.steer_left_rad) + front_y * sinf(out.steer_left_rad);
        out.v_FR = fr_x * cosf(out.steer_right_rad) + front_y * sinf(out.steer_right_rad);

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

        float max_omega = v_max_wheel_ / R_w_;
        out.omega_FL = fmaxf(-max_omega, fminf(max_omega, (out.v_FL / R_w_) * motor_dir_));
        out.omega_FR = fmaxf(-max_omega, fminf(max_omega, (out.v_FR / R_w_) * motor_dir_));
        out.omega_RL = fmaxf(-max_omega, fminf(max_omega, (out.v_RL / R_w_) * motor_dir_));
        out.omega_RR = fmaxf(-max_omega, fminf(max_omega, (out.v_RR / R_w_) * motor_dir_));

        const float rad_s_to_rpm = 60.0f / (2.0f * PI_F);
        out.rpm_FL = out.omega_FL * rad_s_to_rpm;
        out.rpm_FR = out.omega_FR * rad_s_to_rpm;
        out.rpm_RL = out.omega_RL * rad_s_to_rpm;
        out.rpm_RR = out.omega_RR * rad_s_to_rpm;

        out.vtrq_min_FL = out.vtrq_min_FR = -kVtrqFullV;
        out.vtrq_max_FL = out.vtrq_max_FR =  kVtrqFullV;
        out.vtrq_ff_FL  = out.vtrq_ff_FR  = fmaxf(-kVtrqFullV, fminf(kVtrqFullV, v_ff_front_base));
        out.vtrq_min_RL = out.vtrq_min_RR = -kVtrqFullV;
        out.vtrq_max_RL = out.vtrq_max_RR =  kVtrqFullV;
        out.vtrq_ff_RL  = out.vtrq_ff_RR  = fmaxf(-kVtrqFullV, fminf(kVtrqFullV, v_ff_rear_base));
        return out;
    }

    // -------------------------------------------------------------------------
    // 8. Cornering State Classification (5-State Model + Hysteretic Telemetry)
    // -------------------------------------------------------------------------
    out.yaw_error = gyro_z_rad_s - omega_ref_;
    float os_frac = 0.0f;
    float us_frac = 0.0f;
    VehicleSlipState raw_slip = SLIP_NORMAL;

    if (speed_abs >= v_slip_gate_) {
        os_frac = fmaxf(0.0f, fminf(1.0f, (fabsf(excess_yaw) - 0.25f) / 0.95f));

        if (os_frac <= 0.05f && fabsf(driver_steer_rad_) > 0.08f && fabsf(omega_tv_target_) > 0.60f) {
            float turn_sign = (driver_steer_rad_ >= 0.0f) ? 1.0f : -1.0f;
            float us_deficit = (omega_tv_target_ - gyro_z_rad_s) * turn_sign;
            us_frac = fmaxf(0.0f, fminf(1.0f, (us_deficit - 0.20f) / 1.80f));
        }

        float dyn_thresh = fmaxf(fmaxf(thresh_spin_ * 2.2f, 0.95f), 0.40f * fabsf(omega_ref_));
        if (omega_ref_ > 0.10f && (gyro_z_rad_s - omega_ref_) > dyn_thresh) {
            raw_slip = SLIP_OVERSTEER;
        } else if (omega_ref_ < -0.10f && (omega_ref_ - gyro_z_rad_s) > dyn_thresh) {
            raw_slip = SLIP_OVERSTEER;
        } else if (fabsf(gyro_z_rad_s) > fmaxf(1.35f, thresh_spin_ * 3.0f) && (gyro_z_rad_s * omega_ref_ <= 0.0f)) {
            raw_slip = SLIP_OVERSTEER;
        } else if (us_frac > 0.45f) {
            raw_slip = SLIP_UNDERSTEER;
        }
    }

    if (speed_abs < v_slip_gate_) {
        slip_state_ = SLIP_NORMAL;
        slip_hold_timer_ = 0.0f;
        understeer_confirm_timer_ = 0.0f;
        oversteer_confirm_timer_ = 0.0f;
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
        }
    }
    out.slip_state = slip_state_;

    bool is_os = (os_frac > 0.05f);
    bool is_countersteering = is_os && ((current_steer_rad_ * gyro_z_rad_s) <= 0.03f * fabsf(gyro_z_rad_s));
    out.is_countersteering = is_countersteering;

    // -------------------------------------------------------------------------
    // 9. Front Axle Envelope (Zero L/R Vectoring + Steering-Coupled V_trq)
    // -------------------------------------------------------------------------
    float steer_norm = fmaxf(0.0f, fminf(1.0f, fabsf(current_steer_rad_) / fmaxf(1e-4f, max_steer_rad_)));
    float front_steer_scale = kFrontTurnMinFrac + (1.0f - kFrontTurnMinFrac) * (1.0f - steer_norm) * (1.0f - steer_norm);
    bool is_braking = (effective_target_v < v_curr - 0.10f) && (v_curr > 0.20f);

    float v_f_max = front_env_enabled_ ? (kVtrqFullV * front_steer_scale) : kVtrqFullV;
    float v_f_min = is_braking ? (-kVtrqFullV * fmaxf(0.35f, front_steer_scale)) : 0.0f;
    float v_f_ff  = (a_x_cmd > 0.0f && front_env_enabled_) ? (v_ff_front_base * front_steer_scale) : v_ff_front_base;

    if (is_countersteering) {
        // Oversteer (countersteering): boost front V_trq_max + FF to drag nose straight
        v_f_max = kVtrqFullV * (0.65f + 0.35f * os_frac);
        v_f_min = 0.0f;
        v_f_ff  = fmaxf(v_ff_front_base, 1.5f + 2.5f * os_frac);
    } else if (is_os) {
        // Oversteer (still turned in): keep front low/neutral so pulling won't worsen spin
        v_f_max = fminf(v_f_max, 0.20f + 0.80f * (1.0f - os_frac));
        v_f_min = 0.0f;
        v_f_ff  = 0.0f;
    } else if (us_frac > 0.0f && front_env_enabled_) {
        // Understeer: drop front drive toward freewheel so 100% of front grip steers
        v_f_max *= (1.0f - 0.85f * us_frac);
        v_f_min = 0.0f;
        v_f_ff  *= (1.0f - us_frac);
    }

#if TEMP_3WD_FR_DEAD
    v_f_max = fminf(v_f_max, kTemp3wdFlVtrqCap);
#endif
    v_f_ff = fmaxf(v_f_min, fminf(v_f_max, v_f_ff));
    out.vtrq_min_FL = out.vtrq_min_FR = v_f_min;
    out.vtrq_max_FL = out.vtrq_max_FR = v_f_max;
    out.vtrq_ff_FL  = out.vtrq_ff_FR  = v_f_ff;

    // -------------------------------------------------------------------------
    // 10. Rear Torque Vectoring (Bounded Slip Bias + Coordinated Yaw Damping)
    // -------------------------------------------------------------------------
    float delta_omega_tv = 0.0f;
    float delta_vtrq_tv = 0.0f;
    if (rear_tv_enabled_ && speed_abs >= v_slip_gate_) {
        float e_psi = is_os ? (-excess_yaw) : (omega_tv_target_ - gyro_z_rad_s);
        float e_db = 0.0f;
        if (fabsf(e_psi) > kTvDeadbandRadS) {
            e_db = copysignf(fabsf(e_psi) - kTvDeadbandRadS, e_psi);
        }

        // Option B1 guardrail: zero Rear TV Kd while delta_gyro servo is actively countersteering
        float tv_kd_scale = fmaxf(0.0f, 1.0f - fabsf(delta_gyro) / 0.06f);
        float delta_omega_max = fmaxf(kTvSlipFloorRadS, kTvSlipRatioMax * (speed_abs / R_w_));
        float d_term_tv = (fabsf(e_psi) > 0.15f || fabsf(gyro_z_dot_filt_) > 3.0f)
                              ? (tv_kd_scale * tv_kd_psi_ * gyro_z_dot_filt_)
                              : 0.0f;

        delta_omega_tv = fmaxf(-delta_omega_max, fminf(delta_omega_max, tv_kp_psi_ * e_db - d_term_tv));
        delta_vtrq_tv = (delta_omega_tv / fmaxf(1.0f, delta_omega_max)) * tv_max_vff_;
    }
    out.delta_omega_tv = delta_omega_tv;
    out.delta_vtrq_tv = delta_vtrq_tv;

    // -------------------------------------------------------------------------
    // 11. Asymmetric Rear V_trq Bounds Across Cornering States
    // -------------------------------------------------------------------------
    float v_min_RL, v_max_RL, v_ff_RL;
    float v_min_RR, v_max_RR, v_ff_RR;

    if (is_os) {
        bool spin_left = (fabsf(gyro_z_rad_s) > 0.05f) ? (gyro_z_rad_s > 0.0f) : (excess_yaw > 0.0f);
        float outer_max = kVtrqFullV * fmaxf(0.0f, 1.0f - 1.4f * os_frac);
        float inner_max = is_countersteering
                              ? (kVtrqFullV * fmaxf(0.10f, 0.35f * (1.0f - 0.6f * os_frac)))
                              : (kVtrqFullV * 0.45f);

        if (spin_left) {
            // Left spin: RR is outer (cut V_trq -> 0); RL is inner
            v_min_RR = 0.0f;
            v_max_RR = outer_max;
            v_ff_RR  = 0.0f;

            v_min_RL = 0.0f;
            v_max_RL = inner_max;
            v_ff_RL  = is_countersteering ? 0.0f : fmaxf(0.0f, fminf(inner_max, v_ff_rear_base - delta_vtrq_tv));
        } else {
            // Right spin: RL is outer (cut V_trq -> 0); RR is inner
            v_min_RL = 0.0f;
            v_max_RL = outer_max;
            v_ff_RL  = 0.0f;

            v_min_RR = 0.0f;
            v_max_RR = inner_max;
            v_ff_RR  = is_countersteering ? 0.0f : fmaxf(0.0f, fminf(inner_max, v_ff_rear_base + delta_vtrq_tv));
        }
    } else {
        float lat_sig = v_yaw * gyro_z_rad_s + 0.35f * v_yaw * omega_tv_target_;
        float lat_load = fmaxf(0.0f, fminf(1.0f, fabsf(v_yaw * gyro_z_rad_s) / fmaxf(1.0f, safe_mu * GRAVITY_G_CONST)));

        float outer_max = kVtrqFullV;
        float outer_min = is_braking ? -kVtrqFullV : -1.0f;
        float inner_max = kVtrqFullV * (1.0f - 0.60f * lat_load);
        float inner_min = is_braking ? (-kVtrqFullV * (1.0f - 0.50f * lat_load)) : 0.0f;

        if (lat_sig >= 0.0f) {
            // Left turn: RR loaded outside, RL unloaded inside
            v_min_RL = inner_min;
            v_max_RL = inner_max;
            v_min_RR = outer_min;
            v_max_RR = outer_max;
        } else {
            // Right turn: RL loaded outside, RR unloaded inside
            v_min_RL = outer_min;
            v_max_RL = outer_max;
            v_min_RR = inner_min;
            v_max_RR = inner_max;
        }
        v_ff_RL = fmaxf(v_min_RL, fminf(v_max_RL, v_ff_rear_base - delta_vtrq_tv));
        v_ff_RR = fmaxf(v_min_RR, fminf(v_max_RR, v_ff_rear_base + delta_vtrq_tv));
    }

    out.vtrq_min_RL = v_min_RL;
    out.vtrq_max_RL = v_max_RL;
    out.vtrq_ff_RL  = v_ff_RL;
    out.vtrq_min_RR = v_min_RR;
    out.vtrq_max_RR = v_max_RR;
    out.vtrq_ff_RR  = v_ff_RR;

    // -------------------------------------------------------------------------
    // 12. 4-Wheel Speed Allocation + Bounded Rear TV Slip Bias
    // -------------------------------------------------------------------------
    float half_Wr_omega_rear = out.omega_cmd * (W_r_ * 0.5f);
    float half_Wf_omega = out.omega_cmd * (W_f_ * 0.5f);

    float v_tv_bias = delta_omega_tv * R_w_;
    out.v_RL = (rate_limited_speed_ - half_Wr_omega_rear) - v_tv_bias;
    out.v_RR = (rate_limited_speed_ + half_Wr_omega_rear) + v_tv_bias;

    // Front axle (zero L/R TV, vector projection along steered knuckle angles)
    float fl_x = rate_limited_speed_ - half_Wf_omega;
    float fr_x = rate_limited_speed_ + half_Wf_omega;
    float front_y = L_ * out.omega_cmd;
    out.v_FL = fl_x * cosf(out.steer_left_rad) + front_y * sinf(out.steer_left_rad);
    out.v_FR = fr_x * cosf(out.steer_right_rad) + front_y * sinf(out.steer_right_rad);

    // Proportional saturation scaling (preserve turning radius at speed limit)
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

    // Convert linear velocities to shaft angular velocities & RPM
    float max_omega = v_max_wheel_ / R_w_;
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

