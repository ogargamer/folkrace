#include "electronic_diff.h"

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

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
    rate_limited_speed_(0.0f) {}

void ElectronicDifferential4WD::reset() {
    current_steer_rad_ = 0.0f;
    rate_limited_speed_ = 0.0f;
}

ElectronicDiffOutputs IRAM_ATTR ElectronicDifferential4WD::update(
    float target_v,
    float target_steer_rad,
    float gyro_z_rad_s,
    float dt,
    float mu
) {
    ElectronicDiffOutputs out;

    // -------------------------------------------------------------------------
    // 1. Steering Slew Rate Limiter (Physical Servo Dynamics @ 15.4 rad/s)
    // -------------------------------------------------------------------------
    float target_delta = fmaxf(-max_steer_rad_, fminf(target_steer_rad, max_steer_rad_));
    float max_d_steer = max_servo_rate_ * dt;
    float steer_diff = target_delta - current_steer_rad_;

    if (fabsf(steer_diff) <= max_d_steer) {
        current_steer_rad_ = target_delta;
    } else {
        current_steer_rad_ += copysignf(max_d_steer, steer_diff);
    }
    current_steer_rad_ = fmaxf(-max_steer_rad_, fminf(current_steer_rad_, max_steer_rad_));

    out.steer_rad = current_steer_rad_;
    out.steer_deg = current_steer_rad_ * (180.0f / PI_F);

    // Single Servo PWM calculation (333 Hz Digital Servo on GPIO 21)
    // Neutral 1520 us = 0.0 rad, +/- 90 deg (+/- 1.5708 rad) = +/- 1000 us
    float pulse_offset = (current_steer_rad_ / (PI_F * 0.5f)) * 1000.0f;
    out.servo_pulse_us = fmaxf(SERVO_PULSE_MIN_US, fminf(SERVO_PULSE_MAX_US, SERVO_PULSE_NEUTRAL_US + pulse_offset));

    // -------------------------------------------------------------------------
    // 2. Dynamic Traction & Acceleration Rate Limiter (from Webots Harness)
    // -------------------------------------------------------------------------
    float steer_mag = fabsf(current_steer_rad_);
    float lateral_fraction = fminf(0.80f, fabsf(sinf(steer_mag)));
    const float front_lat_reserve_min = 0.20f;
    float max_longitudinal_fraction = sqrtf(fmaxf(front_lat_reserve_min * front_lat_reserve_min, 1.0f - lateral_fraction * lateral_fraction));

    float a_limit = fminf(ACCEL_FWD_MAX, ACCEL_FWD_RATIO * mu * 9.80665f);
    float d_limit = fminf(BRAKE_DECEL_MAX, BRAKE_DECEL_RATIO * mu * 9.80665f * max_longitudinal_fraction);

    float speed_err = target_v - rate_limited_speed_;
    bool is_accel = (rate_limited_speed_ >= 0.0f && speed_err > 0.0f) || (rate_limited_speed_ <= 0.0f && speed_err < 0.0f);
    float max_dv = (is_accel ? a_limit : d_limit) * dt;

    if (fabsf(speed_err) <= max_dv) {
        rate_limited_speed_ = target_v;
    } else {
        rate_limited_speed_ += copysignf(max_dv, speed_err);
    }
    out.rate_limited_v = rate_limited_speed_;

    // -------------------------------------------------------------------------
    // 3. Kinematic Yaw Rate Command
    // -------------------------------------------------------------------------
    float sin_d = sinf(current_steer_rad_);
    float cos_d = cosf(current_steer_rad_);
    float tan_d = tanf(current_steer_rad_);
    out.omega_cmd = (rate_limited_speed_ * tan_d) / L_;

    // -------------------------------------------------------------------------
    // 4. 40% Geometric Ackermann Knuckle Headings
    // -------------------------------------------------------------------------
    if (fabsf(tan_d) > 1e-4f) {
        float half_Wk_sin = (W_k_ * 0.5f) * sin_d;
        float L_cos = L_ * cos_d;
        float L_sin = L_ * sin_d;
        float steer_left_true = atan2f(L_sin, L_cos - half_Wk_sin);
        float steer_right_true = atan2f(L_sin, L_cos + half_Wk_sin);
        out.steer_left_rad = current_steer_rad_ + ackermann_percent_ * (steer_left_true - current_steer_rad_);
        out.steer_right_rad = current_steer_rad_ + ackermann_percent_ * (steer_right_true - current_steer_rad_);
    } else {
        out.steer_left_rad = current_steer_rad_;
        out.steer_right_rad = current_steer_rad_;
    }

    // -------------------------------------------------------------------------
    // 5. Standstill Guard
    // -------------------------------------------------------------------------
    if (fabsf(rate_limited_speed_) < 0.001f) {
        out.v_FL = out.v_FR = out.v_RL = out.v_RR = 0.0f;
        out.omega_FL = out.omega_FR = out.omega_RL = out.omega_RR = 0.0f;
        out.rpm_FL = out.rpm_FR = out.rpm_RL = out.rpm_RR = 0.0f;
        out.scale_factor = 1.0f;
        out.slip_state = SLIP_NORMAL;
        out.yaw_error = 0.0f;
        return out;
    }

    // -------------------------------------------------------------------------
    // 6. 4-Wheel Speed Allocation (Forward & Reverse)
    // -------------------------------------------------------------------------
    float half_Wr_omega = out.omega_cmd * (W_r_ * 0.5f);
    float half_Wf_omega = out.omega_cmd * (W_f_ * 0.5f);

    // Rear axle (pure longitudinal roll)
    out.v_RL = rate_limited_speed_ - half_Wr_omega;
    out.v_RR = rate_limited_speed_ + half_Wr_omega;

    // Front axle (vector projection along steered knuckle angles)
    float fl_x = rate_limited_speed_ - half_Wf_omega;
    float fr_x = rate_limited_speed_ + half_Wf_omega;
    float front_y = L_ * out.omega_cmd;

    out.v_FL = fl_x * cosf(out.steer_left_rad) + front_y * sinf(out.steer_left_rad);
    out.v_FR = fr_x * cosf(out.steer_right_rad) + front_y * sinf(out.steer_right_rad);

    // -------------------------------------------------------------------------
    // 7. Proportional Saturation Scaling (Preserve turning radius at limit)
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
    // 8. Convert Linear Velocities to Actuator Rotational Velocities & RPM
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

    // -------------------------------------------------------------------------
    // 9. Reliability-Focused IMU Slip Detection
    // -------------------------------------------------------------------------
    out.slip_state = SLIP_NORMAL;
    out.yaw_error = gyro_z_rad_s - out.omega_cmd;

    if (fabsf(rate_limited_speed_) >= v_slip_gate_) {
        float dyn_thresh = fmaxf(thresh_spin_, 0.40f * fabsf(out.omega_cmd));

        if (out.omega_cmd > 0.05f && (gyro_z_rad_s - out.omega_cmd) > dyn_thresh) {
            out.slip_state = SLIP_OVERSTEER;
        } else if (out.omega_cmd < -0.05f && (out.omega_cmd - gyro_z_rad_s) > dyn_thresh) {
            out.slip_state = SLIP_OVERSTEER;
        } else if (fabsf(gyro_z_rad_s) > fmaxf(1.20f, thresh_spin_ * 2.0f) && (gyro_z_rad_s * out.omega_cmd <= 0.0f)) {
            out.slip_state = SLIP_OVERSTEER;
        } else {
            float omega_achievable = fminf(fabsf(out.omega_cmd), (0.90f * 9.80665f) / fmaxf(fabsf(rate_limited_speed_), 0.50f));
            if (fabsf(current_steer_rad_) > 0.15f && omega_achievable > 1.20f && fabsf(gyro_z_rad_s) < (0.35f * omega_achievable)) {
                out.slip_state = SLIP_UNDERSTEER;
            }
        }
    }

    return out;
}

