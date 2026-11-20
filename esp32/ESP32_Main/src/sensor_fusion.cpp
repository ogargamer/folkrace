#include "sensor_fusion.h"
#include <utility>

#ifndef PI_F
#define PI_F 3.14159265358979323846f
#endif

IMUEncoderFusion::IMUEncoderFusion(
    float wheel_radius,
    float track_rear,
    float track_front,
    float wheelbase,
    float motor_dir,
    float dt
) : R_w_(wheel_radius),
    W_r_(track_rear),
    W_f_(track_front),
    L_(wheelbase),
    motor_dir_(motor_dir),
    dt_(dt),
    v_est_(0.0f),
    yaw_est_(0.0f),
    total_distance_m_(0.0f),
    slip_ratio_(0.0f) {}

void IMUEncoderFusion::reset() {
    v_est_ = 0.0f;
    yaw_est_ = 0.0f;
    total_distance_m_ = 0.0f;
    slip_ratio_ = 0.0f;
}

FusionOutputs IRAM_ATTR IMUEncoderFusion::update(
    float fl_rpm,
    float fr_rpm,
    float rl_rpm,
    float rr_rpm,
    float gyro_z_rad_s,
    float accel_fwd_m_s2,
    float steer_rad,
    float dt,
    float front_vtrq_avg,
    float rear_vtrq_avg
) {
    FusionOutputs out;
    float step_dt = (dt > 0.0f) ? dt : dt_;

    // 1. Convert shaft RPM to linear rolling speeds (m/s)
    // Motor direction convention: positive RPM moves vehicle forward with VEHICLE_MOTOR_DIR = 1.0
    const float rpm_to_mps = (2.0f * PI_F / 60.0f) * R_w_ * motor_dir_;
    float v_fl = fl_rpm * rpm_to_mps;
    float v_fr = fr_rpm * rpm_to_mps;
    float v_rl = rl_rpm * rpm_to_mps;
    float v_rr = rr_rpm * rpm_to_mps;

    // 2. Rear Axle unsteered speed
    float v_rear_avg = 0.5f * (v_rl + v_rr);
    out.v_rear_avg = v_rear_avg;

    // 3. Front Axle: Project steered wheels along vehicle longitudinal body axis
    float cos_steer = cosf(steer_rad);
    float v_fl_proj = v_fl * cos_steer;
    float v_fr_proj = v_fr * cos_steer;
#if TEMP_3WD_FR_DEAD
    (void)v_fr_proj;
    float v_front_avg = v_fl_proj;
#else
    float v_front_avg = 0.5f * (v_fl_proj + v_fr_proj);
#endif
    out.v_front_avg = v_front_avg;

    // 4. Project each wheel onto chassis centerline (sign: +gyro_z = left turn)
    float v_rl_body = v_rl + 0.5f * W_r_ * gyro_z_rad_s;
    float v_rr_body = v_rr - 0.5f * W_r_ * gyro_z_rad_s;
    float v_fl_body = v_fl_proj + 0.5f * W_f_ * gyro_z_rad_s;
    float v_fr_body = v_fr_proj - 0.5f * W_f_ * gyro_z_rad_s;

    // 5. Reference wheel selection by torque sign
    float v_trq_avg = 0.5f * (front_vtrq_avg + rear_vtrq_avg);
    float v_wheel_ref = 0.0f;

#if TEMP_3WD_FR_DEAD
    (void)v_fr_body;
    // 3WD fallback: sort [RL, RR, FL] ascending
    float w0 = v_rl_body;
    float w1 = v_rr_body;
    float w2 = v_fl_body;
    if (w0 > w1) std::swap(w0, w1);
    if (w1 > w2) std::swap(w1, w2);
    if (w0 > w1) std::swap(w0, w1);

    if (v_trq_avg > 0.35f) {
        // Positive drive: lowest (capped front wheel) unless dead-zero guard trips
        v_wheel_ref = (fabsf(w1) > 0.30f && fabsf(w0) < 0.05f) ? w1 : w0;
    } else if (v_trq_avg < -0.35f) {
        // Braking / negative drive: highest unless single-wheel upward outlier
        v_wheel_ref = (fabsf(w2 - w1) < 1.50f) ? w2 : w1;
    } else {
        v_wheel_ref = w1;
    }
#else
    // 4WD: sort [RL, RR, FL, FR] ascending into w0 <= w1 <= w2 <= w3
    float w0 = v_rl_body;
    float w1 = v_rr_body;
    float w2 = v_fl_body;
    float w3 = v_fr_body;
    if (w0 > w1) std::swap(w0, w1);
    if (w2 > w3) std::swap(w2, w3);
    if (w0 > w2) std::swap(w0, w2);
    if (w1 > w3) std::swap(w1, w3);
    if (w1 > w2) std::swap(w1, w2);

    if (v_trq_avg > 0.35f) {
        // Positive drive: 2nd-lowest (rejects rear spin + 1 faulty low outlier)
        v_wheel_ref = w1;
    } else if (v_trq_avg < -0.35f) {
        // Braking: 2nd-highest (rejects locking wheels + 1 faulty high outlier)
        v_wheel_ref = w2;
    } else {
        // Coasting / light load: median of 4
        v_wheel_ref = 0.5f * (w1 + w2);
    }
#endif
    out.v_wheels = v_wheel_ref;

    // 6. Dynamic Slip Estimation
    float rear_diff_yaw_rate = (v_rr - v_rl) / fmaxf(0.01f, W_r_);
    float diff_error = fabsf(rear_diff_yaw_rate - gyro_z_rad_s);
    float axle_discrepancy = fabsf(v_front_avg - v_rear_avg);
    float wheel_accel = (v_wheel_ref - v_est_) / fmaxf(1e-4f, step_dt);
    float accel_mismatch = fabsf(wheel_accel - accel_fwd_m_s2);

    float slip_indicator = fminf(1.0f,
        (diff_error / 4.0f) * 0.40f +
        (axle_discrepancy / 0.50f) * 0.35f +
        (accel_mismatch / 8.0f) * 0.25f
    );
    slip_ratio_ = slip_indicator;
    out.slip_ratio = slip_indicator;

    // 7. Inertial Prediction & Torque/Slip-Gated Complementary Fusion
    float min_axle_vtrq = fminf(fabsf(front_vtrq_avg), fabsf(rear_vtrq_avg));
    float max_axle_vtrq = fmaxf(fabsf(front_vtrq_avg), fabsf(rear_vtrq_avg));
    float trq_t = fminf(1.0f, fmaxf(0.0f, (min_axle_vtrq - 0.80f) / (2.20f - 0.80f)));
    float beta_trq = 0.20f - trq_t * (0.20f - 0.035f);
    float beta_slip = 0.20f - 0.165f * slip_indicator;
    float beta = fminf(beta_trq, beta_slip);

    bool is_crash_impact = (fabsf(v_wheel_ref) < 0.20f && fabsf(accel_fwd_m_s2) > 20.0f);

    if (fabsf(v_wheel_ref) < 0.02f && fabsf(accel_fwd_m_s2) < 0.20f && max_axle_vtrq < 0.50f) {
        v_est_ = 0.0f;
    } else if (is_crash_impact) {
        v_est_ = 0.0f;
    } else {
        float v_pred = v_est_ + accel_fwd_m_s2 * step_dt;
        float envelope_margin = 1.50f;
        float v_pred_clamped = fminf(v_wheel_ref + envelope_margin,
                                     fmaxf(v_wheel_ref - envelope_margin, v_pred));

        v_est_ = (1.0f - beta) * v_pred_clamped + beta * v_wheel_ref;
    }

    // 8. Distance and Heading Integration
    total_distance_m_ += v_est_ * step_dt;
    yaw_est_ += gyro_z_rad_s * step_dt;

    out.v_est = v_est_;
    out.yaw_rate = gyro_z_rad_s;
    out.yaw_est = yaw_est_;
    out.total_distance_m = total_distance_m_;

    return out;
}

