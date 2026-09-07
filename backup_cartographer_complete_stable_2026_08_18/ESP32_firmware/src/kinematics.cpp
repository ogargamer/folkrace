#include "kinematics.h"
#include <cmath>
#include <algorithm>

template<typename T>
static inline T clamp_val(T v, T lo, T hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

KinematicsEngine::KinematicsEngine() : dir_(1.0f) {}

void KinematicsEngine::reset() {
    dir_ = 1.0f;
}

uint16_t KinematicsEngine::interpolateSteeringLUT(float delta_deg) {
    float clamped_deg = clamp_val(delta_deg, STEER_LUT_ANGLES_DEG[0], STEER_LUT_ANGLES_DEG[STEER_LUT_SIZE - 1]);

    if (clamped_deg <= STEER_LUT_ANGLES_DEG[0]) {
        return static_cast<uint16_t>(STEER_LUT_PWM_US[0]);
    }
    if (clamped_deg >= STEER_LUT_ANGLES_DEG[STEER_LUT_SIZE - 1]) {
        return static_cast<uint16_t>(STEER_LUT_PWM_US[STEER_LUT_SIZE - 1]);
    }

    for (size_t i = 0; i < STEER_LUT_SIZE - 1; ++i) {
        if (clamped_deg >= STEER_LUT_ANGLES_DEG[i] && clamped_deg <= STEER_LUT_ANGLES_DEG[i + 1]) {
            float range_angle = STEER_LUT_ANGLES_DEG[i + 1] - STEER_LUT_ANGLES_DEG[i];
            float range_pwm = STEER_LUT_PWM_US[i + 1] - STEER_LUT_PWM_US[i];
            float t = (clamped_deg - STEER_LUT_ANGLES_DEG[i]) / range_angle;
            float pwm_val = STEER_LUT_PWM_US[i] + t * range_pwm;
            return static_cast<uint16_t>(std::round(pwm_val));
        }
    }

    return SERVO_NEUTRAL_PWM_US;
}

uint16_t KinematicsEngine::computeSteeringPWM(float v_profiled, float omega_profiled, float* out_delta_rad) {
    float delta_raw_rad = 0.0f;

    // Launch standstill guard: |v_profiled| < 0.05 m/s => delta = 0 rad (0 deg)
    if (std::abs(v_profiled) < LAUNCH_STEER_V_DEADBAND) {
        delta_raw_rad = 0.0f;
    } else {
        delta_raw_rad = std::atan2(WHEELBASE_M * omega_profiled, std::abs(v_profiled));
        delta_raw_rad = clamp_val(delta_raw_rad, -DELTA_MAX_RAD, DELTA_MAX_RAD);
    }


    if (out_delta_rad != nullptr) {
        *out_delta_rad = delta_raw_rad;
    }

    float delta_raw_deg = delta_raw_rad * (180.0f / M_PI);
    return interpolateSteeringLUT(delta_raw_deg);
}

WheelSpeeds KinematicsEngine::computeKinematicSpeeds(float v_profiled, float omega_profiled) {
    // 1. Zero-Crossing Direction Hysteresis Update
    if (v_profiled > HYSTERESIS_V_THRESH) {
        dir_ = 1.0f;
    } else if (v_profiled < -HYSTERESIS_V_THRESH) {
        dir_ = -1.0f;
    }
    // If within [-0.01, +0.01], keep previous dir_

    WheelSpeeds speeds;

    // 2. Rear Wheel Speeds (REAR_CONTACT_HALF_TRACK = 0.0485 m)
    speeds.rl = v_profiled - omega_profiled * REAR_CONTACT_HALF_TRACK;
    speeds.rr = v_profiled + omega_profiled * REAR_CONTACT_HALF_TRACK;

    // 3. Front Wheel Speeds (FRONT_INTERMEDIATE_HALF_TRACK = 0.04225 m)
    if (std::abs(omega_profiled) < 1e-4f || std::abs(v_profiled) < LOW_SPEED_DIFF_THRESH) {
        // Front differential speed suppression below 0.02 m/s or straight-line driving
        speeds.fl = v_profiled;
        speeds.fr = v_profiled;
    } else {
        // Hypotenuse Ackermann speed kinematics with intermediate half track width
        float R = std::abs(v_profiled) / omega_profiled;

        // Ensure R magnitude is not smaller than R_MIN while preserving sign
        if (std::abs(R) < R_MIN) {
            R = (R >= 0.0f) ? R_MIN : -R_MIN;
        }

        float term_l = R - FRONT_INTERMEDIATE_HALF_TRACK;
        float term_r = R + FRONT_INTERMEDIATE_HALF_TRACK;
        float L_sq = WHEELBASE_M * WHEELBASE_M;

        speeds.fl = dir_ * std::abs(omega_profiled) * std::sqrt(term_l * term_l + L_sq);
        speeds.fr = dir_ * std::abs(omega_profiled) * std::sqrt(term_r * term_r + L_sq);
    }

    return speeds;
}

WheelRPMs KinematicsEngine::computeWheelRPMs(float v_profiled, float omega_profiled, float delta_v_corr, float& out_k_tcs) {
    WheelSpeeds kin_speeds = computeKinematicSpeeds(v_profiled, omega_profiled);

    // Global Ratio TCS Scaling (K_tcs)
    // Guarded at |v_profiled| >= 0.15 m/s to prevent low-speed ratio mathematical blowups
    if (std::abs(v_profiled) >= TCS_ABS_V_MIN) {
        out_k_tcs = (v_profiled + delta_v_corr) / v_profiled;
    } else {
        out_k_tcs = 1.0f;
    }

    WheelRPMs rpms;
    rpms.fl = (out_k_tcs * kin_speeds.fl) * K_v2rpm;
    rpms.fr = (out_k_tcs * kin_speeds.fr) * K_v2rpm;
    rpms.rl = (out_k_tcs * kin_speeds.rl) * K_v2rpm;
    rpms.rr = (out_k_tcs * kin_speeds.rr) * K_v2rpm;

    return rpms;
}
