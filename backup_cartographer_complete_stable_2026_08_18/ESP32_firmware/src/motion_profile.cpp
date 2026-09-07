#include "motion_profile.h"
#include <cmath>
#include <algorithm>

MotionProfiler::MotionProfiler()
    : v_profiled_(0.0f), omega_profiled_(0.0f), a_prof_(0.0f) {}

void MotionProfiler::reset(float v, float omega) {
    v_profiled_ = v;
    omega_profiled_ = omega;
    a_prof_ = 0.0f;
}

static inline int sgn(float val) {
    if (val > 0.0f) return 1;
    if (val < 0.0f) return -1;
    return 0;
}

void MotionProfiler::update(float v_cmd, float omega_cmd, float dt) {
    if (dt <= 0.0f) return;

    float v_prev = v_profiled_;
    float delta_v = v_cmd - v_profiled_;

    // Speed Magnitude Rules:
    // If speed magnitude increases (sgn(v_current) == sgn(delta_v) or v_current == 0),
    // limit acceleration rate using drive torque limit a_max (5.0 m/s²).
    // If speed magnitude decreases or direction is reversing (sgn(v_current) != sgn(delta_v)),
    // limit rate using motor braking capability d_max (8.0 m/s²).
    float a_limit = A_MAX;
    if (v_profiled_ != 0.0f && sgn(v_profiled_) != sgn(delta_v)) {
        a_limit = D_MAX;
    }

    float max_delta_v = a_limit * dt;

    if (delta_v > 0.0f) {
        float step = std::min(delta_v, max_delta_v);
        // Ensure zero crossing is passed cleanly when decelerating to rest
        if (v_prev < 0.0f && (v_prev + step) > 0.0f) {
            v_profiled_ = 0.0f;
        } else {
            v_profiled_ = v_prev + step;
        }
    } else if (delta_v < 0.0f) {
        float step = std::max(delta_v, -max_delta_v);
        // Ensure zero crossing is passed cleanly when decelerating to rest
        if (v_prev > 0.0f && (v_prev + step) < 0.0f) {
            v_profiled_ = 0.0f;
        } else {
            v_profiled_ = v_prev + step;
        }
    }

    // Compute profiler linear acceleration derivative (m/s²)
    a_prof_ = (v_profiled_ - v_prev) / dt;

    // Profile angular velocity with alpha_max limit (20.0 rad/s²)
    float delta_omega = omega_cmd - omega_profiled_;
    float max_delta_omega = ALPHA_MAX * dt;
    if (delta_omega > 0.0f) {
        omega_profiled_ += std::min(delta_omega, max_delta_omega);
    } else if (delta_omega < 0.0f) {
        omega_profiled_ += std::max(delta_omega, -max_delta_omega);
    }
}
