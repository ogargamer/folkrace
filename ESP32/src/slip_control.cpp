#include "slip_control.h"
#include <cmath>
#include <algorithm>

template<typename T>
static inline T clamp_val(T v, T lo, T hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

SlipController::SlipController() : delta_v_corr_(0.0f), first_run_(true) {}

void SlipController::reset() {
    delta_v_corr_ = 0.0f;
    first_run_.store(true, std::memory_order_release);
}

void SlipController::triggerReSeed() {
    first_run_.store(true, std::memory_order_release);
}

float SlipController::update(float v_profiled, float a_prof, float a_x_body_imu) {
    // Thread-safe cross-core state re-seeding check
    if (first_run_.exchange(false, std::memory_order_acq_rel)) {
        delta_v_corr_ = 0.0f;
    }

    // Reverse Gear Safety Override: Disable TCS/ABS when reversing (v_profiled < 0)
    if (v_profiled < 0.0f) {
        delta_v_corr_ = 0.0f;
        return delta_v_corr_;
    }

    // Calculate signed acceleration error: Esigned = a_prof - a_x_body_imu
    float E_signed = a_prof - a_x_body_imu;

    // Spinout Condition: Accelerating forward & positive acceleration error > 1.0 m/s²
    if (a_prof > 0.0f && E_signed > TCS_ABS_THRESH) {
        delta_v_corr_ = std::max(-DELTA_V_CORR_MAX, delta_v_corr_ - TCS_ABS_STEP);
    }
    // Lockup Condition: Decelerating forward & negative acceleration error < -1.0 m/s²
    else if (a_prof < 0.0f && E_signed < -TCS_ABS_THRESH) {
        delta_v_corr_ = std::min(+DELTA_V_CORR_MAX, delta_v_corr_ + TCS_ABS_STEP);
    }
    // Normal Traction (|Esigned| <= 1.0 m/s²): Rapidly decay delta_v_corr toward 0.0 m/s (0.025 m/s per tick)
    else {
        if (delta_v_corr_ > 0.0f) {
            delta_v_corr_ = std::max(0.0f, delta_v_corr_ - TCS_ABS_DECAY);
        } else if (delta_v_corr_ < 0.0f) {
            delta_v_corr_ = std::min(0.0f, delta_v_corr_ + TCS_ABS_DECAY);
        }
    }

    delta_v_corr_ = clamp_val(delta_v_corr_, -DELTA_V_CORR_MAX, +DELTA_V_CORR_MAX);
    return delta_v_corr_;
}
