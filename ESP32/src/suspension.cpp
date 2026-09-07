#include "suspension.h"
#include <cmath>
#include <algorithm>

template<typename T>
static inline T clamp_val(T v, T lo, T hi) {
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

ActiveSuspension::ActiveSuspension() : pwm_state_(1500.0f) {}

void ActiveSuspension::reset() {
    pwm_state_ = 1500.0f;
}

uint16_t ActiveSuspension::update(float v_profiled, float omega_profiled, float dt) {
    if (dt <= 0.0f) {
        return static_cast<uint16_t>(std::round(pwm_state_));
    }

    float pwm_target = 1500.0f;

    // Reverse Gear Override: Force active suspension to neutral (1500 µs) when v_profiled < 0
    if (v_profiled < 0.0f) {
        pwm_target = 1500.0f;
    } else {
        // Forward cornering: Signed lateral acceleration a_y_ff = v_profiled * omega_profiled
        float a_y_ff = v_profiled * omega_profiled;
        float stroke = clamp_val(K_SUSP * a_y_ff, -500.0f, +500.0f);
        pwm_target = 1500.0f + stroke;
    }

    // 1. Exponential Moving Average LPF (alpha = 0.15)
    float pwm_lpf = pwm_state_ + ALPHA_SUSP * (pwm_target - pwm_state_);

    // 2. Slew Rate Limiter (SUSP_SLEW_LIMIT = 25000 µs/s)
    float max_delta_pwm = SUSP_SLEW_LIMIT * dt; // 25000 * 0.005 = 125 µs per tick
    float delta_pwm = pwm_lpf - pwm_state_;
    delta_pwm = clamp_val(delta_pwm, -max_delta_pwm, +max_delta_pwm);

    pwm_state_ += delta_pwm;
    pwm_state_ = clamp_val(pwm_state_, 1000.0f, 2000.0f);

    return static_cast<uint16_t>(std::round(pwm_state_));
}
