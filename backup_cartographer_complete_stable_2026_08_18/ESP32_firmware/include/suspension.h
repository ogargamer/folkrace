#ifndef SUSPENSION_H
#define SUSPENSION_H

#include "robot_config.h"

/**
 * @brief Feedforward Active Roll Suspension operating via single central servo
 * 
 * Computes signed lateral acceleration a_y_ff = v_profiled * omega_profiled.
 * Positive a_y_ff (left turn) compresses inner left suspension arm (PWM > 1500 µs).
 * Negative a_y_ff (right turn) compresses inner right suspension arm (PWM < 1500 µs).
 * In reverse gear (v_profiled < 0), locks output to neutral (1500 µs).
 * Applies EMA LPF (alpha = 0.15) and slew rate limit (SUSP_SLEW_LIMIT = 25000 µs/s).
 */
class ActiveSuspension {
public:
    ActiveSuspension();

    /**
     * @brief Reset active suspension internal filter & slew state to neutral (1500 µs)
     */
    void reset();

    /**
     * @brief Update active suspension servo PWM command
     * @param v_profiled Profiled linear velocity (m/s)
     * @param omega_profiled Profiled angular velocity (rad/s)
     * @param dt Loop time step (seconds)
     * @return uint16_t Servo PWM pulse width in microseconds (µs)
     */
    uint16_t update(float v_profiled, float omega_profiled, float dt = CONTROL_LOOP_DT);

    uint16_t getLastPWM() const { return static_cast<uint16_t>(std::round(pwm_state_)); }

private:
    float pwm_state_{1500.0f};
};

#endif // SUSPENSION_H
