#ifndef SLIP_CONTROL_H
#define SLIP_CONTROL_H

#include "robot_config.h"
#include <atomic>

/**
 * @brief Unified Global Ratio TCS/ABS Slip Controller operating on signed acceleration error
 * 
 * Computes signed acceleration error Esigned = a_prof - a_x_body_imu.
 * Adjusts an additive velocity correction term delta_v_corr in range [-1.0, +1.0] m/s.
 * Bypasses TCS/ABS in reverse gear (v_profiled < 0).
 */
class SlipController {
public:
    SlipController();

    /**
     * @brief Reset slip controller state to zero correction
     */
    void reset();

    /**
     * @brief Trigger thread-safe state re-seeding via std::atomic<bool> first_run
     */
    void triggerReSeed();

    /**
     * @brief Update TCS/ABS additive correction term delta_v_corr
     * @param v_profiled Profiled linear velocity (m/s)
     * @param a_prof Profiled linear acceleration (m/s²)
     * @param a_x_body_imu Body-frame linear acceleration from IMU (m/s²)
     * @return float Additive correction term delta_v_corr (m/s)
     */
    float update(float v_profiled, float a_prof, float a_x_body_imu);

    float getDeltaVCorr() const { return delta_v_corr_; }

private:
    float delta_v_corr_{0.0f};
    std::atomic<bool> first_run_{true};
};

#endif // SLIP_CONTROL_H
