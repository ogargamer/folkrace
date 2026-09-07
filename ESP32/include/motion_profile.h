#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include "robot_config.h"

/**
 * @brief Dual-Axis Motion Profiler with Kinetic Energy Bounding
 * 
 * Applies drive acceleration limit a_max (5.0 m/s²) when increasing speed magnitude,
 * and motor FOC braking limit d_max (8.0 m/s²) during deceleration or direction reversals.
 * Limits angular acceleration rate by alpha_max (20.0 rad/s²).
 */
class MotionProfiler {
public:
    MotionProfiler();

    /**
     * @brief Reset internal profiled states
     * @param v Initial linear velocity (m/s)
     * @param omega Initial angular velocity (rad/s)
     */
    void reset(float v = 0.0f, float omega = 0.0f);

    /**
     * @brief Update profiler state for one control loop iteration dt
     * @param v_cmd Linear velocity target (m/s)
     * @param omega_cmd Angular velocity target (rad/s)
     * @param dt Loop time step (seconds)
     */
    void update(float v_cmd, float omega_cmd, float dt = CONTROL_LOOP_DT);

    float getVProfiled() const { return v_profiled_; }
    float getOmegaProfiled() const { return omega_profiled_; }
    float getAProf() const { return a_prof_; }

private:
    float v_profiled_{0.0f};
    float omega_profiled_{0.0f};
    float a_prof_{0.0f};
};

#endif // MOTION_PROFILE_H
