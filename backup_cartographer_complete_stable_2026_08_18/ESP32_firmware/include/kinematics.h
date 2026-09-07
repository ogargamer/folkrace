#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "robot_config.h"
#include "hal_interfaces.h"

struct WheelSpeeds {
    float fl{0.0f}; // Front Left (m/s)
    float fr{0.0f}; // Front Right (m/s)
    float rl{0.0f}; // Rear Left (m/s)
    float rr{0.0f}; // Rear Right (m/s)
};

struct WheelRPMs {
    float fl{0.0f}; // Front Left (RPM)
    float fr{0.0f}; // Front Right (RPM)
    float rl{0.0f}; // Rear Left (RPM)
    float rr{0.0f}; // Rear Right (RPM)
};

/**
 * @brief Kinematics Engine for 4WD Direct-Drive Hypotenuse Ackermann Steering & Speed Allocation
 */
class KinematicsEngine {
public:
    KinematicsEngine();

    /**
     * @brief Reset kinematics internal direction hysteresis state
     */
    void reset();

    /**
     * @brief Compute target steering angle with launch standstill deadband guard and map through signed 9-point LUT
     * @param v_profiled Profiled linear velocity (m/s)
     * @param omega_profiled Profiled angular velocity (rad/s)
     * @param out_delta_rad Pointer to receive raw computed steering angle in radians (optional)
     * @return uint16_t Servo pulse width in microseconds (µs)
     */
    uint16_t computeSteeringPWM(float v_profiled, float omega_profiled, float* out_delta_rad = nullptr);

    /**
     * @brief Compute uncorrected 4-wheel kinematic linear speeds (m/s)
     * @param v_profiled Profiled linear velocity (m/s)
     * @param omega_profiled Profiled angular velocity (rad/s)
     * @return WheelSpeeds Linear speeds for FL, FR, RL, RR
     */
    WheelSpeeds computeKinematicSpeeds(float v_profiled, float omega_profiled);

    /**
     * @brief Compute 4-wheel target RPMs scaled by Global Ratio TCS scalar (K_tcs)
     * @param v_profiled Profiled linear velocity (m/s)
     * @param omega_profiled Profiled angular velocity (rad/s)
     * @param delta_v_corr Additive TCS/ABS velocity correction term (m/s)
     * @param out_k_tcs Reference to receive computed K_tcs ratio scalar
     * @return WheelRPMs Target motor RPMs for FL, FR, RL, RR
     */
    WheelRPMs computeWheelRPMs(float v_profiled, float omega_profiled, float delta_v_corr, float& out_k_tcs);

    /**
     * @brief Interpolate steering angle in degrees through signed 9-point Ackermann LUT
     * @param delta_deg Steering angle in degrees [-45..+45]
     * @return uint16_t Servo pulse width in microseconds (µs)
     */
    static uint16_t interpolateSteeringLUT(float delta_deg);

    float getDir() const { return dir_; }

private:
    float dir_{1.0f}; // Direction sign hysteresis state (+1.0 or -1.0)
};

#endif // KINEMATICS_H
