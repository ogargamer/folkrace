#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include <cmath>
#include "config.h"

struct FusionOutputs {
    float v_est;              // Estimated true longitudinal ground speed (m/s)
    float v_wheels;           // Outlier-rejected median wheel surface speed (m/s)
    float v_rear_avg;         // Average unsteered rear axle speed (m/s)
    float v_front_avg;        // Average projected front axle speed (m/s)
    float slip_ratio;         // Dynamic slip indicator [0.0 = full grip, 1.0 = severe slip]
    float yaw_rate;           // Calibrated Z-axis yaw rate (rad/s)
    float yaw_est;            // Estimated vehicle heading (rad)
    float total_distance_m;   // Integrated distance traveled (m)
};

class IMUEncoderFusion {
public:
    IMUEncoderFusion(
        float wheel_radius = VEHICLE_WHEEL_RADIUS_M,
        float track_rear = VEHICLE_TRACK_REAR_M,
        float track_front = VEHICLE_TRACK_FRONT_M,
        float wheelbase = VEHICLE_WHEELBASE_M,
        float motor_dir = VEHICLE_MOTOR_DIR,
        float dt = CONTROL_LOOP_DT_S
    );

    void reset();

    /**
     * @brief Executes one 400 Hz sensor fusion cycle.
     * Guaranteed single-precision math (float) with zero dynamic allocation.
     */
    FusionOutputs update(
        float fl_rpm,
        float fr_rpm,
        float rl_rpm,
        float rr_rpm,
        float gyro_z_rad_s,
        float accel_fwd_m_s2,
        float steer_rad,
        float dt = CONTROL_LOOP_DT_S,
        float front_vtrq_avg = 0.0f,
        float rear_vtrq_avg = 0.0f
    );

    float getVelocityEst() const { return v_est_; }
    float getYawEst() const { return yaw_est_; }
    float getDistanceM() const { return total_distance_m_; }
    float getSlipRatio() const { return slip_ratio_; }

private:
    float R_w_;
    float W_r_;
    float W_f_;
    float L_;
    float motor_dir_;
    float dt_;

    // Filter state
    float v_est_;
    float yaw_est_;
    float total_distance_m_;
    float slip_ratio_;
};

#endif // SENSOR_FUSION_H

