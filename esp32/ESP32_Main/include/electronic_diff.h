#ifndef ELECTRONIC_DIFF_H
#define ELECTRONIC_DIFF_H

#include <Arduino.h>
#include <cmath>
#include "config.h"

enum VehicleSlipState : uint8_t {
    SLIP_NORMAL = 0,
    SLIP_UNDERSTEER = 1,
    SLIP_OVERSTEER = 2
};

struct ElectronicDiffOutputs {
    // Steering
    float steer_rad;          // Physical center steering angle after gyro assist & slew limiting (rad)
    float steer_deg;          // Physical center steering angle (deg)
    float driver_steer_rad;   // Slew-limited driver/MPC steering angle before gyro assist (rad)
    float driver_steer_deg;   // Driver/MPC steering angle (deg)
    float delta_gyro_rad;     // Continuous PD gyro countersteer contribution (rad)
    float steer_left_rad;     // Geometric Ackermann left knuckle angle (rad)
    float steer_right_rad;    // Geometric Ackermann right knuckle angle (rad)
    float servo_pulse_us;     // Commanded pulse width for 333 Hz digital servo (us)

    // Linear wheel speeds (m/s)
    float rate_limited_v;     // Speed after dynamic traction limiter (m/s)
    float v_FL;
    float v_FR;
    float v_RL;
    float v_RR;

    // Motor shaft setpoints
    float omega_FL;           // Rad/s
    float omega_FR;
    float omega_RL;
    float omega_RR;
    float rpm_FL;             // Motor RPM
    float rpm_FR;
    float rpm_RL;
    float rpm_RR;

    // Kinematics & Health
    float omega_cmd;          // Theoretical kinematic yaw rate from physical steer (rad/s)
    float omega_ref;          // Grip-capped, lagged reference yaw rate (rad/s)
    float scale_factor;       // RPM saturation scale factor (1.0 = nominal)
    VehicleSlipState slip_state;
    float yaw_error;          // Measured gyro_z minus omega_ref (rad/s)
};

class ElectronicDifferential4WD {
public:
    ElectronicDifferential4WD(
        float wheelbase = VEHICLE_WHEELBASE_M,
        float track_rear = VEHICLE_TRACK_REAR_M,
        float track_front = VEHICLE_TRACK_FRONT_M,
        float track_kingpin = VEHICLE_TRACK_KINGPIN_M,
        float wheel_radius = VEHICLE_WHEEL_RADIUS_M,
        float max_steer = VEHICLE_MAX_STEER_RAD,
        float max_servo_rate = VEHICLE_SERVO_MAX_RATE,
        float ackermann_pct = VEHICLE_ACKERMANN_PCT,
        float motor_dir = VEHICLE_MOTOR_DIR,
        float v_max = VEHICLE_MAX_WHEEL_SPEED,
        float slip_gate = SLIP_GATE_SPEED_MPS,
        float thresh_spin = SLIP_THRESH_SPIN_RAD_S
    );

    void reset();

    /**
     * @brief Computes 4WD electronic differential setpoints at 400 Hz.
     * Guaranteed single-precision math (float) with zero dynamic allocation.
     */
    ElectronicDiffOutputs update(
        float target_v,
        float target_steer_rad,
        float gyro_z_rad_s,
        float dt = CONTROL_LOOP_DT_S,
        float mu = DEFAULT_ROAD_MU,
        float v_curr = 0.0f
    );

    float getCurrentSteerRad() const { return current_steer_rad_; }
    float getRateLimitedSpeed() const { return rate_limited_speed_; }

private:
    float L_;
    float W_r_;
    float W_f_;
    float W_k_;
    float R_w_;
    float max_steer_rad_;
    float max_servo_rate_;
    float ackermann_percent_;
    float motor_dir_;
    float v_max_wheel_;
    float v_slip_gate_;
    float thresh_spin_;

    // Internal continuous state
    float current_steer_rad_;
    float driver_steer_rad_;
    float prev_delta_gyro_rad_;
    float rate_limited_speed_;
    float omega_ref_;
    float prev_gyro_z_;
    float gyro_z_dot_filt_;
    bool has_prev_gyro_;
    VehicleSlipState slip_state_;
    float slip_hold_timer_;
    float understeer_confirm_timer_;
    float oversteer_confirm_timer_;
    float slip_entry_speed_;
    float diff_split_gain_;
    float early_us_filt_;
    float early_os_;
    float a_lat_obs_filt_;
};

#endif // ELECTRONIC_DIFF_H

