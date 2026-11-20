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

    // Per-wheel torque voltage bounds & feedforward (V, robot forward convention)
    float vtrq_min_FL;
    float vtrq_max_FL;
    float vtrq_ff_FL;
    float vtrq_min_FR;
    float vtrq_max_FR;
    float vtrq_ff_FR;
    float vtrq_min_RL;
    float vtrq_max_RL;
    float vtrq_ff_RL;
    float vtrq_min_RR;
    float vtrq_max_RR;
    float vtrq_ff_RR;

    // Kinematics & Health
    float omega_cmd;          // Theoretical kinematic yaw rate from physical steer (rad/s)
    float omega_ref;          // Grip-capped, lagged reference yaw rate for oversteer/gyro (rad/s)
    float omega_tv_target;    // Short-lag uncapped kinematic yaw target for Rear TV (rad/s)
    float delta_omega_tv;     // Rear TV bounded slip bias (rad/s)
    float delta_vtrq_tv;      // Rear TV feedforward torque differential (V)
    float yaw_accel_filt;     // Clock-beat-guarded filtered yaw acceleration (rad/s^2)
    float scale_factor;       // RPM saturation scale factor (1.0 = nominal)
    VehicleSlipState slip_state;
    float yaw_error;          // Measured gyro_z minus omega_ref (rad/s)
    bool is_countersteering;  // True when steering is opposing active oversteer spin
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
     * @brief Computes 4WD electronic differential & torque envelope setpoints at 400 Hz.
     * Guaranteed single-precision math (float) with zero dynamic allocation.
     */
    ElectronicDiffOutputs update(
        float target_v,
        float target_steer_rad,
        float gyro_z_rad_s,
        float dt = CONTROL_LOOP_DT_S,
        float mu = DEFAULT_ROAD_MU,
        float v_curr = 0.0f,
        uint32_t gyro_seq = 0,
        uint64_t gyro_ts_us = 0
    );

    void setGyroAssistEnabled(bool enabled) { gyro_assist_enabled_ = enabled; }
    void setRearTVEnabled(bool enabled) { rear_tv_enabled_ = enabled; }
    void setFrontEnvelopeEnabled(bool enabled) { front_env_enabled_ = enabled; }
    void setRearTVGains(float kp_psi, float kd_psi, float max_vff) {
        tv_kp_psi_ = fmaxf(0.0f, kp_psi);
        tv_kd_psi_ = fmaxf(0.0f, kd_psi);
        tv_max_vff_ = fmaxf(0.0f, max_vff);
    }
    void setOversteerMuCap(float mu_os_cap) {
        mu_os_cap_ = fmaxf(0.40f, mu_os_cap);
    }

    bool isGyroAssistEnabled() const { return gyro_assist_enabled_; }
    bool isRearTVEnabled() const { return rear_tv_enabled_; }
    bool isFrontEnvelopeEnabled() const { return front_env_enabled_; }
    float getTvKpPsi() const { return tv_kp_psi_; }
    float getTvKdPsi() const { return tv_kd_psi_; }
    float getTvMaxVff() const { return tv_max_vff_; }
    float getOversteerMuCap() const { return mu_os_cap_; }

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
    float omega_tv_target_;
    float prev_gyro_z_;
    float gyro_z_dot_filt_;
    uint32_t prev_gyro_seq_;
    uint64_t prev_gyro_ts_us_;
    bool has_prev_gyro_;
    VehicleSlipState slip_state_;
    float slip_hold_timer_;
    float understeer_confirm_timer_;
    float oversteer_confirm_timer_;

    // Tunable runtime bring-up flags & gains
    bool gyro_assist_enabled_;
    bool rear_tv_enabled_;
    bool front_env_enabled_;
    float tv_kp_psi_;
    float tv_kd_psi_;
    float tv_max_vff_;
    float mu_os_cap_;
};

#endif // ELECTRONIC_DIFF_H

