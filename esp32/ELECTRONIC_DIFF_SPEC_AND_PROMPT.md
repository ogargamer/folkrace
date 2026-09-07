# 4WD Electronic Differential & Reliability-Focused Slip Detection Specification
## High-Performance Real-Time Control for ESP32-S3 (200 Hz / 5.0 ms Loop)

---

### 1. Executive Summary & Hardware Context

This specification defines the firmware requirements and exact mathematical formulations for an active **4WD Electronic Differential, True Geometric Ackermann Kinematics, and Reliability-Focused IMU Slip Detection** running on an **ESP32-S3 microcontroller at 200 Hz (5.0 ms loop time)**.

#### Hardware Architecture:
- **MCU**: ESP32-S3 dual-core Xtensa LX7 @ 240 MHz.
- **Actuators**: 4 independent brushless DC hub / drive motors controlled via closed-loop Field Oriented Control (FOC) drivers.
- **Control Interface**: Velocity setpoints in radians/second (or m/s linear) sent to each FOC motor controller. No phase ammeters/torque feedback are available, so motor speed tracking is stiff and guaranteed by local FOC PID.
- **Steering**: Front-wheel Ackermann steering driven by independent left/right servos or single servo linkage with physical lock at +/- 30 degrees (0.5236 rad).
- **IMU**: BNO08x 9-DOF IMU positioned near Center of Gravity (COG), reporting calibrated Z-axis yaw rate (rad/s) and lateral acceleration (m/s^2).
- **Host Interface (Raspberry Pi 5)**: Computes high-level path planning, obstacle avoidance, trajectory curvature, and lookahead speed profiling, streaming target forward speed `v` (m/s) and virtual center steering angle `delta` (rad) to the ESP32-S3 over UART/CAN.

---

### 2. Vehicle Geometric Parameters

Values extracted directly from the physical vehicle design:

| Parameter | Symbol | Value (SI) | Value (Imperial/Design) | Description |
| :--- | :--- | :--- | :--- | :--- |
| **Wheelbase** | `L` | `0.1325 m` | `132.5 mm` | Distance between front and rear axle centers |
| **Rear Track Width** | `W_r` | `0.1000 m` | `100.0 mm` | Distance between rear left and right tire centers |
| **Front Track Width** | `W_f` | `0.0822 m` | `82.2 mm` | Distance between front left and right tire centers |
| **Kingpin Pivot Track**| `W_k` | `0.0620 m` | `62.0 mm` | Distance between front steering kingpin pivot axes |
| **Wheel Rolling Radius**| `R_w` | `0.0275 m` | `27.5 mm` | Effective rolling radius of tires |
| **Max Steering Angle** | `MAX_STEER`| `0.5236 rad` | `30.0 deg` | Physical steering lock limit |
| **Max Wheel Speed** | `v_max_wheel`| `3.85 m/s` | `140 rad/s` | Motor hardware RPM ceiling |
| **Slip Detection Gate**| `v_slip_gate`| `0.30 m/s` | ~1.1 km/h | Low-speed threshold to bypass IMU noise |
| **Spinout Threshold** | `thresh_spin`| `0.45 rad/s` | ~25.8 deg/s | Gross yaw rate divergence limit for oversteer |

---

### 3. Mathematical Formulations

#### A. Input Clamping & Kinematic Yaw Rate Command
To eliminate all division-by-zero singularities (which occur on straightaways if using radius `R = L / tan(delta)`), compute the commanded kinematic yaw rate `omega_cmd` directly:

1. `delta_clamped = clamp(delta, -MAX_STEER, MAX_STEER)`
2. `omega_cmd = (v * tan(delta_clamped)) / L`

*Properties:*
- At straight running (`delta == 0`), `tan(0) = 0` => `omega_cmd = 0.0 rad/s`.
- At standstill (`v == 0`), `omega_cmd = 0.0 rad/s`.
- In reverse (`v < 0`), `omega_cmd` automatically inverts sign, naturally matching vehicle backing dynamics.

---

#### B. 100% Perfect Independent Front Ackermann Angles
For dual-servo or independent virtual steering actuators, calculate the exact geometric steering angle for inner and outer wheels based on kingpin spacing `W_k`:

If `abs(tan(delta_clamped)) > 1e-4`:
- `steer_left = atan2(L * sin(delta_clamped), L * cos(delta_clamped) - (W_k * 0.5) * sin(delta_clamped))`
- `steer_right = atan2(L * sin(delta_clamped), L * cos(delta_clamped) + (W_k * 0.5) * sin(delta_clamped))`
Else:
- `steer_left = delta_clamped`
- `steer_right = delta_clamped`

Clamp both to `[-MAX_STEER, MAX_STEER]`.

---

#### C. 4-Wheel Speed Allocation (Forward & Reverse)
Each wheel follows a unique kinematic circular arc around the Center of Instantaneous Rotation (CIR):

1. Direction sign:
   `sign_v = (v >= 0.0) ? 1.0 : -1.0`

2. Intermediate axle differentials:
   `half_Wr_omega = omega_cmd * (W_r * 0.5)`
   `half_Wf_omega = omega_cmd * (W_f * 0.5)`

3. Rear Axle Speeds (pure rolling along longitudinal heading):
   `v_RL = v - half_Wr_omega`
   `v_RR = v + half_Wr_omega`

4. Front Axle Speeds (along individual Ackermann wheel heading vectors):
   `fl_x = v - half_Wf_omega`
   `fr_x = v + half_Wf_omega`
   `front_y = L * omega_cmd`
   `v_FL = sign_v * sqrt(fl_x * fl_x + front_y * front_y)`
   `v_FR = sign_v * sqrt(fr_x * fr_x + front_y * front_y)`

---

#### D. Standstill Guard & Proportional RPM Saturation Scaling
1. **Standstill Guard:**
   If `fabs(v) < 0.001 m/s`:
   - Force `v_FL = v_FR = v_RL = v_RR = 0.0 m/s`.
   - Reset all slip detection debounce counters and filters.

2. **Proportional Saturation Scaling (ROS / ArduPilot approach):**
   In hard cornering at high speed, the outer wheels demand higher velocity than the vehicle center `v`. If any wheel exceeds physical motor limits, scale **all 4 wheels proportionally** so the vehicle turning radius is preserved without uncommanded yaw jerks:
   - `v_peak = max(fabs(v_FL), max(fabs(v_FR), max(fabs(v_RL), fabs(v_RR))))`
   - If `v_peak > v_max_wheel`:
     - `scale = v_max_wheel / v_peak`
     - `v_FL = v_FL * scale`
     - `v_FR = v_FR * scale`
     - `v_RL = v_RL * scale`
     - `v_RR = v_RR * scale`

3. **Rotational Output (rad/s):**
   - `omega_FL = (v_FL / R_w) * MOTOR_DIR`
   - `omega_FR = (v_FR / R_w) * MOTOR_DIR`
   - `omega_RL = (v_RL / R_w) * MOTOR_DIR`
   - `omega_RR = (v_RR / R_w) * MOTOR_DIR`

---

##### E. Reliability-Focused BNO08x Slip Detection Logic

By setting the reference target directly to `r_target = omega_cmd`, we achieve **0.0% artificial baseline error** across all steering angles (avoiding the small-angle tangent discrepancy that fails on small-scale vehicles).

1. **Low-Speed Gating:**
   If `fabs(v) < v_slip_gate`:
   - State is `SLIP_NORMAL`. (Sensor noise/vibration at crawl speed must never cause false tripping).

2. **Dynamic Threshold Scaling (Tire Slip Compliance):**
   `dyn_thresh = fmaxf(thresh_spin, 0.40f * fabsf(omega_cmd))`
   - Base threshold covers IMU noise at gentle turns.
   - At high speeds, tire slip angle naturally causes measured yaw rate to be 15% to 30% lower than pure kinematic roll. Scaling prevents normal cornering lag from falsely tripping oversteer.

3. **Classification:**
   - **True Oversteer (Rotational Runaway in Turn Direction):**
     `if (omega_cmd > 0.05f && (omega_gyro_z - omega_cmd) > dyn_thresh)` -> `SLIP_OVERSTEER`
     `else if (omega_cmd < -0.05f && (omega_cmd - omega_gyro_z) > dyn_thresh)` -> `SLIP_OVERSTEER`
   - **Counter-Steer Slide / Severe Spinout:**
     `else if (fabsf(omega_gyro_z) > fmaxf(1.20f, thresh_spin * 2.0f) && (omega_gyro_z * omega_cmd <= 0.0f))` -> `SLIP_OVERSTEER`
   - **Severe Understeer / Wall Plow (Wheels steered hard, vehicle plowing straight):**
     `else if (fabsf(delta) > 0.12f && fabsf(omega_cmd) > 0.80f && fabsf(omega_gyro_z) < (0.35f * fabsf(omega_cmd)))` -> `SLIP_UNDERSTEER`
   - **Normal Tracking:**
     `else` -> `SLIP_NORMAL`

---

### 4. ESP32-S3 Reference C++ Header Implementation

```cpp
#pragma once
#include <Arduino.h>
#include <math.h>

enum VehicleSlipState {
    SLIP_NORMAL = 0,
    SLIP_OVERSTEER = 1,
    SLIP_UNDERSTEER = 2
};

struct ElectronicDiffOutputs {
    float steer_left_rad;
    float steer_right_rad;
    float v_FL_m_s;
    float v_FR_m_s;
    float v_RL_m_s;
    float v_RR_m_s;
    float omega_FL_rad_s;
    float omega_FR_rad_s;
    float omega_RL_rad_s;
    float omega_RR_rad_s;
    float omega_cmd_rad_s;
    float scale_factor;
    VehicleSlipState slip_state;
    float yaw_error;
};

class ElectronicDifferential4WD {
public:
    // Vehicle Physical Geometry (meters)
    float L;                // Wheelbase: 0.1325 m
    float W_r;              // Rear Track: 0.1000 m
    float W_f;              // Front Track: 0.0822 m
    float W_k;              // Kingpin Track: 0.0620 m
    float R_w;              // Wheel Radius: 0.0275 m
    float max_steer_rad;    // Physical limit: +/- 0.5236 rad (30 deg)
    float motor_dir;        // Direction scalar (+1.0 or -1.0)
    float v_max_wheel;      // Motor velocity ceiling: 4.32 m/s (1500 RPM)

    // Slip Detection Thresholds
    float v_slip_gate;      // Minimum speed: 0.30 m/s
    float thresh_spin;      // Base yaw rate divergence threshold: 0.45 rad/s

    ElectronicDifferential4WD(float wheelbase = 0.1325f,
                               float track_rear = 0.1000f,
                               float track_front = 0.0822f,
                               float track_kingpin = 0.0620f,
                               float wheel_radius = 0.0275f,
                               float max_steer = 0.5236f,
                               float motor_direction = -1.0f,
                               float v_max = 4.32f,
                               float slip_gate = 0.30f,
                               float spin_thresh = 0.45f)
        : L(wheelbase), W_r(track_rear), W_f(track_front), W_k(track_kingpin),
          R_w(wheel_radius), max_steer_rad(max_steer), motor_dir(motor_direction),
          v_max_wheel(v_max), v_slip_gate(slip_gate), thresh_spin(spin_thresh) {}

    ElectronicDiffOutputs update(float target_v, float target_steer_rad, float gyro_z_rad_s) {
        ElectronicDiffOutputs out;

        // 1. Clamp Steering Command
        float delta = fmaxf(-max_steer_rad, fminf(target_steer_rad, max_steer_rad));

        // 2. Exact Dual-Servo Geometric Ackermann
        float sin_d = sinf(delta);
        float cos_d = cosf(delta);
        float L_sin = L * sin_d;
        float L_cos = L * cos_d;
        float half_Wk_sin = (W_k * 0.5f) * sin_d;

        out.steer_left_rad = atan2f(L_sin, L_cos - half_Wk_sin);
        out.steer_right_rad = atan2f(L_sin, L_cos + half_Wk_sin);

        // 3. Command Yaw Rate
        out.omega_cmd_rad_s = (target_v * tanf(delta)) / L;

        // 4. Standstill Guard
        if (fabsf(target_v) < 0.001f) {
            out.v_FL_m_s = out.v_FR_m_s = out.v_RL_m_s = out.v_RR_m_s = 0.0f;
            out.omega_FL_rad_s = out.omega_FR_rad_s = out.omega_RL_rad_s = out.omega_RR_rad_s = 0.0f;
            out.omega_cmd_rad_s = 0.0f;
            out.scale_factor = 1.0f;
            out.slip_state = SLIP_NORMAL;
            out.yaw_error = 0.0f;
            return out;
        }

        // 5. 4-Wheel Speed Allocation
        float sign_v = (target_v >= 0.0f) ? 1.0f : -1.0f;
        float half_Wr_omega = out.omega_cmd_rad_s * (W_r * 0.5f);
        float half_Wf_omega = out.omega_cmd_rad_s * (W_f * 0.5f);

        out.v_RL_m_s = target_v - half_Wr_omega;
        out.v_RR_m_s = target_v + half_Wr_omega;

        float fl_x = target_v - half_Wf_omega;
        float fr_x = target_v + half_Wf_omega;
        float front_y = L * out.omega_cmd_rad_s;

        out.v_FL_m_s = sign_v * hypotf(fl_x, front_y);
        out.v_FR_m_s = sign_v * hypotf(fr_x, front_y);

        // 6. Proportional Saturation Scaling
        float v_peak = fmaxf(fmaxf(fabsf(out.v_FL_m_s), fabsf(out.v_FR_m_s)),
                             fmaxf(fabsf(out.v_RL_m_s), fabsf(out.v_RR_m_s)));
        out.scale_factor = 1.0f;
        if (v_peak > v_max_wheel) {
            out.scale_factor = v_max_wheel / v_peak;
            out.v_FL_m_s *= out.scale_factor;
            out.v_FR_m_s *= out.scale_factor;
            out.v_RL_m_s *= out.scale_factor;
            out.v_RR_m_s *= out.scale_factor;
        }

        // 7. Angular Velocities for FOC Controllers
        out.omega_FL_rad_s = (out.v_FL_m_s / R_w) * motor_dir;
        out.omega_FR_rad_s = (out.v_FR_m_s / R_w) * motor_dir;
        out.omega_RL_rad_s = (out.v_RL_m_s / R_w) * motor_dir;
        out.omega_RR_rad_s = (out.v_RR_m_s / R_w) * motor_dir;

        // 8. Reliability-Focused IMU Slip Detection
        out.slip_state = SLIP_NORMAL;
        out.yaw_error = gyro_z_rad_s - out.omega_cmd_rad_s;

        if (fabsf(target_v) >= v_slip_gate) {
            float dyn_thresh = fmaxf(thresh_spin, 0.40f * fabsf(out.omega_cmd_rad_s));

            if (out.omega_cmd_rad_s > 0.05f && (gyro_z_rad_s - out.omega_cmd_rad_s) > dyn_thresh) {
                out.slip_state = SLIP_OVERSTEER;
            } else if (out.omega_cmd_rad_s < -0.05f && (out.omega_cmd_rad_s - gyro_z_rad_s) > dyn_thresh) {
                out.slip_state = SLIP_OVERSTEER;
            } else if (fabsf(gyro_z_rad_s) > fmaxf(1.20f, thresh_spin * 2.0f) && (gyro_z_rad_s * out.omega_cmd_rad_s <= 0.0f)) {
                out.slip_state = SLIP_OVERSTEER;
            } else {
                float omega_achievable = fminf(fabsf(out.omega_cmd_rad_s), (0.90f * 9.81f) / fmaxf(fabsf(target_v), 0.50f));
                if (fabsf(delta) > 0.15f && omega_achievable > 1.20f && fabsf(gyro_z_rad_s) < (0.35f * omega_achievable)) {
                    out.slip_state = SLIP_UNDERSTEER;
                }
            }
        }

        return out;
    }
};
```
