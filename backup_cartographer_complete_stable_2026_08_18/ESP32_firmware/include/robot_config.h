#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <cstdint>
#include <cmath>
#include <cstddef>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ============================================================================
// 1. LOOP TIMING & RATES
// ============================================================================
constexpr uint32_t CONTROL_LOOP_NOMINAL_HZ = 200;
constexpr float CONTROL_LOOP_DT = 1.0f / static_cast<float>(CONTROL_LOOP_NOMINAL_HZ); // 0.005s (5.0 ms)
constexpr uint32_t HOST_CMD_FREQ_HZ = 60;

// ============================================================================
// 2. CHASSIS PHYSICAL PARAMETERS & KINEMATICS
// ============================================================================
constexpr float ROBOT_MASS_KG = 1.0f;               // 1.0 kg mass
constexpr float COM_HEIGHT_M = 0.050f;               // 50 mm CoM height
constexpr float TRACK_WIDTH_CONTACT_M = 0.097f;      // 97 mm nominal contact patch center-to-center
constexpr float TRACK_WIDTH_KINGPIN_M = 0.072f;      // 72 mm kingpin pivot-to-pivot
constexpr float WHEELBASE_M = 0.127f;                // 127 mm wheelbase
constexpr float TIRE_RADIUS_M = 0.027f;              // 27 mm tire radius (54mm diameter)
constexpr float SCRUB_OFFSET_M = 0.0125f;            // 12.5 mm kingpin scrub offset

// Standard Aliases
constexpr float WHEELBASE = WHEELBASE_M;
constexpr float TIRE_RADIUS = TIRE_RADIUS_M;

// Derived Kinematic Half-Track Constants (constexpr)
constexpr float FRONT_KINGPIN_HALF_TRACK = TRACK_WIDTH_KINGPIN_M / 2.0f; // 0.036m (36mm)
constexpr float REAR_CONTACT_HALF_TRACK = TRACK_WIDTH_CONTACT_M / 2.0f;  // 0.0485m (48.5mm)
constexpr float FRONT_INTERMEDIATE_HALF_TRACK = 0.04225f;                // 0.04225m (42.25mm compromise track width)

// Speed to RPM conversion factor: RPM = v (m/s) * K_v2rpm
// K_v2rpm = 60.0 / (2.0 * M_PI * r) ~= 353.67765f RPM / (m/s)
constexpr float K_v2rpm = 60.0f / (2.0f * M_PI * TIRE_RADIUS_M);

// Steering Limits & Turning Radius Constraints
constexpr float DELTA_MAX_DEG = 45.0f;
constexpr float DELTA_MAX_RAD = DELTA_MAX_DEG * (M_PI / 180.0f); // ~0.785398f rad

// Minimum Turning Radius R_min = WHEELBASE / tan(DELTA_MAX_RAD)
// tan(45 deg) = 1.0, so R_MIN = 0.127m
constexpr float R_MIN = WHEELBASE_M / 1.0f; // tan(M_PI/4.0f) == 1.0f

// Hysteresis & Deadband Thresholds
constexpr float LAUNCH_STEER_V_DEADBAND = 0.05f;  // |v_profiled| < 0.05 m/s => delta = 0 deg
constexpr float HYSTERESIS_V_THRESH = 0.01f;       // dir hysteresis threshold (+0.01 / -0.01 m/s)
constexpr float LOW_SPEED_DIFF_THRESH = 0.02f;     // |v_profiled| < 0.02 m/s => diff suppression

// ============================================================================
// 3. ACTUATOR & SERVO LUT PARAMETERS
// ============================================================================
constexpr size_t STEER_LUT_SIZE = 9;
constexpr float STEER_LUT_ANGLES_DEG[STEER_LUT_SIZE] = {
    -45.0f, -28.0f, -15.0f, -5.0f, 0.0f, 5.0f, 15.0f, 28.0f, 45.0f
};

// Default Microseconds Servo Calibration for Signed Steering LUT
constexpr float STEER_LUT_PWM_US[STEER_LUT_SIZE] = {
    1000.0f, 1180.0f, 1330.0f, 1440.0f, 1500.0f, 1560.0f, 1670.0f, 1820.0f, 2000.0f
};

constexpr uint16_t SERVO_NEUTRAL_PWM_US = 1500;

// Active Suspension Single Servo Constants
constexpr float K_SUSP = 40.0f;              // µs / (m/s²)
constexpr float ALPHA_SUSP = 0.15f;           // LPF alpha constant
constexpr float SUSP_SLEW_LIMIT = 25000.0f;  // µs/s limit

// ============================================================================
// 4. MOTION PROFILING & TCS/ABS PARAMETERS
// ============================================================================
constexpr float A_MAX = 5.0f;                 // m/s² (Drive acceleration limit when increasing speed magnitude)
constexpr float D_MAX = 8.0f;                 // m/s² (Motor braking capability when decelerating / reversing)
constexpr float ALPHA_MAX = 20.0f;            // rad/s² (Max angular acceleration)

// TCS/ABS Global Engine Constants
constexpr float TCS_ABS_THRESH = 1.0f;        // m/s² signed acceleration error threshold
constexpr float TCS_ABS_STEP = 0.05f;         // m/s correction step per tick
constexpr float TCS_ABS_DECAY = 0.025f;       // m/s rapid decay rate per tick toward 0.0
constexpr float DELTA_V_CORR_MAX = 1.0f;      // m/s hard clamp bound delta_v_corr in [-1.0, +1.0]
constexpr float TCS_ABS_V_MIN = 0.15f;        // m/s min profiled speed to evaluate K_tcs ratio scaling

#endif // ROBOT_CONFIG_H
