#include <iostream>
#include <cassert>
#include <cmath>
#include "motion_profile.h"
#include "kinematics.h"
#include "hal_interfaces.h"

// Helper macro for floating-point equality assertions
#define ASSERT_FLOAT_EQ(val1, val2, tol) \
    do { \
        float diff = std::abs((val1) - (val2)); \
        if (diff > (tol)) { \
            std::cerr << "Assertion Failed at " << __FILE__ << ":" << __LINE__ \
                      << " | Expected: " << (val2) << ", Got: " << (val1) \
                      << " (diff: " << diff << " > tol: " << (tol) << ")" << std::endl; \
            std::exit(1); \
        } \
    } while(0)

#define ASSERT_TRUE(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion Failed at " << __FILE__ << ":" << __LINE__ \
                      << " | " << msg << std::endl; \
            std::exit(1); \
        } \
    } while(0)

void test_kinetic_energy_profiling() {
    std::cout << "[TEST 1] Kinetic Energy Profiling..." << std::endl;
    MotionProfiler profiler;

    // 1. Step target v_cmd = 3.0 m/s from rest (v = 0.0)
    profiler.update(3.0f, 0.0f, CONTROL_LOOP_DT);
    // Expected v_profiled after 1 tick: 0.0 + 5.0 * 0.005 = 0.025 m/s
    ASSERT_FLOAT_EQ(profiler.getVProfiled(), 0.025f, 1e-5f);
    ASSERT_FLOAT_EQ(profiler.getAProf(), 5.0f, 1e-4f); // Respects a_max = 5.0 m/s²

    // Run until profiler reaches 3.0 m/s
    for (int i = 0; i < 150; ++i) {
        profiler.update(3.0f, 0.0f, CONTROL_LOOP_DT);
    }
    ASSERT_FLOAT_EQ(profiler.getVProfiled(), 3.0f, 1e-5f);

    // 2. Step target v_cmd = -2.0 m/s from +3.0 m/s
    // Deceleration down to zero must respect d_max = 8.0 m/s²
    profiler.update(-2.0f, 0.0f, CONTROL_LOOP_DT);
    // Expected v_profiled after 1 tick of braking: 3.0 - 8.0 * 0.005 = 2.96 m/s
    ASSERT_FLOAT_EQ(profiler.getVProfiled(), 2.96f, 1e-5f);
    ASSERT_FLOAT_EQ(profiler.getAProf(), -8.0f, 1e-4f); // Respects d_max = 8.0 m/s²

    // Step through deceleration until speed reaches 0.0 m/s (2.96 / 0.04 = 74 more ticks)
    for (int i = 0; i < 74; ++i) {
        profiler.update(-2.0f, 0.0f, CONTROL_LOOP_DT);
    }
    ASSERT_FLOAT_EQ(profiler.getVProfiled(), 0.0f, 1e-5f);

    // 3. Accelerating backward from rest (v_current = 0.0 m/s) towards -2.0 m/s must respect a_max = 5.0 m/s²
    profiler.update(-2.0f, 0.0f, CONTROL_LOOP_DT);
    // Expected v_profiled after 1 tick from rest: 0.0 - 5.0 * 0.005 = -0.025 m/s
    ASSERT_FLOAT_EQ(profiler.getVProfiled(), -0.025f, 1e-5f);
    ASSERT_FLOAT_EQ(profiler.getAProf(), -5.0f, 1e-4f); // Respects a_max = 5.0 m/s²

    std::cout << "  -> PASSED!" << std::endl;
}

void test_signed_lut_interpolation() {
    std::cout << "[TEST 2] Signed LUT Interpolation..." << std::endl;

    // Check control points directly
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(-45.0f) == 1000, "LUT -45 deg failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(-28.0f) == 1180, "LUT -28 deg failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(-15.0f) == 1330, "LUT -15 deg failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(-5.0f) == 1440, "LUT -5 deg failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(0.0f) == 1500, "LUT 0 deg failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(5.0f) == 1560, "LUT +5 deg failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(15.0f) == 1670, "LUT +15 deg failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(28.0f) == 1820, "LUT +28 deg failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(45.0f) == 2000, "LUT +45 deg failed");

    // Check mid-point interpolations
    // Between -5 deg (1440) and 0 deg (1500) at -2.5 deg => 1470
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(-2.5f) == 1470, "LUT -2.5 deg failed");
    // Between 0 deg (1500) and +5 deg (1560) at +2.5 deg => 1530
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(2.5f) == 1530, "LUT +2.5 deg failed");

    // Out-of-bounds clamping check
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(-50.0f) == 1000, "LUT -50 deg clamp failed");
    ASSERT_TRUE(KinematicsEngine::interpolateSteeringLUT(50.0f) == 2000, "LUT +50 deg clamp failed");

    std::cout << "  -> PASSED!" << std::endl;
}

void test_launch_steering_deadband() {
    std::cout << "[TEST 3] Launch Steering Deadband..." << std::endl;
    KinematicsEngine engine;

    float delta_rad = -999.0f;
    uint16_t pwm = engine.computeSteeringPWM(0.0f, 0.0f, &delta_rad);
    ASSERT_FLOAT_EQ(delta_rad, 0.0f, 1e-6f);
    ASSERT_TRUE(pwm == 1500, "Launch deadband at rest (v=0, omega=0) failed");

    // Low speed below deadband threshold (0.03 m/s < 0.05 m/s) with non-zero angular rate
    pwm = engine.computeSteeringPWM(0.03f, 1.5f, &delta_rad);
    ASSERT_FLOAT_EQ(delta_rad, 0.0f, 1e-6f);
    ASSERT_TRUE(pwm == 1500, "Launch deadband below threshold (v=0.03 m/s) failed");

    // Speed above deadband threshold (0.10 m/s >= 0.05 m/s)
    pwm = engine.computeSteeringPWM(0.10f, 0.0f, &delta_rad);
    ASSERT_FLOAT_EQ(delta_rad, 0.0f, 1e-6f);
    ASSERT_TRUE(pwm == 1500, "Steering at v=0.10 m/s straight line failed");

    std::cout << "  -> PASSED!" << std::endl;
}

void test_global_ratio_scaling_and_guard() {
    std::cout << "[TEST 4] Global Ratio Scaling (K_tcs) & Threshold Guard..." << std::endl;
    KinematicsEngine engine;

    // 1. At v_profiled = 2.0 m/s (>= 0.15 m/s) with delta_v_corr = -0.5 m/s
    // Expected K_tcs = (2.0 + (-0.5)) / 2.0 = 0.75
    float k_tcs_calc = 0.0f;
    WheelRPMs uncorrected_rpms = engine.computeWheelRPMs(2.0f, 0.0f, 0.0f, k_tcs_calc);
    ASSERT_FLOAT_EQ(k_tcs_calc, 1.0f, 1e-6f);

    WheelRPMs corrected_rpms = engine.computeWheelRPMs(2.0f, 0.0f, -0.5f, k_tcs_calc);
    ASSERT_FLOAT_EQ(k_tcs_calc, 0.75f, 1e-6f);

    // Verify all 4 wheel target RPMs scale by exactly K_tcs = 0.75 relative to uncorrected
    ASSERT_FLOAT_EQ(corrected_rpms.fl, 0.75f * uncorrected_rpms.fl, 1e-4f);
    ASSERT_FLOAT_EQ(corrected_rpms.fr, 0.75f * uncorrected_rpms.fr, 1e-4f);
    ASSERT_FLOAT_EQ(corrected_rpms.rl, 0.75f * uncorrected_rpms.rl, 1e-4f);
    ASSERT_FLOAT_EQ(corrected_rpms.rr, 0.75f * uncorrected_rpms.rr, 1e-4f);

    // 2. Low-speed guard active: v_profiled = 0.10 m/s (< 0.15 m/s) with delta_v_corr = -0.5 m/s
    // K_tcs must remain forced at 1.0
    WheelRPMs low_speed_rpms = engine.computeWheelRPMs(0.10f, 0.0f, -0.5f, k_tcs_calc);
    ASSERT_FLOAT_EQ(k_tcs_calc, 1.0f, 1e-6f);

    std::cout << "  -> PASSED!" << std::endl;
}

void test_reverse_wheel_speed_and_hysteresis() {
    std::cout << "[TEST 5] Reverse Wheel Speed Differentiation & Direction Hysteresis..." << std::endl;
    KinematicsEngine engine;

    // 1. Pass cornering command in reverse: v = -1.5 m/s, omega = 2.0 rad/s
    float k_tcs = 1.0f;
    WheelRPMs rpms = engine.computeWheelRPMs(-1.5f, 2.0f, 0.0f, k_tcs);

    // In reverse (dir = -1.0):
    // FR is the outer front wheel, FL is the inner front wheel.
    // Assert outer front wheel target RPM magnitude is GREATER than inner front wheel magnitude
    ASSERT_TRUE(std::abs(rpms.fr) > std::abs(rpms.fl), "Outer front wheel RPM magnitude must be greater than inner front wheel");
    ASSERT_TRUE(rpms.fl < 0.0f && rpms.fr < 0.0f, "Front wheel RPMs must be negative in reverse");
    ASSERT_FLOAT_EQ(engine.getDir(), -1.0f, 1e-6f);

    // 2. Direction Hysteresis across slow zero transitions
    // v_profiled moving through zero: -0.008, 0.005, 0.009 (all within [-0.01, +0.01])
    engine.computeKinematicSpeeds(-0.008f, 0.0f);
    ASSERT_FLOAT_EQ(engine.getDir(), -1.0f, 1e-6f); // Retains -1.0

    engine.computeKinematicSpeeds(0.005f, 0.0f);
    ASSERT_FLOAT_EQ(engine.getDir(), -1.0f, 1e-6f); // Retains -1.0 across zero crossing

    engine.computeKinematicSpeeds(0.009f, 0.0f);
    ASSERT_FLOAT_EQ(engine.getDir(), -1.0f, 1e-6f); // Still retains -1.0 below +0.01 m/s threshold

    // Exceed +0.01 m/s threshold => switches to +1.0
    engine.computeKinematicSpeeds(0.02f, 0.0f);
    ASSERT_FLOAT_EQ(engine.getDir(), 1.0f, 1e-6f); // Switched to +1.0

    std::cout << "  -> PASSED!" << std::endl;
}

int main() {
    std::cout << "====================================================" << std::endl;
    std::cout << " RUNNING PHASE 1 NATIVE KINEMATICS UNIT TESTS      " << std::endl;
    std::cout << "====================================================" << std::endl;

    test_kinetic_energy_profiling();
    test_signed_lut_interpolation();
    test_launch_steering_deadband();
    test_global_ratio_scaling_and_guard();
    test_reverse_wheel_speed_and_hysteresis();

    std::cout << "====================================================" << std::endl;
    std::cout << " ALL PHASE 1 VIRTUAL UNIT TESTS PASSED SUCCESSFULLY!" << std::endl;
    std::cout << "====================================================" << std::endl;
    return 0;
}
