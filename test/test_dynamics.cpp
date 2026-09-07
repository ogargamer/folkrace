#include <iostream>
#include <cassert>
#include <cmath>
#include "slip_control.h"
#include "suspension.h"
#include "robot_config.h"

// Helper assertion macros
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

void test_tcs_spinout_step_and_decay() {
    std::cout << "[TEST DYNAMICS 1] TCS Spinout Step & Rapid Decay..." << std::endl;
    SlipController controller;

    float v_profiled = 2.0f;
    float a_prof = 5.0f;
    float a_x_imu = 1.0f; // Esigned = +4.0 m/s² > 1.0 m/s² threshold

    // 1. Step down -0.05 m/s per tick for 20 ticks until clamped at -1.0 m/s
    for (int tick = 1; tick <= 20; ++tick) {
        float corr = controller.update(v_profiled, a_prof, a_x_imu);
        float expected = -0.05f * tick;
        ASSERT_FLOAT_EQ(corr, expected, 1e-4f);
    }

    // 21st tick: must remain clamped at -1.0 m/s
    float corr_clamped = controller.update(v_profiled, a_prof, a_x_imu);
    ASSERT_FLOAT_EQ(corr_clamped, -1.0f, 1e-4f);

    // 2. Normal traction restored: Esigned = 0.0 m/s² (a_x_imu = a_prof = 5.0 m/s²)
    // Rapid decay toward 0.0 m/s at 0.025 m/s per tick (40 ticks for full release = 0.2s)
    a_x_imu = 5.0f;
    for (int tick = 1; tick <= 40; ++tick) {
        float corr = controller.update(v_profiled, a_prof, a_x_imu);
        float expected = -1.0f + 0.025f * tick;
        ASSERT_FLOAT_EQ(corr, expected, 1e-4f);
    }

    // After 40 ticks, delta_v_corr must be back to 0.0 m/s
    float corr_zero = controller.update(v_profiled, a_prof, a_x_imu);
    ASSERT_FLOAT_EQ(corr_zero, 0.0f, 1e-6f);

    std::cout << "  -> PASSED!" << std::endl;
}

void test_abs_lockup() {
    std::cout << "[TEST DYNAMICS 2] ABS Lockup Step Up..." << std::endl;
    SlipController controller;

    float v_profiled = 2.0f;
    float a_prof = -8.0f;
    float a_x_imu = -2.0f; // Esigned = -8.0 - (-2.0) = -6.0 m/s² < -1.0 m/s² threshold

    // Step up +0.05 m/s per tick up to +1.0 m/s (20 ticks)
    for (int tick = 1; tick <= 20; ++tick) {
        float corr = controller.update(v_profiled, a_prof, a_x_imu);
        float expected = +0.05f * tick;
        ASSERT_FLOAT_EQ(corr, expected, 1e-4f);
    }

    // 21st tick: clamped at +1.0 m/s
    float corr_clamped = controller.update(v_profiled, a_prof, a_x_imu);
    ASSERT_FLOAT_EQ(corr_clamped, +1.0f, 1e-4f);

    std::cout << "  -> PASSED!" << std::endl;
}

void test_signed_active_suspension() {
    std::cout << "[TEST DYNAMICS 3] Signed Feedforward Active Suspension..." << std::endl;
    ActiveSuspension suspension;

    // 1. Left Turn: v = 2.0 m/s, omega = 1.5 rad/s => a_y_ff = +3.0 m/s²
    // Target PWM = 1500 + 40 * 3.0 = 1620 µs (> 1500 µs, compresses inner left arm)
    uint16_t pwm_left = 1500;
    for (int i = 0; i < 50; ++i) {
        pwm_left = suspension.update(2.0f, 1.5f, CONTROL_LOOP_DT);
    }
    ASSERT_TRUE(pwm_left > 1500, "Left turn must produce PWM > 1500 µs");
    ASSERT_FLOAT_EQ(static_cast<float>(pwm_left), 1620.0f, 1.0f);

    // Reset suspension for clean right-turn test
    suspension.reset();

    // 2. Right Turn: v = 2.0 m/s, omega = -1.5 rad/s => a_y_ff = -3.0 m/s²
    // Target PWM = 1500 - 40 * 3.0 = 1380 µs (< 1500 µs, compresses inner right arm)
    uint16_t pwm_right = 1500;
    for (int i = 0; i < 50; ++i) {
        pwm_right = suspension.update(2.0f, -1.5f, CONTROL_LOOP_DT);
    }
    ASSERT_TRUE(pwm_right < 1500, "Right turn must produce PWM < 1500 µs");
    ASSERT_FLOAT_EQ(static_cast<float>(pwm_right), 1380.0f, 1.0f);

    std::cout << "  -> PASSED!" << std::endl;
}

void test_reverse_gear_overrides() {
    std::cout << "[TEST DYNAMICS 4] Reverse Gear Overrides & Mode Re-seeding..." << std::endl;
    SlipController controller;
    ActiveSuspension suspension;

    // Build up non-zero delta_v_corr in forward gear
    controller.update(2.0f, 5.0f, 1.0f);
    ASSERT_TRUE(controller.getDeltaVCorr() < 0.0f, "Forward spinout should produce negative delta_v_corr");

    // Enter Reverse gear (v_profiled < 0)
    float corr_rev = controller.update(-1.5f, -2.0f, -1.0f);
    ASSERT_FLOAT_EQ(corr_rev, 0.0f, 1e-6f); // Bypassed: forced to 0.0

    uint16_t pwm_rev = suspension.update(-1.5f, 2.0f, CONTROL_LOOP_DT);
    ASSERT_TRUE(pwm_rev == 1500, "Reverse gear suspension must remain locked at 1500 µs neutral");

    // Trigger re-seed before forward transition
    controller.triggerReSeed();
    float corr_reseeded = controller.update(0.01f, 0.0f, 0.0f);
    ASSERT_FLOAT_EQ(corr_reseeded, 0.0f, 1e-6f);

    std::cout << "  -> PASSED!" << std::endl;
}

int main() {
    std::cout << "====================================================" << std::endl;
    std::cout << " RUNNING PHASE 2 NATIVE DYNAMICS UNIT TESTS        " << std::endl;
    std::cout << "====================================================" << std::endl;

    test_tcs_spinout_step_and_decay();
    test_abs_lockup();
    test_signed_active_suspension();
    test_reverse_gear_overrides();

    std::cout << "====================================================" << std::endl;
    std::cout << " ALL PHASE 2 VIRTUAL UNIT TESTS PASSED SUCCESSFULLY!" << std::endl;
    std::cout << "====================================================" << std::endl;
    return 0;
}
