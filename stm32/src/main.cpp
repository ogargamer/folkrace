#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "config_pins.h"
#include "mt6816_spi.h"

// ============================================================================
// DA WINNA — 20.0 kHz HARD REAL-TIME DUAL-MOTOR SIMPLEFOC ENGINE (STM32G474)
// ============================================================================
// Hardware: STM32G474CBT6 (170 MHz Cortex-M4F, HW FPU)
// Timers: TIM1 (M1 PWM 25kHz), TIM8 (M2 PWM 25kHz), TIM6 (20 kHz Control ISR)
// Sensors: Dual MT6816 14-Bit Magnetic Encoders on Shared SPI1 @ 10.6 MHz
// Telemetry: Exact 26-Byte Binary Telemetry Frame @ 25 Hz + ASCII Console
// ============================================================================

// ----------------------------------------------------------------------------
// 26-Byte Binary Telemetry Frame (Exact match with Python GUI struct)
// ----------------------------------------------------------------------------
struct __attribute__((packed)) TelemetryPacket {
    uint8_t  header[2];      // 0xAA, 0x55 (Sync word)          [Bytes 0..1]
    uint16_t m1_raw_angle;   // 0 .. 16383 (Raw 14-bit count)    [Bytes 2..3]
    int16_t  m1_actual_rpm;  // Scaled x10 (e.g. 4000 = 400.0)  [Bytes 4..5]
    int16_t  m1_target_rpm;  // Scaled x10                       [Bytes 6..7]
    int16_t  m1_uq_mv;       // Phase voltage Uq (mV)            [Bytes 8..9]
    uint8_t  m1_status;      // Bitfield: En, LUT, Mag, Par, St  [Byte 10]
    uint16_t m2_raw_angle;   // 0 .. 16383 (Raw 14-bit count)    [Bytes 11..12]
    int16_t  m2_actual_rpm;  // Scaled x10                       [Bytes 13..14]
    int16_t  m2_target_rpm;  // Scaled x10                       [Bytes 15..16]
    int16_t  m2_uq_mv;       // Phase voltage Uq (mV)            [Bytes 17..18]
    uint8_t  m2_status;      // Bitfield: En, LUT, Mag, Par, St  [Byte 19]
    uint16_t foc_khz_x10;    // 200 = 20.0 kHz                   [Bytes 20..21]
    uint8_t  isr_lat_us;     // Peak ISR latency in µs           [Byte 22]
    uint8_t  spikes_count;   // Jitter spikes counter            [Byte 23]
    uint8_t  calib_progress; // 0 .. 100% calibration progress   [Byte 24]
    uint8_t  checksum;       // XOR sum over bytes 2..24         [Byte 25]
};
static_assert(sizeof(TelemetryPacket) == 26, "TelemetryPacket must be exactly 26 bytes");

// ----------------------------------------------------------------------------
// Global Peripherals & Motor Instances
// ----------------------------------------------------------------------------
MT6816Sensor encoder1(PIN_M1_CS);
MT6816Sensor encoder2(PIN_M2_CS);

BLDCDriver3PWM driver1(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);
BLDCDriver3PWM driver2(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN);

BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(MOTOR_POLE_PAIRS);

HardwareTimer *foc_timer = nullptr;

// ----------------------------------------------------------------------------
// System & Real-Time Operational State
// ----------------------------------------------------------------------------
volatile bool m1_enabled = false;
volatile bool m2_enabled = false;
volatile bool m1_calibrated = false;
volatile bool m2_calibrated = false;

volatile float m1_target_rpm = 0.0f;
volatile float m2_target_rpm = 0.0f;

volatile bool m1_calib_active = false;
volatile bool m2_calib_active = false;
volatile uint8_t live_calib_progress = 0; // 0..100%

// DWT Latency & Real-Time Profiling Variables
volatile uint32_t foc_tick_count = 0;
volatile uint32_t isr_peak_cycles = 0;
volatile uint32_t isr_last_cycles = 0;
volatile uint8_t  isr_jitter_spikes = 0;

// Stall & Safety Watchdog Tracking
bool stall_watchdog_enabled = true;
uint32_t m1_stall_timer = 0;
uint32_t m2_stall_timer = 0;
bool m1_is_stalled = false;
bool m2_is_stalled = false;

// Telemetry & Diagnostic Timers
uint32_t last_telemetry_time = 0;
uint32_t last_led_time = 0;
bool led_state = false;
bool show_telemetry = true;

// Serial Line Buffer for Command Parsing
char rx_line[256];
uint16_t rx_idx = 0;

// ----------------------------------------------------------------------------
// Hard Real-Time 20.0 kHz Hardware Timer ISR (TIM6, NVIC Priority 2)
// ----------------------------------------------------------------------------
void focTimerISR() {
    uint32_t start_cycles = DWT->CYCCNT;
    foc_tick_count++;

    // 1. Single-Read SPI Caching: Poll MT6816 sensor strictly ONCE per tick
    encoder1.readHardwareSPI();
    // encoder2.readHardwareSPI(); // Disabled for single-encoder SPI1 bench test

    // 2. Motor 1 Commutation Loop
    if (m1_enabled) {
        if (!m1_calib_active) {
            motor1.loopFOC(); // Calls encoder1.update() -> cached angle, ZERO SPI overhead!
        }
    }

    // 3. Motor 2 Commutation Loop
    if (m2_enabled) {
        if (!m2_calib_active) {
            motor2.loopFOC(); // Calls encoder2.update() -> cached angle, ZERO SPI overhead!
        }
    }

    // 4. Downsampled Velocity Motion Control (Every 10th tick = 2.0 kHz / 500 us)
    if ((foc_tick_count % MOTION_DOWNSAMPLE_RATIO) == 0) {
        if (m1_enabled && !m1_calib_active) {
            motor1.move(m1_target_rpm * 0.104719755f);
        } else if (!m1_enabled) {
            encoder1.update();
        }

        if (m2_enabled && !m2_calib_active) {
            motor2.move(m2_target_rpm * 0.104719755f);
        } else if (!m2_enabled) {
            encoder2.update();
        }
    }

    // 5. Profiling & Jitter Tracking
    uint32_t elapsed = DWT->CYCCNT - start_cycles;
    isr_last_cycles = elapsed;
    if (elapsed > isr_peak_cycles) {
        isr_peak_cycles = elapsed;
    }
    // 25.0 us @ 170 MHz = 4,250 cycles
    if (elapsed > 4250) {
        isr_jitter_spikes++;
    }
}

// ----------------------------------------------------------------------------
// Pre-Loaded Zero Angle Initialization Helper
// ----------------------------------------------------------------------------
void initMotorFOC(int motor_id) {
    if (motor_id == 1) {
        driver1.enable();
        if (M1_DEFAULT_ZERO_ELECTRIC_ANGLE != 0.0f) {
            motor1.zero_electric_angle = M1_DEFAULT_ZERO_ELECTRIC_ANGLE;
            motor1.sensor_direction = Direction::CW;
            motor1.initFOC();
            m1_calibrated = true;
            Serial.println(">>> M1: Silent alignment loaded from config (No jerk).");
        } else {
            Serial.println(">>> M1: Running standard SimpleFOC alignment...");
            if (motor1.initFOC()) {
                m1_calibrated = true;
                Serial.print(">>> M1 ALIGN OK: ZeroAngle=");
                Serial.print(motor1.zero_electric_angle, 4);
                Serial.print(" Dir=");
                Serial.println(motor1.sensor_direction == Direction::CW ? "CW" : "CCW");
            } else {
                m1_calibrated = false;
                m1_enabled = false;
                driver1.disable();
                Serial.println(">>> M1 ALIGN FAILED!");
            }
        }
    } else if (motor_id == 2) {
        driver2.enable();
        if (M2_DEFAULT_ZERO_ELECTRIC_ANGLE != 0.0f) {
            motor2.zero_electric_angle = M2_DEFAULT_ZERO_ELECTRIC_ANGLE;
            motor2.sensor_direction = Direction::CW;
            motor2.initFOC();
            m2_calibrated = true;
            Serial.println(">>> M2: Silent alignment loaded from config (No jerk).");
        } else {
            Serial.println(">>> M2: Running standard SimpleFOC alignment...");
            if (motor2.initFOC()) {
                m2_calibrated = true;
                Serial.print(">>> M2 ALIGN OK: ZeroAngle=");
                Serial.print(motor2.zero_electric_angle, 4);
                Serial.print(" Dir=");
                Serial.println(motor2.sensor_direction == Direction::CW ? "CW" : "CCW");
            } else {
                m2_calibrated = false;
                m2_enabled = false;
                driver2.disable();
                Serial.println(">>> M2 ALIGN FAILED!");
            }
        }
    }
}

// ----------------------------------------------------------------------------
// Calibration Helpers & Angle Normalization
// ----------------------------------------------------------------------------
void sendBinaryTelemetry(); // Forward declaration for real-time telemetry streaming

static inline float normalizeAngleRad(float angle) {
    float a = fmodf(angle, _2PI);
    return (a >= 0.0f) ? a : (a + _2PI);
}

static inline float wrapAngleRad(float angle) {
    while (angle > _PI) angle -= _2PI;
    while (angle < -_PI) angle += _2PI;
    return angle;
}

static inline bool checkCalibrationAbort() {
    if (Serial.available()) {
        char c = (char)Serial.peek();
        if (c == 'x' || c == 'X' || c == 's' || c == 'S' || c == ' ') {
            while (Serial.available()) Serial.read();
            return true;
        }
    }
    return false;
}

// Circular 1D inverse-distance interpolation from measured samples to 128 raw LUT segments
static float interpolateLUTAtAngle(float target_rad, const float* sample_angles, const float* sample_errors, int n) {
    target_rad = normalizeAngleRad(target_rad);
    float best_dist1 = 1e9f, best_dist2 = 1e9f;
    int idx1 = 0, idx2 = 0;

    for (int i = 0; i < n; i++) {
        float d = fabsf(wrapAngleRad(sample_angles[i] - target_rad));
        if (d < best_dist1) {
            best_dist2 = best_dist1;
            idx2 = idx1;
            best_dist1 = d;
            idx1 = i;
        } else if (d < best_dist2) {
            best_dist2 = d;
            idx2 = i;
        }
    }

    float total_dist = best_dist1 + best_dist2;
    if (total_dist < 1e-6f) return sample_errors[idx1];
    float w1 = best_dist2 / total_dist;
    float w2 = best_dist1 / total_dist;
    return (w1 * sample_errors[idx1] + w2 * sample_errors[idx2]);
}

// ----------------------------------------------------------------------------
// Stepped 128-Point Calibration Sweep (Dual-Pass Bidirectional + Stall Checks)
// ----------------------------------------------------------------------------
void runSteppedCalibration(int motor_id) {
    Serial.println("\n================================================================================");
    Serial.print(">>> 🔬 STARTING 128-POINT DUAL-PASS LUT CALIBRATION FOR MOTOR ");
    Serial.println(motor_id);
    Serial.println("================================================================================");
    Serial.println("Motor Model        : GM3506 (11 Pole Pairs on STM32G474)");
    Serial.printf ("Step / Hold Voltage: %.2f V (move) / %.2f V (hold & sample)\n", CALIB_ALIGN_VOLTAGE, CALIB_HOLD_VOLTAGE);
    Serial.println("Methodology        : 128-Step Dual-Pass Bidirectional Mechanical Sweep");
    Serial.printf ("Oversampling       : %d Samples / Step with Micro-Dither\n", CALIB_OVERSAMPLE_COUNT);
    Serial.printf ("Anomaly Guard      : |err| > %.3f rad (%.1f deg) -> FAIL (pole slip/stall)\n",
                   LUT_ANOMALY_THRESHOLD_RAD, LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f);
    Serial.println("Press 'x' or 'STOP' anytime to abort");
    Serial.println("--------------------------------------------------------------------------------");

    BLDCMotor &motor = (motor_id == 1) ? motor1 : motor2;
    BLDCDriver3PWM &driver = (motor_id == 1) ? driver1 : driver2;
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;
    volatile bool &calib_active = (motor_id == 1) ? m1_calib_active : m2_calib_active;
    bool prev_enabled = (motor_id == 1) ? m1_enabled : m2_enabled;

    calib_active = true;
    live_calib_progress = 0;
    encoder.clearLUT(); // Disable old LUT to measure pure raw sensor characteristics
    driver.enable();

    // ------------------------------------------------------------------------
    // Step 1: Pre-flight Hardware Health Check
    // ------------------------------------------------------------------------
    Serial.print("1. Checking MT6816 Sensor Health... ");
    delay(50); // Allow 20 kHz ISR to sample sensor at least 1,000 times

    if (encoder.last_no_mag) {
        Serial.println("FAILED!");
        Serial.println("   ❌ ABORT: MT6816 reports NO_MAG flag (airgap too large or magnet missing).");
        Serial.println("   FIX: Position diametric magnet 1.0 - 2.0 mm directly above MT6816 chip.");
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        if (!prev_enabled) driver.disable();
        calib_active = false;
        live_calib_progress = 0;
        Serial.print("[CALIB_FAILED]:M"); Serial.print(motor_id); Serial.println(" Magnet Missing");
        return;
    }

    if (!encoder.last_parity_ok || encoder.parity_errors > 200) {
        Serial.println("FAILED!");
        Serial.printf ("   ❌ ABORT: MT6816 SPI Parity Errors detected (total: %lu)!\n", encoder.parity_errors);
        Serial.println("   FIX: Check SPI1 wiring (SCK/MISO/MOSI/CS), grounding, or termination.");
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        if (!prev_enabled) driver.disable();
        calib_active = false;
        live_calib_progress = 0;
        Serial.print("[CALIB_FAILED]:M"); Serial.print(motor_id); Serial.println(" SPI Parity Error");
        return;
    }
    Serial.println("OK (Magnet present, Parity OK)");

    // ------------------------------------------------------------------------
    // Step 2: Commutation Direction & Initial Rotor Stall Verification
    // ------------------------------------------------------------------------
    Serial.print("2. Determining Sensor Direction & Verifying Motion (Stall Check)... ");

    // Settle at electrical zero vector
    motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, _3PI_2);
    delay(150);
    motor.setPhaseVoltage(CALIB_HOLD_VOLTAGE, 0.0f, _3PI_2);
    delay(150);

    float start_raw = encoder.getRawMechanicalAngle();

    // Sweep 200 micro-steps across 2 electrical revolutions
    const int DIR_STEPS = 200;
    for (int step = 0; step <= DIR_STEPS; step++) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            if (!prev_enabled) driver.disable();
            calib_active = false;
            live_calib_progress = 0;
            return;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, el_ang);
        delayMicroseconds(1500);
    }
    delay(100);
    float mid_raw = encoder.getRawMechanicalAngle();

    // Sweep back to starting point
    for (int step = DIR_STEPS; step >= 0; step--) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            if (!prev_enabled) driver.disable();
            calib_active = false;
            live_calib_progress = 0;
            return;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, el_ang);
        delayMicroseconds(1500);
    }
    delay(100);

    float d_ang = wrapAngleRad(mid_raw - start_raw);
    float moved_deg = fabsf(d_ang) * 57.2957795f;
    float expected_deg = (360.0f * 2.0f / (float)MOTOR_POLE_PAIRS) * 0.30f; // ~19.6 deg minimum

    if (fabsf(d_ang) < 0.15f) { // Less than 8.6 deg mech
        Serial.println("FAILED!");
        Serial.println("   -------------------------------------------------------------");
        Serial.printf ("   ❌ ROTOR MOTION CHECK FAILED: Rotor moved only %.2f deg mech (expected >= %.2f deg)!\n",
                       moved_deg, expected_deg);
        Serial.printf ("      Start Angle: %.4f rad (%.1f deg) | Mid Angle: %.4f rad (%.1f deg)\n",
                       start_raw, start_raw * 57.2957795f, mid_raw, mid_raw * 57.2957795f);
        Serial.println("   ⚠️ ROOT CAUSE DIAGNOSTICS:");
        Serial.println("      1. 12V DC Bench Supply is switched OFF or current limit tripped.");
        Serial.println("      2. Motor phase connector unplugged or high-resistance solder joint.");
        Serial.println("      3. Inverter Gate Driver is disabled or damaged.");
        Serial.println("      4. Physical mechanical binding (bearing locked, gear jamming).");
        Serial.println("   -------------------------------------------------------------");
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        if (!prev_enabled) driver.disable();
        calib_active = false;
        live_calib_progress = 0;
        Serial.print("[CALIB_FAILED]:M"); Serial.print(motor_id); Serial.println(" Rotor Stalled (No Motion)");
        return;
    }

    Direction sensor_dir = (d_ang > 0.0f) ? Direction::CW : Direction::CCW;
    int dir_sign = (sensor_dir == Direction::CW) ? 1 : -1;
    Serial.printf("OK: Detected %s (%+.2f deg mech)\n", (dir_sign == 1) ? "CW (+1)" : "CCW (-1)", moved_deg);

    // ------------------------------------------------------------------------
    // Step 3: Bidirectional 128-Step Sweep Across 360 Mechanical Degrees
    // ------------------------------------------------------------------------
    Serial.println("\n3. Executing Bidirectional 128-Step Sweep Across 360° Mechanical:");
    const int TOTAL_STEPS = CALIB_TOTAL_STEPS;
    const float step_mech_rad = _2PI / (float)TOTAL_STEPS;

    float fwd_measured[TOTAL_STEPS];
    float rev_measured[TOTAL_STEPS];
    int consecutive_stalls = 0;

    // --- PASS 1: FORWARD SWEEP ---
    for (int i = 0; i < TOTAL_STEPS; i++) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            if (!prev_enabled) driver.disable();
            calib_active = false;
            live_calib_progress = 0;
            return;
        }

        float target_mech = (float)i * step_mech_rad;
        float target_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * target_mech;

        // Smooth micro-stepping transition from previous step
        if (i > 0) {
            const int INTERP = 8;
            float prev_mech = (float)(i - 1) * step_mech_rad;
            float prev_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * prev_mech;
            for (int s = 1; s <= INTERP; s++) {
                float el = prev_elec + (target_elec - prev_elec) * ((float)s / (float)INTERP);
                motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, el);
                delayMicroseconds(1000);
            }
        } else {
            motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, target_elec);
            delay(150);
        }

        // Micro-dither stiction cancellation
        for (int d = 4; d >= 1; d--) {
            float dither = 0.05f * ((float)d / 4.0f);
            motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, target_elec + dither);
            delayMicroseconds(400);
            motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, target_elec - dither);
            delayMicroseconds(400);
        }

        // Thermal hold & settle
        motor.setPhaseVoltage(CALIB_HOLD_VOLTAGE, 0.0f, target_elec);
        delay(25);

        // Oversampled vector averaging
        float sum_sin = 0.0f, sum_cos = 0.0f;
        for (int s = 0; s < CALIB_OVERSAMPLE_COUNT; s++) {
            float a = encoder.getRawMechanicalAngle();
            sum_sin += sinf(a);
            sum_cos += cosf(a);
            delayMicroseconds(100);
        }
        fwd_measured[i] = normalizeAngleRad(atan2f(sum_sin, sum_cos));

        // Per-Step Stall & Slip Monitor
        if (i > 0) {
            float step_delta = wrapAngleRad(fwd_measured[i] - fwd_measured[i - 1]);
            if ((float)dir_sign * step_delta <= 0.003f) {
                consecutive_stalls++;
                if (consecutive_stalls >= 4) {
                    Serial.println("\n❌ FORWARD SWEEP FAILED: ROTOR STALLED (No movement over 4 steps)!");
                    Serial.printf ("   Step: %d / %d | Last Step Delta: %+.4f rad\n", i, TOTAL_STEPS, step_delta);
                    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
                    if (!prev_enabled) driver.disable();
                    calib_active = false;
                    live_calib_progress = 0;
                    Serial.print("[CALIB_FAILED]:M"); Serial.print(motor_id); Serial.println(" Forward Stall");
                    return;
                }
            } else {
                consecutive_stalls = 0;
            }
        }

        live_calib_progress = (uint8_t)((i * 50) / TOTAL_STEPS);
        sendBinaryTelemetry();

        if ((i + 1) % 16 == 0 || i == TOTAL_STEPS - 1) {
            Serial.printf("   Forward Sweep: %d%% Complete\n", (int)(((float)(i + 1) / (float)TOTAL_STEPS) * 100.0f));
            Serial.printf("[CALIB_PROG]:%d%%\n", live_calib_progress);
        }
    }

    // --- THERMAL RELIEF PAUSE (1.0s) ---
    Serial.println(">>> ❄️ Thermal Relief Pause (1.0s)...");
    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    delay(1000);

    // --- PASS 2: REVERSE SWEEP ---
    consecutive_stalls = 0;
    float rev_start_mech = (float)(TOTAL_STEPS - 1) * step_mech_rad;
    float rev_start_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * rev_start_mech;
    motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, rev_start_elec);
    delay(100);

    for (int i = TOTAL_STEPS - 1; i >= 0; i--) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            if (!prev_enabled) driver.disable();
            calib_active = false;
            live_calib_progress = 0;
            return;
        }

        float target_mech = (float)i * step_mech_rad;
        float target_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * target_mech;

        // Smooth micro-stepping transition
        if (i < TOTAL_STEPS - 1) {
            const int INTERP = 8;
            float prev_mech = (float)(i + 1) * step_mech_rad;
            float prev_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * prev_mech;
            for (int s = 1; s <= INTERP; s++) {
                float el = prev_elec + (target_elec - prev_elec) * ((float)s / (float)INTERP);
                motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, el);
                delayMicroseconds(1000);
            }
        }

        // Micro-dither stiction cancellation
        for (int d = 4; d >= 1; d--) {
            float dither = 0.05f * ((float)d / 4.0f);
            motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, target_elec + dither);
            delayMicroseconds(400);
            motor.setPhaseVoltage(CALIB_ALIGN_VOLTAGE, 0.0f, target_elec - dither);
            delayMicroseconds(400);
        }

        // Thermal hold & settle
        motor.setPhaseVoltage(CALIB_HOLD_VOLTAGE, 0.0f, target_elec);
        delay(25);

        // Oversampled vector averaging
        float sum_sin = 0.0f, sum_cos = 0.0f;
        for (int s = 0; s < CALIB_OVERSAMPLE_COUNT; s++) {
            float a = encoder.getRawMechanicalAngle();
            sum_sin += sinf(a);
            sum_cos += cosf(a);
            delayMicroseconds(100);
        }
        rev_measured[i] = normalizeAngleRad(atan2f(sum_sin, sum_cos));

        // Per-Step Stall Monitor (Reverse)
        if (i < TOTAL_STEPS - 1) {
            float step_delta = wrapAngleRad(rev_measured[i] - rev_measured[i + 1]);
            if ((float)dir_sign * step_delta >= -0.003f) {
                consecutive_stalls++;
                if (consecutive_stalls >= 4) {
                    Serial.println("\n❌ REVERSE SWEEP FAILED: ROTOR STALLED (No movement over 4 steps)!");
                    Serial.printf ("   Step: %d / %d | Last Step Delta: %+.4f rad\n", i, TOTAL_STEPS, step_delta);
                    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
                    if (!prev_enabled) driver.disable();
                    calib_active = false;
                    live_calib_progress = 0;
                    Serial.print("[CALIB_FAILED]:M"); Serial.print(motor_id); Serial.println(" Reverse Stall");
                    return;
                }
            } else {
                consecutive_stalls = 0;
            }
        }

        live_calib_progress = 50 + (uint8_t)(((TOTAL_STEPS - 1 - i) * 50) / TOTAL_STEPS);
        sendBinaryTelemetry();

        if (i % 16 == 0 || i == 0) {
            Serial.printf("   Reverse Sweep: %d%% Complete\n", (int)(((float)(TOTAL_STEPS - i) / (float)TOTAL_STEPS) * 100.0f));
            Serial.printf("[CALIB_PROG]:%d%%\n", live_calib_progress);
        }
    }

    // ------------------------------------------------------------------------
    // Step 4: Midpoint Synthesis, Filtering & Quality Diagnostics
    // ------------------------------------------------------------------------
    float start_mid = normalizeAngleRad(atan2f(sinf(fwd_measured[0]) + sinf(rev_measured[0]),
                                               cosf(fwd_measured[0]) + cosf(rev_measured[0])));
    float raw_errors[TOTAL_STEPS];
    float mid_angles[TOTAL_STEPS];
    float filtered_errors[TOTAL_STEPS];

    for (int i = 0; i < TOTAL_STEPS; i++) {
        float mid_meas = normalizeAngleRad(atan2f(sinf(fwd_measured[i]) + sinf(rev_measured[i]),
                                                   cosf(fwd_measured[i]) + cosf(rev_measured[i])));
        float expected_meas = normalizeAngleRad(start_mid + (float)dir_sign * ((float)i * step_mech_rad));
        raw_errors[i] = wrapAngleRad(mid_meas - expected_meas);
        mid_angles[i] = mid_meas;
    }

    // 3-point circular median filter (eliminates isolated single-point glitches)
    float max_err = -100.0f, min_err = 100.0f, sum_sq_err = 0.0f;
    int anomaly_count = 0;

    for (int i = 0; i < TOTAL_STEPS; i++) {
        int prev_i = (i - 1 + TOTAL_STEPS) % TOTAL_STEPS;
        int next_i = (i + 1) % TOTAL_STEPS;
        float a = raw_errors[prev_i];
        float b = raw_errors[i];
        float c = raw_errors[next_i];
        float med = ((a >= b && a <= c) || (a <= b && a >= c)) ? a :
                    ((b >= a && b <= c) || (b <= a && b >= c)) ? b : c;

        float filtered_err = (fabsf(b - med) > 0.08f) ? med : b;
        filtered_errors[i] = filtered_err;

        if (filtered_err > max_err) max_err = filtered_err;
        if (filtered_err < min_err) min_err = filtered_err;
        sum_sq_err += filtered_err * filtered_err;

        if (fabsf(filtered_err) > LUT_ANOMALY_THRESHOLD_RAD) {
            anomaly_count++;
            if (anomaly_count <= 4) {
                Serial.printf("   ⚠️ Anomaly idx[%3d] @ %.1f deg mech | err = %+.4f rad (%+.2f deg)\n",
                              i, ((float)i / (float)TOTAL_STEPS) * 360.0f, filtered_err, filtered_err * 57.2957795f);
            }
        }
    }

    // Fourier harmonic extraction (Harmonic 1 = runout, Harmonic 2 = tilt)
    float f_a1 = 0.0f, f_b1 = 0.0f, f_a2 = 0.0f, f_b2 = 0.0f;
    for (int i = 0; i < TOTAL_STEPS; i++) {
        float angle_mech = (float)i * step_mech_rad;
        f_a1 += filtered_errors[i] * cosf(angle_mech);
        f_b1 += filtered_errors[i] * sinf(angle_mech);
        f_a2 += filtered_errors[i] * cosf(2.0f * angle_mech);
        f_b2 += filtered_errors[i] * sinf(2.0f * angle_mech);
    }
    float h1_amp_deg = (2.0f * sqrtf(f_a1 * f_a1 + f_b1 * f_b1) / (float)TOTAL_STEPS) * 57.2957795f;
    float h2_amp_deg = (2.0f * sqrtf(f_a2 * f_a2 + f_b2 * f_b2) / (float)TOTAL_STEPS) * 57.2957795f;

    float pk_pk_rad = max_err - min_err;
    float pk_pk_deg = pk_pk_rad * 57.2957795f;
    float rms_deg   = sqrtf(sum_sq_err / (float)TOTAL_STEPS) * 57.2957795f;

    Serial.println("--------------------------------------------------------------------------------");
    Serial.println("128-POINT NON-LINEARITY CALIBRATION METRICS:");
    Serial.printf ("  • Peak-to-Peak Error Span : %.3f deg mech (±%.3f deg)\n", pk_pk_deg, pk_pk_deg * 0.5f);
    Serial.printf ("  • RMS Residual Error      : %.3f deg mech\n", rms_deg);
    Serial.printf ("  • Harmonic 1 (Eccentricity): ±%.3f deg\n", h1_amp_deg);
    Serial.printf ("  • Harmonic 2 (Tilt)       : ±%.3f deg\n", h2_amp_deg);
    Serial.printf ("  • Anomaly Points (>%.1f°) : %d\n", LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f, anomaly_count);

    // ------------------------------------------------------------------------
    // Step 5: Quality Gate & Activation
    // ------------------------------------------------------------------------
    bool lut_valid = (anomaly_count <= 6) && (pk_pk_rad <= LUT_PKPK_LIMIT_RAD);

    if (!lut_valid) {
        Serial.println("--------------------------------------------------------------------------------");
        Serial.println("❌ LUT CALIBRATION REJECTED — ROTOR STALL OR EXTREME DISTORTION DETECTED!");
        Serial.println("--------------------------------------------------------------------------------");
        if (anomaly_count > 6) {
            Serial.printf ("  ⚠️ Excessive slip/stall points (%d anomalies exceed limit of 6).\n", anomaly_count);
            Serial.println("     ROOT CAUSE: Motor stalled, lost steps, or phase current was insufficient.");
            Serial.println("     FIX: Check power supply 12V rail and ensure motor shaft turns freely.");
        }
        if (pk_pk_rad > LUT_PKPK_LIMIT_RAD) {
            Serial.printf ("  ⚠️ Peak-to-Peak error %.3f° exceeds allowable limit %.3f°!\n", pk_pk_deg, LUT_PKPK_LIMIT_RAD * 57.2957795f);
            Serial.println("     ROOT CAUSE: Magnet runout > 0.15mm off-center or sensor tilt.");
            Serial.println("     FIX: Re-center magnet on motor shaft axis.");
        }
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        if (!prev_enabled) driver.disable();
        calib_active = false;
        live_calib_progress = 0;
        Serial.print("[CALIB_FAILED]:M"); Serial.print(motor_id); Serial.println(" Quality Gate Failure");
        return;
    }

    // Interpolate circular error table onto the 128 raw encoder segments
    for (int s = 0; s < TOTAL_STEPS; s++) {
        float seg_angle = (float)s * step_mech_rad;
        encoder.lut_errors[s] = interpolateLUTAtAngle(seg_angle, mid_angles, filtered_errors, TOTAL_STEPS);
    }
    encoder.is_lut_active = true;

    live_calib_progress = 100;
    sendBinaryTelemetry();

    // Safe shutdown of test vectors
    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    calib_active = false;
    if (!prev_enabled) {
        driver.disable();
    }

    Serial.print("[LUT_READY]:M"); Serial.println(motor_id);
    Serial.println(">>> 128-POINT CALIBRATION COMPLETE & LUT ACTIVATED! <<<");
    Serial.println("================================================================================\n");
}

// ----------------------------------------------------------------------------
// 8-Line LUT Upload Parser (sscanf)
// ----------------------------------------------------------------------------
void parseLUTUpload(const char *cmd) {
    int m_id = 1;
    int line_idx = 0;
    float vals[16];

    // Format: LUT <motor_id> <line_idx> <f0,f1,...,f15>
    int read_count = sscanf(cmd, "LUT %d %d %f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                            &m_id, &line_idx,
                            &vals[0], &vals[1], &vals[2], &vals[3],
                            &vals[4], &vals[5], &vals[6], &vals[7],
                            &vals[8], &vals[9], &vals[10], &vals[11],
                            &vals[12], &vals[13], &vals[14], &vals[15]);

    if (read_count == 18 && line_idx >= 0 && line_idx < 8) {
        if (m_id == 1) {
            encoder1.setLUTLine(line_idx, vals, 16);
            if (line_idx == 7) {
                Serial.println("[LUT_READY]:M1");
            } else {
                Serial.print("[LUT_ACK]:M1:");
                Serial.println(line_idx);
            }
        } else if (m_id == 2) {
            encoder2.setLUTLine(line_idx, vals, 16);
            if (line_idx == 7) {
                Serial.println("[LUT_READY]:M2");
            } else {
                Serial.print("[LUT_ACK]:M2:");
                Serial.println(line_idx);
            }
        }
    } else {
        Serial.println("ERR: Invalid LUT format. Expected 'LUT <m_id> <line_idx> <16 csv floats>'");
    }
}

// ----------------------------------------------------------------------------
// Reset Parameters to GM3506 Defaults
// ----------------------------------------------------------------------------
void resetDefaults(int motor_id) {
    if (motor_id == 1 || motor_id == 0) {
        motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor1.PID_velocity.limit = motor1.voltage_limit;
        motor1.velocity_limit = MOTOR_VELOCITY_LIMIT;
        motor1.PID_velocity.P = 0.12f;
        motor1.PID_velocity.I = 1.20f;
        motor1.PID_velocity.D = 0.001f;
        motor1.PID_velocity.output_ramp = 1000.0f;
        motor1.LPF_velocity.Tf = 0.015f;
        m1_target_rpm = 0.0f;
    }
    if (motor_id == 2 || motor_id == 0) {
        motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor2.PID_velocity.limit = motor2.voltage_limit;
        motor2.velocity_limit = MOTOR_VELOCITY_LIMIT;
        motor2.PID_velocity.P = 0.12f;
        motor2.PID_velocity.I = 1.20f;
        motor2.PID_velocity.D = 0.001f;
        motor2.PID_velocity.output_ramp = 1000.0f;
        motor2.LPF_velocity.Tf = 0.015f;
        m2_target_rpm = 0.0f;
    }
    Serial.println(">>> [RESET] Restored GM3506 12V Optimized Defaults (SpaceVectorPWM).");
}

// ----------------------------------------------------------------------------
// Text Help & Status Menu
// ----------------------------------------------------------------------------
void printHelp() {
    Serial.println("\n================================================================================");
    Serial.println("  🏎️  DA WINNA — STM32G474 DUAL-MOTOR 20.0 kHz SIMPLEFOC ENGINE (GM3506)");
    Serial.println("================================================================================");
    Serial.print  ("  Motor 1 State    : "); Serial.println(m1_enabled ? "ENABLED" : "DISABLED");
    Serial.print  ("  Motor 2 State    : "); Serial.println(m2_enabled ? "ENABLED" : "DISABLED");
    Serial.print  ("  FOC Timer        : TIM6 Hardware Timer @ 20.0 kHz (50.0 us tick, Priority 2)\n");
    Serial.print  ("  Motion Control   : Downsampled 2.0 kHz (500 us tick, downsample = 10)\n");
    Serial.print  ("  Shared Bus       : SPI1 @ 10.6 MHz (PA5/6/7) | CS1=PA2, CS2=PA3\n");
    Serial.print  ("  PWM Inverters    : TIM1 (M1: PA8/9/10, PB3) | TIM8 (M2: PB6/8/9, PB0) @ 25 kHz\n");
    Serial.print  ("  Stall Watchdog   : "); Serial.println(stall_watchdog_enabled ? "ACTIVE (Target >= 5 RPM & Actual == 0)" : "DISABLED");
    Serial.println("--------------------------------------------------------------------------------");
    Serial.println("Commands:");
    Serial.println("  E1 / E2 / E      : Enable Motor 1 / Motor 2 / Both");
    Serial.println("  D1 / D2 / D      : Disable Motor 1 / Motor 2 / Both");
    Serial.println("  STOP or x or spc : Emergency Global Stop (Disable all motors & reset targets)");
    Serial.println("  T1 <rpm>         : Set Motor 1 Target Speed in RPM");
    Serial.println("  T2 <rpm>         : Set Motor 2 Target Speed in RPM");
    Serial.println("  T <rpm>          : Set Both Motors Target Speed in RPM");
    Serial.println("  C1 / C2 / C      : Run SimpleFOC Electrical Alignment");
    Serial.println("  CALIB 1 / 2      : Run Stepped 128-Point Sweep Calibration & Activate LUT");
    Serial.println("  LUT <m> <idx> .. : Upload 16 floats of 128-point LUT curve");
    Serial.println("  P1/P2 <val>      : Set Velocity P Gain");
    Serial.println("  I1/I2 <val>      : Set Velocity I Gain");
    Serial.println("  D1/D2 <val>      : Set Velocity D Gain");
    Serial.println("  F1/F2 <val>      : Set Velocity LPF Tf in seconds");
    Serial.println("  V1/V2 <val>      : Set Voltage Limit in Volts");
    Serial.println("  R1 / R2 / R      : Reset to GM3506 Defaults");
    Serial.println("  Z1 / Z2 / Z      : Zero / Tare Current Encoder Angle");
    Serial.println("  K                : Toggle Stall Watchdog");
    Serial.println("  G                : Toggle Telemetry Output");
    Serial.println("  ENC              : Print live Encoder 3 & 4 diagnostic readout");
    Serial.println("  H or ?           : Show this help menu");
    Serial.println("================================================================================\n");
}

// ----------------------------------------------------------------------------
// Command Interpreter
// ----------------------------------------------------------------------------
void processCommand(const char *cmd) {
    if (strlen(cmd) == 0) return;

    // Check LUT upload
    if (strncmp(cmd, "LUT", 3) == 0) {
        parseLUTUpload(cmd);
        return;
    }

    // Check CALIB command
    if (strncmp(cmd, "CALIB", 5) == 0) {
        int m = 1;
        sscanf(cmd, "CALIB %d", &m);
        runSteppedCalibration(m);
        return;
    }

    // Single-character or prefix commands
    if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "x") == 0 || strcmp(cmd, "X") == 0 || strcmp(cmd, " ") == 0) {
        m1_target_rpm = 0.0f;
        m2_target_rpm = 0.0f;
        m1_enabled = false;
        m2_enabled = false;
        driver1.disable();
        driver2.disable();
        Serial.println(">>> 🛑 EMERGENCY STOP: BOTH MOTORS DISABLED.");
        return;
    }

    if (strcmp(cmd, "E1") == 0) {
        m1_enabled = true;
        if (!m1_calibrated) initMotorFOC(1);
        else driver1.enable();
        Serial.println(">>> M1: ENABLED");
        return;
    }
    if (strcmp(cmd, "E2") == 0) {
        m2_enabled = true;
        if (!m2_calibrated) initMotorFOC(2);
        else driver2.enable();
        Serial.println(">>> M2: ENABLED");
        return;
    }
    if (strcmp(cmd, "E") == 0 || strcmp(cmd, "e") == 0) {
        m1_enabled = true;
        m2_enabled = true;
        if (!m1_calibrated) initMotorFOC(1);
        else driver1.enable();
        if (!m2_calibrated) initMotorFOC(2);
        else driver2.enable();
        Serial.println(">>> BOTH MOTORS: ENABLED");
        return;
    }

    if (strcmp(cmd, "D1") == 0) {
        m1_enabled = false;
        driver1.disable();
        m1_target_rpm = 0.0f;
        Serial.println(">>> M1: DISABLED");
        return;
    }
    if (strcmp(cmd, "D2") == 0) {
        m2_enabled = false;
        driver2.disable();
        m2_target_rpm = 0.0f;
        Serial.println(">>> M2: DISABLED");
        return;
    }
    if (strcmp(cmd, "D") == 0 || strcmp(cmd, "d") == 0) {
        m1_enabled = false;
        m2_enabled = false;
        driver1.disable();
        driver2.disable();
        m1_target_rpm = 0.0f;
        m2_target_rpm = 0.0f;
        Serial.println(">>> BOTH MOTORS: DISABLED");
        return;
    }

    if (strcmp(cmd, "C1") == 0) {
        initMotorFOC(1);
        return;
    }
    if (strcmp(cmd, "C2") == 0) {
        initMotorFOC(2);
        return;
    }
    if (strcmp(cmd, "C") == 0 || strcmp(cmd, "c") == 0) {
        initMotorFOC(1);
        initMotorFOC(2);
        return;
    }

    if (strcmp(cmd, "R1") == 0) { resetDefaults(1); return; }
    if (strcmp(cmd, "R2") == 0) { resetDefaults(2); return; }
    if (strcmp(cmd, "R") == 0 || strcmp(cmd, "r") == 0) { resetDefaults(0); return; }

    if (strcmp(cmd, "Z1") == 0) { encoder1.zeroCurrentPosition(); Serial.println(">>> M1: Tared."); return; }
    if (strcmp(cmd, "Z2") == 0) { encoder2.zeroCurrentPosition(); Serial.println(">>> M2: Tared."); return; }
    if (strcmp(cmd, "Z") == 0 || strcmp(cmd, "z") == 0) {
        encoder1.zeroCurrentPosition();
        encoder2.zeroCurrentPosition();
        Serial.println(">>> BOTH MOTORS: Tared.");
        return;
    }

    if (strcmp(cmd, "K") == 0 || strcmp(cmd, "k") == 0) {
        stall_watchdog_enabled = !stall_watchdog_enabled;
        Serial.print(">>> Stall Watchdog: ");
        Serial.println(stall_watchdog_enabled ? "ENABLED" : "DISABLED");
        return;
    }

    if (strcmp(cmd, "G") == 0 || strcmp(cmd, "g") == 0) {
        show_telemetry = !show_telemetry;
        Serial.print(">>> Telemetry Output: ");
        Serial.println(show_telemetry ? "ON" : "OFF");
        return;
    }

    if (strcmp(cmd, "ENC") == 0 || strcmp(cmd, "enc") == 0 || strcmp(cmd, "ENC4") == 0 || strcmp(cmd, "enc4") == 0) {
        Serial.println("\n--- [MT6816 ENCODER LIVE DIAGNOSTICS] ---");
        Serial.print("Encoder 3 (PB2 - M1/RL): Raw14=");
        Serial.print(encoder1.last_raw_14bit);
        Serial.print(" | Angle=");
        Serial.print(encoder1.getSingleTurnDeg(), 2);
        Serial.print(" deg | Parity=");
        Serial.print(encoder1.last_parity_ok ? "OK" : "FAIL");
        Serial.print(" | Mag=");
        Serial.print(encoder1.last_no_mag ? "NO_MAG" : "OK");
        Serial.print(" | TotalReads=");
        Serial.print(encoder1.total_reads);
        Serial.print(" | Retries=");
        Serial.print(encoder1.parity_retries);
        Serial.print(" | ParityErrs=");
        Serial.println(encoder1.parity_errors);

        Serial.print("Encoder 4 (PB3 - M2/RR): Raw14=");
        Serial.print(encoder2.last_raw_14bit);
        Serial.print(" | Angle=");
        Serial.print(encoder2.getSingleTurnDeg(), 2);
        Serial.print(" deg | Parity=");
        Serial.print(encoder2.last_parity_ok ? "OK" : "FAIL");
        Serial.print(" | Mag=");
        Serial.print(encoder2.last_no_mag ? "NO_MAG" : "OK");
        Serial.print(" | TotalReads=");
        Serial.print(encoder2.total_reads);
        Serial.print(" | Retries=");
        Serial.print(encoder2.parity_retries);
        Serial.print(" | ParityErrs=");
        Serial.println(encoder2.parity_errors);
        Serial.println("------------------------------------------\n");
        return;
    }

    if (strcmp(cmd, "DFU") == 0 || strcmp(cmd, "dfu") == 0 || strcmp(cmd, "BOOT") == 0) {
        Serial.println(">>> 🔄 JUMPING TO STM32 SYSTEM DFU BOOTLOADER...");
        Serial.flush();
        delay(50);
        __disable_irq();
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;
        for (int i = 0; i < 8; i++) {
            NVIC->ICER[i] = 0xFFFFFFFF;
            NVIC->ICPR[i] = 0xFFFFFFFF;
        }
        const uint32_t boot_addr = 0x1FFF0000;
        uint32_t jump_addr = *(__IO uint32_t *)(boot_addr + 4);
        void (*boot_jump)(void) = (void (*)(void))jump_addr;
        __set_MSP(*(__IO uint32_t *)boot_addr);
        boot_jump();
        while (1);
    }

    if (strcmp(cmd, "H") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0) {
        printHelp();
        return;
    }

    // Parameter tuning with float parsing
    if (strncmp(cmd, "T1", 2) == 0) {
        m1_target_rpm = atof(cmd + 2);
        Serial.print(">>> M1 Target Speed: "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
        return;
    }
    if (strncmp(cmd, "T2", 2) == 0) {
        m2_target_rpm = atof(cmd + 2);
        Serial.print(">>> M2 Target Speed: "); Serial.print(m2_target_rpm, 1); Serial.println(" RPM");
        return;
    }
    if (strncmp(cmd, "T", 1) == 0) {
        float val = atof(cmd + 1);
        m1_target_rpm = val;
        m2_target_rpm = val;
        Serial.print(">>> Coupled Target Speed: "); Serial.print(val, 1); Serial.println(" RPM");
        return;
    }

    if (strncmp(cmd, "P1", 2) == 0) {
        float val = atof(cmd + 2);
        if (val >= 0.0f) { motor1.PID_velocity.P = val; Serial.print(">>> M1 P: "); Serial.println(val, 4); }
        return;
    }
    if (strncmp(cmd, "P2", 2) == 0) {
        float val = atof(cmd + 2);
        if (val >= 0.0f) { motor2.PID_velocity.P = val; Serial.print(">>> M2 P: "); Serial.println(val, 4); }
        return;
    }

    if (strncmp(cmd, "I1", 2) == 0) {
        float val = atof(cmd + 2);
        if (val >= 0.0f) { motor1.PID_velocity.I = val; Serial.print(">>> M1 I: "); Serial.println(val, 4); }
        return;
    }
    if (strncmp(cmd, "I2", 2) == 0) {
        float val = atof(cmd + 2);
        if (val >= 0.0f) { motor2.PID_velocity.I = val; Serial.print(">>> M2 I: "); Serial.println(val, 4); }
        return;
    }

    if (strncmp(cmd, "D1", 2) == 0) {
        float val = atof(cmd + 2);
        if (val >= 0.0f) { motor1.PID_velocity.D = val; Serial.print(">>> M1 D: "); Serial.println(val, 5); }
        return;
    }
    if (strncmp(cmd, "D2", 2) == 0) {
        float val = atof(cmd + 2);
        if (val >= 0.0f) { motor2.PID_velocity.D = val; Serial.print(">>> M2 D: "); Serial.println(val, 5); }
        return;
    }

    if (strncmp(cmd, "F1", 2) == 0) {
        float val = atof(cmd + 2);
        if (val > 0.0f) { motor1.LPF_velocity.Tf = val; Serial.print(">>> M1 LPF Tf: "); Serial.println(val, 4); }
        return;
    }
    if (strncmp(cmd, "F2", 2) == 0) {
        float val = atof(cmd + 2);
        if (val > 0.0f) { motor2.LPF_velocity.Tf = val; Serial.print(">>> M2 LPF Tf: "); Serial.println(val, 4); }
        return;
    }

    if (strncmp(cmd, "V1", 2) == 0) {
        float val = atof(cmd + 2);
        if (val > 0.0f) { motor1.voltage_limit = val; motor1.PID_velocity.limit = val; Serial.print(">>> M1 V-Limit: "); Serial.println(val, 2); }
        return;
    }
    if (strncmp(cmd, "V2", 2) == 0) {
        float val = atof(cmd + 2);
        if (val > 0.0f) { motor2.voltage_limit = val; motor2.PID_velocity.limit = val; Serial.print(">>> M2 V-Limit: "); Serial.println(val, 2); }
        return;
    }

    Serial.print("ERR: Unrecognized command: ");
    Serial.println(cmd);
}

// ----------------------------------------------------------------------------
// Autonomous Stall Protection Watchdog
// ----------------------------------------------------------------------------
void checkStallWatchdog() {
    if (!stall_watchdog_enabled) return;

    // Motor 1 Stall Check
    if (m1_enabled) {
        float actual1 = fabsf(motor1.shaftVelocity() * 9.54929659f);
        float target1 = fabsf(m1_target_rpm);
        if (target1 >= 5.0f && actual1 < 1.0f) {
            if (m1_stall_timer == 0) m1_stall_timer = millis();
            else if (millis() - m1_stall_timer >= 2000) m1_is_stalled = true;
        } else {
            m1_stall_timer = 0;
        }

        if (m1_is_stalled) {
            Serial.println("\n>>> 🚨 SAFETY SHUTDOWN: Motor 1 Stalled (Target >= 5 RPM, Actual == 0 RPM)!");
            m1_enabled = false;
            driver1.disable();
            m1_target_rpm = 0.0f;
            m1_is_stalled = false;
            m1_stall_timer = 0;
        }
    }

    // Motor 2 Stall Check
    if (m2_enabled) {
        float actual2 = fabsf(motor2.shaftVelocity() * 9.54929659f);
        float target2 = fabsf(m2_target_rpm);
        if (target2 >= 5.0f && actual2 < 1.0f) {
            if (m2_stall_timer == 0) m2_stall_timer = millis();
            else if (millis() - m2_stall_timer >= 2000) m2_is_stalled = true;
        } else {
            m2_stall_timer = 0;
        }

        if (m2_is_stalled) {
            Serial.println("\n>>> 🚨 SAFETY SHUTDOWN: Motor 2 Stalled (Target >= 5 RPM, Actual == 0 RPM)!");
            m2_enabled = false;
            driver2.disable();
            m2_target_rpm = 0.0f;
            m2_is_stalled = false;
            m2_stall_timer = 0;
        }
    }
}

// ----------------------------------------------------------------------------
// 26-Byte Telemetry Frame Generator (25 Hz)
// ----------------------------------------------------------------------------
void sendBinaryTelemetry() {
    TelemetryPacket pkt;
    pkt.header[0] = 0xAA;
    pkt.header[1] = 0x55;

    // Motor 1 Fields
    pkt.m1_raw_angle  = encoder1.last_raw_14bit;
    pkt.m1_actual_rpm = (int16_t)(motor1.shaftVelocity() * 9.54929659f * 10.0f);
    pkt.m1_target_rpm = (int16_t)(m1_target_rpm * 10.0f);
    pkt.m1_uq_mv      = (int16_t)(motor1.voltage.q * 1000.0f);

    uint8_t s1 = 0;
    if (m1_enabled)            s1 |= (1 << 0);
    if (encoder1.is_lut_active) s1 |= (1 << 1);
    if (!encoder1.last_no_mag) s1 |= (1 << 2);
    if (encoder1.last_parity_ok) s1 |= (1 << 3);
    if (m1_is_stalled)         s1 |= (1 << 4);
    if (m1_calib_active)       s1 |= (1 << 5);
    pkt.m1_status = s1;

    // Motor 2 Fields
    pkt.m2_raw_angle  = encoder2.last_raw_14bit;
    pkt.m2_actual_rpm = (int16_t)(motor2.shaftVelocity() * 9.54929659f * 10.0f);
    pkt.m2_target_rpm = (int16_t)(m2_target_rpm * 10.0f);
    pkt.m2_uq_mv      = (int16_t)(motor2.voltage.q * 1000.0f);

    uint8_t s2 = 0;
    if (m2_enabled)            s2 |= (1 << 0);
    if (encoder2.is_lut_active) s2 |= (1 << 1);
    if (!encoder2.last_no_mag) s2 |= (1 << 2);
    if (encoder2.last_parity_ok) s2 |= (1 << 3);
    if (m2_is_stalled)         s2 |= (1 << 4);
    if (m2_calib_active)       s2 |= (1 << 5);
    pkt.m2_status = s2;

    // Real-Time Control & Profiler Stats
    pkt.foc_khz_x10    = 200; // 20.0 kHz
    // Peak ISR latency converted from cycles to microseconds (170 cycles/us)
    uint32_t lat_us    = isr_peak_cycles / 170;
    pkt.isr_lat_us     = (lat_us > 255) ? 255 : (uint8_t)lat_us;
    pkt.spikes_count   = isr_jitter_spikes;
    pkt.calib_progress = live_calib_progress;

    // Reset peak cycles every telemetry period to track rolling max latency
    isr_peak_cycles = isr_last_cycles;

    // Compute XOR checksum over bytes [2 .. 24] (23 bytes total)
    uint8_t chk = 0;
    const uint8_t *raw_ptr = (const uint8_t *)&pkt;
    for (int i = 2; i < 25; i++) {
        chk ^= raw_ptr[i];
    }
    pkt.checksum = chk;

    // Atomic binary write to serial buffer
    Serial.write(raw_ptr, sizeof(TelemetryPacket));
}

// ----------------------------------------------------------------------------
// Arduino Setup: Autonomous Standalone Initialization (< 150 ms)
// ----------------------------------------------------------------------------
void setup() {
    // 1. Standalone DWT Unlock (Essential for standalone battery/bench operation without ST-Link)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    // 2. Status LED setup
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    // 3. Serial Port Initialization (921600 Baud for Ultra High Speed Telemetry)
    Serial.begin(921600);

    // 4. Initialize MT6816 Encoder 1 on SPI1
    pinMode(PIN_M2_CS, OUTPUT);
    digitalWrite(PIN_M2_CS, HIGH); // Ensure Encoder 2 CS is deselected
    encoder1.init();

    // 5. Configure Drivers (TIM1 & TIM8 @ 25 kHz Center-Aligned SpaceVectorPWM)
    driver1.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver1.voltage_limit = MOTOR_SUPPLY_VOLTAGE;
    driver1.pwm_frequency = 25000;
    driver1.init();

    driver2.voltage_power_supply = MOTOR_SUPPLY_VOLTAGE;
    driver2.voltage_limit = MOTOR_SUPPLY_VOLTAGE;
    driver2.pwm_frequency = 25000;
    driver2.init();

    // 6. Configure Motors (GM3506 Optimized Dynamics)
    motor1.linkDriver(&driver1);
    motor1.linkSensor(&encoder1);
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor1.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor1.velocity_limit = MOTOR_VELOCITY_LIMIT;
    motor1.KV_rating = MOTOR_KV_RATING;
    motor1.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
    motor1.controller = MotionControlType::velocity;
    motor1.PID_velocity.P = 0.12f;
    motor1.PID_velocity.I = 1.20f;
    motor1.PID_velocity.D = 0.001f;
    motor1.PID_velocity.output_ramp = 1000.0f;
    motor1.PID_velocity.limit = motor1.voltage_limit;
    motor1.LPF_velocity.Tf = 0.015f;
    motor1.init();
    driver1.disable();

    motor2.linkDriver(&driver2);
    motor2.linkSensor(&encoder2);
    motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor2.velocity_limit = MOTOR_VELOCITY_LIMIT;
    motor2.KV_rating = MOTOR_KV_RATING;
    motor2.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
    motor2.controller = MotionControlType::velocity;
    motor2.PID_velocity.P = 0.12f;
    motor2.PID_velocity.I = 1.20f;
    motor2.PID_velocity.D = 0.001f;
    motor2.PID_velocity.output_ramp = 1000.0f;
    motor2.PID_velocity.limit = motor2.voltage_limit;
    motor2.LPF_velocity.Tf = 0.015f;
    motor2.init();
    driver2.disable();

    // 7. Silent Pre-Loaded Alignment Check (Bypasses Startup Rotation Jerk)
    if (M1_DEFAULT_ZERO_ELECTRIC_ANGLE != 0.0f) {
        motor1.zero_electric_angle = M1_DEFAULT_ZERO_ELECTRIC_ANGLE;
        motor1.sensor_direction = Direction::CW;
        m1_calibrated = true;
    }

    if (M2_DEFAULT_ZERO_ELECTRIC_ANGLE != 0.0f) {
        motor2.zero_electric_angle = M2_DEFAULT_ZERO_ELECTRIC_ANGLE;
        motor2.sensor_direction = Direction::CW;
        m2_calibrated = true;
    }

    // 8. Configure & Start TIM6 Hardware Timer Interrupt @ 20.0 kHz (Priority 2)
    foc_timer = new HardwareTimer(TIM6);
    foc_timer->setOverflow(FOC_COMMUTATION_HZ, HERTZ_FORMAT);
    foc_timer->attachInterrupt(focTimerISR);
    foc_timer->setInterruptPriority(2, 0);
    foc_timer->resume();

    last_telemetry_time = millis();
    last_led_time = millis();

    printHelp();
}

// ----------------------------------------------------------------------------
// Background Task Loop (Serial Parsing, Telemetry, Safety)
// ----------------------------------------------------------------------------
void loop() {
    // 1. Process Incoming Serial Characters into Line Buffer
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_line[rx_idx] = '\0';
                processCommand(rx_line);
                rx_idx = 0;
            }
        } else if (rx_idx < sizeof(rx_line) - 1) {
            rx_line[rx_idx++] = c;
        }
    }

    // 2. Autonomous Stall Protection
    checkStallWatchdog();

    // 3. Periodic Telemetry Stream (25 Hz = 40 ms)
    uint32_t now = millis();
    if (show_telemetry && (now - last_telemetry_time >= (1000 / TELEMETRY_RATE_HZ))) {
        last_telemetry_time = now;
        sendBinaryTelemetry();
    }

    // 4. Heartbeat LED Toggle (500 ms)
    if (now - last_led_time >= 500) {
        last_led_time = now;
        led_state = !led_state;
        digitalWrite(STATUS_LED, led_state ? LOW : HIGH);
    }
}
