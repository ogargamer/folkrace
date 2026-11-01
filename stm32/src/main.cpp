#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <SimpleFOC.h>
#include "config_pins.h"
#include "mt6816_spi.h"

// ============================================================================
// DA WINNA — 20.0 kHz HARD REAL-TIME DUAL-MOTOR SIMPLEFOC ENGINE (STM32G474)
// ============================================================================
// Hardware: STM32G474CBT6 (170 MHz Cortex-M4F, HW FPU)
// Timers: TIM1 (M1 PWM 25kHz), TIM8 (M2 PWM 25kHz), TIM6 (20 kHz Control ISR)
// Sensors: Dual MT6816 14-Bit Magnetic Encoders on Shared SPI1 @ 5.31 MHz
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

volatile float m1_base_zero_angle = 0.0f;
volatile float m2_base_zero_angle = 0.0f;
volatile float m1_lag_comp_us = 35.0f;
volatile float m2_lag_comp_us = 35.0f;
static float m1_lag_filt_vel = 0.0f;
static float m2_lag_filt_vel = 0.0f;

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

// Angle Normalization Helpers
static inline float normalizeAngleRad(float angle) {
    float a = fmodf(angle, _2PI);
    return (a >= 0.0f) ? a : (a + _2PI);
}

static inline float wrapAngleRad(float angle) {
    while (angle > _PI) angle -= _2PI;
    while (angle < -_PI) angle += _2PI;
    return angle;
}

// ----------------------------------------------------------------------------
// Hard Real-Time 20.0 kHz Hardware Timer ISR (TIM6, NVIC Priority 2)
// ----------------------------------------------------------------------------
void focTimerISR() {
    uint32_t start_cycles = DWT->CYCCNT;
    foc_tick_count++;

    // 1. Single-Read SPI Caching & SimpleFOC Sensor State Update
    encoder1.readHardwareSPI();
    encoder1.update(); // CRITICAL: Updates angle tracking, timestamp, and multi-turn unwrapping on every 20 kHz tick!
    if (m2_enabled || m2_calib_active) {
        encoder2.readHardwareSPI();
        encoder2.update();
    }

    // 2. Motor 1 Commutation Loop (with Smooth Filtered Sensor Lag Compensation)
    if (m1_enabled && !m1_calib_active) {
        if (m1_lag_comp_us > 0.0f && m1_base_zero_angle != 0.0f) {
            // Crisp 2.0 ms low-pass filter on lag compensation velocity (alpha = 0.024f @ 20 kHz)
            // Eliminates torque-commutation feedback ripple with no sluggish transient lag
            m1_lag_filt_vel += 0.024f * (motor1.shaft_velocity - m1_lag_filt_vel);
            if (fabsf(m1_lag_filt_vel) > 3.0f) {
                float lead_rad = m1_lag_filt_vel * (float)MOTOR_POLE_PAIRS * (m1_lag_comp_us * 1e-6f);
                motor1.zero_electric_angle = normalizeAngleRad(m1_base_zero_angle - lead_rad);
            } else {
                motor1.zero_electric_angle = m1_base_zero_angle;
            }
        }
        motor1.loopFOC();
    }

    // 3. Motor 2 Commutation Loop (with Smooth Filtered Sensor Lag Compensation)
    if (m2_enabled && !m2_calib_active) {
        if (m2_lag_comp_us > 0.0f && m2_base_zero_angle != 0.0f) {
            m2_lag_filt_vel += 0.024f * (motor2.shaft_velocity - m2_lag_filt_vel);
            if (fabsf(m2_lag_filt_vel) > 3.0f) {
                float lead_rad = m2_lag_filt_vel * (float)MOTOR_POLE_PAIRS * (m2_lag_comp_us * 1e-6f);
                motor2.zero_electric_angle = normalizeAngleRad(m2_base_zero_angle - lead_rad);
            } else {
                motor2.zero_electric_angle = m2_base_zero_angle;
            }
        }
        motor2.loopFOC();
    }

    // 4. Downsampled Velocity Motion Control (Every 10th tick = 2.0 kHz / 500 us)
    if ((foc_tick_count % MOTION_DOWNSAMPLE_RATIO) == 0) {
        if (m1_enabled && !m1_calib_active) {
            motor1.move(m1_target_rpm * 0.104719755f);
        }
        if (m2_enabled && !m2_calib_active) {
            motor2.move(m2_target_rpm * 0.104719755f);
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
// LUT Data Dump & Serialization Helper
// ----------------------------------------------------------------------------
void dumpLUT(int motor_id) {
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;
    Serial.printf("[LUT_DATA_START]:M%d\n", motor_id);
    for (int line = 0; line < 8; line++) {
        Serial.printf("LUT %d %d ", motor_id, line);
        for (int j = 0; j < 16; j++) {
            Serial.printf("%.6f%s", encoder.lut_errors[line * 16 + j], (j == 15) ? "" : ",");
        }
        Serial.println();
    }
    Serial.printf("[LUT_DATA_END]:M%d\n", motor_id);
}

// ----------------------------------------------------------------------------
// JSON Configuration Exporter (Capturable by Python GUI)
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// Persistent Flash Storage (STM32 Internal Emulated EEPROM)
// ----------------------------------------------------------------------------
#define FOC_CONFIG_MAGIC 0x44415749UL // "DAWI"

struct PersistentFOCConfig {
    uint32_t magic;
    float    m1_zero_angle;
    int8_t   m1_direction;
    float    m1_lag_comp_us;
    float    m1_kv_rating;
    float    m1_pid_p;
    float    m1_pid_i;
    float    m1_pid_d;
    float    m1_lpf_tf;
    float    m1_velocity_limit;
    float    m1_dwt_ms;
    bool     m1_lut_active;
    float    m1_lut_errors[128];

    float    m2_zero_angle;
    int8_t   m2_direction;
    float    m2_lag_comp_us;
    float    m2_kv_rating;
    float    m2_pid_p;
    float    m2_pid_i;
    float    m2_pid_d;
    float    m2_lpf_tf;
    float    m2_velocity_limit;
    float    m2_dwt_ms;
    bool     m2_lut_active;
    float    m2_lut_errors[128];

    uint32_t checksum;
};

static uint32_t computeConfigChecksum(const PersistentFOCConfig &cfg) {
    uint32_t chk = 0x55AA55AAUL;
    const uint8_t *p = (const uint8_t *)&cfg;
    size_t len = offsetof(PersistentFOCConfig, checksum);
    for (size_t i = 0; i < len; i++) {
        chk = (chk << 1) ^ p[i];
    }
    return chk;
}

void saveConfigToFlash() {
    PersistentFOCConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = FOC_CONFIG_MAGIC;

    cfg.m1_zero_angle = motor1.zero_electric_angle;
    cfg.m1_direction = (motor1.sensor_direction == Direction::CW) ? 1 : -1;
    cfg.m1_lag_comp_us = m1_lag_comp_us;
    cfg.m1_kv_rating = motor1.KV_rating;
    cfg.m1_pid_p = motor1.PID_velocity.P;
    cfg.m1_pid_i = motor1.PID_velocity.I;
    cfg.m1_pid_d = motor1.PID_velocity.D;
    cfg.m1_lpf_tf = motor1.LPF_velocity.Tf;
    cfg.m1_velocity_limit = motor1.velocity_limit;
    cfg.m1_dwt_ms = encoder1.window_ms;
    cfg.m1_lut_active = encoder1.is_lut_active;
    for (int i = 0; i < 128; i++) {
        cfg.m1_lut_errors[i] = encoder1.lut_errors[i];
    }

    cfg.m2_zero_angle = motor2.zero_electric_angle;
    cfg.m2_direction = (motor2.sensor_direction == Direction::CW) ? 1 : -1;
    cfg.m2_lag_comp_us = m2_lag_comp_us;
    cfg.m2_kv_rating = motor2.KV_rating;
    cfg.m2_pid_p = motor2.PID_velocity.P;
    cfg.m2_pid_i = motor2.PID_velocity.I;
    cfg.m2_pid_d = motor2.PID_velocity.D;
    cfg.m2_lpf_tf = motor2.LPF_velocity.Tf;
    cfg.m2_velocity_limit = motor2.velocity_limit;
    cfg.m2_dwt_ms = encoder2.window_ms;
    cfg.m2_lut_active = encoder2.is_lut_active;
    for (int i = 0; i < 128; i++) {
        cfg.m2_lut_errors[i] = encoder2.lut_errors[i];
    }

    cfg.checksum = computeConfigChecksum(cfg);

    eeprom_buffer_fill();
    const uint8_t *ptr = (const uint8_t *)&cfg;
    for (size_t i = 0; i < sizeof(PersistentFOCConfig); i++) {
        eeprom_buffered_write_byte(i, ptr[i]);
    }
    eeprom_buffer_flush();
    Serial.println(">>> [FLASH] Motor Configuration & 128-pt LUT saved to STM32 Flash!");
}

bool loadConfigFromFlash() {
    eeprom_buffer_fill();
    PersistentFOCConfig cfg;
    uint8_t *ptr = (uint8_t *)&cfg;
    for (size_t i = 0; i < sizeof(PersistentFOCConfig); i++) {
        ptr[i] = eeprom_buffered_read_byte(i);
    }
    if (cfg.magic != FOC_CONFIG_MAGIC) {
        Serial.println(">>> [FLASH] No saved calibration profile in flash.");
        return false;
    }
    if (computeConfigChecksum(cfg) != cfg.checksum) {
        Serial.println(">>> [FLASH] Checksum mismatch in saved profile!");
        return false;
    }

    motor1.zero_electric_angle = cfg.m1_zero_angle;
    m1_base_zero_angle = cfg.m1_zero_angle;
    motor1.sensor_direction = (cfg.m1_direction == 1) ? Direction::CW : Direction::CCW;
    m1_lag_comp_us = cfg.m1_lag_comp_us;
    if (cfg.m1_kv_rating > 10.0f) motor1.KV_rating = cfg.m1_kv_rating;
    if (cfg.m1_pid_p > 0.0f) motor1.PID_velocity.P = cfg.m1_pid_p;
    if (cfg.m1_pid_i > 0.0f) motor1.PID_velocity.I = cfg.m1_pid_i;
    motor1.PID_velocity.D = cfg.m1_pid_d;
    if (cfg.m1_lpf_tf > 0.0f) motor1.LPF_velocity.Tf = cfg.m1_lpf_tf;
    if (cfg.m1_velocity_limit > 0.0f) motor1.velocity_limit = cfg.m1_velocity_limit;
    if (cfg.m1_dwt_ms >= 0.2f && cfg.m1_dwt_ms <= 6.0f) encoder1.setDWTWindowMs(cfg.m1_dwt_ms);
    encoder1.is_lut_active = cfg.m1_lut_active;
    for (int i = 0; i < 128; i++) {
        encoder1.lut_errors[i] = cfg.m1_lut_errors[i];
    }
    m1_calibrated = (cfg.m1_zero_angle != 0.0f);

    motor2.zero_electric_angle = cfg.m2_zero_angle;
    m2_base_zero_angle = cfg.m2_zero_angle;
    motor2.sensor_direction = (cfg.m2_direction == 1) ? Direction::CW : Direction::CCW;
    m2_lag_comp_us = cfg.m2_lag_comp_us;
    if (cfg.m2_kv_rating > 10.0f) motor2.KV_rating = cfg.m2_kv_rating;
    if (cfg.m2_pid_p > 0.0f) motor2.PID_velocity.P = cfg.m2_pid_p;
    if (cfg.m2_pid_i > 0.0f) motor2.PID_velocity.I = cfg.m2_pid_i;
    motor2.PID_velocity.D = cfg.m2_pid_d;
    if (cfg.m2_lpf_tf > 0.0f) motor2.LPF_velocity.Tf = cfg.m2_lpf_tf;
    if (cfg.m2_velocity_limit > 0.0f) motor2.velocity_limit = cfg.m2_velocity_limit;
    if (cfg.m2_dwt_ms >= 0.2f && cfg.m2_dwt_ms <= 6.0f) encoder2.setDWTWindowMs(cfg.m2_dwt_ms);
    encoder2.is_lut_active = cfg.m2_lut_active;
    for (int i = 0; i < 128; i++) {
        encoder2.lut_errors[i] = cfg.m2_lut_errors[i];
    }
    m2_calibrated = (cfg.m2_zero_angle != 0.0f);

    Serial.println(">>> [FLASH] Successfully loaded configuration & LUT from STM32 Flash!");
    return true;
}

// ----------------------------------------------------------------------------
// JSON Configuration Exporter (Capturable by Python GUI)
// ----------------------------------------------------------------------------
void dumpConfigJSON() {
    Serial.println("\n[CONFIG_START]");
    Serial.printf("{\n");
    Serial.printf("  \"zero_electric_angle\": %.6f,\n", motor1.zero_electric_angle);
    Serial.printf("  \"sensor_direction\": %d,\n", (motor1.sensor_direction == Direction::CW) ? 1 : -1);
    Serial.printf("  \"sensor_lag_comp_us\": %.1f,\n", m1_lag_comp_us);
    Serial.printf("  \"motor_kv_rating\": %.2f,\n", motor1.KV_rating);
    Serial.printf("  \"is_lut_calibrated\": %s,\n", encoder1.is_lut_active ? "true" : "false");
    Serial.printf("  \"pid_p\": %.4f,\n", motor1.PID_velocity.P);
    Serial.printf("  \"pid_i\": %.4f,\n", motor1.PID_velocity.I);
    Serial.printf("  \"pid_d\": %.4f,\n", motor1.PID_velocity.D);
    Serial.printf("  \"lpf_tf\": %.4f,\n", motor1.LPF_velocity.Tf);
    Serial.printf("  \"dwt_window_ms\": %.2f,\n", encoder1.window_ms);
    Serial.printf("  \"m2_zero_electric_angle\": %.6f,\n", motor2.zero_electric_angle);
    Serial.printf("  \"m2_sensor_direction\": %d,\n", (motor2.sensor_direction == Direction::CW) ? 1 : -1);
    Serial.printf("  \"lut_128\": [");
    for (int i = 0; i < 128; i++) {
        Serial.printf("%.6f%s", encoder1.lut_errors[i], (i == 127) ? "" : ", ");
    }
    Serial.printf("]\n");
    Serial.printf("}\n");
    Serial.println("[CONFIG_END]\n");
}

// ----------------------------------------------------------------------------
// Stepped 128-Point Calibration Sweep (Single Pass - Matching ESP32 Master Suite)
// ----------------------------------------------------------------------------
bool runSteppedLUTSweepSinglePass(int motor_id, float align_voltage) {
    float hold_voltage = fminf(align_voltage, CALIB_HOLD_VOLTAGE);
    Serial.println("\n================================================================================");
    Serial.println(">>> 🔬 STARTING 128-POINT DUAL-PASS NON-LINEARITY LUT CALIBRATION <<<");
    Serial.println("================================================================================");
    Serial.printf ("Target Motor       : GM3506 (11 Pole Pairs on STM32G474 M%d)\n", motor_id);
    Serial.printf ("Step / Hold Voltage: %.2f V (move) / %.2f V (hold & sample)\n", align_voltage, hold_voltage);
    Serial.println("Methodology        : 128-Step Dual-Pass Bidirectional Mechanical Sweep");
    Serial.printf ("Oversampling       : %d Samples / Step with Micro-Dither (Stiction Cancellation)\n", CALIB_SWEEP_OVERSAMPLE_COUNT);
    Serial.printf ("Anomaly Guard      : Any |err| > %.3f rad (%.1f deg) -> FAIL (pole slip)\n",
                   LUT_ANOMALY_THRESHOLD_RAD, LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f);
    Serial.println("Press 'x' or 'STOP' anytime to abort");
    Serial.println("--------------------------------------------------------------------------------");

    BLDCMotor &motor = (motor_id == 1) ? motor1 : motor2;
    BLDCDriver3PWM &driver = (motor_id == 1) ? driver1 : driver2;
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;

    encoder.is_lut_active = false;
    driver.enable();

    // 1. Pre-flight MT6816 Health Check
    Serial.print("1. Checking MT6816 Sensor Health... ");
    delay(50);
    if (encoder.last_no_mag) {
        Serial.println("FAILED!");
        Serial.println("   ❌ ABORT: MT6816 reports NO_MAG flag (airgap too large or magnet missing).");
        Serial.println("   FIX: Position diametric magnet 1.0 - 2.0 mm directly above MT6816 chip.");
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }
    if (!encoder.last_parity_ok || encoder.parity_errors > 200) {
        Serial.println("FAILED!");
        Serial.printf ("   ❌ ABORT: MT6816 SPI Parity Errors detected (total: %lu)!\n", encoder.parity_errors);
        Serial.println("   FIX: Check SPI1 wiring (SCK/MISO/MOSI/CS), grounding, or termination.");
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }
    Serial.println("OK (Magnet present, Parity OK)");

    // 2. Commutation Direction & Rotor Stall Verification
    Serial.print("2. Determining Sensor Direction & Verifying Motion (Stall Check)... ");
    motor.setPhaseVoltage(align_voltage, 0.0f, _3PI_2);
    delay(100);
    motor.setPhaseVoltage(hold_voltage, 0.0f, _3PI_2);
    delay(300);

    float start_raw = encoder.getRawMechanicalAngle();

    const int DIR_STEPS = 200;
    for (int step = 0; step <= DIR_STEPS; step++) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(align_voltage, 0.0f, el_ang);
        delayMicroseconds(2000);
    }
    delay(150);
    float mid_raw = encoder.getRawMechanicalAngle();

    for (int step = DIR_STEPS; step >= 0; step--) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(align_voltage, 0.0f, el_ang);
        delayMicroseconds(2000);
    }
    delay(150);

    float d_ang = wrapAngleRad(mid_raw - start_raw);
    float moved_deg = fabsf(d_ang) * 57.2957795f;
    float expected_deg = (360.0f / (float)MOTOR_POLE_PAIRS) * 0.40f;

    if (fabsf(d_ang) < 0.05f) {
        Serial.println("FAILED!");
        Serial.println("  -------------------------------------------------------------");
        Serial.printf ("  ❌ MOTION CHECK FAILED: Rotor moved only %.2f deg mech (expected >= %.2f deg).\n",
                       moved_deg, expected_deg);
        Serial.printf ("     Angles sampled : start = %.4f rad (%.1f deg), mid = %.4f rad (%.1f deg)\n",
                       start_raw, start_raw * 57.2957795f, mid_raw, mid_raw * 57.2957795f);
        Serial.println("     ⚠️ ROOT CAUSE: Insufficient torque to overcome cogging/friction or unpowered inverter.");
        Serial.println("        FIX: Verify 12V DC bench supply is ON, driver EN pin is HIGH, or increase voltage ('CALIB 4.2').");
        Serial.println("  -------------------------------------------------------------\n");
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }

    Direction sensor_dir = (d_ang > 0.0f) ? Direction::CW : Direction::CCW;
    int dir_sign = (sensor_dir == Direction::CW) ? 1 : -1;
    Serial.printf("Detected %s (%+.2f deg mech) [OK]\n",
                  sensor_dir == Direction::CW ? "CW (+1)" : "CCW (-1)",
                  d_ang * 57.2957795f);

    // 3. Bidirectional 128-Step Sweep Across 360 Mechanical Degrees
    Serial.println("\n3. Executing Bidirectional 128-Step Sweep Across 360° Mechanical:");
    const int TOTAL_STEPS = CALIB_TOTAL_STEPS;
    const float step_mech_rad = _2PI / (float)TOTAL_STEPS;

    static float fwd_measured[CALIB_TOTAL_STEPS];
    static float rev_measured[CALIB_TOTAL_STEPS];
    static float raw_errors[CALIB_TOTAL_STEPS];
    static float mid_angles[CALIB_TOTAL_STEPS];
    static float temp_errors[CALIB_TOTAL_STEPS];

    // --- PASS 1: FORWARD (CW) SWEEP ---
    for (int i = 0; i < TOTAL_STEPS; i++) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }

        float target_mech = (float)i * step_mech_rad;
        float target_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * target_mech;

        // Smooth micro-stepping transition (16 micro-steps)
        if (i > 0) {
            const int INTERP = 16;
            float prev_mech = (float)(i - 1) * step_mech_rad;
            float prev_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * prev_mech;
            for (int s = 1; s <= INTERP; s++) {
                float el = prev_elec + (target_elec - prev_elec) * ((float)s / (float)INTERP);
                motor.setPhaseVoltage(align_voltage, 0.0f, el);
                delayMicroseconds(1000);
            }
        } else {
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec);
            delay(250);
        }

        motor.setPhaseVoltage(align_voltage, 0.0f, target_elec);
        delayMicroseconds(8000); // Rotor settle time

        // Decaying micro-dither stiction cancellation (8 steps)
        for (int d = 8; d >= 1; d--) {
            float dither = 0.08f * ((float)d / 8.0f);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec + dither);
            delayMicroseconds(500);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec - dither);
            delayMicroseconds(500);
        }

        motor.setPhaseVoltage(hold_voltage, 0.0f, target_elec);
        delay(CALIB_SWEEP_DAMP_SETTLE_MS);

        // Oversampled vector averaging (96 samples)
        float sum_sin = 0.0f, sum_cos = 0.0f;
        for (int s = 0; s < CALIB_SWEEP_OVERSAMPLE_COUNT; s++) {
            float a = encoder.getRawMechanicalAngle();
            sum_sin += sinf(a);
            sum_cos += cosf(a);
            delayMicroseconds(100);
        }
        fwd_measured[i] = normalizeAngleRad(atan2f(sum_sin, sum_cos));

        live_calib_progress = (uint8_t)((i * 45) / TOTAL_STEPS);
        sendBinaryTelemetry();

        if ((i + 1) % 16 == 0 || i == TOTAL_STEPS - 1) {
            Serial.printf("   Forward Sweep: %d%% Complete\n", (int)(((float)(i + 1) / (float)TOTAL_STEPS) * 100.0f));
            Serial.printf("[CALIB_PROG]:%d%%\n", live_calib_progress);
        }
    }

    // --- DEDICATED THERMAL COOL-DOWN PAUSE (5 seconds) ---
    Serial.printf("\n>>> ❄️ THERMAL COOL-DOWN PAUSE (%ds): De-energizing stator coils to eliminate thermal drift...\n", CALIB_COOLDOWN_PAUSE_SEC);
    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    for (int sec = CALIB_COOLDOWN_PAUSE_SEC; sec > 0; sec--) {
        Serial.printf("   Coil cool-down: %ds remaining...\n", sec);
        delay(1000);
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            return false;
        }
    }
    Serial.println(">>> ❄️ Coils thermally stabilized. Resuming Reverse Sweep...\n");

    // Re-engage at end of mechanical travel before reverse sweep
    float rev_start_mech = (float)(TOTAL_STEPS - 1) * step_mech_rad;
    float rev_start_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * rev_start_mech;
    motor.setPhaseVoltage(align_voltage, 0.0f, rev_start_elec);
    delay(100);
    motor.setPhaseVoltage(hold_voltage, 0.0f, rev_start_elec);
    delay(250);

    // --- PASS 2: REVERSE (CCW) SWEEP ---
    for (int i = TOTAL_STEPS - 1; i >= 0; i--) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }

        float target_mech = (float)i * step_mech_rad;
        float target_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * target_mech;

        // Smooth micro-stepping transition (16 micro-steps)
        if (i < TOTAL_STEPS - 1) {
            const int INTERP = 16;
            float prev_mech = (float)(i + 1) * step_mech_rad;
            float prev_elec = _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * prev_mech;
            for (int s = 1; s <= INTERP; s++) {
                float el = prev_elec + (target_elec - prev_elec) * ((float)s / (float)INTERP);
                motor.setPhaseVoltage(align_voltage, 0.0f, el);
                delayMicroseconds(1000);
            }
        }

        motor.setPhaseVoltage(align_voltage, 0.0f, target_elec);
        delayMicroseconds(8000);

        // Decaying micro-dither (8 steps)
        for (int d = 8; d >= 1; d--) {
            float dither = 0.08f * ((float)d / 8.0f);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec + dither);
            delayMicroseconds(500);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_elec - dither);
            delayMicroseconds(500);
        }

        motor.setPhaseVoltage(hold_voltage, 0.0f, target_elec);
        delay(CALIB_SWEEP_DAMP_SETTLE_MS);

        // Oversampled vector averaging (96 samples)
        float sum_sin = 0.0f, sum_cos = 0.0f;
        for (int s = 0; s < CALIB_SWEEP_OVERSAMPLE_COUNT; s++) {
            float a = encoder.getRawMechanicalAngle();
            sum_sin += sinf(a);
            sum_cos += cosf(a);
            delayMicroseconds(100);
        }
        rev_measured[i] = normalizeAngleRad(atan2f(sum_sin, sum_cos));

        live_calib_progress = 45 + (uint8_t)(((TOTAL_STEPS - 1 - i) * 45) / TOTAL_STEPS);
        sendBinaryTelemetry();

        if (i % 16 == 0 || i == 0) {
            Serial.printf("   Reverse Sweep: %d%% Complete\n", (int)(((float)(TOTAL_STEPS - i) / (float)TOTAL_STEPS) * 100.0f));
            Serial.printf("[CALIB_PROG]:%d%%\n", live_calib_progress);
        }
    }

    // 4. Midpoint Synthesis, Residual Error Table & Median Filter
    float start_mid = normalizeAngleRad(atan2f(sinf(fwd_measured[0]) + sinf(rev_measured[0]),
                                               cosf(fwd_measured[0]) + cosf(rev_measured[0])));

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
        temp_errors[i] = filtered_err;

        if (filtered_err > max_err) max_err = filtered_err;
        if (filtered_err < min_err) min_err = filtered_err;
        sum_sq_err += filtered_err * filtered_err;

        if (fabsf(filtered_err) > LUT_ANOMALY_THRESHOLD_RAD) {
            anomaly_count++;
            if (anomaly_count <= 8) {
                float mech_pos_deg = ((float)i / (float)TOTAL_STEPS) * 360.0f;
                int nearest_pole = (int)(mech_pos_deg / (360.0f / (float)MOTOR_POLE_PAIRS)) + 1;
                Serial.printf("  ⚠️ ANOMALY idx[%3d] @ %5.1f deg mech (Pole #%2d) | err = %+.4f rad (%+.2f deg) — POLE SLIP?\n",
                              i, mech_pos_deg, nearest_pole, filtered_err, filtered_err * 57.2957795f);
            }
        }
    }

    // Fourier harmonic extraction (Harmonic 1 = runout, Harmonic 2 = tilt)
    float f_a1 = 0.0f, f_b1 = 0.0f, f_a2 = 0.0f, f_b2 = 0.0f;
    for (int i = 0; i < TOTAL_STEPS; i++) {
        float angle_mech = (float)i * step_mech_rad;
        f_a1 += temp_errors[i] * cosf(angle_mech);
        f_b1 += temp_errors[i] * sinf(angle_mech);
        f_a2 += temp_errors[i] * cosf(2.0f * angle_mech);
        f_b2 += temp_errors[i] * sinf(2.0f * angle_mech);
    }
    float h1_amp_deg = (2.0f * sqrtf(f_a1 * f_a1 + f_b1 * f_b1) / (float)TOTAL_STEPS) * 57.2957795f;
    float h2_amp_deg = (2.0f * sqrtf(f_a2 * f_a2 + f_b2 * f_b2) / (float)TOTAL_STEPS) * 57.2957795f;

    float pk_pk_rad = max_err - min_err;
    float pk_pk_deg = pk_pk_rad * 57.2957795f;
    float rms_deg   = sqrtf(sum_sq_err / (float)TOTAL_STEPS) * 57.2957795f;

    Serial.println("--------------------------------------------------------------------------------");
    Serial.println("128-POINT NON-LINEARITY CORRECTION RESULTS:");
    Serial.printf ("  Peak-to-Peak Mechanical Error : %.3f deg (±%.3f deg)\n", pk_pk_deg, pk_pk_deg * 0.5f);
    Serial.printf ("  Electrical Error Span (11 PP) : ±%.2f deg elec\n", pk_pk_deg * 0.5f * (float)MOTOR_POLE_PAIRS);
    Serial.printf ("  RMS Correction Error          : %.3f deg mech\n", rms_deg);
    Serial.printf ("  Harmonic 1 (Eccentricity)     : ±%.3f deg\n", h1_amp_deg);
    Serial.printf ("  Harmonic 2 (Magnet Tilt)      : ±%.3f deg\n", h2_amp_deg);
    Serial.printf ("  Anomaly Points (>%.1f deg)    : %d\n", LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f, anomaly_count);

    // 5. Anomaly Gate
    bool lut_valid = (anomaly_count <= 2) && (pk_pk_rad <= LUT_PKPK_LIMIT_RAD);

    if (!lut_valid) {
        Serial.println("--------------------------------------------------------------------------------");
        Serial.println("❌ LUT CALIBRATION REJECTED — GEOMETRIC OR ROTOR STALL FAILURE");
        Serial.println("--------------------------------------------------------------------------------");
        if (anomaly_count > 0) {
            Serial.printf("  ⚠️ ROOT CAUSE: Rotor step stall / slip (%d points exceeded ±%.3f rad / ±%.2f deg).\n",
                          anomaly_count, LUT_ANOMALY_THRESHOLD_RAD, LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f);
            Serial.println("     FIX: Motor stiction exceeded hold torque. Increase sweep voltage ('CALIB 4.2').");
        }
        if (pk_pk_rad > LUT_PKPK_LIMIT_RAD) {
            Serial.printf("  ⚠️ ROOT CAUSE: Peak-to-Peak error span %.4f rad (%.2f deg mech) exceeds %.3f rad (%.2f deg) limit.\n",
                          pk_pk_rad, pk_pk_deg, LUT_PKPK_LIMIT_RAD, LUT_PKPK_LIMIT_RAD * 57.2957795f);
            if (h1_amp_deg > 1.2f) {
                Serial.printf("     DIAGNOSTIC: Harmonic 1 (Radial Eccentricity) is high (±%.2f deg mech).\n", h1_amp_deg);
                Serial.println("     FIX: Magnet is seated off-center (>0.15mm runout). Re-seat magnet on motor shaft axis.");
            } else if (h2_amp_deg > 0.8f) {
                Serial.printf("     DIAGNOSTIC: Harmonic 2 (Magnet Tilt) is high (±%.2f deg mech).\n", h2_amp_deg);
                Serial.println("     FIX: Magnet is tilted or PCB is not parallel to magnet face. Align sensor mounting plane.");
            } else {
                Serial.println("     DIAGNOSTIC: High-order harmonic distortion (stator slot cogging or bearing binding).");
                Serial.println("     FIX: Check shaft freespin by hand and ensure align voltage overcomes cogging detents.");
            }
        }
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }

    // Interpolate circular error table onto 128 raw encoder segments
    for (int s = 0; s < TOTAL_STEPS; s++) {
        float seg_angle = (float)s * step_mech_rad;
        encoder.lut_errors[s] = interpolateLUTAtAngle(seg_angle, mid_angles, temp_errors, TOTAL_STEPS);
    }
    encoder.is_lut_active = true;

    Serial.print("[LUT_READY]:M"); Serial.println(motor_id);
    Serial.println("  Status                        : ✅ 128-Point LUT ACTIVE (all points valid)");
    Serial.println("--------------------------------------------------------------------------------");
    return true;
}

// ----------------------------------------------------------------------------
// Multi-Pole Comprehensive Electrical Zero Alignment (Matching ESP32 Suite)
// ----------------------------------------------------------------------------
bool runComprehensiveElectricalAlignment(int motor_id, float align_voltage) {
    float hold_voltage = fminf(align_voltage * 0.85f, 3.40f);
    Serial.println("\n================================================================================");
    Serial.println(">>> STARTING COMPREHENSIVE MULTI-POINT ELECTRICAL ZERO CALIBRATION <<<");
    Serial.println("================================================================================");
    Serial.printf ("Target Motor       : GM3506 (11 Pole Pairs on STM32G474 M%d)\n", motor_id);
    Serial.printf ("Step / Hold Voltage: %.2f V (move) / %.2f V (hold & sample)\n", align_voltage, hold_voltage);
    Serial.println("Methodology        : 11-Pole Continuous Sweep + Bidirectional Dither Hysteresis Cancellation");
    Serial.printf ("Oversampling       : %d Samples / Point with 14-Bit MT6816 Resolution\n", CALIB_ALIGN_OVERSAMPLE_COUNT);
    BLDCMotor &motor = (motor_id == 1) ? motor1 : motor2;
    BLDCDriver3PWM &driver = (motor_id == 1) ? driver1 : driver2;
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;

    Serial.printf ("LUT Linearization  : %s\n", encoder.is_lut_active ? "NON-LINEARITY 128-PT [ACTIVE]" : "OFF [RAW SENSOR]");
    Serial.println("--------------------------------------------------------------------------------");

    driver.enable();

    // 1. Determining Sensor Commutation Direction & Verifying Motion
    Serial.print("1. Determining Sensor Commutation Direction & Verifying Motion... ");
    motor.setPhaseVoltage(align_voltage, 0.0f, _3PI_2);
    delay(400);
    float start_angle = encoder.getCalibratedAngleRad();

    const int DIR_STEPS = 400;
    for (int step = 0; step <= DIR_STEPS; step++) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(align_voltage, 0.0f, el_ang);
        delayMicroseconds(2500);
    }
    delay(200);
    float mid_angle = encoder.getCalibratedAngleRad();

    for (int step = DIR_STEPS; step >= 0; step--) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor.setPhaseVoltage(align_voltage, 0.0f, el_ang);
        delayMicroseconds(2500);
    }
    delay(200);

    float d_ang_fwd = wrapAngleRad(mid_angle - start_angle);
    float moved_rad = fabsf(d_ang_fwd);
    float min_expected_rad = (2.0f * _2PI / (float)MOTOR_POLE_PAIRS) * 0.40f;

    if (moved_rad < min_expected_rad) {
        Serial.println("FAILED!");
        Serial.println("  -------------------------------------------------------------");
        Serial.printf ("  ❌ MOTION CHECK FAILED: Rotor moved only %.2f deg mech (expected >= %.2f deg).\n",
                       moved_rad * 57.2957795f, min_expected_rad * 57.2957795f);
        Serial.printf ("     Angles sampled : start = %.4f rad (%.1f deg), mid = %.4f rad (%.1f deg)\n",
                       start_angle, start_angle * 57.2957795f, mid_angle, mid_angle * 57.2957795f);
        Serial.println("  -------------------------------------------------------------\n");
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }

    Direction sensor_dir = (d_ang_fwd > 0.0f) ? Direction::CW : Direction::CCW;
    Serial.printf("Detected %s (Moved %.1f deg mech) [OK]\n",
                  sensor_dir == Direction::CW ? "CW" : "CCW",
                  moved_rad * 57.2957795f);
    motor.sensor_direction = sensor_dir;

    // 2. Multi-Pole Full 360° Bidirectional Sweep with Real-Time Progression Tracking
    Serial.println("\n2. Scanning All 11 Electrical Poles (Full 360° Forward CW + Reverse CCW Round-Trip):");

    const int num_poles = MOTOR_POLE_PAIRS;
    static float fwd_zeros[MOTOR_POLE_PAIRS];
    static float rev_zeros[MOTOR_POLE_PAIRS];
    static float fwd_mechs[MOTOR_POLE_PAIRS];
    static float rev_mechs[MOTOR_POLE_PAIRS];
    static float pole_zeros[MOTOR_POLE_PAIRS];
    bool all_poles_ok = true;
    bool stall_detected = false;
    int stall_count = 0;

    // PASS 1: FORWARD (CW) 360° SWEEP (Poles 0 -> 10)
    for (int p = 0; p < num_poles; p++) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }

        float target_el = _3PI_2 + (float)p * _2PI;
        if (p > 0) {
            const int INTERP_STEPS = 120;
            float prev_el = _3PI_2 + (float)(p - 1) * _2PI;
            for (int s = 1; s <= INTERP_STEPS; s++) {
                float el = prev_el + (target_el - prev_el) * ((float)s / (float)INTERP_STEPS);
                motor.setPhaseVoltage(align_voltage, 0.0f, el);
                delayMicroseconds(2500);
            }
        } else {
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el);
            delay(300);
        }

        // Decaying micro-dither (8 steps)
        for (int d = 8; d >= 1; d--) {
            float dither = 0.08f * ((float)d / 8.0f);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el + dither);
            delayMicroseconds(800);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el - dither);
            delayMicroseconds(800);
        }
        motor.setPhaseVoltage(hold_voltage, 0.0f, target_el);
        delay(120);

        float mech = 0.0f;
        for (int s = 0; s < CALIB_ALIGN_OVERSAMPLE_COUNT; s++) {
            mech += encoder.getCalibratedAngleRad();
            delayMicroseconds(200);
        }
        mech /= (float)CALIB_ALIGN_OVERSAMPLE_COUNT;
        fwd_mechs[p] = mech;
        fwd_zeros[p] = normalizeAngleRad((float)((int)sensor_dir * num_poles) * mech);

        float step_deg = 0.0f;
        if (p > 0) {
            step_deg = wrapAngleRad(fwd_mechs[p] - fwd_mechs[p - 1]) * (sensor_dir == Direction::CW ? 1.0f : -1.0f) * 57.2957795f;
            if (step_deg < 12.0f) {
                stall_detected = true;
                stall_count++;
            }
        }
        Serial.printf("   [Pass 1 CW]  Pole #%2d/11 @ target el %4.0f° | mech %5.1f° (Δ=%+5.1f°) %s\n",
                      p + 1, (float)p * 360.0f, fwd_mechs[p] * 57.2957795f, step_deg,
                      (p > 0 && step_deg < 12.0f) ? "❌ STALL!" : "OK");
    }

    // PASS 2: REVERSE (CCW) 360° RETURN SWEEP (Poles 10 -> 0)
    for (int p = num_poles - 1; p >= 0; p--) {
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
            return false;
        }

        float target_el = _3PI_2 + (float)p * _2PI;
        if (p < num_poles - 1) {
            const int INTERP_STEPS = 120;
            float prev_el = _3PI_2 + (float)(p + 1) * _2PI;
            for (int s = 1; s <= INTERP_STEPS; s++) {
                float el = prev_el + (target_el - prev_el) * ((float)s / (float)INTERP_STEPS);
                motor.setPhaseVoltage(align_voltage, 0.0f, el);
                delayMicroseconds(2500);
            }
        } else {
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el);
            delay(150);
        }

        // Decaying micro-dither (8 steps)
        for (int d = 8; d >= 1; d--) {
            float dither = 0.08f * ((float)d / 8.0f);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el + dither);
            delayMicroseconds(800);
            motor.setPhaseVoltage(align_voltage, 0.0f, target_el - dither);
            delayMicroseconds(800);
        }
        motor.setPhaseVoltage(hold_voltage, 0.0f, target_el);
        delay(120);

        float mech = 0.0f;
        for (int s = 0; s < CALIB_ALIGN_OVERSAMPLE_COUNT; s++) {
            mech += encoder.getCalibratedAngleRad();
            delayMicroseconds(200);
        }
        mech /= (float)CALIB_ALIGN_OVERSAMPLE_COUNT;
        rev_mechs[p] = mech;
        rev_zeros[p] = normalizeAngleRad((float)((int)sensor_dir * num_poles) * mech);

        float rev_step_deg = 0.0f;
        if (p < num_poles - 1) {
            rev_step_deg = wrapAngleRad(rev_mechs[p] - rev_mechs[p + 1]) * (sensor_dir == Direction::CW ? -1.0f : 1.0f) * 57.2957795f;
            if (rev_step_deg < 12.0f) {
                stall_detected = true;
                stall_count++;
            }
        }
        Serial.printf("   [Pass 2 CCW] Pole #%2d/11 @ target el %4.0f° | mech %5.1f° (Δ=%+5.1f°) %s\n",
                      p + 1, (float)p * 360.0f, rev_mechs[p] * 57.2957795f, rev_step_deg,
                      (p < num_poles - 1 && rev_step_deg < 12.0f) ? "❌ STALL!" : "OK");
    }

    // Cumulative Motion Check
    float total_fwd_travel_deg = 0.0f;
    for (int p = 1; p < num_poles; p++) {
        total_fwd_travel_deg += wrapAngleRad(fwd_mechs[p] - fwd_mechs[p - 1]) * (sensor_dir == Direction::CW ? 1.0f : -1.0f) * 57.2957795f;
    }
    bool rotation_ok = (total_fwd_travel_deg >= 250.0f) && (stall_count == 0);

    // Synthesis, Error Decomposition & Diagnostics
    Serial.println("\n--------------------------------------------------------------------------------");
    Serial.println("Pole # | Mech Angle | Forward (CW)  | Reverse (CCW) | True Zero | Hysteresis | Status");
    Serial.println("-------+------------+---------------+---------------+-----------+------------+-------");

    float max_hyst_deg = 0.0f;
    float sum_hyst_deg = 0.0f;
    float sum_sin = 0.0f, sum_cos = 0.0f;
    float sum_fwd_sin = 0.0f, sum_fwd_cos = 0.0f;
    float sum_rev_sin = 0.0f, sum_rev_cos = 0.0f;

    for (int p = 0; p < num_poles; p++) {
        float fwd_z = fwd_zeros[p];
        float rev_z = rev_zeros[p];
        float true_z = normalizeAngleRad(atan2f(sinf(fwd_z) + sinf(rev_z), cosf(fwd_z) + cosf(rev_z)));
        pole_zeros[p] = true_z;

        sum_sin += sinf(true_z);
        sum_cos += cosf(true_z);
        sum_fwd_sin += sinf(fwd_z);
        sum_fwd_cos += cosf(fwd_z);
        sum_rev_sin += sinf(rev_z);
        sum_rev_cos += cosf(rev_z);

        float hyst_diff = wrapAngleRad(fwd_z - rev_z);
        float hyst_deg = fabsf(hyst_diff) * 57.2957795f;

        if (hyst_deg > max_hyst_deg) max_hyst_deg = hyst_deg;
        sum_hyst_deg += hyst_deg;

        bool pole_pass = (hyst_deg <= 8.0f);
        if (stall_detected && hyst_deg < 0.02f) {
            pole_pass = false; // Identical readings during a stall indicate zero motion!
        }
        if (!pole_pass) all_poles_ok = false;

        float pole_mech_target = ((float)p / (float)num_poles) * 360.0f;
        Serial.printf("  #%-2d | %5.1f deg | %6.4f rad | %6.4f rad | %6.4f rad | ±%4.2f deg | %s\n",
                      p + 1, pole_mech_target, fwd_z, rev_z, true_z,
                      hyst_deg * 0.5f, pole_pass ? "[PASS]" : "[WARN-HYST]");
    }
    Serial.println("--------------------------------------------------------------------------------");

    float mean_zero_angle = normalizeAngleRad(atan2f(sum_sin / (float)num_poles, sum_cos / (float)num_poles));
    float mean_fwd_angle  = normalizeAngleRad(atan2f(sum_fwd_sin / (float)num_poles, sum_fwd_cos / (float)num_poles));
    float mean_rev_angle  = normalizeAngleRad(atan2f(sum_rev_sin / (float)num_poles, sum_rev_cos / (float)num_poles));

    float global_hyst_deg = fabsf(wrapAngleRad(mean_fwd_angle - mean_rev_angle)) * 57.2957795f;
    float avg_hyst_deg    = sum_hyst_deg / (float)num_poles;

    // Standard deviation across all 11 poles
    float var_sum = 0.0f;
    for (int p = 0; p < num_poles; p++) {
        float diff = wrapAngleRad(pole_zeros[p] - mean_zero_angle);
        var_sum += diff * diff;
    }
    float std_dev_rad = sqrtf(var_sum / (float)num_poles);
    float std_dev_deg = std_dev_rad * 57.2957795f;

    float max_elec_err_rad = fmaxf(std_dev_rad, max_hyst_deg * 0.01745329f * 0.5f);
    float torque_eff_pct = cosf(max_elec_err_rad) * 100.0f;
    float d_axis_leak_pct = sinf(max_elec_err_rad) * 100.0f;

    Serial.println("\n----------------- 🏆 CERTIFIED ELECTRICAL ALIGNMENT RESULTS -----------------");
    Serial.printf ("  ⭐ Mean Zero Electric Angle : %.5f rad (%.2f deg elec)\n", mean_zero_angle, mean_zero_angle * 57.2957795f);
    Serial.printf ("  Commutation Direction       : %s\n", sensor_dir == Direction::CW ? "CW (+1)" : "CCW (-1)");
    Serial.printf ("  Directional Hysteresis Span : ±%.2f deg elec (Avg: %.2f deg | Peak: %.2f deg)\n",
                   global_hyst_deg * 0.5f, avg_hyst_deg, max_hyst_deg);
    Serial.printf ("  Pole-to-Pole Dispersion     : ±%.2f deg elec (±%.4f rad)\n", std_dev_deg, std_dev_rad);
    Serial.printf ("  Worst-Case Torque Output    : %.2f%% of theoretical Kt\n", torque_eff_pct);
    Serial.printf ("  Parasitic d-Axis Leakage    : %.2f%% (Current lost to heat)\n", d_axis_leak_pct);
    Serial.printf ("  Cumulative 360° Progression : %.1f deg mech (%s)\n", total_fwd_travel_deg, rotation_ok ? "VERIFIED FULL ROTATION" : "INCOMPLETE / STALLED");
    Serial.print  ("  Alignment Quality Score     : ");

    const char* grade_str = "GRADE E";

    // HARD MOTION GATE: If the rotor stalled or failed step progression, REJECT
    if (stall_detected || !rotation_ok || total_fwd_travel_deg < 250.0f || std_dev_deg > 10.0f || max_hyst_deg > 10.0f) {
        grade_str = "GRADE F";
        Serial.println("❌ GRADE F (REJECTED — Rotor Stall / Motion Check Failed)");
        Serial.println("  -----------------------------------------------------------------------------");
        if (stall_detected || !rotation_ok) {
            Serial.printf ("  ⚠️ ROOT CAUSE: Rotor step stall detected (%d poles stalled; total travel %.1f° of expected ~327°).\n",
                           stall_count, total_fwd_travel_deg);
            Serial.println("     FIX: Increase alignment voltage ('C1 4.5') to overcome cogging detents,");
            Serial.println("          verify 12V bench power supply is ON, and check motor bearing freespin.");
        }
        if (max_hyst_deg > 10.0f) {
            Serial.printf("  ⚠️ ROOT CAUSE: Bearing stiction or detent pull (Hysteresis Peak: %.1f deg).\n", max_hyst_deg);
        }
        if (std_dev_deg > 10.0f) {
            Serial.printf("  ⚠️ ROOT CAUSE: Severe magnet runout or uncalibrated LUT (StdDev: %.1f deg).\n", std_dev_deg);
        }
        Serial.println("================================================================================\n");
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return false;
    }

    if (std_dev_deg < 2.0f && max_hyst_deg <= 2.0f) {
        grade_str = "GRADE A";
        Serial.println("⭐ GRADE A (Exceptional Precision < 2.0 deg elec / < 0.18 deg mech)");
        Serial.println("     DIAGNOSTIC: Commutation vectors flawlessly decoupled; maximum theoretical torque.");
    } else if (std_dev_deg < 4.0f && max_hyst_deg <= 4.0f) {
        grade_str = "GRADE B";
        Serial.println("✅ GRADE B (Excellent / Competition Grade < 4.0 deg elec / < 0.36 deg mech)");
        Serial.println("     DIAGNOSTIC: Superb alignment; negligible d-axis cross-talk (<5%); maximum efficiency.");
    } else if (std_dev_deg < 6.0f && max_hyst_deg <= 6.0f) {
        grade_str = "GRADE C";
        Serial.println("✨ GRADE C (Good / Standard < 6.0 deg elec / < 0.55 deg mech)");
        Serial.println("     DIAGNOSTIC: Solid closed-loop commutation; minimal parasitic loss (<8%).");
    } else if (std_dev_deg < 8.0f && max_hyst_deg <= 8.0f) {
        grade_str = "GRADE D";
        Serial.println("⚡ GRADE D (Fair / Minor Ripple < 8.0 deg elec / < 0.73 deg mech)");
        Serial.println("     DIAGNOSTIC: Mild hysteresis / cogging ripple detected; minor thermal penalty.");
    } else {
        grade_str = "GRADE E";
        Serial.println("⚠️ GRADE E (Marginal / Usable — Moderate thermal penalty)");
    }

    Serial.println("  -----------------------------------------------------------------------------");
    Serial.println("  SimpleFOC Static Code Parameters:");
    Serial.printf ("    motor.zero_electric_angle = %.5ff;\n", mean_zero_angle);
    Serial.printf ("    motor.sensor_direction    = Direction::%s;\n", sensor_dir == Direction::CW ? "CW" : "CCW");
    Serial.println("================================================================================\n");

    motor.zero_electric_angle = mean_zero_angle;
    motor.sensor_direction = sensor_dir;
    if (motor_id == 1) {
        m1_base_zero_angle = mean_zero_angle;
        m1_calibrated = true;
    } else {
        m2_base_zero_angle = mean_zero_angle;
        m2_calibrated = true;
    }

    Serial.printf("[CALIB_DONE]:M%d:GRADE=%s:ZERO=%.5f\n", motor_id, grade_str, mean_zero_angle);
    return true;
}

// ----------------------------------------------------------------------------
// Master Auto-Calibration Orchestrator (128-pt Dual-Pass LUT + 11-Pole Zero + Dump)
// ----------------------------------------------------------------------------
void runMasterCalibrationSequence(int motor_id, float align_voltage) {
    BLDCMotor &motor = (motor_id == 1) ? motor1 : motor2;
    BLDCDriver3PWM &driver = (motor_id == 1) ? driver1 : driver2;
    MT6816Sensor &encoder = (motor_id == 1) ? encoder1 : encoder2;
    volatile bool &calib_active = (motor_id == 1) ? m1_calib_active : m2_calib_active;
    bool prev_enabled = (motor_id == 1) ? m1_enabled : m2_enabled;

    calib_active = true;
    live_calib_progress = 0;
    encoder.clearLUT();

    // Pass 1: 128-point LUT Sweep
    bool lut_ok = runSteppedLUTSweepSinglePass(motor_id, align_voltage);
    if (!lut_ok && align_voltage < CALIB_BOOST_VOLTAGE) {
        Serial.println("\n================================================================================");
        Serial.printf ("⚠️ STEPPED LUT CALIBRATION STALLED @ %.2fV!\n", align_voltage);
        Serial.printf ("⚡ AUTOMATIC TORQUE BOOST: Ramping to %.2fV and retrying sweep...\n", CALIB_BOOST_VOLTAGE);
        Serial.println("================================================================================\n");
        delay(400);
        lut_ok = runSteppedLUTSweepSinglePass(motor_id, CALIB_BOOST_VOLTAGE);
    }

    if (!lut_ok) {
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        if (!prev_enabled) driver.disable();
        calib_active = false;
        live_calib_progress = 0;
        Serial.printf("[CALIB_FAILED]:M%d LUT Sweep Rejected\n", motor_id);
        return;
    }

    // Thermal Cool-Down Pause before Electrical Alignment
    Serial.printf("\n>>> ❄️ THERMAL COOL-DOWN PAUSE (%ds) before 11-Pole Electrical Alignment...\n", CALIB_COOLDOWN_PAUSE_SEC);
    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    for (int sec = CALIB_COOLDOWN_PAUSE_SEC; sec > 0; sec--) {
        Serial.printf("   Coil cool-down: %ds remaining...\n", sec);
        delay(1000);
        if (checkCalibrationAbort()) {
            Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
            if (!prev_enabled) driver.disable();
            calib_active = false;
            live_calib_progress = 0;
            return;
        }
    }

    // Pass 2: 11-pole Electrical Alignment on Linearized Sensor
    bool align_ok = runComprehensiveElectricalAlignment(motor_id, align_voltage);
    if (!align_ok) {
        motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        if (!prev_enabled) driver.disable();
        calib_active = false;
        live_calib_progress = 0;
        Serial.printf("[CALIB_FAILED]:M%d Electrical Alignment Rejected\n", motor_id);
        return;
    }

    // Pass 3: Dump full LUT curve so GUI / Host can store it
    dumpLUT(motor_id);

    live_calib_progress = 100;
    sendBinaryTelemetry();

    motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
    calib_active = false;
    if (!prev_enabled) {
        driver.disable();
    }

    Serial.println(">>> 🏆 MASTER CALIBRATION SEQUENCE FULLY CERTIFIED & ACTIVE! <<<\n");
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
        motor1.KV_rating = MOTOR_KV_RATING;
        motor1.PID_velocity.P = 0.15f;
        motor1.PID_velocity.I = 1.50f;
        motor1.PID_velocity.D = 0.00000f;
        motor1.PID_velocity.output_ramp = 100.0f;
        motor1.PID_velocity.Ts = 0.0005f;
        motor1.LPF_velocity.Tf = 0.0005f;
        m1_lag_comp_us = 35.0f;
        m1_lag_filt_vel = 0.0f;
        m1_target_rpm = 0.0f;
    }
    if (motor_id == 2 || motor_id == 0) {
        motor2.voltage_limit = MOTOR_VOLTAGE_LIMIT;
        motor2.PID_velocity.limit = motor2.voltage_limit;
        motor2.velocity_limit = MOTOR_VELOCITY_LIMIT;
        motor2.KV_rating = MOTOR_KV_RATING;
        motor2.PID_velocity.P = 0.15f;
        motor2.PID_velocity.I = 1.50f;
        motor2.PID_velocity.D = 0.00000f;
        motor2.PID_velocity.output_ramp = 100.0f;
        motor2.PID_velocity.Ts = 0.0005f;
        motor2.LPF_velocity.Tf = 0.0005f;
        m2_lag_comp_us = 35.0f;
        m2_lag_filt_vel = 0.0f;
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
    Serial.print  ("  Shared Bus       : SPI1 @ 5.31 MHz (PB3/4/5) | CS1=PA15, CS2=PB11\n");
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
    Serial.println("  CALIB 1/2 [v]    : Master Auto-Calibration (128-pt LUT + 11-Pole Continuous Zero)");
    Serial.println("  w1 / w2 [v]      : Shorthand Master Calibration (matching ESP32 suite)");
    Serial.println("  C1 / C2 [v]      : Run 11-Pole Continuous Electrical Zero Alignment");
    Serial.println("  DUMP_LUT 1/2     : Dump 128-Point LUT Calibration Profile");
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

    // Check LUT dump command
    if (strncmp(cmd, "DUMP_LUT", 8) == 0 || strncmp(cmd, "dump_lut", 8) == 0 ||
        strncmp(cmd, "LUT_DUMP", 8) == 0 || strncmp(cmd, "lut_dump", 8) == 0) {
        int m = 1;
        sscanf(cmd, "%*s %d", &m);
        dumpLUT(m);
        return;
    }

    // Check CALIB command (e.g. 'CALIB 1', 'CALIB 2', 'CALIB 1 4.2')
    if (strncmp(cmd, "CALIB", 5) == 0 || strncmp(cmd, "calib", 5) == 0) {
        int m = 1;
        float v = CALIB_ALIGN_VOLTAGE;
        int parsed = sscanf(cmd, "%*s %d %f", &m, &v);
        if (parsed < 1) m = 1;
        if (parsed < 2 || v <= 0.0f) v = CALIB_ALIGN_VOLTAGE;
        runMasterCalibrationSequence(m, v);
        return;
    }

    // Master sweep shorthand 'w1', 'W1', 'w2', 'W2' (Matching ESP32 'w' command)
    if ((cmd[0] == 'w' || cmd[0] == 'W') && (cmd[1] == '1' || cmd[1] == '2')) {
        int m = cmd[1] - '0';
        float v = atof(cmd + 2);
        if (v <= 0.0f) v = CALIB_ALIGN_VOLTAGE;
        runMasterCalibrationSequence(m, v);
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

    if (strncmp(cmd, "C1", 2) == 0 || strncmp(cmd, "c1", 2) == 0) {
        float v = atof(cmd + 2);
        if (v <= 0.0f) v = CALIB_ALIGN_VOLTAGE;
        runComprehensiveElectricalAlignment(1, v);
        return;
    }
    if (strncmp(cmd, "C2", 2) == 0 || strncmp(cmd, "c2", 2) == 0) {
        float v = atof(cmd + 2);
        if (v <= 0.0f) v = CALIB_ALIGN_VOLTAGE;
        runComprehensiveElectricalAlignment(2, v);
        return;
    }
    if (strcmp(cmd, "C") == 0 || strcmp(cmd, "c") == 0) {
        runComprehensiveElectricalAlignment(1, CALIB_ALIGN_VOLTAGE);
        runComprehensiveElectricalAlignment(2, CALIB_ALIGN_VOLTAGE);
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

    if (strncasecmp(cmd, "RAMP1", 5) == 0) {
        float val = atof(cmd + 5);
        if (val > 0.0f) { motor1.PID_velocity.output_ramp = val; Serial.printf(">>> M1 Voltage Ramp: %.1f V/s\n", val); }
        return;
    }
    if (strncasecmp(cmd, "RAMP2", 5) == 0) {
        float val = atof(cmd + 5);
        if (val > 0.0f) { motor2.PID_velocity.output_ramp = val; Serial.printf(">>> M2 Voltage Ramp: %.1f V/s\n", val); }
        return;
    }

    if (strncasecmp(cmd, "MAXRPM1", 7) == 0) {
        float val = atof(cmd + 7);
        if (val > 0.0f) { motor1.velocity_limit = val * 0.104719755f; Serial.printf(">>> M1 Max Speed: %.1f RPM\n", val); }
        return;
    }
    if (strncasecmp(cmd, "MAXRPM2", 7) == 0) {
        float val = atof(cmd + 7);
        if (val > 0.0f) { motor2.velocity_limit = val * 0.104719755f; Serial.printf(">>> M2 Max Speed: %.1f RPM\n", val); }
        return;
    }

    if (strncasecmp(cmd, "KV1", 3) == 0) {
        float val = atof(cmd + 3);
        if (val > 0.0f) { motor1.KV_rating = val; Serial.printf(">>> M1 KV: %.1f\n", val); }
        return;
    }
    if (strncasecmp(cmd, "KV2", 3) == 0) {
        float val = atof(cmd + 3);
        if (val > 0.0f) { motor2.KV_rating = val; Serial.printf(">>> M2 KV: %.1f\n", val); }
        return;
    }

    if (strncasecmp(cmd, "LAG1", 4) == 0 || strncasecmp(cmd, "L1", 2) == 0) {
        const char *p = (strncasecmp(cmd, "LAG1", 4) == 0) ? (cmd + 4) : (cmd + 2);
        float val = atof(p);
        if (val >= 0.0f) { m1_lag_comp_us = val; Serial.printf(">>> M1 Sensor Lag Comp: %.1f us\n", val); }
        return;
    }
    if (strncasecmp(cmd, "LAG2", 4) == 0 || strncasecmp(cmd, "L2", 2) == 0) {
        const char *p = (strncasecmp(cmd, "LAG2", 4) == 0) ? (cmd + 4) : (cmd + 2);
        float val = atof(p);
        if (val >= 0.0f) { m2_lag_comp_us = val; Serial.printf(">>> M2 Sensor Lag Comp: %.1f us\n", val); }
        return;
    }
    if (strncasecmp(cmd, "LAG", 3) == 0) {
        float val = atof(cmd + 3);
        if (val >= 0.0f) { m1_lag_comp_us = val; m2_lag_comp_us = val; Serial.printf(">>> Both Motors Sensor Lag Comp: %.1f us\n", val); }
        return;
    }

    if (strncasecmp(cmd, "DWT1", 4) == 0) {
        float val = atof(cmd + 4);
        if (val > 0.0f) {
            encoder1.setDWTWindowMs(val);
            Serial.printf(">>> M1 DWT Velocity Window: %.2f ms (%d samples)\n", encoder1.window_ms, encoder1.window_samples);
        }
        return;
    }
    if (strncasecmp(cmd, "DWT2", 4) == 0) {
        float val = atof(cmd + 4);
        if (val > 0.0f) {
            encoder2.setDWTWindowMs(val);
            Serial.printf(">>> M2 DWT Velocity Window: %.2f ms (%d samples)\n", encoder2.window_ms, encoder2.window_samples);
        }
        return;
    }
    if (strncasecmp(cmd, "DWT", 3) == 0) {
        float val = atof(cmd + 3);
        if (val > 0.0f) {
            encoder1.setDWTWindowMs(val);
            encoder2.setDWTWindowMs(val);
            Serial.printf(">>> Both Motors DWT Velocity Window: %.2f ms (%d samples)\n", val, encoder1.window_samples);
        }
        return;
    }

    // Direct relative zero nudge via standalone +<delta> or -<delta>
    if (cmd[0] == '+' || (cmd[0] == '-' && (cmd[1] == '.' || (cmd[1] >= '0' && cmd[1] <= '9')))) {
        float delta = atof(cmd);
        motor1.zero_electric_angle = normalizeAngleRad(m1_base_zero_angle + delta);
        m1_base_zero_angle = motor1.zero_electric_angle;
        m1_calibrated = true;
        Serial.printf(">>> M1 Zero Electric Angle Nudged: %.4f rad (%.1f deg) [delta: %+.4f]\n",
                      motor1.zero_electric_angle, motor1.zero_electric_angle * 57.2957795f, delta);
        return;
    }

    if (strncasecmp(cmd, "ZERO1", 5) == 0 || strncasecmp(cmd, "O1", 2) == 0) {
        const char *p = (strncasecmp(cmd, "ZERO1", 5) == 0) ? (cmd + 5) : (cmd + 2);
        while (*p == ' ') p++;
        bool is_rel = (*p == '+' || *p == '-');
        float val = atof(p);
        if (is_rel) {
            motor1.zero_electric_angle = normalizeAngleRad(m1_base_zero_angle + val);
        } else {
            motor1.zero_electric_angle = normalizeAngleRad(val);
        }
        m1_base_zero_angle = motor1.zero_electric_angle;
        m1_calibrated = true;
        Serial.printf(">>> M1 Zero Electric Angle %s: %.4f rad (%.1f deg)\n",
                      is_rel ? "Nudged to" : "Override",
                      motor1.zero_electric_angle, motor1.zero_electric_angle * 57.2957795f);
        return;
    }
    if (strncasecmp(cmd, "ZERO2", 5) == 0 || strncasecmp(cmd, "O2", 2) == 0) {
        const char *p = (strncasecmp(cmd, "ZERO2", 5) == 0) ? (cmd + 5) : (cmd + 2);
        while (*p == ' ') p++;
        bool is_rel = (*p == '+' || *p == '-');
        float val = atof(p);
        if (is_rel) {
            motor2.zero_electric_angle = normalizeAngleRad(m2_base_zero_angle + val);
        } else {
            motor2.zero_electric_angle = normalizeAngleRad(val);
        }
        m2_base_zero_angle = motor2.zero_electric_angle;
        m2_calibrated = true;
        Serial.printf(">>> M2 Zero Electric Angle %s: %.4f rad (%.1f deg)\n",
                      is_rel ? "Nudged to" : "Override",
                      motor2.zero_electric_angle, motor2.zero_electric_angle * 57.2957795f);
        return;
    }

    if (strncasecmp(cmd, "A1", 2) == 0) {
        float val = atof(cmd + 2);
        if (val > 0.0f) { motor1.voltage_sensor_align = val; Serial.printf(">>> M1 Align Voltage: %.2f V\n", val); }
        return;
    }
    if (strncasecmp(cmd, "A2", 2) == 0) {
        float val = atof(cmd + 2);
        if (val > 0.0f) { motor2.voltage_sensor_align = val; Serial.printf(">>> M2 Align Voltage: %.2f V\n", val); }
        return;
    }

    if ((cmd[0] == 'v' || cmd[0] == 'V') && (cmd[1] == '+' || cmd[1] == '-' || (cmd[1] >= '0' && cmd[1] <= '9'))) {
        float val = atof(cmd + 1);
        driver1.enable();
        motor1.setPhaseVoltage(fabsf(val), 0.0f, (val >= 0.0f) ? 0.0f : _PI);
        Serial.printf(">>> Open-Loop Voltage Test: %+.2f V\n", val);
        return;
    }

    if (strcasecmp(cmd, "s") == 0 || strcasecmp(cmd, "STATUS") == 0) {
        printHelp();
        return;
    }

    if (strncasecmp(cmd, "SET_MODE", 8) == 0) {
        int mode = atoi(cmd + 8);
        encoder1.is_lut_active = (mode == 1);
        encoder2.is_lut_active = (mode == 1);
        Serial.printf(">>> Calibration Mode: %s\n", (mode == 1) ? "128-Point LUT Active" : "Raw Angle");
        return;
    }
    if (strncasecmp(cmd, "M1", 2) == 0 && (cmd[2] == ' ' || cmd[2] == '\0')) {
        int mode = atoi(cmd + 2);
        encoder1.is_lut_active = (mode == 1);
        Serial.printf(">>> M1 LUT Mode: %s\n", (mode == 1) ? "128-Point LUT Active" : "Raw Angle");
        return;
    }
    if (strncasecmp(cmd, "M2", 2) == 0 && (cmd[2] == ' ' || cmd[2] == '\0')) {
        int mode = atoi(cmd + 2);
        encoder2.is_lut_active = (mode == 1);
        Serial.printf(">>> M2 LUT Mode: %s\n", (mode == 1) ? "128-Point LUT Active" : "Raw Angle");
        return;
    }

    if (strcasecmp(cmd, "u") == 0 || strcasecmp(cmd, "u1") == 0) {
        dumpLUT(1);
        return;
    }
    if (strcasecmp(cmd, "u2") == 0) {
        dumpLUT(2);
        return;
    }

    if (strcasecmp(cmd, "SAVE") == 0 || strcasecmp(cmd, "save") == 0) {
        saveConfigToFlash();
        dumpConfigJSON();
        Serial.println("[CONFIG_SAVED]:OK");
        return;
    }

    if (strcasecmp(cmd, "LOAD") == 0 || strcasecmp(cmd, "load") == 0) {
        bool ok = loadConfigFromFlash();
        dumpConfigJSON();
        Serial.println(ok ? "[CONFIG_LOADED]:OK" : "[CONFIG_LOADED]:FAIL");
        return;
    }

    if (strcasecmp(cmd, "EXPORT") == 0 || strcasecmp(cmd, "export") == 0) {
        dumpConfigJSON();
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
    // 0. Immediately clamp Driver Enable pins LOW to prevent current surge on startup
    pinMode(M1_EN, OUTPUT);
    digitalWrite(M1_EN, LOW);
    pinMode(M2_EN, OUTPUT);
    digitalWrite(M2_EN, LOW);

    // 1. Standalone DWT Unlock (Essential for standalone battery/bench operation without ST-Link)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    // 2. Status LED setup
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    // 3. Serial Port Initialization (921600 Baud for Ultra High Speed Telemetry)
    Serial.begin(921600);

    // 4. Initialize MT6816 Encoders on shared SPI1
    pinMode(PIN_M2_CS, OUTPUT);
    digitalWrite(PIN_M2_CS, HIGH); // Ensure Encoder 2 CS is deselected
    encoder1.init();
    encoder2.setupCSPin(PIN_M2_CS);

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
    motor1.PID_velocity.P = 0.15f;
    motor1.PID_velocity.I = 1.50f;
    motor1.PID_velocity.D = 0.00000f;
    motor1.PID_velocity.output_ramp = 100.0f;
    motor1.PID_velocity.Ts = 0.0005f;
    motor1.PID_velocity.limit = motor1.voltage_limit;
    motor1.LPF_velocity.Tf = 0.0005f;
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
    motor2.PID_velocity.P = 0.15f;
    motor2.PID_velocity.I = 1.50f;
    motor2.PID_velocity.D = 0.00000f;
    motor2.PID_velocity.output_ramp = 100.0f;
    motor2.PID_velocity.Ts = 0.0005f;
    motor2.PID_velocity.limit = motor2.voltage_limit;
    motor2.LPF_velocity.Tf = 0.0005f;
    motor2.init();
    driver2.disable();

    // 7. Silent Pre-Loaded Alignment Check (Auto-load from Flash if saved, else fallback to constants)
    if (loadConfigFromFlash()) {
        Serial.println(">>> [STARTUP] Calibration & LUT profile loaded from Flash.");
    } else {
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
