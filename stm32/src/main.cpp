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

    // 1. Single-Read SPI Caching: Poll both MT6816 sensors strictly ONCE per tick
    encoder1.readHardwareSPI();
    encoder2.readHardwareSPI();

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
// Stepped 128-Point Calibration Sweep (Background Task)
// ----------------------------------------------------------------------------
void runSteppedCalibration(int motor_id) {
    Serial.print(">>> STARTING 128-POINT STEPPED CALIBRATION FOR MOTOR ");
    Serial.println(motor_id);

    if (motor_id == 1) {
        m1_calib_active = true;
        driver1.enable();
    } else {
        m2_calib_active = true;
        driver2.enable();
    }

    const float align_volt = 3.5f;
    const int total_steps = 128;
    float recorded_errors[128];

    // Sweep 128 electrical angles across one complete mechanical revolution
    float elec_step = (2.0f * _PI * (float)MOTOR_POLE_PAIRS) / (float)total_steps;

    for (int i = 0; i < total_steps; i++) {
        live_calib_progress = (uint8_t)((i * 100) / total_steps);
        float test_elec_angle = (float)i * elec_step;

        if (motor_id == 1) {
            motor1.setPhaseVoltage(align_volt, 0.0f, test_elec_angle);
        } else {
            motor2.setPhaseVoltage(align_volt, 0.0f, test_elec_angle);
        }

        delay(30); // Allow rotor mechanical settling

        // Read angle from ISR's cached buffer with zero bus contention
        float measured_rad = (motor_id == 1) ? encoder1.getCalibratedAngleRad() : encoder2.getCalibratedAngleRad();
        float expected_rad = (float)i * (2.0f * _PI / (float)total_steps);
        float err = measured_rad - expected_rad;

        // Wrap error to [-PI, +PI]
        while (err > _PI) err -= 2.0f * _PI;
        while (err < -_PI) err += 2.0f * _PI;

        recorded_errors[i] = err;

        if (i % 16 == 0) {
            Serial.print("[CALIB_PROG]:");
            Serial.print(live_calib_progress);
            Serial.println("%");
        }
    }

    live_calib_progress = 100;

    // Apply recorded errors to encoder LUT
    for (int line = 0; line < 8; line++) {
        if (motor_id == 1) {
            encoder1.setLUTLine(line, &recorded_errors[line * 16], 16);
        } else {
            encoder2.setLUTLine(line, &recorded_errors[line * 16], 16);
        }
    }

    if (motor_id == 1) {
        m1_calib_active = false;
        motor1.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        Serial.println("[LUT_READY]:M1");
    } else {
        m2_calib_active = false;
        motor2.setPhaseVoltage(0.0f, 0.0f, 0.0f);
        Serial.println("[LUT_READY]:M2");
    }

    Serial.println(">>> 128-POINT CALIBRATION COMPLETE & LUT ACTIVATED! <<<");
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

    // 4. Initialize MT6816 Dual Encoders on Shared SPI1 @ 10.6 MHz
    encoder1.init();
    encoder2.init();

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
