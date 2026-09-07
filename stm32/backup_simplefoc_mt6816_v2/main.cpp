#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

// ============================================================================
// DA WINNA — MT6816 SPI ENCODER & SIMPLEFOC SUITE (GM3506 OPTIMIZED @ 12V)
// ============================================================================

// SPI1 Master Pins (STM32G474 Relaxed & Strict compatible)
#define PIN_SCK      PA5    // Clock (Board Pin A5)
#define PIN_MOSI     PA7    // Master Out / 3-Wire SDAT (Board Pin A7)
#define PIN_MISO     PB4    // Master In (Board Pin B4)

// Dedicated CS Pins
#define PIN_CS_PA3   PA3    // Relaxed ENC1 (Board Pin A3)
#define PIN_CS_PA15  PA15   // Strict ENC1 (Board Pin A15)
#define PIN_CS_PB1   PB1    // ENC2/3 (Board Pin B1)
#define PIN_CS_PB2   PB2    // ENC3/4 (Board Pin B2)
#define PIN_CS_PB3   PB3    // Board Pin B3

// MT6816 PWM Pulse Capture Pin (Ground Truth Diagnostic)
#define PIN_PWM_IN   PA0

// Motor 1 Inverter Driver Pins (Relaxed: PA8, PA9, PA10, PB3)
#define M1_PWM_A     PA8    // TIM1_CH1 (Board Pin A8)
#define M1_PWM_B     PA9    // TIM1_CH2 (Board Pin A9)
#define M1_PWM_C     PA10   // TIM1_CH3 (Board Pin A10)
#define M1_EN        PB3    // Gate Driver Enable (Board Pin B3)

// Status LED
#define STATUS_LED   PC13

// GM3506 Motor Specs: 11 Pole Pairs (24N22P), 12V Supply
#define MOTOR_POLE_PAIRS        11
#define MOTOR_POWER_SUPPLY_VOLT 12.0f
#define MOTOR_ALIGN_VOLTAGE     3.0f

// PWM Capture Global ISR Variables
volatile uint32_t pwm_rise_us = 0;
volatile uint32_t pwm_high_us = 0;
volatile uint32_t pwm_period_us = 0;
volatile bool pwm_new_data = false;

void pwmPinInterrupt() {
    uint32_t now = micros();
    if (digitalRead(PIN_PWM_IN) == HIGH) {
        if (pwm_rise_us != 0) {
            pwm_period_us = now - pwm_rise_us;
        }
        pwm_rise_us = now;
    } else {
        if (pwm_rise_us != 0) {
            pwm_high_us = now - pwm_rise_us;
            pwm_new_data = true;
        }
    }
}

// ============================================================================
// MT6816 High-Performance Sensor Driver (Custom Filtered Velocity Engine)
// ============================================================================
class MT6816Sensor : public Sensor {
public:
    int cs_pin;
    uint32_t spi_speed;
    uint8_t spi_mode;
    bool use_3wire;

    // Diagnostics & Tracking
    uint32_t total_reads;
    uint32_t parity_errors;
    uint32_t mag_faults;
    bool last_no_mag;
    bool last_parity_ok;
    uint8_t last_raw_h;
    uint8_t last_raw_l;
    uint16_t last_raw_14bit;
    uint16_t prev_raw_count;
    int32_t full_rotations;
    float zero_offset_deg;

    // Velocity Estimation State
    uint32_t last_vel_time_us;
    float prev_vel_angle;
    float filtered_velocity_rad_s;

    MT6816Sensor()
        : cs_pin(PIN_CS_PA3), spi_speed(4000000), spi_mode(SPI_MODE3), use_3wire(false),
          total_reads(0), parity_errors(0), mag_faults(0), last_no_mag(false),
          last_parity_ok(true), last_raw_h(0), last_raw_l(0), last_raw_14bit(0),
          prev_raw_count(0), full_rotations(0), zero_offset_deg(0.0f),
          last_vel_time_us(0), prev_vel_angle(0.0f), filtered_velocity_rad_s(0.0f) {
    }

    void setMode(bool three_wire) {
        use_3wire = three_wire;
        if (use_3wire) {
            SPI.end();
            pinMode(PIN_SCK, OUTPUT);
            digitalWrite(PIN_SCK, HIGH);
            pinMode(PIN_MOSI, INPUT_PULLUP);
        } else {
            SPI.setSCLK(PIN_SCK);
            SPI.setMISO(PIN_MISO);
            SPI.setMOSI(PIN_MOSI);
            SPI.begin();
        }
    }

    void init() {
        pinMode(PIN_CS_PA3, OUTPUT);  digitalWrite(PIN_CS_PA3, HIGH);
        pinMode(PIN_CS_PA15, OUTPUT); digitalWrite(PIN_CS_PA15, HIGH);
        pinMode(PIN_CS_PB1, OUTPUT);  digitalWrite(PIN_CS_PB1, HIGH);
        pinMode(PIN_CS_PB2, OUTPUT);  digitalWrite(PIN_CS_PB2, HIGH);
        pinMode(PIN_CS_PB3, OUTPUT);  digitalWrite(PIN_CS_PB3, HIGH);

        // 1. Force 4-Wire Hardware SPI default
        setMode(false);

        // 2. Transmit register command to force 4-Wire mode in MT6816 RAM
        digitalWrite(cs_pin, LOW);
        delayMicroseconds(2);
        SPI.transfer16(0x0100);
        delayMicroseconds(2);
        digitalWrite(cs_pin, HIGH);
        delayMicroseconds(5);

        // 3. Test 4-Wire response
        uint8_t h = 0, l = 0;
        uint16_t raw14 = 0;
        bool no_mag = false, parity_ok = false;
        readCurrent(h, l, raw14, no_mag, parity_ok);

        if (h == 0xFF && l == 0xFF) {
            setMode(true);
            readCurrent(h, l, raw14, no_mag, parity_ok);
        }

        last_raw_h = h;
        last_raw_l = l;
        last_raw_14bit = raw14;
        prev_raw_count = raw14;
        last_no_mag = no_mag;
        last_parity_ok = parity_ok;

        last_vel_time_us = micros();
        prev_vel_angle = ((float)raw14 / 16384.0f) * _2PI;
        filtered_velocity_rad_s = 0.0f;
    }

    static bool parityCheck(uint16_t data) {
        int count = 0;
        for (int i = 0; i < 16; i++) {
            if (data & (1 << i)) count++;
        }
        return (count % 2 == 0); // Even Parity
    }

    // 4-Wire Hardware SPI Transfer
    void read4WireHW(int target_cs, uint8_t &h, uint8_t &l) {
        SPISettings settings(spi_speed, MSBFIRST, spi_mode);
        noInterrupts();

        SPI.beginTransaction(settings);
        digitalWrite(target_cs, LOW);
        delayMicroseconds(1);
        uint16_t rx_h = SPI.transfer16(0x8300);
        delayMicroseconds(1);
        digitalWrite(target_cs, HIGH);
        SPI.endTransaction();
        delayMicroseconds(1);

        SPI.beginTransaction(settings);
        digitalWrite(target_cs, LOW);
        delayMicroseconds(1);
        uint16_t rx_l = SPI.transfer16(0x8400);
        delayMicroseconds(1);
        digitalWrite(target_cs, HIGH);
        SPI.endTransaction();
        delayMicroseconds(1);

        interrupts();
        h = rx_h & 0xFF;
        l = rx_l & 0xFF;
    }

    // 3-Wire Atomic Half-Duplex Transfer on PA7 (SDAT)
    static uint8_t transfer3WireByte(uint8_t out_byte, bool read_mode) {
        uint8_t in_byte = 0;
        if (!read_mode) {
            pinMode(PIN_MOSI, OUTPUT);
            for (int i = 7; i >= 0; i--) {
                digitalWrite(PIN_MOSI, (out_byte & (1 << i)) ? HIGH : LOW);
                delayMicroseconds(1);
                digitalWrite(PIN_SCK, LOW);
                delayMicroseconds(1);
                digitalWrite(PIN_SCK, HIGH);
                delayMicroseconds(1);
            }
        } else {
            pinMode(PIN_MOSI, INPUT_PULLUP);
            for (int i = 7; i >= 0; i--) {
                digitalWrite(PIN_SCK, LOW);
                delayMicroseconds(1);
                digitalWrite(PIN_SCK, HIGH);
                delayMicroseconds(1);
                if (digitalRead(PIN_MOSI)) in_byte |= (1 << i);
            }
        }
        return in_byte;
    }

    void read3WireAtomic(int target_cs, uint8_t &h, uint8_t &l) {
        pinMode(PIN_SCK, OUTPUT);
        digitalWrite(PIN_SCK, HIGH);

        noInterrupts();

        digitalWrite(target_cs, LOW);
        delayMicroseconds(2);
        transfer3WireByte(0x83, false);
        h = transfer3WireByte(0x00, true);
        delayMicroseconds(2);
        digitalWrite(target_cs, HIGH);
        delayMicroseconds(2);

        digitalWrite(target_cs, LOW);
        delayMicroseconds(2);
        transfer3WireByte(0x84, false);
        l = transfer3WireByte(0x00, true);
        delayMicroseconds(2);
        digitalWrite(target_cs, HIGH);
        delayMicroseconds(2);

        interrupts();
    }

    bool readCurrent(uint8_t &h, uint8_t &l, uint16_t &raw14, bool &no_mag, bool &parity_ok) {
        if (use_3wire) {
            read3WireAtomic(cs_pin, h, l);
        } else {
            read4WireHW(cs_pin, h, l);
        }

        uint16_t angle_data = ((uint16_t)h << 8) | l;
        raw14 = ((uint16_t)h << 6) | ((l & 0xFC) >> 2);
        no_mag = (l & 0x02) != 0;
        parity_ok = parityCheck(angle_data);

        total_reads++;
        if (!parity_ok) parity_errors++;
        if (no_mag) mag_faults++;

        return parity_ok;
    }

    // Core SimpleFOC update: reads sensor & updates multi-turn rotation count
    void update() override {
        uint8_t h, l;
        uint16_t raw;
        bool no_mag, parity_ok;
        readCurrent(h, l, raw, no_mag, parity_ok);

        if (parity_ok || last_raw_14bit == 0) {
            last_raw_h = h;
            last_raw_l = l;
            last_raw_14bit = raw;
            last_no_mag = no_mag;
            last_parity_ok = true;

            int32_t delta = (int32_t)raw - (int32_t)prev_raw_count;
            if (delta > 8192) full_rotations--;
            else if (delta < -8192) full_rotations++;
            prev_raw_count = raw;
        } else {
            last_parity_ok = false;
        }
    }

    // Single-turn mechanical angle in [0, 2PI)
    float getSensorAngle() override {
        return ((float)last_raw_14bit / 16384.0f) * _2PI;
    }

    // Continuous multi-turn mechanical angle
    float getAngle() override {
        return ((float)full_rotations * _2PI) + (((float)last_raw_14bit / 16384.0f) * _2PI);
    }

    // Accurate, Low-Pass Filtered Velocity (rad/s)
    float getVelocity() override {
        uint32_t now_us = micros();
        float dt = (now_us - last_vel_time_us) * 1e-6f;
        if (dt < 0.0005f) { // Sample velocity at max 2 kHz
            return filtered_velocity_rad_s;
        }
        last_vel_time_us = now_us;

        float current_angle = ((float)last_raw_14bit / 16384.0f) * _2PI;
        float delta_angle = current_angle - prev_vel_angle;
        prev_vel_angle = current_angle;

        // Wrap delta angle to [-PI, +PI]
        if (delta_angle > _PI) delta_angle -= _2PI;
        else if (delta_angle < -_PI) delta_angle += _2PI;

        float raw_vel = delta_angle / dt;

        // Clean low-pass filter (Tf = 25ms)
        float alpha = dt / (dt + 0.025f);
        filtered_velocity_rad_s += alpha * (raw_vel - filtered_velocity_rad_s);

        // Immediate zero clamp when stationary
        if (fabsf(filtered_velocity_rad_s) < 0.08f && fabsf(delta_angle) < 1e-5f) {
            filtered_velocity_rad_s = 0.0f;
        }

        return filtered_velocity_rad_s;
    }

    float getSingleTurnDeg() {
        float d = ((float)last_raw_14bit / 16384.0f) * 360.0f - zero_offset_deg;
        while (d < 0.0f) d += 360.0f;
        while (d >= 360.0f) d -= 360.0f;
        return d;
    }

    void zeroCurrentPosition() {
        zero_offset_deg = ((float)last_raw_14bit / 16384.0f) * 360.0f;
        full_rotations = 0;
    }
};

MT6816Sensor encoder1;

// SimpleFOC Motor 1 Driver & BLDC Motor
BLDCDriver3PWM driver1(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);
BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);

// Control States
MotionControlType control_mode = MotionControlType::velocity;
bool is_enabled = false;
bool is_calibrated = false;
bool show_telemetry = true;
float m1_target_rpm = 0.0f;

// Stall & Safety Watchdog Parameters (Enabled by default)
bool stall_watchdog_enabled = true;
uint32_t stall_timer = 0;
bool is_stalled = false;

uint32_t last_telemetry_time = 0;
uint32_t last_led_time = 0;
bool led_state = false;

const char* getCSName(int p) {
    if (p == PIN_CS_PA3) return "PA3 (A3)";
    if (p == PIN_CS_PA15) return "PA15 (A15)";
    if (p == PIN_CS_PB1) return "PB1 (B1)";
    if (p == PIN_CS_PB2) return "PB2 (B2)";
    if (p == PIN_CS_PB3) return "PB3 (B3)";
    return "UNKNOWN";
}

void printVisualDial(float deg) {
    int index = (int)(deg / 22.5f) % 16;
    Serial.print(" [");
    for (int i = 0; i < 16; i++) {
        if (i == index) Serial.print("O");
        else if (i == 0 || i == 4 || i == 8 || i == 12) Serial.print("+");
        else Serial.print("-");
    }
    Serial.print("]");
}

void printHelp() {
    Serial.println("\n================================================================================");
    Serial.println("  DA WINNA — MT6816 4-WIRE/3-WIRE SPI ENCODER & SIMPLEFOC SUITE (GM3506)");
    Serial.println("================================================================================");
    Serial.print  ("  Driver State     : "); Serial.println(is_enabled ? "ENABLED" : "DISABLED");
    Serial.print  ("  Control Mode     : "); Serial.println(control_mode == MotionControlType::velocity_openloop ? "OPEN-LOOP" : "CLOSED-LOOP FOC");
    Serial.print  ("  Target Speed     : "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
    Serial.print  ("  Voltage Limit    : "); Serial.print(motor1.voltage_limit, 1); Serial.println(" V");
    Serial.print  ("  Velocity PID     : P="); Serial.print(motor1.PID_velocity.P, 3);
    Serial.print  (" | I="); Serial.print(motor1.PID_velocity.I, 3);
    Serial.print  (" | D="); Serial.print(motor1.PID_velocity.D, 4);
    Serial.print  (" | Tf="); Serial.print(motor1.LPF_velocity.Tf, 4); Serial.println("s");
    Serial.print  ("  Active CS Pin    : "); Serial.println(getCSName(encoder1.cs_pin));
    Serial.print  ("  Active Bus Mode  : "); Serial.println(encoder1.use_3wire ? "3-Wire Half-Duplex (PA7 SDAT)" : "4-Wire Full Hardware SPI1");
    Serial.print  ("  Stall Watchdog   : "); Serial.println(stall_watchdog_enabled ? "ACTIVE (800ms timeout)" : "DISABLED");
    Serial.println("--------------------------------------------------------------------------------");
    Serial.println("Actions (No Enter key required):");
    Serial.println("  r             : Reset to GM3506 12V Optimized Defaults (P=0.12, I=1.2, D=0.001, Tf=0.025, V=12V)");
    Serial.println("  w             : Toggle 4-Wire Hardware SPI vs 3-Wire Half-Duplex");
    Serial.println("  c             : Start FOC closed-loop alignment calibration");
    Serial.println("  o             : Switch to Open-Loop mode (No sensor required)");
    Serial.println("  e             : Toggle Enable / Disable driver");
    Serial.println("  x or Space    : Disable motor and stop (0 RPM)");
    Serial.println("  k             : Toggle Stall Detection Watchdog ON/OFF");
    Serial.println("  g             : Toggle telemetry streaming ON / OFF");
    Serial.println("  t<float>      : Set target speed in RPM (e.g. 't120' or 't-60')");
    Serial.println("  p<float>      : Set velocity P gain (e.g. 'p0.12')");
    Serial.println("  i<float>      : Set velocity I gain (e.g. 'i1.20')");
    Serial.println("  d<float>      : Set velocity D gain (e.g. 'd0.001')");
    Serial.println("  f<float>      : Set velocity LPF Tf in seconds (e.g. 'f0.025')");
    Serial.println("  v<float>      : Set motor voltage limit (e.g. 'v12.0')");
    Serial.println("  z             : Tare / Zero current encoder angle");
    Serial.println("  h or ?        : Show this status menu");
    Serial.println("================================================================================\n");
}

void setup() {
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    pinMode(PIN_PWM_IN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_PWM_IN), pwmPinInterrupt, CHANGE);

    Serial.begin(921600);
    Serial.setTimeout(25);
    SimpleFOCDebug::enable(&Serial);

    uint32_t start_time = millis();
    while (!Serial && (millis() - start_time < 3000)) {
        digitalWrite(STATUS_LED, (millis() % 200 < 100) ? LOW : HIGH);
    }
    digitalWrite(STATUS_LED, HIGH);

    // 1. Initialize MT6816 Sensor (Default 4-Wire Hardware SPI)
    encoder1.init();

    // 2. Initialize Driver 1 (12V Supply)
    driver1.voltage_power_supply = MOTOR_POWER_SUPPLY_VOLT;
    driver1.voltage_limit = MOTOR_POWER_SUPPLY_VOLT;
    driver1.init();

    // 3. Initialize Motor 1 (GM3506 Optimized Defaults)
    motor1.linkDriver(&driver1);
    motor1.linkSensor(&encoder1);
    motor1.voltage_limit = 12.0f;
    motor1.velocity_limit = 50.0f;
    motor1.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;

    // Velocity PID parameters optimized for GM3506 Gimbal Motor @ 12V
    motor1.controller = MotionControlType::velocity;
    motor1.PID_velocity.P = 0.12f;
    motor1.PID_velocity.I = 1.20f;
    motor1.PID_velocity.D = 0.001f;
    motor1.PID_velocity.output_ramp = 1000.0f;
    motor1.PID_velocity.limit = 12.0f;
    motor1.LPF_velocity.Tf = 0.025f;

    motor1.init();
    driver1.disable(); // Disabled on boot for safety

    printHelp();

    last_telemetry_time = millis();
    last_led_time = millis();
}

void loop() {
    // 1. Fast FOC Commutation Loop
    if (is_enabled) {
        motor1.loopFOC();
        float target_rad_s = m1_target_rpm * 0.104719755f;
        motor1.move(target_rad_s);
    } else {
        // When motor is disabled, update sensor periodically for accurate manual rotation tracking
        static uint32_t last_enc_update = 0;
        uint32_t now_us = micros();
        if (now_us - last_enc_update >= 1000) { // 1 kHz
            last_enc_update = now_us;
            encoder1.update();
        }
    }

    // 2. Universal Active Stall & Overload Safety Watchdog
    if (is_enabled && stall_watchdog_enabled && control_mode == MotionControlType::velocity) {
        float actual_rpm = encoder1.getVelocity() * 9.54929659f;
        float vel_error = fabsf(m1_target_rpm - actual_rpm);
        float applied_u = fabsf(motor1.voltage.q);

        bool is_stall_condition = false;

        // Condition 1: Target commanded (|Target| >= 3 RPM), but motor cannot spin (|Actual| < 2.0 RPM or vel_error > 0.4*Target)
        //              and driver voltage is demanding torque (applied_u >= 1.2V)
        if (fabsf(m1_target_rpm) >= 3.0f) {
            if ((vel_error > 0.4f * fabsf(m1_target_rpm) || fabsf(actual_rpm) < 2.0f) && applied_u >= 1.2f) {
                is_stall_condition = true;
            }
        } 
        // Condition 2: Target is ~0 RPM, but high external torque/block forces voltage >= 2.2V
        else if (applied_u >= 2.2f) {
            is_stall_condition = true;
        }

        if (is_stall_condition) {
            if (stall_timer == 0) {
                stall_timer = millis();
            } else if (millis() - stall_timer >= 800) { // 800 ms trigger
                is_stalled = true;
            }
        } else {
            stall_timer = 0;
        }

        if (is_stalled) {
            Serial.println("\n>>> 🚨 SAFETY SHUTDOWN: Motor Stall / Overload Detected! <<<");
            Serial.print  (">>> Target: "); Serial.print(m1_target_rpm, 1);
            Serial.print  (" RPM | Actual: "); Serial.print(actual_rpm, 1);
            Serial.print  (" RPM | Applied Uq: "); Serial.print(applied_u, 2); Serial.println(" V");
            Serial.println(">>> Driver disabled for safety. Click Enable (e) to re-enable.");
            is_enabled = false;
            driver1.disable();
            m1_target_rpm = 0.0f;
            is_stalled = false;
            stall_timer = 0;
        }
    }

    // 3. Process Real-Time Serial Commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        switch (cmd) {
            case '0':
                m1_target_rpm = 0.0f;
                Serial.println("Target: 0 RPM");
                break;
            case '1':
                m1_target_rpm = 50.0f;
                Serial.println("Target: 50 RPM");
                break;
            case '2':
                m1_target_rpm = 100.0f;
                Serial.println("Target: 100 RPM");
                break;
            case '3':
                m1_target_rpm = 200.0f;
                Serial.println("Target: 200 RPM");
                break;
            case '4':
                m1_target_rpm = 500.0f;
                Serial.println("Target: 500 RPM");
                break;

            case '+':
            case '=':
                m1_target_rpm += 10.0f;
                Serial.print("Target: "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
                break;
            case '-':
            case '_':
                m1_target_rpm -= 10.0f;
                Serial.print("Target: "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
                break;

            case 'r':
            case 'R': {
                motor1.voltage_limit = 12.0f;
                motor1.PID_velocity.limit = 12.0f;
                motor1.PID_velocity.P = 0.12f;
                motor1.PID_velocity.I = 1.20f;
                motor1.PID_velocity.D = 0.001f;
                motor1.LPF_velocity.Tf = 0.025f;
                m1_target_rpm = 0.0f;
                Serial.println("\n>>> [RESET] Restored GM3506 Optimized Defaults (12V Supply):");
                Serial.println("  Voltage Limit : 12.0 V");
                Serial.println("  Velocity P    : 0.1200");
                Serial.println("  Velocity I    : 1.2000");
                Serial.println("  Velocity D    : 0.0010");
                Serial.println("  LPF Tf        : 0.0250 s");
                Serial.println("  Target Speed  : 0.0 RPM\n");
                break;
            }

            case 'w':
            case 'W':
                encoder1.setMode(!encoder1.use_3wire);
                Serial.print(">>> Switched SPI Bus Mode to: ");
                Serial.println(encoder1.use_3wire ? "3-Wire Half-Duplex (PA7 SDAT)" : "4-Wire Full Hardware SPI1");
                break;

            case 'k':
            case 'K':
                stall_watchdog_enabled = !stall_watchdog_enabled;
                Serial.print(">>> Stall Watchdog: ");
                Serial.println(stall_watchdog_enabled ? "ENABLED" : "DISABLED");
                break;

            case 'e':
            case 'E':
                is_enabled = !is_enabled;
                if (is_enabled) {
                    driver1.enable();
                    Serial.println(">>> Driver ENABLED.");
                } else {
                    driver1.disable();
                    Serial.println(">>> Driver DISABLED.");
                }
                break;

            case 'x':
            case 'X':
            case ' ':
                m1_target_rpm = 0.0f;
                is_enabled = false;
                driver1.disable();
                Serial.println(">>> MOTOR STOPPED & DISABLED.");
                break;

            case 'c':
            case 'C': {
                Serial.println("\n==================================================");
                Serial.println(">>> STARTING SIMPLEFOC ELECTRICAL ALIGNMENT <<<");
                Serial.println("==================================================");
                driver1.enable();
                motor1.controller = MotionControlType::velocity;
                control_mode = MotionControlType::velocity;

                if (motor1.initFOC()) {
                    is_calibrated = true;
                    is_enabled = true;
                    // Target speed stays at 0 RPM after calibration
                    m1_target_rpm = 0.0f;
                    Serial.println("\n✅ [SUCCESS] FOC Calibration Complete! Closed-Loop Ready at 0 RPM.");
                    Serial.print  ("  Zero Electric Angle: "); Serial.print(motor1.zero_electric_angle, 4); Serial.println(" rad");
                    Serial.print  ("  Sensor Direction   : "); Serial.println(motor1.sensor_direction == Direction::CW ? "CW" : "CCW");
                    Serial.print  ("  Pole Pairs Check   : "); Serial.println(motor1.pole_pairs);
                } else {
                    is_calibrated = false;
                    is_enabled = false;
                    driver1.disable();
                    Serial.println("\n❌ [FAILED] FOC Alignment FAILED! Motor disabled for safety.");
                }
                Serial.println("==================================================\n");
                break;
            }

            case 'o':
            case 'O':
                Serial.println("\n>>> Activating OPEN-LOOP Velocity Mode (No sensor required)...");
                driver1.enable();
                motor1.controller = MotionControlType::velocity_openloop;
                control_mode = MotionControlType::velocity_openloop;
                is_enabled = true;
                if (m1_target_rpm == 0.0f) m1_target_rpm = 30.0f;
                break;

            case 'g':
            case 'G':
                show_telemetry = !show_telemetry;
                Serial.print(">>> Telemetry stream: "); Serial.println(show_telemetry ? "ON" : "OFF");
                break;

            // Live Parameter Tuning (Prefix + Float Parsing)
            case 'p':
            case 'P': {
                float val = Serial.parseFloat();
                if (val >= 0.0f) {
                    motor1.PID_velocity.P = val;
                    Serial.print(">>> Updated Velocity P: "); Serial.println(motor1.PID_velocity.P, 4);
                }
                break;
            }
            case 'i':
            case 'I': {
                float val = Serial.parseFloat();
                if (val >= 0.0f) {
                    motor1.PID_velocity.I = val;
                    Serial.print(">>> Updated Velocity I: "); Serial.println(motor1.PID_velocity.I, 4);
                }
                break;
            }
            case 'd':
            case 'D': {
                float val = Serial.parseFloat();
                if (val >= 0.0f) {
                    motor1.PID_velocity.D = val;
                    Serial.print(">>> Updated Velocity D: "); Serial.println(motor1.PID_velocity.D, 5);
                }
                break;
            }
            case 'f':
            case 'F': {
                float val = Serial.parseFloat();
                if (val > 0.0f) {
                    motor1.LPF_velocity.Tf = val;
                    Serial.print(">>> Updated Velocity LPF Tf: "); Serial.println(motor1.LPF_velocity.Tf, 4);
                }
                break;
            }
            case 'v':
            case 'V': {
                float val = Serial.parseFloat();
                if (val > 0.0f) {
                    motor1.voltage_limit = val;
                    motor1.PID_velocity.limit = val;
                    Serial.print(">>> Updated Voltage Limit: "); Serial.print(motor1.voltage_limit, 2); Serial.println(" V");
                }
                break;
            }
            case 't':
            case 'T': {
                float val = Serial.parseFloat();
                m1_target_rpm = val;
                Serial.print(">>> Updated Target Speed: "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
                break;
            }

            case 'z':
            case 'Z':
                encoder1.zeroCurrentPosition();
                Serial.println(">>> ZERO / TARE ANGLE SET! <<<");
                break;

            case 'h':
            case 'H':
            case '?':
                printHelp();
                break;

            default:
                break;
        }
    }

    // 4. Periodic Status & Comprehensive Debug Telemetry (20 Hz)
    uint32_t now = millis();
    if (show_telemetry && (now - last_telemetry_time >= 50)) {
        last_telemetry_time = now;

        float angle_deg = encoder1.getSingleTurnDeg();
        float actual_rpm = encoder1.getVelocity() * 9.54929659f;
        float uq = motor1.voltage.q;
        float ud = motor1.voltage.d;

        if (pwm_period_us > 800 && pwm_period_us < 1200) {
            float pwm_duty = (float)pwm_high_us / (float)pwm_period_us;
            float pwm_angle = pwm_duty * 360.0f;
            float pwm_freq = 1000000.0f / (float)pwm_period_us;
            Serial.print("[PWM@PA0]: ");
            if (pwm_angle < 100.0f) Serial.print(" ");
            if (pwm_angle < 10.0f) Serial.print(" ");
            Serial.print(pwm_angle, 1);
            Serial.print(" deg (");
            Serial.print(pwm_duty * 100.0f, 1);
            Serial.print("% @ ");
            Serial.print((int)pwm_freq);
            Serial.print("Hz) [OK] || ");
        }

        Serial.print("Ctrl: ");
        if (!is_enabled) Serial.print("DISBL ");
        else if (control_mode == MotionControlType::velocity_openloop) Serial.print("OPEN  ");
        else Serial.print("CLOSE ");

        Serial.print("| Target: ");
        if (m1_target_rpm >= 0.0f) Serial.print("+");
        Serial.print(m1_target_rpm, 1);
        Serial.print(" RPM | Actual: ");
        if (actual_rpm >= 0.0f) Serial.print("+");
        Serial.print(actual_rpm, 1);
        Serial.print(" RPM | Angle: ");
        if (angle_deg < 100.0f) Serial.print(" ");
        if (angle_deg < 10.0f) Serial.print(" ");
        Serial.print(angle_deg, 2);
        Serial.print(" deg | Uq: ");
        if (uq >= 0.0f) Serial.print("+");
        Serial.print(uq, 2);
        Serial.print("V | Ud: ");
        if (ud >= 0.0f) Serial.print("+");
        Serial.print(ud, 2);
        Serial.print("V | Raw: ");
        if (encoder1.last_raw_14bit < 10000) Serial.print(" ");
        if (encoder1.last_raw_14bit < 1000) Serial.print(" ");
        if (encoder1.last_raw_14bit < 100) Serial.print(" ");
        Serial.print(encoder1.last_raw_14bit);
        Serial.print("/16383 | Mode: ");
        Serial.print(encoder1.use_3wire ? "3-Wire" : "4-Wire");
        Serial.print(" | Mag: ");
        if (encoder1.last_raw_h == 0xFF && encoder1.last_raw_l == 0xFF) {
            Serial.print("[ NO SENSOR ]");
        } else if (encoder1.last_no_mag) {
            Serial.print("[ WEAK/NO MAG ]");
        } else {
            Serial.print("[ OK ]");
        }
        Serial.print(" | Parity: ");
        Serial.print(encoder1.last_parity_ok ? "[PASS]" : "[FAIL]");

        printVisualDial(angle_deg);
        Serial.println();
    }

    // 5. Heartbeat LED
    if (now - last_led_time >= 500) {
        last_led_time = now;
        led_state = !led_state;
        digitalWrite(STATUS_LED, led_state ? LOW : HIGH);
    }
}
