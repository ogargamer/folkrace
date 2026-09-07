#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

// ============================================================================
// DA WINNA — MT6816 / MT6818 3-WIRE / 4-WIRE SPI & PWM BENCHMARK SUITE
// ============================================================================

// SPI1 Master Pins (Relaxed & Strict compatible)
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

// Motor 1 Inverter Pins (Relaxed: PA8, PA9, PA10, PB3)
#define M1_PWM_A     PA8    // TIM1_CH1
#define M1_PWM_B     PA9    // TIM1_CH2
#define M1_PWM_C     PA10   // TIM1_CH3
#define M1_EN        PB3    // Gate Driver Enable

// Status LED
#define STATUS_LED   PC13

#define MOTOR_POLE_PAIRS   11
#define MOTOR_VOLTAGE_SUPPLY 12.0f
#define MOTOR_TEST_VOLTAGE_LIMIT 3.0f

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
// MT6816 High-Performance Sensor Driver (SimpleFOC Compatible)
// ============================================================================
class MT6816Sensor : public Sensor {
public:
    int cs_pin;
    uint32_t spi_speed;
    uint8_t spi_mode;
    bool use_3wire;
    float cpr;

    // Diagnostics & Tracking
    uint32_t total_reads;
    uint32_t parity_errors;
    uint32_t mag_faults;
    bool last_no_mag;
    bool last_parity_ok;
    uint8_t last_raw_h;
    uint8_t last_raw_l;
    uint16_t last_raw_14bit;
    float zero_offset_deg;

    int32_t full_rotations;
    uint16_t prev_raw_angle;

    MT6816Sensor()
        : cs_pin(PIN_CS_PA3), spi_speed(4000000), spi_mode(SPI_MODE3), use_3wire(true), cpr(16384.0f),
          total_reads(0), parity_errors(0), mag_faults(0), last_no_mag(false),
          last_parity_ok(true), last_raw_h(0), last_raw_l(0), last_raw_14bit(0),
          zero_offset_deg(0.0f), full_rotations(0), prev_raw_angle(0) {
    }

    void init() {
        pinMode(PIN_CS_PA3, OUTPUT);  digitalWrite(PIN_CS_PA3, HIGH);
        pinMode(PIN_CS_PA15, OUTPUT); digitalWrite(PIN_CS_PA15, HIGH);
        pinMode(PIN_CS_PB1, OUTPUT);  digitalWrite(PIN_CS_PB1, HIGH);
        pinMode(PIN_CS_PB2, OUTPUT);  digitalWrite(PIN_CS_PB2, HIGH);
        pinMode(PIN_CS_PB3, OUTPUT);  digitalWrite(PIN_CS_PB3, HIGH);

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

        delay(10);
        this->Sensor::init();

        // Initial prime reading
        readCurrent(last_raw_h, last_raw_l, last_raw_14bit, last_no_mag, last_parity_ok);
        prev_raw_angle = last_raw_14bit;
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

    float getSensorAngle() override {
        uint8_t h, l;
        uint16_t raw;
        bool no_mag, parity_ok;
        readCurrent(h, l, raw, no_mag, parity_ok);

        // Parity Glitch Filter: Only commit new angle if parity is valid
        if (parity_ok || last_raw_14bit == 0) {
            last_raw_h = h;
            last_raw_l = l;
            last_raw_14bit = raw;
            last_no_mag = no_mag;
            last_parity_ok = true;

            int32_t delta = (int32_t)last_raw_14bit - (int32_t)prev_raw_angle;
            if (delta > 8192) full_rotations--;
            else if (delta < -8192) full_rotations++;
            prev_raw_angle = last_raw_14bit;
        } else {
            last_parity_ok = false;
        }

        return ((float)full_rotations * _2PI) + (((float)last_raw_14bit / cpr) * _2PI);
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

// Operational States
bool live_streaming_active = true;
enum TestMode {
    MODE_STREAM = 1,
    MODE_RAW_INSPECTOR,
    MODE_BENCHMARK,
    MODE_MOTOR_OPENLOOP,
    MODE_MOTOR_FOC
};

TestMode current_mode = MODE_STREAM;
uint32_t last_telemetry_time = 0;
uint32_t last_led_time = 0;
bool led_state = false;
float motor_openloop_target = 0.0f;
bool motor_enabled = false;

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

void printBanner() {
    Serial.println();
    Serial.println("================================================================================");
    Serial.println("   DA WINNA — MT6816 SPI ENCODER & SIMPLEFOC SUITE");
    Serial.println("================================================================================");
    Serial.print  ("  Active CS: "); Serial.print(getCSName(encoder1.cs_pin));
    Serial.print  (" | Bus Mode: "); Serial.print(encoder1.use_3wire ? "3-Wire Half-Duplex (PA7 SDAT)" : "4-Wire Full Hardware SPI1");
    Serial.print  (" | Clock: "); Serial.print(encoder1.spi_speed / 1000000.0f, 1);
    Serial.print  (" MHz | SPI Mode: "); Serial.println(encoder1.spi_mode == SPI_MODE3 ? "Mode 3" : "Mode 0");
    Serial.println("================================================================================");
    Serial.println("  [ACTIVE COMMANDS]:");
    Serial.println("    [1] Toggle Live Streaming (Start / Pause)");
    Serial.println("    [2] Raw Register Inspector (0x03 & 0x04 bit breakdown)");
    Serial.println("    [3] 10,000 Sample High-Speed SPI Benchmark & Jitter Test");
    Serial.println("    [4] Motor 1 Open-Loop Spin Test (+ / - speed, Space stop)");
    Serial.println("    [5] Motor 1 Closed-Loop SimpleFOC Angle Tracking");
    Serial.println("    [t] Toggle 3-Wire vs 4-Wire SPI Mode");
    Serial.println("    [c] Cycle CS Pin (PA3 -> PA15 -> PB1 -> PB2 -> PB3)");
    Serial.println("    [s] Cycle Clock Speed (1MHz -> 2MHz -> 4MHz -> 8MHz -> 16MHz)");
    Serial.println("    [p] Toggle SPI Mode (Mode 3 vs Mode 0)");
    Serial.println("    [z] Tare / Zero Angle Offset");
    Serial.println("================================================================================");
}

void runLiveStream() {
    if (!live_streaming_active) return;

    uint32_t now = millis();
    if (now - last_telemetry_time >= 50) { // 20 Hz Telemetry
        last_telemetry_time = now;

        encoder1.update();
        float angle_deg = encoder1.getSingleTurnDeg();
        float angle_rad = encoder1.getAngle();
        float velocity_rpm = encoder1.getVelocity() * 9.54929659f;

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
            Serial.print("Hz) [OK]  ");
            printVisualDial(pwm_angle);
            Serial.print(" || ");
        }

        Serial.print("[SPI@");
        Serial.print(getCSName(encoder1.cs_pin));
        Serial.print("]: ");
        if (angle_deg < 100.0f) Serial.print(" ");
        if (angle_deg < 10.0f) Serial.print(" ");
        Serial.print(angle_deg, 2);
        Serial.print(" deg (");
        Serial.print(angle_rad, 3);
        Serial.print(" rad) | Raw: ");
        if (encoder1.last_raw_14bit < 10000) Serial.print(" ");
        if (encoder1.last_raw_14bit < 1000) Serial.print(" ");
        if (encoder1.last_raw_14bit < 100) Serial.print(" ");
        Serial.print(encoder1.last_raw_14bit);
        Serial.print("/16383 | Spd: ");
        if (velocity_rpm >= 0.0f) Serial.print("+");
        Serial.print(velocity_rpm, 1);
        Serial.print(" RPM | Reg: 0x");
        if (encoder1.last_raw_h < 16) Serial.print("0");
        Serial.print(encoder1.last_raw_h, HEX);
        Serial.print(" 0x");
        if (encoder1.last_raw_l < 16) Serial.print("0");
        Serial.print(encoder1.last_raw_l, HEX);
        Serial.print(" | Mode: ");
        Serial.print(encoder1.use_3wire ? "3-Wire" : "4-Wire");
        Serial.print(" | Mag: ");
        if (encoder1.last_raw_h == 0xFF && encoder1.last_raw_l == 0xFF) {
            Serial.print("[ NO SENSOR / 0xFF ]");
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
}

void runRawInspector() {
    uint8_t h = 0, l = 0;
    uint16_t raw14 = 0;
    bool no_mag = false, parity_ok = false;

    encoder1.readCurrent(h, l, raw14, no_mag, parity_ok);

    Serial.println("\n------------------------------------------------------------");
    Serial.println("  MT6816 RAW REGISTER INSPECTOR");
    Serial.print  ("  Active CS: "); Serial.print(getCSName(encoder1.cs_pin));
    Serial.print  (" | Bus Mode: "); Serial.print(encoder1.use_3wire ? "3-Wire (SDAT)" : "4-Wire HW SPI");
    Serial.print  (" @ "); Serial.print(encoder1.spi_speed / 1000000.0f, 1); Serial.println(" MHz");
    Serial.println("------------------------------------------------------------");

    Serial.print("  Reg 0x03 (Angle High Byte): 0x");
    if (h < 16) Serial.print("0"); Serial.print(h, HEX);
    Serial.print("  [0b");
    for (int b = 7; b >= 0; b--) Serial.print((h >> b) & 1);
    Serial.println("]");

    Serial.print("  Reg 0x04 (Angle Low Byte) : 0x");
    if (l < 16) Serial.print("0"); Serial.print(l, HEX);
    Serial.print("  [0b");
    for (int b = 7; b >= 0; b--) Serial.print((l >> b) & 1);
    Serial.println("]");

    Serial.print("    |- Angle<5:0>   : 0b");
    for (int b = 7; b >= 2; b--) Serial.print((l >> b) & 1);
    Serial.println();

    Serial.print("    |- No_Mag_Warn  : Bit 1 = ");
    Serial.print((l >> 1) & 1);
    Serial.println(no_mag ? " -> [!! NO MAGNET DETECTED !!]" : " -> [MAGNET OK]");

    Serial.print("    |- Parity Bit   : Bit 0 = ");
    Serial.print(l & 1);
    Serial.println(parity_ok ? " -> [PARITY PASS (Even)]" : " -> [!! PARITY ERROR !!]");

    Serial.print("  Combined 14-Bit Raw Value : ");
    Serial.print(raw14);
    Serial.print(" / 16383 (");
    Serial.print((raw14 / 16384.0f) * 360.0f, 2);
    Serial.println(" deg)");
    Serial.println("------------------------------------------------------------\n");
}

void runBenchmark() {
    Serial.println("\n------------------------------------------------------------");
    Serial.println("  HIGH-SPEED 10,000 SAMPLE SPI BENCHMARK");
    Serial.print  ("  Sampling 10,000 back-to-back angle reads in ");
    Serial.print  (encoder1.use_3wire ? "3-Wire Mode" : "4-Wire Mode");
    Serial.println("...");
    Serial.println("------------------------------------------------------------");

    const int SAMPLES = 10000;
    uint8_t h, l;
    uint16_t raw;
    bool no_mag, parity_ok;
    uint32_t errors = 0;

    uint32_t t0 = micros();
    for (int i = 0; i < SAMPLES; i++) {
        encoder1.readCurrent(h, l, raw, no_mag, parity_ok);
        if (!parity_ok) errors++;
    }
    uint32_t dt = micros() - t0;

    float avg_us = (float)dt / SAMPLES;
    float rate_khz = 1000.0f / avg_us;

    Serial.print("  Total Time for 10,000 Samples : "); Serial.print(dt / 1000.0f, 2); Serial.println(" ms");
    Serial.print("  Average Read Latency          : "); Serial.print(avg_us, 2); Serial.println(" us / sample");
    Serial.print("  Max Theoretical Sampling Rate : "); Serial.print(rate_khz, 1); Serial.println(" kHz (samples/sec)");
    Serial.print("  Parity Error Rate             : "); Serial.print(errors);
    Serial.print(" / "); Serial.print(SAMPLES); Serial.print(" (");
    Serial.print(((float)errors / SAMPLES) * 100.0f, 4); Serial.println("%)");
    Serial.println("------------------------------------------------------------\n");
}

void setupMotor() {
    Serial.print("Initializing Motor 1 (Phase: PA8, PA9, PA10 | EN: PB3)... ");
    driver1.voltage_power_supply = MOTOR_VOLTAGE_SUPPLY;
    driver1.voltage_limit = MOTOR_TEST_VOLTAGE_LIMIT;
    driver1.init();

    motor1.linkDriver(&driver1);
    motor1.voltage_limit = MOTOR_TEST_VOLTAGE_LIMIT;
    motor1.controller = MotionControlType::velocity_openloop;
    motor1.init();

    motor_enabled = true;
    Serial.println("Done.");
}

void runMotorOpenLoop() {
    if (!motor_enabled) setupMotor();
    motor1.move(motor_openloop_target);

    uint32_t now = millis();
    if (now - last_telemetry_time >= 100) {
        last_telemetry_time = now;
        encoder1.update();

        float angle_deg = encoder1.getSingleTurnDeg();
        float target_rpm = motor_openloop_target * 9.54929659f;

        Serial.print("[MOTOR 1 OPEN-LOOP] Target: ");
        Serial.print(target_rpm, 1);
        Serial.print(" RPM | Enc Angle: ");
        Serial.print(angle_deg, 1);
        Serial.print(" deg | Raw: ");
        Serial.print(encoder1.last_raw_14bit);
        printVisualDial(angle_deg);
        Serial.println("  [Controls: + / - / Space]");
    }
}

void setup() {
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH);

    pinMode(PIN_PWM_IN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_PWM_IN), pwmPinInterrupt, CHANGE);

    Serial.begin(921600);
    uint32_t start_time = millis();
    while (!Serial && (millis() - start_time < 3000)) {
        digitalWrite(STATUS_LED, (millis() % 200 < 100) ? LOW : HIGH);
    }
    digitalWrite(STATUS_LED, HIGH);

    encoder1.init();
    printBanner();

    last_telemetry_time = millis();
    last_led_time = millis();
}

void loop() {
    // 1. Process User Interactive Keyboard Commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        switch (cmd) {
            case '1':
                live_streaming_active = !live_streaming_active;
                current_mode = MODE_STREAM;
                if (live_streaming_active) {
                    Serial.println("\n>>> [1] LIVE STREAMING RESUMED <<<");
                } else {
                    Serial.println("\n>>> [1] LIVE STREAMING PAUSED (Press 1 or Click Stream to Resume) <<<");
                }
                break;
            case '2':
                current_mode = MODE_RAW_INSPECTOR;
                runRawInspector();
                current_mode = MODE_STREAM;
                break;
            case '3':
                current_mode = MODE_BENCHMARK;
                runBenchmark();
                current_mode = MODE_STREAM;
                break;
            case '4':
                current_mode = MODE_MOTOR_OPENLOOP;
                motor_openloop_target = 3.0f;
                Serial.println("\n>>> [4] MOTOR 1 OPEN-LOOP ROTATION (3.0 rad/s) <<<");
                Serial.println("  Controls: [+] Fast FWD, [-] Reverse, [Space] Stop");
                break;
            case '5':
                Serial.println("\n>>> [5] MOTOR 1 SIMPLEFOC CLOSED-LOOP ALIGNMENT <<<");
                if (!motor_enabled) setupMotor();
                motor1.linkSensor(&encoder1);
                motor1.controller = MotionControlType::velocity;
                motor1.initFOC();
                Serial.println(">>> SimpleFOC Alignment Complete! Ready for Closed-Loop. <<<");
                current_mode = MODE_STREAM;
                break;
            case 't':
            case 'T':
                encoder1.use_3wire = !encoder1.use_3wire;
                if (!encoder1.use_3wire) {
                    SPI.setSCLK(PIN_SCK);
                    SPI.setMISO(PIN_MISO);
                    SPI.setMOSI(PIN_MOSI);
                    SPI.begin();
                } else {
                    SPI.end();
                    pinMode(PIN_SCK, OUTPUT);
                    digitalWrite(PIN_SCK, HIGH);
                    pinMode(PIN_MOSI, INPUT_PULLUP);
                }
                Serial.print(">>> Bus Mode Switched to: ");
                Serial.println(encoder1.use_3wire ? "3-Wire Half-Duplex (PA7 SDAT)" : "4-Wire Full Hardware SPI1");
                break;
            case 'c':
            case 'C': {
                if (encoder1.cs_pin == PIN_CS_PA3) encoder1.cs_pin = PIN_CS_PA15;
                else if (encoder1.cs_pin == PIN_CS_PA15) encoder1.cs_pin = PIN_CS_PB1;
                else if (encoder1.cs_pin == PIN_CS_PB1) encoder1.cs_pin = PIN_CS_PB2;
                else if (encoder1.cs_pin == PIN_CS_PB2) encoder1.cs_pin = PIN_CS_PB3;
                else encoder1.cs_pin = PIN_CS_PA3;
                Serial.print(">>> Active CS Switched to: ");
                Serial.println(getCSName(encoder1.cs_pin));
                break;
            }
            case 's':
            case 'S': {
                if (encoder1.spi_speed == 1000000) encoder1.spi_speed = 2000000;
                else if (encoder1.spi_speed == 2000000) encoder1.spi_speed = 4000000;
                else if (encoder1.spi_speed == 4000000) encoder1.spi_speed = 8000000;
                else if (encoder1.spi_speed == 8000000) encoder1.spi_speed = 16000000;
                else encoder1.spi_speed = 1000000;
                Serial.print(">>> SPI Clock Speed set to: ");
                Serial.print(encoder1.spi_speed / 1000000.0f, 1);
                Serial.println(" MHz <<<");
                break;
            }
            case 'p':
            case 'P': {
                if (encoder1.spi_mode == SPI_MODE3) encoder1.spi_mode = SPI_MODE0;
                else encoder1.spi_mode = SPI_MODE3;
                Serial.print(">>> SPI Mode set to: ");
                Serial.println(encoder1.spi_mode == SPI_MODE3 ? "SPI_MODE3" : "SPI_MODE0");
                break;
            }
            case '+':
            case '=':
                if (current_mode == MODE_MOTOR_OPENLOOP) {
                    motor_openloop_target += 2.0f;
                    Serial.print(">> Speed: "); Serial.print(motor_openloop_target * 9.54929659f, 1); Serial.println(" RPM");
                }
                break;
            case '-':
            case '_':
                if (current_mode == MODE_MOTOR_OPENLOOP) {
                    motor_openloop_target -= 2.0f;
                    Serial.print(">> Speed: "); Serial.print(motor_openloop_target * 9.54929659f, 1); Serial.println(" RPM");
                }
                break;
            case ' ':
                if (current_mode == MODE_MOTOR_OPENLOOP) {
                    motor_openloop_target = 0.0f;
                    if (motor_enabled) motor1.move(0.0f);
                    Serial.println(">> MOTOR STOPPED <<");
                } else {
                    live_streaming_active = !live_streaming_active;
                    if (live_streaming_active) Serial.println("\n>> STREAM RESUMED <<");
                    else Serial.println("\n>> STREAM PAUSED <<");
                }
                break;
            case 'z':
            case 'Z':
                encoder1.zeroCurrentPosition();
                Serial.println(">>> ZERO / TARE OFFSET SET! <<<");
                break;
            case 'm':
            case 'M':
            case '?':
                printBanner();
                break;
            default:
                break;
        }
    }

    // 2. Dispatch Active Modes
    switch (current_mode) {
        case MODE_STREAM:
            runLiveStream();
            break;
        case MODE_MOTOR_OPENLOOP:
            runMotorOpenLoop();
            break;
        default:
            break;
    }

    // 3. Heartbeat LED
    uint32_t now = millis();
    if (now - last_led_time >= 500) {
        last_led_time = now;
        led_state = !led_state;
        digitalWrite(STATUS_LED, led_state ? LOW : HIGH);
    }
}
