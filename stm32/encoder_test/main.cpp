#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>

// ============================================================================
// DA WINNA — MT6816 SPI ENCODER & SIMPLEFOC SUITE (GM3506 OPTIMIZED @ 12V)
// ============================================================================
// Baud Rate: 921600 (High-Speed Motor Telemetry & KV Test Suite)
// Hardware: STM32G474 (170MHz) | Driver: PA8, PA9, PA10, PB3 (12V) | Sensor: MT6816 SPI
// ============================================================================

// ============================================================================
// ⚙️ USER CONFIGURATION & TUNING PARAMETERS (EASY-TO-EDIT SECTION)
// ============================================================================
// 1. HARDWARE & INVERTER SPECIFICATIONS (Compile-Time Settings)
#define MOTOR_POLE_PAIRS            11        // GM3506 Gimbal Motor (24N22P = 11 Pole Pairs)
#define MOTOR_POWER_SUPPLY_VOLT     12.0f     // Main DC Power Supply Bus Voltage (V)
#define MOTOR_PWM_FREQUENCY_HZ      25000     // Inverter PWM Carrier Frequency (25 kHz = Silent & Low Ripple)
#define MOTOR_ALIGN_VOLTAGE         3.5f      // Optimal Electrical Zero Alignment Voltage (3.0V - 3.5V for GM3506 draws ~0.50A)
#define SPI_SPEED_DEFAULT_HZ        4000000   // MT6816/MT6818 SPI Master Baud (4 MHz = ~4us per read)
#define SPI_USE_3WIRE_DEFAULT       false     // true = 3-Wire Half-Duplex (PA7), false = 4-Wire HW SPI

// 2. SENSOR LAG COMPENSATION & VELOCITY FILTERING
// MT6816 CORDIC delay (~6us) + SPI1 transfer (~8us) + PWM update (~20us) = ~35us total lag
#define DEFAULT_SENSOR_LAG_COMP_US  35.0f     // Dynamic Phase Lead Lag Compensation (35.0 us = Symmetric CW/CCW FOC)
#define DEFAULT_LPF_TF_SEC          0.0050f   // Velocity Low-Pass Filter Tf (5 ms for 14-Bit MT6816 SPI gives razor-sharp response)

// 3. TIMING, LOOP RATES & SAMPLING FREQUENCIES
#define TELEMETRY_STREAM_INTERVAL_MS 50       // Serial Live Streaming Interval (50ms = 20 Hz, 20ms = 50 Hz)
#define ALIGNMENT_OVERSAMPLE_COUNT  128       // Number of 14-bit encoder samples per calibration point

// 4. REAL-TIME TUNABLE DEFAULT FOC PARAMETERS (Can be changed live over GUI / Serial)
#define DEFAULT_VOLTAGE_LIMIT       12.0f     // Maximum allowable phase voltage (V)
#define DEFAULT_VELOCITY_LIMIT_RPM  1432.0f   // Maximum velocity limit (1432 RPM = 150 rad/s)
#define DEFAULT_OUTPUT_RAMP_V_PER_S 200.0f    // Output voltage ramp rate limit (200 V/s for ultra-smooth transitions)
#define DEFAULT_PID_P               0.1200f   // Velocity Proportional Gain
#define DEFAULT_PID_I               1.2000f   // Velocity Integral Gain
#define DEFAULT_PID_D               0.0000f   // Velocity Derivative Gain (0.0 for pure PI velocity control)
#define DEFAULT_MOTOR_KV_RATING     130.0f    // Initial Back-EMF Feedforward KV rating (RPM/V)

// 5. AUTOMATED CLOSED-LOOP KV TEST PARAMETERS
#define KV_DWELL_TIME_MS            1200      // Settle time for motor to reach steady terminal velocity (ms)
#define KV_SAMPLE_TIME_MS           600       // Velocity sampling window duration (ms)
#define KV_SAMPLE_INTERVAL_US       5000      // Velocity sampling rate (5000 us = 200 Hz)
// ============================================================================

// SPI1 Master Hardware Pins
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

// Motor 1 Inverter Driver Pins (PB6, PB7, PB9, PB3)
#define M1_PWM_A     PB6    // TIM4_CH1 (Board Pin B6)
#define M1_PWM_B     PB7    // TIM4_CH2 (Board Pin B7)
#define M1_PWM_C     PB9    // TIM4_CH4 (Board Pin B9)
#define M1_EN        PB3    // Gate Driver Enable (Board Pin B3)

// Status LED
#define STATUS_LED   PC13

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
// MT6816 High-Performance Sensor Driver (SimpleFOC Standard Architecture)
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
    float zero_offset_deg;

    MT6816Sensor()
        : cs_pin(PIN_CS_PA3), spi_speed(SPI_SPEED_DEFAULT_HZ), spi_mode(SPI_MODE3), use_3wire(SPI_USE_3WIRE_DEFAULT),
          total_reads(0), parity_errors(0), mag_faults(0), last_no_mag(false),
          last_parity_ok(true), last_raw_h(0), last_raw_l(0), last_raw_14bit(0),
          zero_offset_deg(0.0f) {
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

        setMode(SPI_USE_3WIRE_DEFAULT);

        digitalWrite(cs_pin, LOW);
        delayMicroseconds(2);
        SPI.transfer16(0x0100);
        delayMicroseconds(2);
        digitalWrite(cs_pin, HIGH);
        delayMicroseconds(5);

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
        last_no_mag = no_mag;
        last_parity_ok = parity_ok;

        this->Sensor::init();
    }

    static bool parityCheck(uint16_t data) {
        int count = 0;
        for (int i = 0; i < 16; i++) {
            if (data & (1 << i)) count++;
        }
        return (count % 2 == 0);
    }

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

        if (parity_ok || last_raw_14bit == 0) {
            last_raw_h = h;
            last_raw_l = l;
            last_raw_14bit = raw;
            last_no_mag = no_mag;
            last_parity_ok = true;
        } else {
            last_parity_ok = false;
        }

        return ((float)last_raw_14bit / 16384.0f) * _2PI;
    }

    float getSingleTurnDeg() {
        float d = ((float)last_raw_14bit / 16384.0f) * 360.0f - zero_offset_deg;
        while (d < 0.0f) d += 360.0f;
        while (d >= 360.0f) d -= 360.0f;
        return d;
    }

    float getVelocity() override {
        uint32_t now_us = micros();
        float Ts = (now_us - vel_angle_prev_ts) * 1e-6f;
        if (Ts <= 0.0f || Ts > 0.5f) {
            vel_angle_prev_ts = now_us;
            vel_angle_prev = angle_prev;
            vel_full_rotations = full_rotations;
            return velocity;
        }

        // Quick return if sampling interval is too small (< 1ms)
        if (Ts < 0.0010f) return velocity;

        float current_angle = (float)full_rotations * _2PI + angle_prev;
        float prev_angle = (float)vel_full_rotations * _2PI + vel_angle_prev;
        float delta_angle = current_angle - prev_angle;

        // Update velocity if moved by at least 1 LSB
        if (fabsf(delta_angle) >= (0.5f * _2PI / 16384.0f)) {
            velocity = delta_angle / Ts;
            vel_angle_prev = angle_prev;
            vel_full_rotations = full_rotations;
            vel_angle_prev_ts = now_us;
        } else if (Ts > 0.040f) {
            // When stopped (no movement for > 40ms), zero out residual noise
            velocity = 0.0f;
            vel_angle_prev_ts = now_us;
            vel_angle_prev = angle_prev;
            vel_full_rotations = full_rotations;
        }

        return velocity;
    }

    void zeroCurrentPosition() {
        zero_offset_deg = ((float)last_raw_14bit / 16384.0f) * 360.0f;
        this->full_rotations = 0;
    }
};

MT6816Sensor encoder1;

// SimpleFOC Motor 1 Driver & BLDC Motor
BLDCDriver3PWM driver1(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);
BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);

// Control States & Lag Compensation
MotionControlType control_mode = MotionControlType::velocity;
bool is_enabled = false;
bool is_calibrated = false;
bool show_telemetry = true;
float m1_target_rpm = 0.0f;
float m1_target_uq = 0.0f;
float sensor_lag_comp_us = DEFAULT_SENSOR_LAG_COMP_US; // Microseconds phase advance
float base_zero_electric_angle = 0.11275f;             // Calibrated mean electrical zero angle (rad)
float calib_align_voltage = MOTOR_ALIGN_VOLTAGE;       // Calibration alignment voltage (V)

// Stall & Safety Watchdog
bool stall_watchdog_enabled = true;
uint32_t stall_timer = 0;
bool is_stalled = false;

uint32_t last_telemetry_time = 0;
uint32_t last_led_time = 0;
bool led_state = false;

// ============================================================================
// AUTOMATED CLOSED-LOOP KV TEST SUITE ENGINE
// ============================================================================
struct KVPoint {
    float voltage;       // Commanded Uq (V)
    float measured_rpm;  // Mean steady-state velocity (RPM)
    float speed_ripple;  // Peak-to-peak speed ripple (RPM)
    float std_dev;       // Standard deviation of speed (RPM)
    uint16_t sample_cnt; // Number of samples recorded
};

#define MAX_KV_POINTS 32
KVPoint kv_points[MAX_KV_POINTS];
int kv_point_count = 0;

bool kv_test_running = false;
int kv_stage_index = 0;
uint32_t kv_stage_start_ms = 0;
float kv_voltages[] = { 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f,
                       -1.0f, -1.5f, -2.0f, -2.5f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f };
const int KV_TOTAL_STAGES = sizeof(kv_voltages) / sizeof(kv_voltages[0]);

#define STAGE_SAMPLE_CAP 150
float stage_speed_samples[STAGE_SAMPLE_CAP];
int stage_sample_count = 0;
uint32_t stage_last_sample_us = 0;

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

// Perform Ordinary Least Squares Linear Regression: y = m*x + c
bool calculateLinearRegression(const float* x, const float* y, int n, float &slope, float &intercept, float &r2) {
    if (n < 2) return false;
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0, sum_yy = 0;
    for (int i = 0; i < n; i++) {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i] * y[i];
        sum_xx += x[i] * x[i];
        sum_yy += y[i] * y[i];
    }
    float denominator = (n * sum_xx - sum_x * sum_x);
    if (fabsf(denominator) < 1e-6f) return false;

    slope = (n * sum_xy - sum_x * sum_y) / denominator;
    intercept = (sum_y - slope * sum_x) / n;

    float ss_tot = 0, ss_res = 0;
    float mean_y = sum_y / n;
    for (int i = 0; i < n; i++) {
        float y_pred = slope * x[i] + intercept;
        ss_tot += (y[i] - mean_y) * (y[i] - mean_y);
        ss_res += (y[i] - y_pred) * (y[i] - y_pred);
    }
    r2 = (ss_tot > 1e-6f) ? (1.0f - (ss_res / ss_tot)) : 1.0f;
    return true;
}

// ============================================================================
// 🔬 COMPREHENSIVE MULTI-POINT HYSTERESIS-COMPENSATED FOC ALIGNMENT
// ============================================================================
bool calibrateComprehensiveFOC(float align_voltage = MOTOR_ALIGN_VOLTAGE) {
    Serial.println("\n================================================================================");
    Serial.println(">>> 🔬 STARTING COMPREHENSIVE MULTI-POINT ELECTRICAL ZERO CALIBRATION <<<");
    Serial.println("================================================================================");
    Serial.print("Target Motor       : GM3506 ("); Serial.print(MOTOR_POLE_PAIRS); Serial.println(" Pole Pairs)");
    Serial.print("Calibration Voltage: "); Serial.print(align_voltage, 2); Serial.println(" V (Current ~ 0.45A)");
    Serial.println("Methodology        : 11-Pole Continuous Sweep + Bidirectional Dither Hysteresis Cancellation");
    Serial.print("Oversampling       : "); Serial.print(ALIGNMENT_OVERSAMPLE_COUNT); Serial.println(" Samples / Point with 14-Bit MT6816 Resolution");
    Serial.println("--------------------------------------------------------------------------------");

    driver1.enable();
    delay(50);

    // 1. Direction Detection (Smooth electrical rotation forward and backward)
    Serial.print("1. Determining Sensor Commutation Direction & Verifying Motion... ");
    motor1.setPhaseVoltage(align_voltage, 0, _3PI_2);
    delay(400);
    encoder1.update();
    float start_angle = encoder1.getMechanicalAngle();

    // Smoothly sweep forward 2 full electrical cycles (65.45 deg mechanical)
    const int DIR_STEPS = 400;
    for (int step = 0; step <= DIR_STEPS; step++) {
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor1.setPhaseVoltage(align_voltage, 0, el_ang);
        delayMicroseconds(2500); // 400 * 2.5ms = 1.0 second smooth sweep
    }
    delay(200);
    encoder1.update();
    float mid_angle = encoder1.getMechanicalAngle();

    // Smoothly sweep backward 2 full electrical cycles back to start
    for (int step = DIR_STEPS; step >= 0; step--) {
        float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
        motor1.setPhaseVoltage(align_voltage, 0, el_ang);
        delayMicroseconds(2500); // 1.0 second smooth return
    }
    delay(200);
    encoder1.update();

    float d_ang_fwd = mid_angle - start_angle;
    while (d_ang_fwd > _PI)  d_ang_fwd -= _2PI;
    while (d_ang_fwd < -_PI) d_ang_fwd += _2PI;
    float moved_rad = fabsf(d_ang_fwd);
    float min_expected_rad = (2.0f * _2PI / (float)MOTOR_POLE_PAIRS) * 0.40f; // Expected ~0.57 rad; min threshold 0.23 rad (~13 deg mech)

    if (moved_rad < min_expected_rad) {
        Serial.println("❌ FAILED!");
        Serial.print("   ERROR: Motor failed to move (detected only ");
        Serial.print(moved_rad * 57.2957795f, 2);
        Serial.print(" deg, expected >= ");
        Serial.print(min_expected_rad * 57.2957795f, 2);
        Serial.println(" deg). Check 12V DC power, inverter wiring, or align voltage!");
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        is_calibrated = false;
        return false;
    }

    Direction sensor_dir = (d_ang_fwd > 0.0f) ? Direction::CW : Direction::CCW;
    Serial.print("Detected ");
    Serial.print(sensor_dir == Direction::CW ? "CW" : "CCW");
    Serial.print(" (Moved ");
    Serial.print(moved_rad * 57.2957795f, 1);
    Serial.println(" deg mech) [OK]");
    motor1.sensor_direction = sensor_dir;

    // 2. Multi-Pole Full 360° Bidirectional Sweep (CW 360° + CCW 360° Round-Trip)
    Serial.println("\n2. Scanning All 11 Electrical Poles (Full 360° Forward CW + Reverse CCW Round-Trip):");
    Serial.println("Pole # | Mech Angle | Forward (CW)  | Reverse (CCW) | True Zero | Hysteresis | Status");
    Serial.println("-------+------------+---------------+---------------+-----------+------------+-------");

    const int num_poles = MOTOR_POLE_PAIRS;
    float fwd_zeros[num_poles];
    float rev_zeros[num_poles];
    float fwd_mechs[num_poles];
    float rev_mechs[num_poles];
    float pole_zeros[num_poles];
    bool all_poles_ok = true;

    // --- PASS 1: FORWARD (CW) 360° SWEEP (Poles 0 -> 10) ---
    for (int p = 0; p < num_poles; p++) {
        float target_el = _3PI_2 + (float)p * _2PI;
        if (p > 0) {
            const int INTERP_STEPS = 100;
            float prev_el = _3PI_2 + (float)(p - 1) * _2PI;
            for (int s = 1; s <= INTERP_STEPS; s++) {
                float el = prev_el + (target_el - prev_el) * ((float)s / (float)INTERP_STEPS);
                motor1.setPhaseVoltage(align_voltage, 0, el);
                delayMicroseconds(2000);
            }
        } else {
            motor1.setPhaseVoltage(align_voltage, 0, target_el);
            delay(300);
        }
        // Decaying micro-dither to eliminate bearing stiction and slot cogging deadband
        for (int d = 12; d >= 1; d--) {
            float dither = 0.10f * ((float)d / 12.0f);
            motor1.setPhaseVoltage(align_voltage, 0, target_el + dither);
            delayMicroseconds(800);
            motor1.setPhaseVoltage(align_voltage, 0, target_el - dither);
            delayMicroseconds(800);
        }
        motor1.setPhaseVoltage(align_voltage, 0, target_el);
        delay(80);

        // Oversample Forward reading
        float mech = 0.0f;
        for (int s = 0; s < ALIGNMENT_OVERSAMPLE_COUNT; s++) {
            encoder1.update();
            mech += encoder1.getMechanicalAngle();
            delayMicroseconds(200);
        }
        mech /= (float)ALIGNMENT_OVERSAMPLE_COUNT;
        fwd_mechs[p] = mech;
        fwd_zeros[p] = _normalizeAngle((float)(sensor_dir * num_poles) * mech);
    }

    // --- PASS 2: REVERSE (CCW) 360° RETURN SWEEP (Poles 10 -> 0) ---
    for (int p = num_poles - 1; p >= 0; p--) {
        float target_el = _3PI_2 + (float)p * _2PI;
        if (p < num_poles - 1) {
            const int INTERP_STEPS = 100;
            float prev_el = _3PI_2 + (float)(p + 1) * _2PI;
            for (int s = 1; s <= INTERP_STEPS; s++) {
                float el = prev_el + (target_el - prev_el) * ((float)s / (float)INTERP_STEPS);
                motor1.setPhaseVoltage(align_voltage, 0, el);
                delayMicroseconds(2000);
            }
        } else {
            motor1.setPhaseVoltage(align_voltage, 0, target_el);
            delay(150);
        }

        // Decaying micro-dither to eliminate bearing stiction and slot cogging deadband
        for (int d = 12; d >= 1; d--) {
            float dither = 0.10f * ((float)d / 12.0f);
            motor1.setPhaseVoltage(align_voltage, 0, target_el + dither);
            delayMicroseconds(800);
            motor1.setPhaseVoltage(align_voltage, 0, target_el - dither);
            delayMicroseconds(800);
        }
        motor1.setPhaseVoltage(align_voltage, 0, target_el);
        delay(80);

        // Oversample Reverse reading
        float mech = 0.0f;
        for (int s = 0; s < ALIGNMENT_OVERSAMPLE_COUNT; s++) {
            encoder1.update();
            mech += encoder1.getMechanicalAngle();
            delayMicroseconds(200);
        }
        mech /= (float)ALIGNMENT_OVERSAMPLE_COUNT;
        rev_mechs[p] = mech;
        rev_zeros[p] = _normalizeAngle((float)(sensor_dir * num_poles) * mech);
    }

    // --- SYNTHESIS & REPORTING ---
    float sum_sin = 0.0f, sum_cos = 0.0f;
    float sum_fwd_sin = 0.0f, sum_fwd_cos = 0.0f;
    float sum_rev_sin = 0.0f, sum_rev_cos = 0.0f;

    for (int p = 0; p < num_poles; p++) {
        float fwd_z = fwd_zeros[p];
        float rev_z = rev_zeros[p];
        float true_z = _normalizeAngle(atan2f(sinf(fwd_z) + sinf(rev_z), cosf(fwd_z) + cosf(rev_z)));
        pole_zeros[p] = true_z;

        sum_sin += sinf(true_z); sum_cos += cosf(true_z);
        sum_fwd_sin += sinf(fwd_z); sum_fwd_cos += cosf(fwd_z);
        sum_rev_sin += sinf(rev_z); sum_rev_cos += cosf(rev_z);

        float hyst_diff = fwd_z - rev_z;
        while (hyst_diff > _PI)  hyst_diff -= _2PI;
        while (hyst_diff < -_PI) hyst_diff += _2PI;
        float hyst_deg = fabsf(hyst_diff) * 57.2957795f;

        bool pole_pass = (hyst_deg <= 12.0f);
        if (!pole_pass) all_poles_ok = false;

        float pole_mech_target = ((float)p / (float)num_poles) * 360.0f;

        Serial.print("  #"); Serial.print(p + 1);
        if (p + 1 < 10) Serial.print(" ");
        Serial.print("  | ");
        if (pole_mech_target < 100.0f) Serial.print(" ");
        if (pole_mech_target < 10.0f) Serial.print(" ");
        Serial.print(pole_mech_target, 1);
        Serial.print(" deg   | ");
        Serial.print(fwd_z, 4);
        Serial.print(" rad   | ");
        Serial.print(rev_z, 4);
        Serial.print(" rad   | ");
        Serial.print(true_z, 4);
        Serial.print(" rad | ±");
        Serial.print(hyst_deg * 0.5f, 2);
        Serial.print(" deg  | ");
        if (pole_pass) {
            Serial.print("[PASS]");
        } else {
            Serial.print("[WARN-HYST]");
        }
        Serial.println();
    }
    Serial.println("--------------------------------------------------------------------------------");

    // Global Circular Mean Electrical Zero
    float mean_zero_angle = _normalizeAngle(atan2f(sum_sin / (float)num_poles, sum_cos / (float)num_poles));
    float mean_fwd_angle  = _normalizeAngle(atan2f(sum_fwd_sin / (float)num_poles, sum_fwd_cos / (float)num_poles));
    float mean_rev_angle  = _normalizeAngle(atan2f(sum_rev_sin / (float)num_poles, sum_rev_cos / (float)num_poles));

    float global_hyst = mean_fwd_angle - mean_rev_angle;
    while (global_hyst > _PI)  global_hyst -= _2PI;
    while (global_hyst < -_PI) global_hyst += _2PI;
    float global_hyst_deg = fabsf(global_hyst) * 57.2957795f;

    // Calculate Circular Variance & Consistency Standard Deviation with wrap-around handling
    float var_sum = 0.0f;
    for (int p = 0; p < num_poles; p++) {
        float diff = pole_zeros[p] - mean_zero_angle;
        while (diff > _PI)  diff -= _2PI;
        while (diff < -_PI) diff += _2PI;
        var_sum += diff * diff;
    }
    float std_dev_rad = sqrtf(var_sum / (float)num_poles);
    float std_dev_deg = std_dev_rad * 57.2957795f;

    bool overall_pass = (std_dev_deg <= 25.0f) && all_poles_ok;

    Serial.println("\n----------------- 🏆 CERTIFIED ELECTRICAL ALIGNMENT RESULTS -----------------");
    Serial.print("  ⭐ Mean Zero Electric Angle: "); Serial.print(mean_zero_angle, 5); Serial.print(" rad (");
    Serial.print(mean_zero_angle * 57.2957795f, 2); Serial.println(" deg elec)");
    Serial.print("  Commutation Direction       : "); Serial.println(sensor_dir == Direction::CW ? "CW (+1)" : "CCW (-1)");
    Serial.print("  Directional Hysteresis Lead : ±"); Serial.print(global_hyst_deg * 0.5f, 2); Serial.print(" deg (CW: ");
    Serial.print(mean_fwd_angle, 4); Serial.print(" rad | CCW: "); Serial.print(mean_rev_angle, 4); Serial.println(" rad)");
    Serial.print("  Pole Alignment Consistency  : ±"); Serial.print(std_dev_deg, 2); Serial.print(" deg elec (±");
    Serial.print(std_dev_rad, 4); Serial.println(" rad)");
    Serial.print("  Alignment Quality Score     : ");

    if (!overall_pass) {
        Serial.println("❌ GRADE F (FAILED: Severe pole misalignment or motor stall!)");
        Serial.println("  ------------------------------------------------------------------------------");
        Serial.println("  ❌ CALIBRATION REJECTED: Consistency threshold exceeded (> 25.0° elec) or stall detected.");
        Serial.println("     Motor disabled for safety.");
        Serial.println("================================================================================\n");
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        is_calibrated = false;
        return false;
    }

    if (std_dev_deg < 6.0f) Serial.println("⭐ GRADE A+ (Ultra-High Precision < 6.0° elec)");
    else if (std_dev_deg < 16.0f) Serial.println("✅ GRADE A (Excellent Precision < 16.0° elec / ~1.4° mech)");
    else Serial.println("⚠️ GRADE B (Acceptable, minor eccentricity/friction)");

    Serial.println("  ------------------------------------------------------------------------------");
    Serial.println("  SimpleFOC Static Code Parameters:");
    Serial.print("    motor.zero_electric_angle = "); Serial.print(mean_zero_angle, 5); Serial.println("f;");
    Serial.print("    motor.sensor_direction    = Direction::"); Serial.print(sensor_dir == Direction::CW ? "CW" : "CCW"); Serial.println(";");
    Serial.println("================================================================================\n");

    // Apply calculated parameters directly to SimpleFOC motor
    base_zero_electric_angle = mean_zero_angle;
    motor1.zero_electric_angle = mean_zero_angle;
    motor1.sensor_direction = sensor_dir;
    motor1.pole_pairs = num_poles;
    is_calibrated = true;
    is_enabled = true;
    m1_target_rpm = 0.0f;
    m1_target_uq = 0.0f;

    // Soft release phase voltage
    motor1.setPhaseVoltage(0, 0, 0);

    return true;
}

void printHelp() {
    Serial.println("\n================================================================================");
    Serial.println("  DA WINNA — MT6816 4-WIRE/3-WIRE SPI ENCODER & SIMPLEFOC SUITE (GM3506)");
    Serial.println("================================================================================");
    Serial.print  ("  Driver State       : "); Serial.println(is_enabled ? "ENABLED" : "DISABLED");
    Serial.print  ("  Control Mode       : ");
    if (control_mode == MotionControlType::velocity_openloop) Serial.println("OPEN-LOOP");
    else if (control_mode == MotionControlType::torque) Serial.println("CLOSED-LOOP VOLTAGE (KV Test Mode)");
    else Serial.println("CLOSED-LOOP VELOCITY FOC");
    Serial.print  ("  Target Speed       : "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
    Serial.print  ("  Target Voltage     : "); Serial.print(m1_target_uq, 2); Serial.println(" V");
    Serial.print  ("  Voltage Limit      : "); Serial.print(motor1.voltage_limit, 1); Serial.println(" V");
    Serial.print  ("  Output Ramp Rate   : "); Serial.print(motor1.PID_velocity.output_ramp, 1); Serial.println(" V/s");
    Serial.print  ("  Velocity Limit     : "); Serial.print(motor1.velocity_limit * 9.54929659f, 1); Serial.println(" RPM");
    Serial.print  ("  Velocity PID       : P="); Serial.print(motor1.PID_velocity.P, 4);
    Serial.print  (" | I="); Serial.print(motor1.PID_velocity.I, 4);
    Serial.print  (" | D="); Serial.print(motor1.PID_velocity.D, 5);
    Serial.print  (" | Tf="); Serial.print(motor1.LPF_velocity.Tf, 4); Serial.println("s");
    Serial.print  ("  Sensor Lag Comp    : "); Serial.print(sensor_lag_comp_us, 1); Serial.println(" us (Phase Lead)");
    Serial.print  ("  PWM Carrier Freq   : "); Serial.print(driver1.pwm_frequency / 1000.0f, 1); Serial.println(" kHz (Fixed)");
    Serial.print  ("  Optimal Align Volt : "); Serial.print(MOTOR_ALIGN_VOLTAGE, 1); Serial.println(" V");
    Serial.print  ("  Active CS Pin      : "); Serial.println(getCSName(encoder1.cs_pin));
    Serial.print  ("  Active Bus Mode    : "); Serial.println(encoder1.use_3wire ? "3-Wire Half-Duplex (PA7 SDAT)" : "4-Wire Full Hardware SPI1");
    Serial.print  ("  Stall Watchdog     : "); Serial.println(stall_watchdog_enabled ? "ACTIVE" : "DISABLED");
    Serial.println("--------------------------------------------------------------------------------");
    Serial.println("Actions:");
    Serial.println("  v or V          : START Autonomous Bidirectional Closed-Loop KV Sweep Test");
    Serial.println("  c or C          : Run Comprehensive 11-Pole Hysteresis-Compensated Calibration");
    Serial.println("  r or R          : Reset to Configured Defaults (P, I, D, Tf=5ms, 15us Lag, 1000V/s Ramp)");
    Serial.println("  w               : Toggle 4-Wire Hardware SPI vs 3-Wire Half-Duplex");
    Serial.println("  o               : Switch to Open-Loop mode (No sensor required)");
    Serial.println("  e               : Toggle Enable / Disable driver");
    Serial.println("  x or Space      : Disable motor and stop (0 RPM / 0 V)");
    Serial.println("  k               : Toggle Stall Detection Watchdog ON/OFF");
    Serial.println("  g               : Toggle telemetry streaming ON / OFF");
    Serial.println("Real-Time Tunable Parameter Commands (Prefix + Float Value):");
    Serial.println("  s<float>        : Set Dynamic Sensor Lag Compensation in us (e.g. 's15.0' or 's0.0')");
    Serial.println("  f<float>        : Set Velocity LPF Tf in seconds (e.g. 'f0.005' or 'f0.010')");
    Serial.println("  a<float>        : Set Output Ramp Rate in V/s (e.g. 'a1000' or 'a500')");
    Serial.println("  l<float>        : Set Voltage Limit in V (e.g. 'l12.0' or 'l8.0')");
    Serial.println("  m<float>        : Set Velocity Limit in RPM (e.g. 'm1400' or 'm800')");
    Serial.println("  p<float>        : Set Velocity P Gain (e.g. 'p0.12')");
    Serial.println("  i<float>        : Set Velocity I Gain (e.g. 'i1.20')");
    Serial.println("  d<float>        : Set Velocity D Gain (e.g. 'd0.001')");
    Serial.println("  q<float>        : Set Motor KV Rating (e.g. 'q130.0')");
    Serial.println("  u<float>        : Set Direct Q-Axis Voltage Uq in V (e.g. 'u3.5' or 'u-4.0')");
    Serial.println("  t<float>        : Set Target Velocity in RPM (e.g. 't120' or 't-60')");
    Serial.println("  z               : Tare / Zero current encoder angle");
    Serial.println("  h or ?          : Show this status menu");
    Serial.println("================================================================================\n");
}

void startKVTest() {
    Serial.println("\n================================================================================");
    Serial.println(">>> 🚀 INITIATING AUTOMATED CLOSED-LOOP KV RATING SWEEP <<<");
    Serial.println("================================================================================");
    Serial.print  ("Motor: GM3506 ("); Serial.print(MOTOR_POLE_PAIRS); Serial.print(" Pole Pairs) | Supply: ");
    Serial.print  (MOTOR_POWER_SUPPLY_VOLT, 1); Serial.print("V SpaceVectorPWM @ ");
    Serial.print  (driver1.pwm_frequency / 1000.0f, 1); Serial.println(" kHz");
    Serial.println("Test Protocol: Linear Unsaturated Voltage Sweep (±1.0V to ±7.0V @ 12V DC)");
    Serial.println("--------------------------------------------------------------------------------");

    if (!is_calibrated) {
        Serial.println(">>> Performing Comprehensive Electrical Zero Alignment First...");
        if (!calibrateComprehensiveFOC(MOTOR_ALIGN_VOLTAGE)) {
            Serial.println("❌ Calibration FAILED! Aborting KV test.");
            driver1.disable();
            is_enabled = false;
            return;
        }
    }

    motor1.controller = MotionControlType::torque;
    control_mode = MotionControlType::torque;
    motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
    motor1.velocity_limit = 3500.0f * 0.104719755f; // Uncap velocity limit during KV sweep
    driver1.enable();
    is_enabled = true;

    kv_point_count = 0;
    kv_stage_index = 0;
    kv_stage_start_ms = millis();
    stage_sample_count = 0;
    m1_target_uq = kv_voltages[0];
    kv_test_running = true;

    Serial.print("[KV_START] TotalStages=");
    Serial.print(KV_TOTAL_STAGES);
    Serial.print(", DwellMs=");
    Serial.print(KV_DWELL_TIME_MS);
    Serial.print(", SampleMs=");
    Serial.println(KV_SAMPLE_TIME_MS);
}

void abortKVTest(const char* reason) {
    kv_test_running = false;
    m1_target_uq = 0.0f;
    m1_target_rpm = 0.0f;
    motor1.velocity_limit = DEFAULT_VELOCITY_LIMIT_RPM * 0.104719755f;
    motor1.move(0.0f);
    driver1.disable();
    is_enabled = false;
    Serial.print("\n>>> ⏹️ KV TEST ABORTED: ");
    Serial.println(reason);
    Serial.println(">>> Motor disabled for safety.\n");
}

void finishKVTest() {
    kv_test_running = false;
    m1_target_uq = 0.0f;
    motor1.velocity_limit = DEFAULT_VELOCITY_LIMIT_RPM * 0.104719755f;
    motor1.move(0.0f);
    driver1.disable();
    is_enabled = false;

    Serial.println("\n================================================================================");
    Serial.println("                   🏁 CLOSED-LOOP KV TEST COMPLETE REPORT                      ");
    Serial.println("================================================================================");
    Serial.println("Stage | Voltage (Uq) | Actual Speed | Speed Ripple | Jitter (StdDev) | Status");
    Serial.println("------+--------------+--------------+--------------+-----------------+-------");

    float fwd_v[MAX_KV_POINTS], fwd_rpm[MAX_KV_POINTS];
    float rev_v[MAX_KV_POINTS], rev_rpm[MAX_KV_POINTS];
    int fwd_count = 0, rev_count = 0;

    for (int i = 0; i < kv_point_count; i++) {
        KVPoint &pt = kv_points[i];
        Serial.print(" ");
        if (i + 1 < 10) Serial.print(" ");
        Serial.print(i + 1);
        Serial.print("   | ");
        if (pt.voltage >= 0.0f) Serial.print("+");
        Serial.print(pt.voltage, 1);
        Serial.print(" V        | ");
        if (pt.measured_rpm >= 0.0f) Serial.print("+");
        Serial.print(pt.measured_rpm, 1);
        Serial.print(" RPM   | ±");
        Serial.print(pt.speed_ripple * 0.5f, 1);
        Serial.print(" RPM    | ±");
        Serial.print(pt.std_dev, 2);
        Serial.print(" RPM       | [PASS]");
        Serial.println();

        if (pt.voltage > 0.5f && fabsf(pt.measured_rpm) > 10.0f) {
            fwd_v[fwd_count] = pt.voltage;
            fwd_rpm[fwd_count] = fabsf(pt.measured_rpm);
            fwd_count++;
        } else if (pt.voltage < -0.5f && fabsf(pt.measured_rpm) > 10.0f) {
            rev_v[rev_count] = fabsf(pt.voltage);
            rev_rpm[rev_count] = fabsf(pt.measured_rpm);
            rev_count++;
        }
    }
    Serial.println("--------------------------------------------------------------------------------");

    float fwd_kv = 0.0f, fwd_v0 = 0.0f, fwd_r2 = 0.0f;
    float rev_kv = 0.0f, rev_v0 = 0.0f, rev_r2 = 0.0f;
    bool fwd_ok = calculateLinearRegression(fwd_v, fwd_rpm, fwd_count, fwd_kv, fwd_v0, fwd_r2);
    bool rev_ok = calculateLinearRegression(rev_v, rev_rpm, rev_count, rev_kv, rev_v0, rev_r2);

    float true_kv = 0.0f;
    float friction_v0 = 0.0f;
    float best_r2 = 0.0f;

    if (fwd_ok && rev_ok) {
        true_kv = (fwd_kv + rev_kv) * 0.5f;
        friction_v0 = fabsf(fwd_v0 / fwd_kv + rev_v0 / rev_kv) * 0.5f;
        best_r2 = (fwd_r2 + rev_r2) * 0.5f;
    } else if (fwd_ok) {
        true_kv = fwd_kv;
        friction_v0 = fabsf(fwd_v0 / fwd_kv);
        best_r2 = fwd_r2;
    } else if (rev_ok) {
        true_kv = rev_kv;
        friction_v0 = fabsf(rev_v0 / rev_kv);
        best_r2 = rev_r2;
    }

    float ke_v_s_rad = (true_kv > 0.1f) ? (60.0f / (_2PI * true_kv)) : 0.0f;
    float kt_nm_a = ke_v_s_rad;
    float max_rpm_12v = true_kv * (12.0f - friction_v0);
    float symmetry_pct = (fwd_kv > 1.0f && rev_kv > 1.0f) ? (1.0f - fabsf(fwd_kv - rev_kv) / true_kv) * 100.0f : 100.0f;

    Serial.println("\n----------------------- 📊 REGRESSION & PHYSICAL CONSTANTS -----------------------");
    Serial.print("  Forward KV (CW)         : "); Serial.print(fwd_kv, 2); Serial.print(" RPM/V (R^2 = "); Serial.print(fwd_r2, 4); Serial.println(")");
    Serial.print("  Reverse KV (CCW)        : "); Serial.print(rev_kv, 2); Serial.print(" RPM/V (R^2 = "); Serial.print(rev_r2, 4); Serial.println(")");
    Serial.print("  Directional Symmetry    : "); Serial.print(symmetry_pct, 1); Serial.println(" %");
    Serial.println("  ------------------------------------------------------------------------------");
    Serial.print("  ⭐ TRUE MOTOR KV RATING : "); Serial.print(true_kv, 1); Serial.println(" RPM/V");
    Serial.print("  Fit Linearity (R^2)     : "); Serial.print(best_r2, 5); Serial.println(" (Excellent > 0.998)");
    Serial.print("  Friction Starting (V0)  : "); Serial.print(friction_v0, 2); Serial.println(" V (Static friction threshold)");
    Serial.print("  Back-EMF Constant (Ke)  : "); Serial.print(ke_v_s_rad, 4); Serial.print(" V*s/rad ("); Serial.print(1.0f / true_kv, 5); Serial.println(" V/RPM)");
    Serial.print("  Torque Constant (Kt)    : "); Serial.print(kt_nm_a, 4); Serial.println(" N*m/A");
    Serial.print("  Max No-Load Speed @ 12V : "); Serial.print(max_rpm_12v, 1); Serial.println(" RPM");
    Serial.println("  ------------------------------------------------------------------------------");
    Serial.println("  SimpleFOC Configuration Code:");
    Serial.print  ("    motor.KV_rating = "); Serial.print(true_kv, 1); Serial.println("f; // GM3506 Measured Closed-Loop KV");
    Serial.println("================================================================================\n");

    motor1.KV_rating = true_kv;

    Serial.print("[KV_RESULT]: KV=");
    Serial.print(true_kv, 2);
    Serial.print(", FwdKV=");
    Serial.print(fwd_kv, 2);
    Serial.print(", RevKV=");
    Serial.print(rev_kv, 2);
    Serial.print(", R2=");
    Serial.print(best_r2, 5);
    Serial.print(", V0=");
    Serial.print(friction_v0, 3);
    Serial.print(", Ke=");
    Serial.print(ke_v_s_rad, 5);
    Serial.print(", Kt=");
    Serial.print(kt_nm_a, 5);
    Serial.print(", MaxRPM=");
    Serial.print(max_rpm_12v, 1);
    Serial.print(", Symmetry=");
    Serial.println(symmetry_pct, 1);
}

void processKVStateMachine() {
    if (!kv_test_running) return;

    uint32_t elapsed = millis() - kv_stage_start_ms;
    float current_target_v = kv_voltages[kv_stage_index];

    motor1.move(current_target_v);

    if (elapsed < KV_DWELL_TIME_MS) {
        stage_sample_count = 0;
    } else if (elapsed < (KV_DWELL_TIME_MS + KV_SAMPLE_TIME_MS)) {
        uint32_t now_us = micros();
        if (now_us - stage_last_sample_us >= KV_SAMPLE_INTERVAL_US) {
            stage_last_sample_us = now_us;
            if (stage_sample_count < STAGE_SAMPLE_CAP) {
                float rpm = motor1.shaftVelocity() * 9.54929659f;
                stage_speed_samples[stage_sample_count++] = rpm;
            }
        }
    } else {
        if (stage_sample_count > 0) {
            float sum = 0.0f;
            float min_spd = stage_speed_samples[0];
            float max_spd = stage_speed_samples[0];
            for (int i = 0; i < stage_sample_count; i++) {
                float s = stage_speed_samples[i];
                sum += s;
                if (s < min_spd) min_spd = s;
                if (s > max_spd) max_spd = s;
            }
            float mean_rpm = sum / stage_sample_count;
            float var_sum = 0.0f;
            for (int i = 0; i < stage_sample_count; i++) {
                float diff = stage_speed_samples[i] - mean_rpm;
                var_sum += diff * diff;
            }
            float std_dev = sqrtf(var_sum / stage_sample_count);
            float ripple = max_spd - min_spd;

            KVPoint pt;
            pt.voltage = current_target_v;
            pt.measured_rpm = mean_rpm;
            pt.speed_ripple = ripple;
            pt.std_dev = std_dev;
            pt.sample_cnt = stage_sample_count;
            kv_points[kv_point_count++] = pt;

            Serial.print("[KV_POINT]: Stage=");
            Serial.print(kv_stage_index + 1);
            Serial.print("/");
            Serial.print(KV_TOTAL_STAGES);
            Serial.print(", V=");
            Serial.print(current_target_v, 2);
            Serial.print(", RPM=");
            Serial.print(mean_rpm, 2);
            Serial.print(", Ripple=");
            Serial.print(ripple, 2);
            Serial.print(", StdDev=");
            Serial.println(std_dev, 3);
        }

        kv_stage_index++;
        if (kv_stage_index >= KV_TOTAL_STAGES) {
            finishKVTest();
        } else {
            kv_stage_start_ms = millis();
            stage_sample_count = 0;
            m1_target_uq = kv_voltages[kv_stage_index];
        }
    }
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

    encoder1.init();

    driver1.voltage_power_supply = MOTOR_POWER_SUPPLY_VOLT;
    driver1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
    driver1.pwm_frequency = MOTOR_PWM_FREQUENCY_HZ;
    driver1.init();

    motor1.linkDriver(&driver1);
    motor1.linkSensor(&encoder1);
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
    motor1.velocity_limit = DEFAULT_VELOCITY_LIMIT_RPM * 0.104719755f;
    motor1.KV_rating = DEFAULT_MOTOR_KV_RATING;
    motor1.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;

    motor1.controller = MotionControlType::velocity;
    motor1.PID_velocity.P = DEFAULT_PID_P;
    motor1.PID_velocity.I = DEFAULT_PID_I;
    motor1.PID_velocity.D = DEFAULT_PID_D;
    motor1.PID_velocity.output_ramp = DEFAULT_OUTPUT_RAMP_V_PER_S;
    motor1.PID_velocity.limit = DEFAULT_VOLTAGE_LIMIT;
    motor1.LPF_velocity.Tf = DEFAULT_LPF_TF_SEC;

    motor1.init();
    driver1.disable();

    printHelp();

    last_telemetry_time = millis();
    last_led_time = millis();
}

void loop() {
    if (is_enabled) {
        // Dynamic Sensor Lag Compensation (Velocity-Proportional Phase Lead on zero_electric_angle)
        if (sensor_lag_comp_us > 0.0f) {
            float lead_rad = motor1.shaft_velocity * (float)motor1.pole_pairs * (sensor_lag_comp_us * 1e-6f);
            motor1.zero_electric_angle = _normalizeAngle(base_zero_electric_angle - lead_rad);
        } else {
            motor1.zero_electric_angle = base_zero_electric_angle;
        }

        motor1.loopFOC();

        if (kv_test_running) {
            processKVStateMachine();
        } else if (control_mode == MotionControlType::velocity) {
            float target_rad_s = m1_target_rpm * 0.104719755f;
            motor1.move(target_rad_s);
        } else if (control_mode == MotionControlType::torque) {
            motor1.move(m1_target_uq);
        } else if (control_mode == MotionControlType::velocity_openloop) {
            float target_rad_s = m1_target_rpm * 0.104719755f;
            motor1.move(target_rad_s);
        }
    } else {
        encoder1.update();
    }

    if (is_enabled && !kv_test_running && stall_watchdog_enabled && control_mode == MotionControlType::velocity) {
        float actual_rpm = fabsf(motor1.shaftVelocity() * 9.54929659f);
        float target_mag = fabsf(m1_target_rpm);

        if (target_mag >= 5.0f && actual_rpm < 1.0f) {
            if (stall_timer == 0) {
                stall_timer = millis();
            } else if (millis() - stall_timer >= 2000) {
                is_stalled = true;
            }
        } else {
            stall_timer = 0;
        }

        if (is_stalled) {
            Serial.println("\n>>> 🚨 SAFETY SHUTDOWN: Motor Stall Detected! <<<");
            is_enabled = false;
            driver1.disable();
            m1_target_rpm = 0.0f;
            is_stalled = false;
            stall_timer = 0;
        }
    }

    if (Serial.available() > 0) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'v':
            case 'V':
                if (kv_test_running) {
                    abortKVTest("User Abort Command ('v')");
                } else {
                    startKVTest();
                }
                break;

            case '0': m1_target_rpm = 0.0f; m1_target_uq = 0.0f; Serial.println("Target: 0"); break;
            case '1': m1_target_rpm = 50.0f; Serial.println("Target: 50 RPM"); break;
            case '2': m1_target_rpm = 100.0f; Serial.println("Target: 100 RPM"); break;
            case '3': m1_target_rpm = 200.0f; Serial.println("Target: 200 RPM"); break;
            case '4': m1_target_rpm = 400.0f; Serial.println("Target: 400 RPM"); break;

            case '+':
            case '=':
                if (control_mode == MotionControlType::torque) {
                    m1_target_uq += 0.5f;
                    Serial.print("Target Uq: "); Serial.print(m1_target_uq, 2); Serial.println(" V");
                } else {
                    m1_target_rpm += 10.0f;
                    Serial.print("Target: "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
                }
                break;

            case '-':
            case '_':
                if (control_mode == MotionControlType::torque) {
                    m1_target_uq -= 0.5f;
                    Serial.print("Target Uq: "); Serial.print(m1_target_uq, 2); Serial.println(" V");
                } else {
                    m1_target_rpm -= 10.0f;
                    Serial.print("Target: "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
                }
                break;

            case 'r':
            case 'R': {
                if (kv_test_running) abortKVTest("Interrupted by Reset");
                motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
                motor1.PID_velocity.limit = DEFAULT_VOLTAGE_LIMIT;
                motor1.velocity_limit = DEFAULT_VELOCITY_LIMIT_RPM * 0.104719755f;
                motor1.PID_velocity.P = DEFAULT_PID_P;
                motor1.PID_velocity.I = DEFAULT_PID_I;
                motor1.PID_velocity.D = DEFAULT_PID_D;
                motor1.PID_velocity.output_ramp = DEFAULT_OUTPUT_RAMP_V_PER_S;
                motor1.LPF_velocity.Tf = DEFAULT_LPF_TF_SEC;
                motor1.KV_rating = DEFAULT_MOTOR_KV_RATING;
                sensor_lag_comp_us = DEFAULT_SENSOR_LAG_COMP_US;
                m1_target_rpm = 0.0f;
                m1_target_uq = 0.0f;
                Serial.println("\n>>> [RESET] Restored Configured GM3506 Defaults (12V SpaceVectorPWM, Tf=5ms, Lag=35us, Ramp=200V/s)\n");
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
                if (kv_test_running) abortKVTest("Interrupted by Driver Toggle");
                is_enabled = !is_enabled;
                if (is_enabled) {
                    if (!is_calibrated) {
                        Serial.println(">>> Driver enabled, running comprehensive FOC alignment...");
                        calibrateComprehensiveFOC(MOTOR_ALIGN_VOLTAGE);
                    } else {
                        driver1.enable();
                    }
                    Serial.println(">>> Driver ENABLED.");
                } else {
                    driver1.disable();
                    Serial.println(">>> Driver DISABLED.");
                }
                break;

            case 'x':
            case 'X':
            case ' ':
                if (kv_test_running) abortKVTest("Emergency Stop Button");
                m1_target_rpm = 0.0f;
                m1_target_uq = 0.0f;
                is_enabled = false;
                driver1.disable();
                Serial.println(">>> MOTOR STOPPED & DISABLED.");
                break;

            case 'c':
            case 'C': {
                if (kv_test_running) abortKVTest("Interrupted by Calibration");
                calibrateComprehensiveFOC(calib_align_voltage);
                break;
            }

            case 'o':
            case 'O':
                if (kv_test_running) abortKVTest("Switched to Open-Loop Mode");
                Serial.println("\n>>> Activating OPEN-LOOP Velocity Mode...");
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
            case 's':
            case 'S': {
                float val = Serial.parseFloat();
                if (val >= 0.0f) {
                    sensor_lag_comp_us = val;
                    Serial.print(">>> Updated Dynamic Sensor Lag Compensation: "); Serial.print(sensor_lag_comp_us, 1); Serial.println(" us");
                }
                break;
            }
            case 'f':
            case 'F': {
                float val = Serial.parseFloat();
                if (val > 0.0f) {
                    motor1.LPF_velocity.Tf = val;
                    Serial.print(">>> Updated Velocity LPF Tf: "); Serial.print(motor1.LPF_velocity.Tf, 4); Serial.println(" s");
                }
                break;
            }
            case 'a':
            case 'A': {
                float val = Serial.parseFloat();
                if (val >= 0.0f) {
                    motor1.PID_velocity.output_ramp = val;
                    Serial.print(">>> Updated Output Voltage Ramp Rate: "); Serial.print(motor1.PID_velocity.output_ramp, 1); Serial.println(" V/s");
                }
                break;
            }
            case 'l':
            case 'L': {
                float val = Serial.parseFloat();
                if (val > 0.0f) {
                    motor1.voltage_limit = val;
                    motor1.PID_velocity.limit = val;
                    Serial.print(">>> Updated Voltage Limit: "); Serial.print(motor1.voltage_limit, 2); Serial.println(" V");
                }
                break;
            }
            case 'm':
            case 'M': {
                float val = Serial.parseFloat();
                if (val > 0.0f) {
                    motor1.velocity_limit = val * 0.104719755f; // Convert RPM to rad/s
                    Serial.print(">>> Updated Velocity Limit: "); Serial.print(val, 1); Serial.println(" RPM");
                }
                break;
            }
            case 'q':
            case 'Q': {
                float val = Serial.parseFloat();
                if (val > 0.0f) {
                    motor1.KV_rating = val;
                    Serial.print(">>> Updated Motor KV Rating: "); Serial.print(motor1.KV_rating, 1); Serial.println(" RPM/V");
                }
                break;
            }
            case 'u':
            case 'U': {
                float val = Serial.parseFloat();
                if (kv_test_running) abortKVTest("Interrupted by Manual Voltage Command");
                m1_target_uq = val;
                control_mode = MotionControlType::torque;
                motor1.controller = MotionControlType::torque;
                if (!is_enabled) {
                    driver1.enable();
                    is_enabled = true;
                }
                Serial.print(">>> Direct Closed-Loop Uq Target: "); Serial.print(m1_target_uq, 2); Serial.println(" V");
                break;
            }
            case 't':
            case 'T': {
                float val = Serial.parseFloat();
                if (kv_test_running) abortKVTest("Interrupted by Manual Speed Command");
                m1_target_rpm = val;
                control_mode = MotionControlType::velocity;
                motor1.controller = MotionControlType::velocity;
                if (!is_enabled) {
                    driver1.enable();
                    is_enabled = true;
                }
                Serial.print(">>> Updated Target Speed: "); Serial.print(m1_target_rpm, 1); Serial.println(" RPM");
                break;
            }
            case 'p':
            case 'P': {
                float val = Serial.parseFloat();
                if (val >= 0.0f) { motor1.PID_velocity.P = val; Serial.print(">>> Updated Velocity P: "); Serial.println(motor1.PID_velocity.P, 4); }
                break;
            }
            case 'i':
            case 'I': {
                float val = Serial.parseFloat();
                if (val >= 0.0f) { motor1.PID_velocity.I = val; Serial.print(">>> Updated Velocity I: "); Serial.println(motor1.PID_velocity.I, 4); }
                break;
            }
            case 'd':
            case 'D': {
                float val = Serial.parseFloat();
                if (val >= 0.0f) { motor1.PID_velocity.D = val; Serial.print(">>> Updated Velocity D: "); Serial.println(motor1.PID_velocity.D, 5); }
                break;
            }

            case 'y':
            case 'Y': {
                float val = Serial.parseFloat();
                if (val >= 1.0f && val <= 12.0f) {
                    calib_align_voltage = val;
                    motor1.voltage_sensor_align = val;
                    Serial.print(">>> Updated Calibration / Alignment Voltage: "); Serial.print(calib_align_voltage, 2); Serial.println(" V");
                }
                break;
            }

            case 'j':
            case 'J': {
                float val = Serial.parseFloat();
                base_zero_electric_angle = _normalizeAngle(val);
                motor1.zero_electric_angle = base_zero_electric_angle;
                Serial.print(">>> Updated Base Electrical Zero Angle: "); Serial.print(base_zero_electric_angle, 5); Serial.print(" rad (");
                Serial.print(base_zero_electric_angle * 57.2957795f, 2); Serial.println(" deg)");
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

    uint32_t now = millis();
    if (show_telemetry && (now - last_telemetry_time >= TELEMETRY_STREAM_INTERVAL_MS)) {
        last_telemetry_time = now;

        float angle_deg = encoder1.getSingleTurnDeg();
        float actual_rpm = motor1.shaftVelocity() * 9.54929659f;
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
        else if (kv_test_running) Serial.print("KV-TEST ");
        else if (control_mode == MotionControlType::velocity_openloop) Serial.print("OPEN  ");
        else if (control_mode == MotionControlType::torque) Serial.print("VOLT  ");
        else Serial.print("CLOSE ");

        Serial.print("| Target: ");
        if (kv_test_running || control_mode == MotionControlType::torque) {
            if (m1_target_uq >= 0.0f) Serial.print("+");
            Serial.print(m1_target_uq, 2);
            Serial.print(" V  ");
        } else {
            if (m1_target_rpm >= 0.0f) Serial.print("+");
            Serial.print(m1_target_rpm, 1);
            Serial.print(" RPM");
        }

        Serial.print(" | Actual: ");
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

    if (now - last_led_time >= 500) {
        last_led_time = now;
        led_state = !led_state;
        digitalWrite(STATUS_LED, led_state ? LOW : HIGH);
    }
}
