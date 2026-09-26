// ============================================================================
// DA WINNA — ESP32-S3 MT6816 SPI ENCODER & SIMPLEFOC SUITE (GM3506 @ 12V)
// 11-POLE CONTINUOUS ZERO VECTOR + 2-HARMONIC FOURIER + DYNAMIC SPEED LAG TRIM
// + NVS FLASH SAVE / LOAD PROFILE PERSISTENCE
// ============================================================================
// Target Board : ESP32-S3-DevKitC-1 (240MHz Dual Core)
// Driver Pins  : GPIO 15 (Phase A), GPIO 16 (Phase B), GPIO 17 (Phase C), GPIO
// 2 (EN) Encoder Pins : GPIO 12 (SCK), GPIO 11 (MOSI), GPIO 13 (MISO), GPIO 10
// (CS) Thermistor   : GPIO 4 (ADC1_CH3)
// ============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include "esp_task_wdt.h"

// 1. MOTOR & HARDWARE CONFIGURATION
#define MOTOR_POLE_PAIRS 11              // GM3506 (24N22P = 11 Pole Pairs)
#define MOTOR_PHASE_RESISTANCE_OHM 5.60f // Phase-to-Phase / 2 = 5.6 Ohm
#define POWER_SUPPLY_VOLTAGE 12.0f       // 12V DC Regulated Bench Supply
#define CALIBRATION_VOLTAGE                                                    \
  3.50f // Safe Open-Loop Calibration Voltage (3.5V ~ 0.45A)

// 2. BUS & PROTOCOL SETTINGS
#define SPI_SPEED_DEFAULT_HZ                                                   \
  4000000 // MT6816 SPI Master Baud (4 MHz with 15cm 22R termination)
#define SPI_USE_3WIRE_DEFAULT false // Default 4-Wire Hardware SPI
#define SENSOR_LAG_COMP_DEFAULT_US                                             \
  35.0f // Default Dynamic Lag Lead Compensation (us)
#define DEFAULT_LPF_TF_SEC                                                     \
  0.0050f // Velocity Low-Pass Filter Tf (5 ms gives razor-sharp response)

// 3. 128-POINT ENCODER NON-LINEARITY LUT CONSTANTS
#define ENCODER_LUT_SIZE 128 // 128-point calibration table
// Maximum allowed single-point correction before pole-slip is declared:
#define LUT_ANOMALY_THRESHOLD_RAD 0.25f // > 14.3 deg mech = pole jump
// Maximum allowed peak-to-peak span across the whole table:
#define LUT_PKPK_LIMIT_RAD 0.50f        // > 28.6 deg mech = bad sweep
#define CALIB_SWEEP_STEPS                                                      \
  256 // Number of micro-steps across 360 mechanical degrees
#define ALIGNMENT_OVERSAMPLE_COUNT                                             \
  128 // Samples per pole point during electrical zero alignment

// 4. TIMING, LOOP RATES & SAMPLING FREQUENCIES
#define TELEMETRY_STREAM_INTERVAL_MS                                           \
  50 // Serial Live Streaming Interval (50ms = 20 Hz)

// 5. REAL-TIME TUNABLE DEFAULT FOC PARAMETERS
#define DEFAULT_VOLTAGE_LIMIT 12.0f // Maximum allowable phase voltage (V)
#define DEFAULT_VELOCITY_LIMIT_RPM                                             \
  1432.0f // Maximum velocity limit (1432 RPM = 150 rad/s)
#define DEFAULT_OUTPUT_RAMP_V_PER_S                                            \
  200.0f                      // Output voltage ramp rate limit (200 V/s)
#define DEFAULT_PID_P 0.1200f // Velocity Proportional Gain
#define DEFAULT_PID_I 1.2000f // Velocity Integral Gain
#define DEFAULT_PID_D 0.0000f // Velocity Derivative Gain
#define DEFAULT_MOTOR_KV_RATING                                                \
  130.0f // Initial Back-EMF Feedforward KV rating (RPM/V)

// 6. AUTOMATED CLOSED-LOOP KV TEST PARAMETERS
#define KV_DWELL_TIME_MS                                                       \
  1200 // Settle time for motor to reach steady terminal velocity (ms)
#define KV_SAMPLE_TIME_MS 600      // Velocity sampling window duration (ms)
#define KV_SAMPLE_INTERVAL_US 5000 // Velocity sampling rate (5000 us = 200 Hz)
#define STAGE_SAMPLE_CAP 200

// ============================================================================
// ESP32-S3 PIN DEFINITIONS
// ============================================================================
#define PIN_SCK 12   // SPI2 SCK (Clock)
#define PIN_MOSI 11  // SPI2 MOSI (Master Out / 3-Wire SDAT)
#define PIN_MISO 13  // SPI2 MISO (Master In)
#define PIN_CS_M1 10 // Dedicated Encoder 1 CS

// Motor 1 Inverter Driver Pins (MCPWM0)
#define M1_PWM_A 15 // Phase A (MCPWM0_OP0A)
#define M1_PWM_B 16 // Phase B (MCPWM0_OP0B)
#define M1_PWM_C 17 // Phase C (MCPWM0_OP1A)
#define M1_EN 2     // Gate Driver Enable

// Analog Sensors
#define PIN_THERM_M1 4 // ADC1_CH3

// ============================================================================
// MT6816 Sensor Driver with Fourier Harmonic Linearization
// ============================================================================
class MT6816Sensor : public Sensor {
public:
  int cs_pin;
  uint32_t spi_speed;
  uint8_t spi_mode;
  bool use_3wire;

  // Fourier Calibration Parameters
  float fourier_A1, fourier_B1;
  float fourier_A2, fourier_B2;
  float lut_errors[ENCODER_LUT_SIZE];
  bool is_lut_calibrated;

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
      : cs_pin(PIN_CS_M1), spi_speed(SPI_SPEED_DEFAULT_HZ), spi_mode(SPI_MODE3),
        use_3wire(SPI_USE_3WIRE_DEFAULT), fourier_A1(0.0f), fourier_B1(0.0f),
        fourier_A2(0.0f), fourier_B2(0.0f), is_lut_calibrated(false),
        total_reads(0), parity_errors(0), mag_faults(0), last_no_mag(false),
        last_parity_ok(true), last_raw_h(0), last_raw_l(0), last_raw_14bit(0),
        zero_offset_deg(0.0f) {
    memset(lut_errors, 0, sizeof(lut_errors));
  }

  void setMode(bool three_wire) {
    use_3wire = three_wire;
    if (use_3wire) {
      SPI.end();
      pinMode(PIN_SCK, OUTPUT);
      digitalWrite(PIN_SCK, HIGH);
      pinMode(PIN_MOSI, INPUT_PULLUP);
    } else {
      SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
      pinMode(PIN_MISO, INPUT_PULLUP);
    }
  }

  void init() {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    setMode(SPI_USE_3WIRE_DEFAULT);
    delay(30);

    for (int i = 0; i < 5; i++) {
      uint8_t h = 0, l = 0;
      uint16_t raw14 = 0;
      bool no_mag = false, parity_ok = false;
      readCurrent(h, l, raw14, no_mag, parity_ok);
      last_raw_h = h;
      last_raw_l = l;
      last_raw_14bit = raw14;
      last_no_mag = no_mag;
      last_parity_ok = parity_ok;
      delay(2);
    }

    this->Sensor::init();
  }

  static bool parityCheck(uint16_t data) {
    int count = 0;
    for (int i = 0; i < 16; i++) {
      if (data & (1 << i))
        count++;
    }
    return (count % 2 == 0);
  }

  void read4WireHW(int target_cs, uint8_t &h, uint8_t &l) {
    SPISettings settings(spi_speed, MSBFIRST, spi_mode);

    SPI.beginTransaction(settings);

    digitalWrite(target_cs, LOW);
    delayMicroseconds(1);
    uint16_t rx_h = SPI.transfer16(0x8300);
    delayMicroseconds(1);
    digitalWrite(target_cs, HIGH);

    delayMicroseconds(1);

    digitalWrite(target_cs, LOW);
    delayMicroseconds(1);
    uint16_t rx_l = SPI.transfer16(0x8400);
    delayMicroseconds(1);
    digitalWrite(target_cs, HIGH);

    SPI.endTransaction();

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
        if (digitalRead(PIN_MOSI))
          in_byte |= (1 << i);
      }
    }
    return in_byte;
  }

  void read3WireAtomic(int target_cs, uint8_t &h, uint8_t &l) {
    pinMode(PIN_SCK, OUTPUT);
    digitalWrite(PIN_SCK, HIGH);

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
  }

  bool readCurrent(uint8_t &h, uint8_t &l, uint16_t &raw14, bool &no_mag,
                   bool &parity_ok) {
    for (int retry = 0; retry < 2; retry++) {
      if (use_3wire) {
        read3WireAtomic(cs_pin, h, l);
      } else {
        read4WireHW(cs_pin, h, l);
      }

      uint16_t angle_data = ((uint16_t)h << 8) | l;
      raw14 = ((uint16_t)h << 6) | ((l & 0xFC) >> 2);
      no_mag = (l & 0x02) != 0;
      parity_ok = parityCheck(angle_data);

      if (parity_ok)
        break;
      delayMicroseconds(2);
    }

    total_reads++;
    if (!parity_ok)
      parity_errors++;
    if (no_mag)
      mag_faults++;

    return parity_ok;
  }

  float getRawMechanicalAngle() {
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

  float getSensorAngle() override {
    float raw_rad = getRawMechanicalAngle();

    if (!is_lut_calibrated) {
      return raw_rad;
    }

    float norm_rad = raw_rad;
    while (norm_rad < 0.0f)
      norm_rad += _2PI;
    while (norm_rad >= _2PI)
      norm_rad -= _2PI;

    const float lut_step = _2PI / (float)ENCODER_LUT_SIZE;
    int idx0 = (int)(norm_rad / lut_step);
    if (idx0 >= ENCODER_LUT_SIZE)
      idx0 = ENCODER_LUT_SIZE - 1;
    int idx1 = (idx0 + 1) % ENCODER_LUT_SIZE;
    float frac = (norm_rad - (float)idx0 * lut_step) / lut_step;

    float offset = (1.0f - frac) * lut_errors[idx0] + frac * lut_errors[idx1];
    float calib_rad = raw_rad - offset;
    return calib_rad;
  }

  float getSingleTurnDeg() {
    float rad = is_lut_calibrated ? getSensorAngle() : getRawMechanicalAngle();
    float d = (rad / _2PI) * 360.0f - zero_offset_deg;
    while (d < 0.0f)
      d += 360.0f;
    while (d >= 360.0f)
      d -= 360.0f;
    return d;
  }

  void zeroCurrentPosition() {
    float rad = is_lut_calibrated ? getSensorAngle() : getRawMechanicalAngle();
    zero_offset_deg = (rad / _2PI) * 360.0f;
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

    if (Ts < 0.001f) {
      return velocity;
    }

    float angle_curr = (float)full_rotations * _2PI + angle_prev;
    float angle_old = (float)vel_full_rotations * _2PI + vel_angle_prev;
    float d_angle = angle_curr - angle_old;

    float raw_vel = d_angle / Ts;
    if (raw_vel > 1500.0f)
      raw_vel = 1500.0f;
    if (raw_vel < -1500.0f)
      raw_vel = -1500.0f;

    velocity = raw_vel;
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = now_us;
    return velocity;
  }
};

// ============================================================================
// GLOBAL OBJECTS & STATE
// ============================================================================
Preferences prefs;
MT6816Sensor encoder1;
BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver1(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);

float target_velocity_rpm = 0.0f;
float target_voltage = 0.0f;
bool is_enabled = false;
bool is_calibrated = false;
bool show_telemetry = true;
uint32_t last_telemetry_time_ms = 0;

float calib_align_voltage = CALIBRATION_VOLTAGE;
float base_zero_electric_angle = 0.0f;
float sensor_lag_comp_us = SENSOR_LAG_COMP_DEFAULT_US;
float motor_kv_rating = DEFAULT_MOTOR_KV_RATING;

// 11-Pole Continuous Zero Vector (Eliminates inter-pole phase dispersion)
float calibrated_pole_zeros[MOTOR_POLE_PAIRS] = {0};
bool use_pole_specific_zeros = true;

// Continuous Interpolation across all 11 Poles
float getInterpolatedElectricalZero(float mech_angle_rad) {
  if (!use_pole_specific_zeros || !is_calibrated) {
    return base_zero_electric_angle;
  }
  float norm = _normalizeAngle(mech_angle_rad);
  if (norm < 0.0f)
    norm += _2PI;
  float pole_pos = (norm / _2PI) * (float)MOTOR_POLE_PAIRS;
  int p0 = (int)pole_pos % MOTOR_POLE_PAIRS;
  int p1 = (p0 + 1) % MOTOR_POLE_PAIRS;
  float frac = pole_pos - (float)((int)pole_pos);

  float z0 = calibrated_pole_zeros[p0];
  float z1 = calibrated_pole_zeros[p1];
  float dz = z1 - z0;
  while (dz > _PI)
    dz -= _2PI;
  while (dz < -_PI)
    dz += _2PI;
  return _normalizeAngle(z0 + frac * dz);
}

// Forward Declarations
bool calibrateComprehensiveFOC(float align_voltage);
extern bool kv_test_running;

// ============================================================================
// THREAD-SAFE DUAL-CORE FOC ISOLATION & TELEMETRY SNAPSHOT
// ============================================================================
portMUX_TYPE telemSpinlock = portMUX_INITIALIZER_UNLOCKED;

struct TelemetrySnapshot {
  float angle_deg;
  float actual_rpm;
  float target_rpm;
  float target_voltage;
  float uq;
  float ud;
  uint16_t raw14;
  uint8_t raw_h;
  uint8_t raw_l;
  bool no_mag;
  bool parity_ok;
  bool is_enabled;
  bool kv_test;
  MotionControlType ctrl_mode;
  float foc_hz;
};

TelemetrySnapshot g_telemetry = {0};
volatile bool foc_paused = false;
TaskHandle_t focTaskHandle = NULL;

void focTaskLoop(void *param) {
  // Disable Task Watchdog on Core 0 so continuous 15kHz FOC loop doesn't trip WDT
  disableCore0WDT();
  TaskHandle_t idle_0 = xTaskGetIdleTaskHandleForCPU(0);
  if (idle_0 != NULL) {
    esp_task_wdt_delete(idle_0);
  }

  uint32_t foc_cycle_count = 0;
  uint32_t last_hz_meas_us = micros();
  float current_foc_hz = 0.0f;
  uint32_t last_snapshot_us = micros();

  while (true) {
    if (foc_paused) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (is_enabled) {
      // 1. Dynamic Sensor Lag Compensation
      if (sensor_lag_comp_us > 0.0f) {
        float lead_rad = motor1.shaft_velocity * (float)motor1.pole_pairs *
                         (sensor_lag_comp_us * 1e-6f);
        motor1.zero_electric_angle =
            _normalizeAngle(base_zero_electric_angle - lead_rad);
      } else {
        motor1.zero_electric_angle = base_zero_electric_angle;
      }

      // 2. High-Frequency Commutation Loop
      motor1.loopFOC();

      // 3. Motion Control Move
      if (kv_test_running) {
        motor1.move(target_voltage);
      } else if (motor1.controller == MotionControlType::velocity) {
        float target_rad_s = target_velocity_rpm * 0.104719755f;
        motor1.move(target_rad_s);
      } else if (motor1.controller == MotionControlType::torque) {
        motor1.move(target_voltage);
      }

      foc_cycle_count++;
    } else {
      // Motor disabled: update sensor angle for live GUI tracking, then idle briefly
      motor1.loopFOC();
      foc_cycle_count++;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Measure FOC loop frequency every 250ms
    uint32_t now_us = micros();
    if (now_us - last_hz_meas_us >= 250000) {
      current_foc_hz = (float)foc_cycle_count * 1000000.0f / (float)(now_us - last_hz_meas_us);
      foc_cycle_count = 0;
      last_hz_meas_us = now_us;
    }

    // Update telemetry snapshot at ~100 Hz (every 10ms)
    if (now_us - last_snapshot_us >= 10000) {
      last_snapshot_us = now_us;

      // Extract single-turn mechanical degree using latest sensor data without extra SPI reads
      float norm_mech = _normalizeAngle(motor1.shaft_angle);
      if (norm_mech < 0.0f)
        norm_mech += _2PI;
      float d = (norm_mech / _2PI) * 360.0f - encoder1.zero_offset_deg;
      while (d < 0.0f)
        d += 360.0f;
      while (d >= 360.0f)
        d -= 360.0f;

      portENTER_CRITICAL(&telemSpinlock);
      g_telemetry.angle_deg = d;
      g_telemetry.actual_rpm = motor1.shaft_velocity * 9.54929658551f;
      g_telemetry.target_rpm = target_velocity_rpm;
      g_telemetry.target_voltage = target_voltage;
      g_telemetry.uq = motor1.voltage.q;
      g_telemetry.ud = motor1.voltage.d;
      g_telemetry.raw14 = encoder1.last_raw_14bit;
      g_telemetry.raw_h = encoder1.last_raw_h;
      g_telemetry.raw_l = encoder1.last_raw_l;
      g_telemetry.no_mag = encoder1.last_no_mag;
      g_telemetry.parity_ok = encoder1.last_parity_ok;
      g_telemetry.is_enabled = is_enabled;
      g_telemetry.kv_test = kv_test_running;
      g_telemetry.ctrl_mode = motor1.controller;
      g_telemetry.foc_hz = current_foc_hz;
      portEXIT_CRITICAL(&telemSpinlock);
    }
  }
}

// ============================================================================
// NVS FLASH PERSISTENT STORAGE
// ============================================================================
void saveConfigToNVS() {
  prefs.begin("foc_config", false);
  prefs.putFloat("zero_angle", base_zero_electric_angle);
  prefs.putInt("sensor_dir", (int)motor1.sensor_direction);
  prefs.putFloat("lag_comp", sensor_lag_comp_us);
  prefs.putFloat("kv_rating", motor_kv_rating);
  prefs.putBool("lut_active", encoder1.is_lut_calibrated);
  prefs.putFloat("f_a1", encoder1.fourier_A1);
  prefs.putFloat("f_b1", encoder1.fourier_B1);
  prefs.putFloat("f_a2", encoder1.fourier_A2);
  prefs.putFloat("f_b2", encoder1.fourier_B2);
  prefs.putBytes("lut_table", encoder1.lut_errors, sizeof(encoder1.lut_errors));
  prefs.putBytes("pole_zeros", calibrated_pole_zeros,
                 sizeof(calibrated_pole_zeros));
  prefs.putFloat("pid_p", motor1.PID_velocity.P);
  prefs.putFloat("pid_i", motor1.PID_velocity.I);
  prefs.putFloat("pid_d", motor1.PID_velocity.D);
  prefs.putFloat("lpf_tf", motor1.LPF_velocity.Tf);
  prefs.putBool("is_valid", true);
  prefs.end();

  Serial.println(
      "\n>>> [NVS] MOTOR CONFIGURATION SAVED TO ESP32 FLASH MEMORY! <<<");
}

bool loadConfigFromNVS() {
  prefs.begin("foc_config", true);
  bool valid = prefs.getBool("is_valid", false);
  if (!valid) {
    prefs.end();
    Serial.println(">>> [NVS] No saved calibration profile in flash. Please "
                   "run calibration ('w' or 'c').");
    return false;
  }

  base_zero_electric_angle =
      prefs.getFloat("zero_angle", base_zero_electric_angle);
  motor1.zero_electric_angle = base_zero_electric_angle;
  motor1.sensor_direction =
      (Direction)prefs.getInt("sensor_dir", (int)Direction::CW);
  sensor_lag_comp_us = prefs.getFloat("lag_comp", sensor_lag_comp_us);
  motor_kv_rating = prefs.getFloat("kv_rating", motor_kv_rating);

  encoder1.is_lut_calibrated = prefs.getBool("lut_active", false);
  encoder1.fourier_A1 = prefs.getFloat("f_a1", 0.0f);
  encoder1.fourier_B1 = prefs.getFloat("f_b1", 0.0f);
  encoder1.fourier_A2 = prefs.getFloat("f_a2", 0.0f);
  encoder1.fourier_B2 = prefs.getFloat("f_b2", 0.0f);

  prefs.getBytes("lut_table", encoder1.lut_errors, sizeof(encoder1.lut_errors));
  prefs.getBytes("pole_zeros", calibrated_pole_zeros,
                 sizeof(calibrated_pole_zeros));

  motor1.PID_velocity.P = prefs.getFloat("pid_p", DEFAULT_PID_P);
  motor1.PID_velocity.I = prefs.getFloat("pid_i", DEFAULT_PID_I);
  motor1.PID_velocity.D = prefs.getFloat("pid_d", DEFAULT_PID_D);
  motor1.LPF_velocity.Tf = prefs.getFloat("lpf_tf", DEFAULT_LPF_TF_SEC);

  prefs.end();

  is_calibrated = true;
  use_pole_specific_zeros = true;

  Serial.println(
      "\n>>> [NVS] MOTOR CONFIGURATION LOADED FROM ESP32 FLASH! <<<");
  Serial.print("  Zero Angle : ");
  Serial.print(base_zero_electric_angle, 5);
  Serial.println(" rad");
  Serial.print("  Direction  : ");
  Serial.println(motor1.sensor_direction == Direction::CW ? "CW" : "CCW");
  Serial.print("  Sensor Lag : ");
  Serial.print(sensor_lag_comp_us, 1);
  Serial.println(" us");
  Serial.print("  LUT Status : ");
  Serial.println(encoder1.is_lut_calibrated ? "128-PT LUT ACTIVE" : "OFF");
  Serial.println(
      "-----------------------------------------------------------\n");
  return true;
}

// Dump Full Config JSON block to Serial
void dumpConfigJSON() {
  Serial.println("\n[CONFIG_START]");
  Serial.print("{\"zero_electric_angle\":");
  Serial.print(base_zero_electric_angle, 6);
  Serial.print(",\"sensor_direction\":");
  Serial.print((int)motor1.sensor_direction);
  Serial.print(",\"sensor_lag_comp_us\":");
  Serial.print(sensor_lag_comp_us, 2);
  Serial.print(",\"motor_kv_rating\":");
  Serial.print(motor_kv_rating, 2);
  Serial.print(",\"is_lut_calibrated\":");
  Serial.print(encoder1.is_lut_calibrated ? "true" : "false");
  Serial.print(",\"fourier_A1\":");
  Serial.print(encoder1.fourier_A1, 6);
  Serial.print(",\"fourier_B1\":");
  Serial.print(encoder1.fourier_B1, 6);
  Serial.print(",\"fourier_A2\":");
  Serial.print(encoder1.fourier_A2, 6);
  Serial.print(",\"fourier_B2\":");
  Serial.print(encoder1.fourier_B2, 6);
  Serial.print(",\"pid_p\":");
  Serial.print(motor1.PID_velocity.P, 4);
  Serial.print(",\"pid_i\":");
  Serial.print(motor1.PID_velocity.I, 4);
  Serial.print(",\"pid_d\":");
  Serial.print(motor1.PID_velocity.D, 5);
  Serial.print(",\"lpf_tf\":");
  Serial.print(motor1.LPF_velocity.Tf, 4);
  Serial.print(",\"pole_zeros\":[");
  for (int i = 0; i < MOTOR_POLE_PAIRS; i++) {
    Serial.print(calibrated_pole_zeros[i], 5);
    if (i < MOTOR_POLE_PAIRS - 1)
      Serial.print(",");
  }
  Serial.print("],\"lut_errors\":[");
  for (int i = 0; i < ENCODER_LUT_SIZE; i++) {
    Serial.print(encoder1.lut_errors[i], 6);
    if (i < ENCODER_LUT_SIZE - 1)
      Serial.print(",");
  }
  Serial.println("]}");
  Serial.println("[CONFIG_END]\n");
}

// KV Test State Machine
bool kv_test_running = false;
const float kv_voltages[] = {1.0f,  2.0f,  3.0f,  4.0f,  5.0f,   6.0f,
                             8.0f,  10.0f, 12.0f, -1.0f, -2.0f,  -3.0f,
                             -4.0f, -5.0f, -6.0f, -8.0f, -10.0f, -12.0f};
const int NUM_KV_STAGES = sizeof(kv_voltages) / sizeof(kv_voltages[0]);
int kv_stage_index = 0;
uint32_t kv_stage_start_ms = 0;
uint32_t stage_last_sample_us = 0;
float stage_speed_samples[STAGE_SAMPLE_CAP];
int stage_sample_count = 0;

float fwd_v[10], fwd_rpm[10];
int fwd_count = 0;
float rev_v[10], rev_rpm[10];
int rev_count = 0;

// Helper: Print Visual Dial Compass
void printVisualDial(float angle_deg) {
  const char *dial_segments[] = {"[  |  ]", "[ /   ]", "[--   ]", "[ \\   ]",
                                 "[  |  ]", "[   / ]", "[   --]", "[   \\ ]"};
  int seg = (int)((angle_deg + 22.5f) / 45.0f) % 8;
  Serial.print(" ");
  Serial.print(dial_segments[seg]);
}

// Helper: Linear Regression
bool calculateLinearRegression(const float *x, const float *y, int n,
                               float &slope, float &intercept, float &r2) {
  if (n < 2)
    return false;
  float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0, sum_yy = 0;
  for (int i = 0; i < n; i++) {
    sum_x += x[i];
    sum_y += y[i];
    sum_xy += x[i] * y[i];
    sum_xx += x[i] * x[i];
    sum_yy += y[i] * y[i];
  }
  float fn = (float)n;
  float denom = (fn * sum_xx - sum_x * sum_x);
  if (fabsf(denom) < 1e-6f)
    return false;
  slope = (fn * sum_xy - sum_x * sum_y) / denom;
  intercept = (sum_y - slope * sum_x) / fn;

  float y_mean = sum_y / fn;
  float ss_tot = 0, ss_res = 0;
  for (int i = 0; i < n; i++) {
    float f_i = slope * x[i] + intercept;
    ss_tot += (y[i] - y_mean) * (y[i] - y_mean);
    ss_res += (y[i] - f_i) * (y[i] - f_i);
  }
  r2 = (ss_tot > 1e-6f) ? (1.0f - (ss_res / ss_tot)) : 1.0f;
  return true;
}

// ============================================================================
// AUTO SPEED LAG & CURRENT MINIMIZATION OPTIMIZER (1000 RPM AUTO-TRIM)
// ============================================================================
void runSpeedLagOptimizer(float test_rpm) {
  if (!is_calibrated) {
    Serial.println("ERROR: Please run 'w' or 'c' calibration first!");
    return;
  }

  Serial.println("\n==========================================================="
                 "=====================");
  Serial.print(">>> ⚡ RUNNING HIGH-SPEED DYNAMIC CURRENT MINIMIZATION (");
  Serial.print(test_rpm, 0);
  Serial.println(" RPM) <<<");
  Serial.println("============================================================="
                 "===================");
  Serial.println("Finding optimal Sensor Lag Lead Compensation (Press 'x' to "
                 "STOP anytime)...");

  motor1.controller = MotionControlType::velocity;
  driver1.enable();
  is_enabled = true;

  float target_rad_s = test_rpm * 0.104719755f;
  float best_lag_us = sensor_lag_comp_us;
  float min_uq = 100.0f;
  bool aborted = false;

  // Settle loop with high-rate FOC and abort checking
  uint32_t settle_start = millis();
  while (millis() - settle_start < 1500) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        aborted = true;
        break;
      }
    }
    motor1.loopFOC();
    motor1.move(target_rad_s);
    delayMicroseconds(200);
  }

  if (aborted) {
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    Serial.println(">>> 🛑 TRIM LAG ABORTED BY USER (EMERGENCY STOP) <<<");
    return;
  }

  // Stall check after initial spin-up
  float init_rpm = encoder1.getVelocity() * 9.54929658551f;
  if (fabsf(init_rpm) < 150.0f) {
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    Serial.print(">>> 🚨 STALL DETECTED (Speed only ");
    Serial.print(init_rpm, 1);
    Serial.println(" RPM)! Safety Shutdown.");
    return;
  }

  Serial.println("Lag (us) | Voltage Uq (V) | Actual RPM | Efficiency Status");
  Serial.println("---------+----------------+------------+------------------");

  for (float test_lag = 10.0f; test_lag <= 65.0f; test_lag += 5.0f) {
    sensor_lag_comp_us = test_lag;

    // Step Settle & Sample with continuous FOC loop
    float uq_sum = 0.0f;
    float rpm_sum = 0.0f;
    int sample_count = 0;
    uint32_t step_start = millis();

    while (millis() - step_start < 400) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'x' || c == 'X') {
          aborted = true;
          break;
        }
      }

      // Lead rad calculation
      float lead_rad = motor1.shaft_velocity * (float)motor1.pole_pairs *
                       (sensor_lag_comp_us * 1e-6f);
      motor1.zero_electric_angle =
          _normalizeAngle(base_zero_electric_angle - lead_rad);

      motor1.loopFOC();
      motor1.move(target_rad_s);

      if (millis() - step_start > 150) {
        uq_sum += fabsf(motor1.voltage.q);
        rpm_sum += encoder1.getVelocity() * 9.54929658551f;
        sample_count++;
      }
      delayMicroseconds(200);
    }

    if (aborted) {
      motor1.setPhaseVoltage(0, 0, 0);
      driver1.disable();
      is_enabled = false;
      Serial.println("\n>>> 🛑 TRIM LAG ABORTED BY USER (EMERGENCY STOP) <<<");
      return;
    }

    float avg_uq = (sample_count > 0) ? (uq_sum / (float)sample_count) : 100.0f;
    float avg_rpm = (sample_count > 0) ? (rpm_sum / (float)sample_count) : 0.0f;

    // Stall guard check
    if (fabsf(avg_rpm) < 150.0f) {
      motor1.setPhaseVoltage(0, 0, 0);
      driver1.disable();
      is_enabled = false;
      Serial.print("\n>>> 🚨 STALL DETECTED AT LAG ");
      Serial.print(test_lag, 1);
      Serial.println(" us! Motor stopped.");
      return;
    }

    Serial.print("  ");
    if (test_lag < 10.0f)
      Serial.print(" ");
    Serial.print(test_lag, 1);
    Serial.print(" us | ");
    Serial.print(avg_uq, 3);
    Serial.print(" V      | ");
    Serial.print(avg_rpm, 1);
    Serial.print(" RPM  | ");

    if (avg_uq < min_uq) {
      min_uq = avg_uq;
      best_lag_us = test_lag;
      Serial.println("[NEW MINIMUM]");
    } else {
      Serial.println();
    }
  }

  sensor_lag_comp_us = best_lag_us;
  motor1.setPhaseVoltage(0, 0, 0);
  driver1.disable();
  is_enabled = false;

  Serial.println("-------------------------------------------------------------"
                 "-------------------");
  Serial.print("⭐ OPTIMAL SENSOR LAG COMPENSATION LOCKED: ");
  Serial.print(best_lag_us, 1);
  Serial.println(" us");
  Serial.print("   Minimum Inverter Demand Voltage: ");
  Serial.print(min_uq, 3);
  Serial.println(" V (Lowest Phase Current)");
  Serial.println(
      "   Motor soft-stopped. Click 'Enable (e)' or set target speed to run.");
  Serial.println("============================================================="
                 "===================\n");
}

// ============================================================================
// 128-POINT NON-LINEARITY LUT CALIBRATION
// ============================================================================
bool calibrate256PointLUT(float align_voltage) {
  Serial.println("\n==========================================================="
                 "=====================");
  Serial.println(
      ">>> 🔬 STARTING 128-POINT DUAL-PASS NON-LINEARITY LUT CALIBRATION <<<");
  Serial.println("============================================================="
                 "===================");
  Serial.println("Target Motor       : GM3506 (11 Pole Pairs on ESP32-S3)");
  Serial.print("Calibration Voltage: ");
  Serial.print(align_voltage, 2);
  Serial.println(" V");
  Serial.println(
      "Methodology        : 128-Step Dual-Pass Bidirectional Mechanical Sweep");
  Serial.println("Oversampling       : 64 Samples / Step with Micro-Dither "
                 "(Stiction Cancellation)");
  Serial.println("Anomaly Guard      : Any |err| > 0.25 rad -> FAIL (pole slip)");
  Serial.println("Press 'x' to STOP anytime");
  Serial.println("-------------------------------------------------------------"
                 "-------------------");

  encoder1.is_lut_calibrated = false;
  motor1.controller = MotionControlType::velocity_openloop;
  driver1.enable();
  is_enabled = true;

  // 1. Determining Commutation Direction & Settling Start Position
  Serial.print("1. Determining Sensor Commutation Direction & Settling... ");
  motor1.setPhaseVoltage(align_voltage, 0, _3PI_2);
  delay(400);
  float start_raw = encoder1.getRawMechanicalAngle();

  const int DIR_STEPS = 200;
  for (int step = 0; step <= DIR_STEPS; step++) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
    float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
    motor1.setPhaseVoltage(align_voltage, 0, el_ang);
    delayMicroseconds(2000);
  }
  delay(150);
  float mid_raw = encoder1.getRawMechanicalAngle();

  for (int step = DIR_STEPS; step >= 0; step--) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
    float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
    motor1.setPhaseVoltage(align_voltage, 0, el_ang);
    delayMicroseconds(2000);
  }
  delay(150);

  float d_ang = mid_raw - start_raw;
  while (d_ang > _PI)
    d_ang -= _2PI;
  while (d_ang < -_PI)
    d_ang += _2PI;

  if (fabsf(d_ang) < 0.05f) {
    Serial.println("FAILED!");
    Serial.println("ERROR: Motor failed to move. Check 12V DC power and gate "
                   "driver enable!");
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    return false;
  }

  Direction sensor_dir = (d_ang > 0.0f) ? Direction::CW : Direction::CCW;
  int dir_sign = (sensor_dir == Direction::CW) ? 1 : -1;
  Serial.print("Detected ");
  Serial.println(sensor_dir == Direction::CW ? "CW (+1)" : "CCW (-1)");

  // 2. High-Resolution Dual-Pass Spatial Error Accumulation
  Serial.println(
      "\n2. Executing Bidirectional 128-Step Sweep Across 360° Mechanical:");
  const int TOTAL_STEPS = ENCODER_LUT_SIZE;
  const float step_mech_rad = _2PI / (float)TOTAL_STEPS;

  float fwd_measured[TOTAL_STEPS];
  float rev_measured[TOTAL_STEPS];

  // PASS 1: FORWARD (CW) SWEEP
  for (int i = 0; i < TOTAL_STEPS; i++) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }

    float target_mech = (float)i * step_mech_rad;
    float target_elec =
        _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * target_mech;

    // Smooth micro-stepping transition from previous step (eliminates jerk and pole slip)
    if (i > 0) {
      const int INTERP = 16;
      float prev_mech = (float)(i - 1) * step_mech_rad;
      float prev_elec =
          _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * prev_mech;
      for (int s = 1; s <= INTERP; s++) {
        float el =
            prev_elec + (target_elec - prev_elec) * ((float)s / (float)INTERP);
        motor1.setPhaseVoltage(align_voltage, 0, el);
        delayMicroseconds(1000);
      }
    } else {
      motor1.setPhaseVoltage(align_voltage, 0, target_elec);
      delay(250);
    }

    motor1.setPhaseVoltage(align_voltage, 0, target_elec);
    delayMicroseconds(8000); // Rotor settle time

    for (int d = 8; d >= 1; d--) {
      float dither = 0.08f * ((float)d / 8.0f);
      motor1.setPhaseVoltage(align_voltage, 0, target_elec + dither);
      delayMicroseconds(500);
      motor1.setPhaseVoltage(align_voltage, 0, target_elec - dither);
      delayMicroseconds(500);
    }
    motor1.setPhaseVoltage(align_voltage, 0, target_elec);
    delay(180); // High-accuracy damp settle before sampling

    float sum_sin = 0.0f, sum_cos = 0.0f;
    for (int s = 0; s < 128; s++) { // 128 static samples for ideal noise rejection
      float a = encoder1.getRawMechanicalAngle();
      sum_sin += sinf(a);
      sum_cos += cosf(a);
      delayMicroseconds(100);
    }
    fwd_measured[i] = _normalizeAngle(atan2f(sum_sin, sum_cos));

    if ((i + 1) % 16 == 0 || i == TOTAL_STEPS - 1) {
      Serial.print("   Forward Sweep: ");
      Serial.print((int)(((float)(i + 1) / (float)TOTAL_STEPS) * 100.0f));
      Serial.println("% Complete");
    }
  }

  // --- DEDICATED THERMAL COOL-DOWN PAUSE (30s) ---
  Serial.println("\n>>> ❄️ THERMAL COOL-DOWN PAUSE (30s): De-energizing stator coils to eliminate thermal drift...");
  motor1.setPhaseVoltage(0, 0, 0);
  for (int sec = 30; sec > 0; sec--) {
    Serial.print("   Coil cool-down: ");
    Serial.print(sec);
    Serial.println("s remaining...");
    delay(1000);
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
  }
  Serial.println(">>> ❄️ Coils thermally stabilized. Resuming Reverse Sweep...\n");

  // Re-engage at end of mechanical travel before reverse sweep
  float rev_start_mech = (float)(TOTAL_STEPS - 1) * step_mech_rad;
  float rev_start_elec =
      _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * rev_start_mech;
  motor1.setPhaseVoltage(align_voltage, 0, rev_start_elec);
  delay(350);

  // PASS 2: REVERSE (CCW) SWEEP
  for (int i = TOTAL_STEPS - 1; i >= 0; i--) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }

    float target_mech = (float)i * step_mech_rad;
    float target_elec =
        _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * target_mech;

    // Smooth micro-stepping transition
    if (i < TOTAL_STEPS - 1) {
      const int INTERP = 16;
      float prev_mech = (float)(i + 1) * step_mech_rad;
      float prev_elec =
          _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * prev_mech;
      for (int s = 1; s <= INTERP; s++) {
        float el =
            prev_elec + (target_elec - prev_elec) * ((float)s / (float)INTERP);
        motor1.setPhaseVoltage(align_voltage, 0, el);
        delayMicroseconds(1000);
      }
    }

    motor1.setPhaseVoltage(align_voltage, 0, target_elec);
    delayMicroseconds(8000); // Rotor settle time

    for (int d = 8; d >= 1; d--) {
      float dither = 0.08f * ((float)d / 8.0f);
      motor1.setPhaseVoltage(align_voltage, 0, target_elec + dither);
      delayMicroseconds(500);
      motor1.setPhaseVoltage(align_voltage, 0, target_elec - dither);
      delayMicroseconds(500);
    }
    motor1.setPhaseVoltage(align_voltage, 0, target_elec);
    delay(180); // High-accuracy damp settle

    float sum_sin = 0.0f, sum_cos = 0.0f;
    for (int s = 0; s < 128; s++) { // 128 static samples
      float a = encoder1.getRawMechanicalAngle();
      sum_sin += sinf(a);
      sum_cos += cosf(a);
      delayMicroseconds(100);
    }
    rev_measured[i] = _normalizeAngle(atan2f(sum_sin, sum_cos));

    if (i % 16 == 0 || i == 0) {
      Serial.print("   Reverse Sweep: ");
      Serial.print(
          (int)(((float)(TOTAL_STEPS - i) / (float)TOTAL_STEPS) * 100.0f));
      Serial.println("% Complete");
    }
  }

  // 3. Synthesize Midpoint & Residual Error Table + Anomaly Detection
  float start_mid =
      _normalizeAngle(atan2f(sinf(fwd_measured[0]) + sinf(rev_measured[0]),
                             cosf(fwd_measured[0]) + cosf(rev_measured[0])));
  float max_err = -100.0f, min_err = 100.0f;
  float sum_sq_err = 0.0f;
  float raw_errors[ENCODER_LUT_SIZE];
  float temp_errors[ENCODER_LUT_SIZE];
  int anomaly_count = 0;

  for (int i = 0; i < TOTAL_STEPS; i++) {
    float mid_meas =
        _normalizeAngle(atan2f(sinf(fwd_measured[i]) + sinf(rev_measured[i]),
                               cosf(fwd_measured[i]) + cosf(rev_measured[i])));
    float expected_meas = _normalizeAngle(
        start_mid + (float)dir_sign * ((float)i * step_mech_rad));
    float err = mid_meas - expected_meas;
    while (err > _PI)
      err -= _2PI;
    while (err < -_PI)
      err += _2PI;

    raw_errors[i] = err;
  }

  // 3-point circular median filter to filter single-sample SPI noise glitches
  for (int i = 0; i < TOTAL_STEPS; i++) {
    int prev_i = (i - 1 + TOTAL_STEPS) % TOTAL_STEPS;
    int next_i = (i + 1) % TOTAL_STEPS;
    float a = raw_errors[prev_i];
    float b = raw_errors[i];
    float c = raw_errors[next_i];
    // Median of 3
    float med = ((a >= b && a <= c) || (a <= b && a >= c)) ? a :
                ((b >= a && b <= c) || (b <= a && b >= c)) ? b : c;
    
    // Use median if single sample deviated by > 0.08 rad from neighbors
    float filtered_err = (fabsf(b - med) > 0.08f) ? med : b;
    temp_errors[i] = filtered_err;

    if (filtered_err > max_err)
      max_err = filtered_err;
    if (filtered_err < min_err)
      min_err = filtered_err;
    sum_sq_err += filtered_err * filtered_err;

    if (fabsf(filtered_err) > LUT_ANOMALY_THRESHOLD_RAD) {
      anomaly_count++;
      if (anomaly_count <= 8) {
        Serial.print("  ⚠️  ANOMALY idx[");
        Serial.print(i);
        Serial.print("] err=");
        Serial.print(filtered_err, 4);
        Serial.print(" rad (");
        Serial.print(filtered_err * 57.2957795f, 2);
        Serial.println(" deg) — POLE SLIP?");
      }
    }
  }

  // Fourier Harmonic Extraction (1st harmonic = eccentricity, 2nd harmonic = tilt)
  float f_a1 = 0.0f, f_b1 = 0.0f, f_a2 = 0.0f, f_b2 = 0.0f;
  for (int i = 0; i < TOTAL_STEPS; i++) {
    float angle_mech = (float)i * step_mech_rad;
    f_a1 += temp_errors[i] * cosf(angle_mech);
    f_b1 += temp_errors[i] * sinf(angle_mech);
    f_a2 += temp_errors[i] * cosf(2.0f * angle_mech);
    f_b2 += temp_errors[i] * sinf(2.0f * angle_mech);
  }
  encoder1.fourier_A1 = 2.0f * f_a1 / (float)TOTAL_STEPS;
  encoder1.fourier_B1 = 2.0f * f_b1 / (float)TOTAL_STEPS;
  encoder1.fourier_A2 = 2.0f * f_a2 / (float)TOTAL_STEPS;
  encoder1.fourier_B2 = 2.0f * f_b2 / (float)TOTAL_STEPS;

  float pk_pk_deg = (max_err - min_err) * 57.2957795f;
  float rms_deg = sqrtf(sum_sq_err / (float)TOTAL_STEPS) * 57.2957795f;
  float pk_pk_rad = max_err - min_err;
  float h1_amp_deg = sqrtf(encoder1.fourier_A1 * encoder1.fourier_A1 +
                           encoder1.fourier_B1 * encoder1.fourier_B1) * 57.2957795f;
  float h2_amp_deg = sqrtf(encoder1.fourier_A2 * encoder1.fourier_A2 +
                           encoder1.fourier_B2 * encoder1.fourier_B2) * 57.2957795f;

  Serial.println("-------------------------------------------------------------"
                 "-------------------");
  Serial.println("128-POINT NON-LINEARITY CORRECTION RESULTS:");
  Serial.print("  Peak-to-Peak Mechanical Error : ");
  Serial.print(pk_pk_deg, 3);
  Serial.print(" deg (±");
  Serial.print(pk_pk_deg * 0.5f, 3);
  Serial.println(" deg)");
  Serial.print("  Electrical Error Span (11 PP) : ±");
  Serial.print(pk_pk_deg * 0.5f * (float)MOTOR_POLE_PAIRS, 2);
  Serial.println(" deg elec");
  Serial.print("  RMS Correction Error          : ");
  Serial.print(rms_deg, 3);
  Serial.println(" deg mech");
  Serial.print("  Harmonic 1 (Eccentricity)     : ±");
  Serial.print(h1_amp_deg, 3);
  Serial.println(" deg");
  Serial.print("  Harmonic 2 (Magnet Tilt)      : ±");
  Serial.print(h2_amp_deg, 3);
  Serial.println(" deg");
  Serial.print("  Anomaly Points (>");
  Serial.print(LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f, 1);
  Serial.print(" deg)     : ");
  Serial.println(anomaly_count);

  // ---- ANOMALY GATE: Reject LUT if persistent pole-slip detected ----
  bool lut_valid = (anomaly_count == 0) && (pk_pk_rad <= LUT_PKPK_LIMIT_RAD);

  if (!lut_valid) {
    Serial.println("-------------------------------------------------------------"
                   "-------------------");
    Serial.println("❌ LUT REJECTED — ANOMALIES DETECTED!");
    if (anomaly_count > 0) {
      Serial.print("   ");
      Serial.print(anomaly_count);
      Serial.println(" points exceeded ±0.25 rad threshold (pole slip or motor stall).");
    }
    if (pk_pk_rad > LUT_PKPK_LIMIT_RAD) {
      Serial.print("   Peak-to-Peak span ");
      Serial.print(pk_pk_rad, 4);
      Serial.println(" rad exceeds 0.50 rad safety limit.");
    }
    Serial.println("   HINT: Check motor is free, try higher align voltage (w 4.2)");
    Serial.println("-------------------------------------------------------------"
                   "-------------------");
    encoder1.is_lut_calibrated = false;
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    return false;
  }

  // Apply verified table
  for (int i = 0; i < TOTAL_STEPS; i++) {
    encoder1.lut_errors[i] = temp_errors[i];
  }
  encoder1.is_lut_calibrated = true;
  Serial.println("  Status                        : ✅ 128-Point LUT ACTIVE (all points valid)");
  Serial.println("-------------------------------------------------------------"
                 "-------------------");

  // Thermal cool-down pause before 11-pole electrical alignment
  Serial.println("\n>>> ❄️ THERMAL COOL-DOWN PAUSE (30s) before 11-Pole Alignment...");
  motor1.setPhaseVoltage(0, 0, 0);
  for (int sec = 30; sec > 0; sec--) {
    Serial.print("   Coil cool-down: ");
    Serial.print(sec);
    Serial.println("s remaining...");
    delay(1000);
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
  }

  // 4. Follow-up 11-pole electrical alignment on linearized sensor
  bool align_ok = calibrateComprehensiveFOC(align_voltage);
  return align_ok;
}

// ============================================================================
// CALIBRATION: FULL 360° BIDIRECTIONAL ROUND-TRIP SWEEP WITH DECAYING DITHER
// ============================================================================
bool calibrateComprehensiveFOC(float align_voltage) {
  Serial.println("\n==========================================================="
                 "=====================");
  Serial.println(
      ">>> STARTING COMPREHENSIVE MULTI-POINT ELECTRICAL ZERO CALIBRATION <<<");
  Serial.println("============================================================="
                 "===================");
  Serial.println("Target Motor       : GM3506 (11 Pole Pairs on ESP32-S3)");
  Serial.print("Calibration Voltage: ");
  Serial.print(align_voltage, 2);
  Serial.println(" V (Current ~ 0.45A)");
  Serial.println("Methodology        : 11-Pole Continuous Sweep + "
                 "Bidirectional Dither Hysteresis Cancellation");
  Serial.print("Oversampling       : ");
  Serial.print(ALIGNMENT_OVERSAMPLE_COUNT);
  Serial.println(" Samples / Point with 14-Bit MT6816 Resolution");
  Serial.print("LUT Linearization  : ");
  Serial.println(encoder1.is_lut_calibrated ? "NON-LINEARITY 128-PT [ACTIVE]"
                                            : "OFF [RAW SENSOR]");
  Serial.println("-------------------------------------------------------------"
                 "-------------------");

  motor1.controller = MotionControlType::velocity_openloop;
  driver1.enable();
  is_enabled = true;

  // 1. Determining Sensor Commutation Direction & Verifying Motion...
  Serial.print(
      "1. Determining Sensor Commutation Direction & Verifying Motion... ");
  motor1.setPhaseVoltage(align_voltage, 0, _3PI_2);
  delay(400);
  encoder1.update();
  float start_angle = encoder1.getMechanicalAngle();

  const int DIR_STEPS = 400;
  for (int step = 0; step <= DIR_STEPS; step++) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
    float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
    motor1.setPhaseVoltage(align_voltage, 0, el_ang);
    delayMicroseconds(2500);
  }
  delay(200);
  encoder1.update();
  float mid_angle = encoder1.getMechanicalAngle();

  for (int step = DIR_STEPS; step >= 0; step--) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
    float el_ang = _3PI_2 + 2.0f * _2PI * ((float)step / (float)DIR_STEPS);
    motor1.setPhaseVoltage(align_voltage, 0, el_ang);
    delayMicroseconds(2500);
  }
  delay(200);
  encoder1.update();

  float d_ang_fwd = mid_angle - start_angle;
  while (d_ang_fwd > _PI)
    d_ang_fwd -= _2PI;
  while (d_ang_fwd < -_PI)
    d_ang_fwd += _2PI;
  float moved_rad = fabsf(d_ang_fwd);
  float min_expected_rad = (2.0f * _2PI / (float)MOTOR_POLE_PAIRS) * 0.40f;

  if (moved_rad < min_expected_rad) {
    Serial.println("FAILED!");
    Serial.print("   ERROR: Motor failed to move (detected only ");
    Serial.print(moved_rad * 57.2957795f, 2);
    Serial.print(" deg, expected >= ");
    Serial.print(min_expected_rad * 57.2957795f, 2);
    Serial.println(
        " deg). Check 12V DC power, inverter wiring, or align voltage!");
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
  Serial.println("\n2. Scanning All 11 Electrical Poles (Full 360° Forward CW "
                 "+ Reverse CCW Round-Trip):");
  Serial.println("Pole # | Mech Angle | Forward (CW)  | Reverse (CCW) | True "
                 "Zero | Hysteresis | Status");
  Serial.println("-------+------------+---------------+---------------+--------"
                 "---+------------+-------");

  const int num_poles = MOTOR_POLE_PAIRS;
  float fwd_zeros[num_poles];
  float rev_zeros[num_poles];
  float fwd_mechs[num_poles];
  float rev_mechs[num_poles];
  float pole_zeros[num_poles];
  bool all_poles_ok = true;

  // PASS 1: FORWARD (CW) 360° SWEEP (Poles 0 -> 10)
  for (int p = 0; p < num_poles; p++) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
    float target_el = _3PI_2 + (float)p * _2PI;
    if (p > 0) {
      const int INTERP_STEPS = 100;
      float prev_el = _3PI_2 + (float)(p - 1) * _2PI;
      for (int s = 1; s <= INTERP_STEPS; s++) {
        float el =
            prev_el + (target_el - prev_el) * ((float)s / (float)INTERP_STEPS);
        motor1.setPhaseVoltage(align_voltage, 0, el);
        delayMicroseconds(2000);
      }
    } else {
      motor1.setPhaseVoltage(align_voltage, 0, target_el);
      delay(300);
    }

    for (int d = 12; d >= 1; d--) {
      float dither = 0.10f * ((float)d / 12.0f);
      motor1.setPhaseVoltage(align_voltage, 0, target_el + dither);
      delayMicroseconds(800);
      motor1.setPhaseVoltage(align_voltage, 0, target_el - dither);
      delayMicroseconds(800);
    }
    motor1.setPhaseVoltage(align_voltage, 0, target_el);
    delay(80);

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

  // PASS 2: REVERSE (CCW) 360° RETURN SWEEP (Poles 10 -> 0)
  for (int p = num_poles - 1; p >= 0; p--) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
    float target_el = _3PI_2 + (float)p * _2PI;
    if (p < num_poles - 1) {
      const int INTERP_STEPS = 100;
      float prev_el = _3PI_2 + (float)(p + 1) * _2PI;
      for (int s = 1; s <= INTERP_STEPS; s++) {
        float el =
            prev_el + (target_el - prev_el) * ((float)s / (float)INTERP_STEPS);
        motor1.setPhaseVoltage(align_voltage, 0, el);
        delayMicroseconds(2000);
      }
    } else {
      motor1.setPhaseVoltage(align_voltage, 0, target_el);
      delay(150);
    }

    for (int d = 12; d >= 1; d--) {
      float dither = 0.10f * ((float)d / 12.0f);
      motor1.setPhaseVoltage(align_voltage, 0, target_el + dither);
      delayMicroseconds(800);
      motor1.setPhaseVoltage(align_voltage, 0, target_el - dither);
      delayMicroseconds(800);
    }
    motor1.setPhaseVoltage(align_voltage, 0, target_el);
    delay(80);

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

  // SYNTHESIS & REPORTING
  float sum_sin = 0.0f, sum_cos = 0.0f;
  float sum_fwd_sin = 0.0f, sum_fwd_cos = 0.0f;
  float sum_rev_sin = 0.0f, sum_rev_cos = 0.0f;

  for (int p = 0; p < num_poles; p++) {
    float fwd_z = fwd_zeros[p];
    float rev_z = rev_zeros[p];
    float true_z = _normalizeAngle(
        atan2f(sinf(fwd_z) + sinf(rev_z), cosf(fwd_z) + cosf(rev_z)));
    pole_zeros[p] = true_z;
    calibrated_pole_zeros[p] = true_z;

    sum_sin += sinf(true_z);
    sum_cos += cosf(true_z);
    sum_fwd_sin += sinf(fwd_z);
    sum_fwd_cos += cosf(fwd_z);
    sum_rev_sin += sinf(rev_z);
    sum_rev_cos += cosf(rev_z);

    float hyst_diff = fwd_z - rev_z;
    while (hyst_diff > _PI)
      hyst_diff -= _2PI;
    while (hyst_diff < -_PI)
      hyst_diff += _2PI;
    float hyst_deg = fabsf(hyst_diff) * 57.2957795f;

    bool pole_pass = (hyst_deg <= 16.0f);
    if (!pole_pass)
      all_poles_ok = false;

    float pole_mech_target = ((float)p / (float)num_poles) * 360.0f;

    Serial.print("  #");
    Serial.print(p + 1);
    if (p + 1 < 10)
      Serial.print(" ");
    Serial.print("  | ");
    if (pole_mech_target < 100.0f)
      Serial.print(" ");
    if (pole_mech_target < 10.0f)
      Serial.print(" ");
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
    Serial.println(pole_pass ? "[PASS]" : "[WARN-HYST]");
  }
  Serial.println("-------------------------------------------------------------"
                 "-------------------");

  float mean_zero_angle = _normalizeAngle(
      atan2f(sum_sin / (float)num_poles, sum_cos / (float)num_poles));
  float mean_fwd_angle = _normalizeAngle(
      atan2f(sum_fwd_sin / (float)num_poles, sum_fwd_cos / (float)num_poles));
  float mean_rev_angle = _normalizeAngle(
      atan2f(sum_rev_sin / (float)num_poles, sum_rev_cos / (float)num_poles));

  float global_hyst = mean_fwd_angle - mean_rev_angle;
  while (global_hyst > _PI)
    global_hyst -= _2PI;
  while (global_hyst < -_PI)
    global_hyst += _2PI;
  float global_hyst_deg = fabsf(global_hyst) * 57.2957795f;

  float var_sum = 0.0f;
  for (int p = 0; p < num_poles; p++) {
    float diff = pole_zeros[p] - mean_zero_angle;
    while (diff > _PI)
      diff -= _2PI;
    while (diff < -_PI)
      diff += _2PI;
    var_sum += diff * diff;
  }
  float std_dev_rad = sqrtf(var_sum / (float)num_poles);
  float std_dev_deg = std_dev_rad * 57.2957795f;

  bool overall_pass = (std_dev_deg <= 25.0f);

  Serial.println("\n----------------- 🏆 CERTIFIED ELECTRICAL ALIGNMENT "
                 "RESULTS -----------------");
  Serial.print("  ⭐ Mean Zero Electric Angle: ");
  Serial.print(mean_zero_angle, 5);
  Serial.print(" rad (");
  Serial.print(mean_zero_angle * 57.2957795f, 2);
  Serial.println(" deg elec)");
  Serial.print("  Commutation Direction       : ");
  Serial.println(sensor_dir == Direction::CW ? "CW (+1)" : "CCW (-1)");
  Serial.print("  Directional Hysteresis Lead : ±");
  Serial.print(global_hyst_deg * 0.5f, 2);
  Serial.print(" deg (CW: ");
  Serial.print(mean_fwd_angle, 4);
  Serial.print(" rad | CCW: ");
  Serial.print(mean_rev_angle, 4);
  Serial.println(" rad)");
  Serial.print("  Pole Alignment Consistency  : ±");
  Serial.print(std_dev_deg, 2);
  Serial.print(" deg elec (±");
  Serial.print(std_dev_rad, 4);
  Serial.println(" rad)");
  Serial.println("  11-Pole Individual Table    : ACTIVE (Interpolated Zero "
                 "Vector eliminates inter-pole dispersion)");
  Serial.print("  Alignment Quality Score     : ");

  if (!overall_pass) {
    Serial.println(
        "❌ GRADE F (FAILED: Severe pole misalignment or motor stall!)");
    Serial.println("  "
                   "-----------------------------------------------------------"
                   "-------------------");
    Serial.println("  ❌ CALIBRATION REJECTED: Consistency threshold exceeded "
                   "(> 25.0 deg elec) or stall detected.");
    Serial.println("==========================================================="
                   "=====================\n");
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    is_calibrated = false;
    return false;
  }

  if (std_dev_deg < 4.0f && all_poles_ok)
    Serial.println(
        "⭐ GRADE A+ (Ultra-High Precision < 4.0 deg elec / ~0.35 deg mech)");
  else if (std_dev_deg < 12.0f)
    Serial.println(
        "✅ GRADE A (Excellent Precision < 12.0 deg elec / ~1.0 deg mech)");
  else if (all_poles_ok)
    Serial.println("⚠️ GRADE B (Acceptable, minor eccentricity/friction)");
  else
    Serial.println("⚠️ GRADE C (Acceptable, minor localized bearing stiction)");

  Serial.println("  "
                 "-------------------------------------------------------------"
                 "-----------------");
  Serial.println("  SimpleFOC Static Code Parameters:");
  Serial.print("    motor.zero_electric_angle = ");
  Serial.print(mean_zero_angle, 5);
  Serial.println("f;");
  Serial.print("    motor.sensor_direction    = Direction::");
  Serial.print(sensor_dir == Direction::CW ? "CW" : "CCW");
  Serial.println(";");
  Serial.println("============================================================="
                 "===================\n");

  base_zero_electric_angle = mean_zero_angle;
  motor1.zero_electric_angle = mean_zero_angle;
  motor1.sensor_direction = sensor_dir;
  motor1.controller = MotionControlType::velocity;
  target_velocity_rpm = 0.0f;
  target_voltage = 0.0f;
  motor1.setPhaseVoltage(0, 0, 0);
  driver1.disable();
  is_enabled = false;
  is_calibrated = true;
  return true;
}

// ============================================================================
// AUTOMATED CLOSED-LOOP KV RATING FINDER
// ============================================================================
void startKVTest() {
  if (!is_calibrated) {
    Serial.println("ERROR: Motor must be calibrated with 'c' or 'w' before "
                   "running KV Finder!");
    return;
  }
  kv_test_running = true;
  kv_stage_index = 0;
  kv_stage_start_ms = millis();
  stage_sample_count = 0;
  fwd_count = 0;
  rev_count = 0;

  motor1.controller = MotionControlType::torque;
  driver1.enable();
  is_enabled = true;

  Serial.println("[KV_START]");
  Serial.println("============================================================="
                 "===================");
  Serial.println(
      ">>> STARTING HIGH-PRECISION CLOSED-LOOP MOTOR KV ANALYZER <<<");
  Serial.println("============================================================="
                 "===================");
}

void abortKVTest(const char *reason) {
  if (!kv_test_running)
    return;
  kv_test_running = false;
  motor1.move(0.0f);
  motor1.setPhaseVoltage(0, 0, 0);
  driver1.disable();
  is_enabled = false;
  Serial.print("KV TEST ABORTED: ");
  Serial.println(reason);
}

void finalizeKVTest() {
  kv_test_running = false;
  motor1.move(0.0f);
  motor1.setPhaseVoltage(0, 0, 0);
  driver1.disable();
  is_enabled = false;

  float fwd_kv = 0, fwd_v0 = 0, fwd_r2 = 0;
  float rev_kv = 0, rev_v0 = 0, rev_r2 = 0;
  bool fwd_ok = calculateLinearRegression(fwd_v, fwd_rpm, fwd_count, fwd_kv,
                                          fwd_v0, fwd_r2);
  bool rev_ok = calculateLinearRegression(rev_v, rev_rpm, rev_count, rev_kv,
                                          rev_v0, rev_r2);

  float true_kv = 0.0f, friction_v0 = 0.0f, best_r2 = 0.0f;
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
  float symmetry_pct = (fwd_kv > 1.0f && rev_kv > 1.0f)
                           ? (1.0f - fabsf(fwd_kv - rev_kv) / true_kv) * 100.0f
                           : 100.0f;

  motor_kv_rating = true_kv;

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
  if (!kv_test_running)
    return;

  uint32_t elapsed = millis() - kv_stage_start_ms;
  float current_target_v = kv_voltages[kv_stage_index];

  // Core 0 FOC task handles the actual commutation at this target voltage
  target_voltage = current_target_v;

  if (elapsed < KV_DWELL_TIME_MS) {
    stage_sample_count = 0;
  } else if (elapsed < (KV_DWELL_TIME_MS + KV_SAMPLE_TIME_MS)) {
    uint32_t now_us = micros();
    if (now_us - stage_last_sample_us >= KV_SAMPLE_INTERVAL_US) {
      stage_last_sample_us = now_us;
      if (stage_sample_count < STAGE_SAMPLE_CAP) {
        float rpm = g_telemetry.actual_rpm;
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
        if (s < min_spd)
          min_spd = s;
        if (s > max_spd)
          max_spd = s;
      }
      float mean_rpm = sum / (float)stage_sample_count;
      float ripple = max_spd - min_spd;

      float var_sum = 0.0f;
      for (int i = 0; i < stage_sample_count; i++) {
        float d = stage_speed_samples[i] - mean_rpm;
        var_sum += d * d;
      }
      float std_dev = sqrtf(var_sum / (float)stage_sample_count);

      if (current_target_v > 0.0f && fwd_count < 10) {
        fwd_v[fwd_count] = current_target_v;
        fwd_rpm[fwd_count] = mean_rpm;
        fwd_count++;
      } else if (current_target_v < 0.0f && rev_count < 10) {
        rev_v[rev_count] = fabsf(current_target_v);
        rev_rpm[rev_count] = fabsf(mean_rpm);
        rev_count++;
      }

      Serial.print("[KV_POINT]: Stage=");
      Serial.print(kv_stage_index + 1);
      Serial.print("/");
      Serial.print(NUM_KV_STAGES);
      Serial.print(", V=");
      Serial.print(current_target_v, 2);
      Serial.print(", RPM=");
      Serial.print(mean_rpm, 2);
      Serial.print(", Ripple=");
      Serial.print(ripple, 2);
      Serial.print(", StdDev=");
      Serial.println(std_dev, 2);
    }

    kv_stage_index++;
    if (kv_stage_index >= NUM_KV_STAGES) {
      finalizeKVTest();
    } else {
      kv_stage_start_ms = millis();
      stage_sample_count = 0;
    }
  }
}

// ============================================================================
// SERIAL COMMAND PROCESSOR (NON-BLOCKING WITH IMMEDIATE E-STOP)
// ============================================================================
void executeCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0)
    return;

  char action = cmd.charAt(0);
  String arg = cmd.substring(1);
  arg.trim();

  if (cmd.equalsIgnoreCase("save")) {
    saveConfigToNVS();
    return;
  }
  if (cmd.equalsIgnoreCase("load")) {
    loadConfigFromNVS();
    return;
  }
  if (cmd.equalsIgnoreCase("export") || cmd.equalsIgnoreCase("dump_cfg")) {
    dumpConfigJSON();
    return;
  }

  switch (action) {
  case 'w':
  case 'W': {
    float v = arg.toFloat();
    if (v <= 0.0f)
      v = calib_align_voltage;
    if (kv_test_running)
      abortKVTest("Interrupted by LUT Calibration");
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(15));
    calibrate256PointLUT(v);
    foc_paused = false;
    break;
  }
  case 'c':
  case 'C': {
    float v = arg.toFloat();
    if (v <= 0.0f)
      v = calib_align_voltage;
    if (kv_test_running)
      abortKVTest("Interrupted by Calibration");
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(15));
    calibrateComprehensiveFOC(v);
    foc_paused = false;
    break;
  }
  case 'n':
  case 'N': {
    float r = arg.toFloat();
    if (r <= 0.0f)
      r = 1000.0f;
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(15));
    runSpeedLagOptimizer(r);
    foc_paused = false;
    break;
  }
  case 's':
  case 'S': {
    float val = arg.toFloat();
    if (val >= 0.0f) {
      sensor_lag_comp_us = val;
      Serial.print(">>> Updated Dynamic Sensor Lag Compensation: ");
      Serial.print(sensor_lag_comp_us, 1);
      Serial.println(" us");
    }
    break;
  }
  case 't':
  case 'T': {
    float val = arg.toFloat();
    if (kv_test_running)
      abortKVTest("Interrupted by Manual Speed Command");
    target_velocity_rpm = val;
    motor1.controller = MotionControlType::velocity;
    driver1.enable();
    is_enabled = true;
    Serial.print(">>> Updated Target Speed: ");
    Serial.print(target_velocity_rpm, 1);
    Serial.println(" RPM");
    break;
  }
  case 'u':
  case 'U': {
    if (!encoder1.is_lut_calibrated) {
      Serial.println("[LUT_DUMP] No active LUT — run 'w' first.");
      break;
    }
    Serial.println(
        "\n// --- 128-POINT NON-LINEARITY CORRECTION LUT (RADIANS) ---");
    Serial.print("const float PROGMEM encoder_lut_128[128] = {");
    for (int i = 0; i < ENCODER_LUT_SIZE; i++) {
      if (i % 8 == 0)
        Serial.print("\n  ");
      Serial.print(encoder1.lut_errors[i], 6);
      Serial.print("f");
      if (i < ENCODER_LUT_SIZE - 1)
        Serial.print(", ");
    }
    Serial.println("\n};\n");
    break;
  }
  case 'v':
  case 'V': {
    float val = arg.toFloat();
    if (kv_test_running)
      abortKVTest("Interrupted by Manual Voltage Command");
    target_voltage = val;
    motor1.controller = MotionControlType::torque;
    driver1.enable();
    is_enabled = true;
    Serial.print(">>> Target Phase Voltage Set: ");
    Serial.print(target_voltage, 2);
    Serial.println(" V");
    break;
  }
  case 'k':
  case 'K': {
    startKVTest();
    break;
  }
  case 'p':
  case 'P': {
    float val = arg.toFloat();
    if (val >= 0.0f) {
      motor1.PID_velocity.P = val;
      Serial.print(">>> Updated Velocity P: ");
      Serial.println(motor1.PID_velocity.P, 4);
    }
    break;
  }
  case 'i':
  case 'I': {
    float val = arg.toFloat();
    if (val >= 0.0f) {
      motor1.PID_velocity.I = val;
      Serial.print(">>> Updated Velocity I: ");
      Serial.println(motor1.PID_velocity.I, 4);
    }
    break;
  }
  case 'd':
  case 'D': {
    float val = arg.toFloat();
    if (val >= 0.0f) {
      motor1.PID_velocity.D = val;
      Serial.print(">>> Updated Velocity D: ");
      Serial.println(motor1.PID_velocity.D, 5);
    }
    break;
  }
  case 'f':
  case 'F': {
    float val = arg.toFloat();
    if (val > 0.0f) {
      motor1.LPF_velocity.Tf = val;
      Serial.print(">>> Updated Velocity LPF Tf: ");
      Serial.print(motor1.LPF_velocity.Tf, 4);
      Serial.println(" s");
    }
    break;
  }
  case 'a':
  case 'A': {
    float val = arg.toFloat();
    if (val >= 0.0f) {
      motor1.PID_velocity.output_ramp = val;
      Serial.print(">>> Updated Output Voltage Ramp Rate: ");
      Serial.print(motor1.PID_velocity.output_ramp, 1);
      Serial.println(" V/s");
    }
    break;
  }
  case 'l':
  case 'L': {
    float val = arg.toFloat();
    if (val > 0.0f) {
      motor1.voltage_limit = val;
      motor1.PID_velocity.limit = val;
      Serial.print(">>> Updated Voltage Limit: ");
      Serial.print(motor1.voltage_limit, 2);
      Serial.println(" V");
    }
    break;
  }
  case 'm':
  case 'M': {
    float val = arg.toFloat();
    if (val > 0.0f) {
      motor1.velocity_limit = val * 0.104719755f;
      Serial.print(">>> Updated Velocity Limit: ");
      Serial.print(val, 1);
      Serial.println(" RPM");
    }
    break;
  }
  case 'q':
  case 'Q': {
    float val = arg.toFloat();
    if (val > 0.0f) {
      motor_kv_rating = val;
      Serial.print(">>> Updated Motor KV Rating: ");
      Serial.print(motor_kv_rating, 1);
      Serial.println(" RPM/V");
    }
    break;
  }
  case 'y':
  case 'Y': {
    float val = arg.toFloat();
    if (val >= 1.0f && val <= 12.0f) {
      calib_align_voltage = val;
      Serial.print(">>> Updated Calibration / Alignment Voltage: ");
      Serial.print(calib_align_voltage, 2);
      Serial.println(" V");
    }
    break;
  }
  case 'j':
  case 'J': {
    float val = arg.toFloat();
    base_zero_electric_angle = _normalizeAngle(val);
    motor1.zero_electric_angle = base_zero_electric_angle;
    use_pole_specific_zeros = false;
    is_calibrated = true;
    Serial.print(">>> Updated Base Electrical Zero Angle: ");
    Serial.print(base_zero_electric_angle, 5);
    Serial.print(" rad (");
    Serial.print(base_zero_electric_angle * 57.2957795f, 2);
    Serial.println(" deg)");
    break;
  }
  case 'z':
  case 'Z': {
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(10));
    encoder1.zeroCurrentPosition();
    foc_paused = false;
    Serial.println(">>> ZERO / TARE ANGLE SET! <<<");
    break;
  }
  case 'e':
  case 'E': {
    is_enabled = !is_enabled;
    if (is_enabled) {
      driver1.enable();
      Serial.println(">>> Driver ENABLED");
    } else {
      motor1.setPhaseVoltage(0, 0, 0);
      driver1.disable();
      Serial.println(">>> Driver DISABLED");
    }
    break;
  }
  case 'x':
  case 'X': {
    if (kv_test_running)
      abortKVTest("Emergency Stop");
    target_velocity_rpm = 0.0f;
    target_voltage = 0.0f;
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    Serial.println(">>> EMERGENCY STOP: Driver Disabled");
    break;
  }
  case 'g':
  case 'G': {
    show_telemetry = !show_telemetry;
    Serial.print(">>> Telemetry stream: ");
    Serial.println(show_telemetry ? "ON" : "OFF");
    break;
  }
  case 'b':
  case 'B': {
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(15));
    encoder1.setMode(!encoder1.use_3wire);
    foc_paused = false;
    Serial.print(">>> Encoder Bus Mode: ");
    Serial.println(encoder1.use_3wire ? "3-Wire Half-Duplex"
                                      : "4-Wire Full Hardware SPI");
    break;
  }
  case 'r':
  case 'R': {
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t h = 0, l = 0;
    uint16_t raw14 = 0;
    bool no_mag = false, parity_ok = false;
    encoder1.readCurrent(h, l, raw14, no_mag, parity_ok);
    foc_paused = false;
    Serial.print(">>> RAW SENSOR DIAGNOSTIC: H=0x");
    Serial.print(h, HEX);
    Serial.print(" L=0x");
    Serial.print(l, HEX);
    Serial.print(" (Raw14=");
    Serial.print(raw14);
    Serial.print("/16383, Angle=");
    Serial.print(((float)raw14 / 16384.0f) * 360.0f, 2);
    Serial.print(" deg, Mag=");
    Serial.print(no_mag ? "[WEAK/NO MAG]" : "[OK]");
    Serial.print(", Parity=");
    Serial.print(parity_ok ? "[PASS]" : "[FAIL]");
    Serial.println(")");
    break;
  }
  case 'h':
  case 'H':
  case '?': {
    Serial.println("\n==================================================");
    Serial.println("  DA WINNA — ESP32-S3 MOTOR TEST BENCH COMMANDS   ");
    Serial.println("==================================================");
    Serial.println("  w [volts]     : 🔬 Master Fourier Calibration + 11-Pole "
                   "Continuous Zero");
    Serial.println(
        "  n [rpm]       : ⚡ 1000 RPM Dynamic Current Minimization Auto-Trim");
    Serial.println("  c [volts]     : Run Standard 11-Pole 360° Alignment");
    Serial.println(
        "  save          : 💾 Save Calibrated Profile to ESP32 Flash (NVS)");
    Serial.println(
        "  load          : 📂 Load Calibrated Profile from ESP32 Flash (NVS)");
    Serial.println("  export        : 📋 Dump Full JSON Configuration String");
    Serial.println(
        "  u             : Dump 128-Point LUT Table to Serial");
    Serial.println("  t <rpm>       : Set Closed-Loop Target Velocity (RPM)");
    Serial.println("  v <volts>     : Set Direct Phase Voltage Uq (V)");
    Serial.println("  k             : Run Automated Closed-Loop KV Analyzer");
    Serial.println("  p, i, d <val> : Set Velocity PID Gains");
    Serial.println("  f <sec>       : Set Velocity Filter Tf (s)");
    Serial.println(
        "  s <us>        : Set Dynamic Sensor Lag Compensation (us)");
    Serial.println("  a <v/s>       : Set Output Ramp Rate (V/s)");
    Serial.println("  l <volts>     : Set Phase Voltage Limit (V)");
    Serial.println("  m <rpm>       : Set Maximum Velocity Limit (RPM)");
    Serial.println("  q <rpm/v>     : Set Motor KV Rating");
    Serial.println("  y <volts>     : Set Calibration Alignment Voltage (V)");
    Serial.println("  j <rad>       : Override Zero Electric Angle directly");
    Serial.println("  z             : Zero / Tare Mechanical Angle");
    Serial.println("  b             : Toggle 4-Wire / 3-Wire Bus Mode");
    Serial.println("  r             : Single Raw SPI Diagnostic Read");
    Serial.println("  e             : Toggle Driver Enable/Disable");
    Serial.println("  x             : Emergency Stop & Disable Driver");
    Serial.println("  g             : Toggle Telemetry Streaming ON/OFF");
    Serial.println("==================================================\n");
    break;
  }
  default:
    break;
  }
}

static char cmd_line_buf[64];
static size_t cmd_line_len = 0;

void processSerialCommand() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'x' || c == 'X') {
      executeCommand("x");
      cmd_line_len = 0;
      continue;
    }
    if (c == '\n' || c == '\r') {
      if (cmd_line_len > 0) {
        cmd_line_buf[cmd_line_len] = '\0';
        executeCommand(String(cmd_line_buf));
        cmd_line_len = 0;
      }
    } else if (cmd_line_len < sizeof(cmd_line_buf) - 1) {
      cmd_line_buf[cmd_line_len++] = c;
    }
  }
}

// ============================================================================
// ARDUINO SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(921600);
  Serial.setTimeout(10);
  delay(500);

  Serial.println("\n==================================================");
  Serial.println("  DA WINNA — ESP32-S3 MOTOR 1 TEST BENCH     ");
  Serial.println("==================================================");
  Serial.println("  Hardware: ESP32-S3 (240MHz) | Driver: GPIO 15,16,17,2 | "
                 "Sensor: MT6816 SPI (GPIO 12,11,13,10)");

  // Initialize Inverter Driver
  driver1.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  driver1.init();

  // Initialize MT6816 SPI Sensor
  encoder1.init();
  motor1.linkSensor(&encoder1);

  // Link Driver & Configure SimpleFOC Motor
  motor1.linkDriver(&driver1);
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::velocity;

  motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT;
  motor1.velocity_limit = (DEFAULT_VELOCITY_LIMIT_RPM * 0.104719755f);
  motor1.PID_velocity.P = DEFAULT_PID_P;
  motor1.PID_velocity.I = DEFAULT_PID_I;
  motor1.PID_velocity.D = DEFAULT_PID_D;
  motor1.PID_velocity.output_ramp = DEFAULT_OUTPUT_RAMP_V_PER_S;
  motor1.LPF_velocity.Tf = DEFAULT_LPF_TF_SEC;

  motor1.init();

  motor1.zero_electric_angle = base_zero_electric_angle;
  motor1.sensor_direction = Direction::CCW;

  driver1.disable();
  is_enabled = false;

  // Attempt Auto-Loading from Flash
  loadConfigFromNVS();

  // Launch Dedicated Real-Time FOC Task on Core 0 (Priority 22)
  xTaskCreatePinnedToCore(
      focTaskLoop,
      "FOC_Task",
      8192,
      NULL,
      22,
      &focTaskHandle,
      0
  );

  Serial.println("  System Ready. FOC Loop running on Core 0 @ high rate.");
  Serial.println("  Type 'w' for Master Fourier Calibration, 'c' for Fast Align, 'n' for Current Trim.");
  Serial.println("==================================================\n");
}

void loop() {
  // 1. Process KV State Machine (Timing & Stage Transitions on Core 1)
  if (kv_test_running) {
    processKVStateMachine();
  }

  // 2. Non-blocking Serial Command Processor
  processSerialCommand();

  // 3. Periodic Telemetry Stream (20 Hz)
  uint32_t now_ms = millis();
  if (show_telemetry &&
      (now_ms - last_telemetry_time_ms >= TELEMETRY_STREAM_INTERVAL_MS)) {
    last_telemetry_time_ms = now_ms;

    // Snapshot from Core 0
    TelemetrySnapshot snap;
    portENTER_CRITICAL(&telemSpinlock);
    snap = g_telemetry;
    portEXIT_CRITICAL(&telemSpinlock);

    const char *ctrl_str = "DISBL";
    if (snap.is_enabled) {
      if (snap.kv_test)
        ctrl_str = "KV-TEST";
      else if (snap.ctrl_mode == MotionControlType::velocity_openloop)
        ctrl_str = "OPEN";
      else if (snap.ctrl_mode == MotionControlType::torque)
        ctrl_str = "VOLT";
      else
        ctrl_str = "CLOSE";
    }

    const char *mag_str = "[ OK ]";
    if (snap.raw_h == 0xFF && snap.raw_l == 0xFF) {
      mag_str = "[ NO SENSOR ]";
    } else if (snap.no_mag) {
      mag_str = "[ WEAK/NO MAG ]";
    }

    const char *parity_str = snap.parity_ok ? "[PASS]" : "[FAIL]";

    const char *dial_segments[] = {"[  |  ]", "[ /   ]", "[--   ]", "[ \\   ]",
                                   "[  |  ]", "[   / ]", "[   --]", "[   \\ ]"};
    int seg = (int)((snap.angle_deg + 22.5f) / 45.0f) % 8;
    if (seg < 0)
      seg = 0;
    if (seg > 7)
      seg = 7;

    char line_buf[256];
    char target_buf[32];
    if (snap.kv_test || snap.ctrl_mode == MotionControlType::torque) {
      snprintf(target_buf, sizeof(target_buf), "%+5.2f V ", snap.target_voltage);
    } else {
      snprintf(target_buf, sizeof(target_buf), "%+6.1f RPM", snap.target_rpm);
    }

    char mode_buf[32];
    if (encoder1.is_lut_calibrated) {
      snprintf(mode_buf, sizeof(mode_buf), "%s(FOURIER:128)",
               encoder1.use_3wire ? "3-Wire" : "4-Wire");
    } else {
      snprintf(mode_buf, sizeof(mode_buf), "%s",
               encoder1.use_3wire ? "3-Wire" : "4-Wire");
    }

    snprintf(
        line_buf, sizeof(line_buf),
        "Ctrl: %-5s | Target: %s | Actual: %+6.1f RPM | Angle: %6.2f deg | "
        "Uq: %+5.2fV | Ud: %+5.2fV | Raw: %5u/16383 | Mode: %s | Mag: %s | "
        "Parity: %s %s | FOC: %.1fkHz\r\n",
        ctrl_str, target_buf, snap.actual_rpm, snap.angle_deg, snap.uq, snap.ud,
        snap.raw14, mode_buf, mag_str, parity_str, dial_segments[seg],
        snap.foc_hz * 0.001f);

    Serial.write((const uint8_t *)line_buf, strlen(line_buf));
  }

  // Yield to FreeRTOS on Core 1
  delay(1);
}
