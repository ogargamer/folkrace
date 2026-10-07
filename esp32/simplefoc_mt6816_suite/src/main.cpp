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

#include "esp_cpu.h"
#include "esp_task_wdt.h"
#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <SimpleFOC.h>

// 1. MOTOR & HARDWARE CONFIGURATION
#define MOTOR_POLE_PAIRS 11              // GM3506 (24N22P = 11 Pole Pairs)
#define MOTOR_PHASE_RESISTANCE_OHM 5.60f // Phase-to-Phase / 2 = 5.6 Ohm
#define POWER_SUPPLY_VOLTAGE 12.0f       // 12V DC Regulated Bench Supply
#define CALIBRATION_VOLTAGE                                                    \
  4.00f // Default Open-Loop Calibration Step Voltage (4.0V ~ 0.65A)
#define CALIBRATION_HOLD_VOLTAGE                                               \
  2.40f // Thermal Relief Hold Voltage during Settle & Sample for LUT (2.4V)
#define CALIBRATION_BOOST_VOLTAGE                                              \
  4.20f // Automatic Boost Voltage on Stall / Pole-Slip Detection (4.2V)

// 2. BUS & PROTOCOL SETTINGS
#define SPI_SPEED_DEFAULT_HZ                                                   \
  4000000 // MT6816 SPI Master Baud (4 MHz with 15cm 22R termination)
#define SPI_USE_3WIRE_DEFAULT false // Default 4-Wire Hardware SPI
#define SENSOR_LAG_COMP_DEFAULT_US                                             \
  35.0f // Default Dynamic Lag Lead Compensation (us)
#define DEFAULT_LPF_TF_MS                                                      \
  3.0f // Velocity Low-Pass Filter Tf in ms (3.0 ms gives razor-sharp response)
#define DEFAULT_LPF_TF_SEC (DEFAULT_LPF_TF_MS * 0.001f)

// 3. 128-POINT ENCODER NON-LINEARITY LUT CONSTANTS
#define ENCODER_LUT_SIZE 128 // 128-point calibration table
// Maximum allowed single-point correction before pole-slip is declared:
#define LUT_ANOMALY_THRESHOLD_RAD                                              \
  0.035f // > 2.0 deg mech (~22 deg elec) = pole slip / stall
// Maximum allowed peak-to-peak span across the whole table:
#define LUT_PKPK_LIMIT_RAD                                                     \
  0.06f // > 3.44 deg mech (~38 deg elec) = bad sweep / binding
#define CALIB_SWEEP_STEPS                                                      \
  256 // Number of micro-steps across 360 mechanical degrees
#define CALIB_SWEEP_DAMP_SETTLE_MS                                             \
  65 // High-accuracy damp settle before sampling (65 ms for conservative
     // margin)
#define CALIB_SWEEP_OVERSAMPLE_COUNT                                           \
  96 // Static samples per sweep point for noise rejection
#define ALIGNMENT_OVERSAMPLE_COUNT                                             \
  96 // Samples per pole point during electrical zero alignment
#define CALIB_COOLDOWN_PAUSE_SEC 5 // Thermal relief cooldown duration between calibration passes (seconds)

// 4. TIMING, LOOP RATES & SAMPLING FREQUENCIES
#define TELEMETRY_STREAM_INTERVAL_MS                                           \
  50 // Serial Live Streaming Interval (50ms = 20 Hz)

// 5. REAL-TIME TUNABLE DEFAULT FOC PARAMETERS
#define DEFAULT_VOLTAGE_LIMIT                                                  \
  (POWER_SUPPLY_VOLTAGE / 1.7320508f) // 6.93V Maximum linear phase voltage (V) for SpaceVectorPWM
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

  // Mode Selector: 0 = Raw, 1 = 128-Point Stepped LUT, 2 = Continuous Analytical Harmonics
  uint8_t correction_mode;

  // Active Harmonic Coefficients (Analytical runtime reconstruction)
  float fourier_A1, fourier_B1; // 1st Harmonic: Eccentricity / Runout
  float fourier_A2, fourier_B2; // 2nd Harmonic: Magnet Tilt / Ellipticity
  float fourier_A3, fourier_B3; // 3rd Harmonic: 3-Point Mechanical Clamping Distortion (Diagnostic)
  float fourier_A4, fourier_B4; // 4th Harmonic: Hall Bridge Mismatch & Flux Saturation
  float lut_errors[ENCODER_LUT_SIZE];
  bool is_lut_calibrated;

  // Harmonic Fit & Physical Diagnostic Metrics
  float harmonic_r_squared;
  float harmonic_pk_pk_deg;
  float harmonic_rms_deg;
  float harmonic_res_pk_pk_deg;
  float harmonic_res_rms_deg;
  float harmonic_mean_lag_deg;
  float harmonic_lag_ripple_deg;
  float harmonic_max_drag_angle_deg;

  // Diagnostics & Tracking
  uint32_t total_reads;
  uint32_t parity_errors;
  uint32_t parity_retries;
  uint32_t mag_faults;
  bool last_no_mag;
  bool last_parity_ok;
  uint8_t last_raw_h;
  uint8_t last_raw_l;
  uint16_t last_raw_14bit;
  float zero_offset_deg;

  MT6816Sensor()
      : cs_pin(PIN_CS_M1), spi_speed(SPI_SPEED_DEFAULT_HZ), spi_mode(SPI_MODE3),
        use_3wire(SPI_USE_3WIRE_DEFAULT), correction_mode(0),
        fourier_A1(0.0f), fourier_B1(0.0f),
        fourier_A2(0.0f), fourier_B2(0.0f),
        fourier_A3(0.0f), fourier_B3(0.0f),
        fourier_A4(0.0f), fourier_B4(0.0f),
        is_lut_calibrated(false),
        harmonic_r_squared(0.0f), harmonic_pk_pk_deg(0.0f), harmonic_rms_deg(0.0f),
        harmonic_res_pk_pk_deg(0.0f), harmonic_res_rms_deg(0.0f),
        harmonic_mean_lag_deg(0.0f), harmonic_lag_ripple_deg(0.0f),
        harmonic_max_drag_angle_deg(0.0f),
        total_reads(0), parity_errors(0), parity_retries(0), mag_faults(0), last_no_mag(false),
        last_parity_ok(true), last_raw_h(0), last_raw_l(0), last_raw_14bit(0),
        zero_offset_deg(0.0f) {
    memset(lut_errors, 0, sizeof(lut_errors));
  }

  void setMode(bool three_wire) {
    if (!use_3wire) {
      SPI.endTransaction();
    }
    use_3wire = three_wire;
    if (use_3wire) {
      SPI.end();
      pinMode(PIN_SCK, OUTPUT);
      digitalWrite(PIN_SCK, HIGH);
      pinMode(PIN_MOSI, INPUT_PULLUP);
    } else {
      SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
      pinMode(PIN_MISO, INPUT_PULLUP);
      SPISettings settings(spi_speed, MSBFIRST, spi_mode);
      SPI.beginTransaction(settings);
    }
  }

  void setSPISpeed(uint32_t speed_hz) {
    spi_speed = speed_hz;
    if (!use_3wire) {
      SPI.endTransaction();
      SPISettings settings(spi_speed, MSBFIRST, spi_mode);
      SPI.beginTransaction(settings);
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
    digitalWrite(target_cs, LOW);
    uint16_t rx_h = SPI.transfer16(0x8300);
    digitalWrite(target_cs, HIGH);

    delayMicroseconds(1); // CS deassertion hold time (t_CS_HIGH >= 150ns)

    digitalWrite(target_cs, LOW);
    uint16_t rx_l = SPI.transfer16(0x8400);
    digitalWrite(target_cs, HIGH);

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

      if (parity_ok) {
        if (retry > 0)
          parity_retries++;
        break;
      }
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

    // Mode 0: Raw (Uncalibrated)
    if (correction_mode == 0) {
      return raw_rad;
    }

    // Mode 1: 128-Point Stepped LUT (Zero-mean piecewise linear interpolation)
    if (correction_mode == 1) {
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
      while (calib_rad < 0.0f)
        calib_rad += _2PI;
      while (calib_rad >= _2PI)
        calib_rad -= _2PI;
      return calib_rad;
    }

    // Mode 2: Continuous Analytical Harmonics (Zero-mean trigonometric evaluation via double-angle identities)
    if (correction_mode == 2) {
      float sin1 = sinf(raw_rad);
      float cos1 = cosf(raw_rad);

      // 2nd Harmonic: double angle products
      float sin2 = 2.0f * sin1 * cos1;
      float cos2 = cos1 * cos1 - sin1 * sin1;

      // 4th Harmonic: double angle of 2nd harmonic
      float sin4 = 2.0f * sin2 * cos2;
      float cos4 = cos2 * cos2 - sin2 * sin2;

      // Zero-mean analytical correction (H1 + H2 + H4)
      float correction = (fourier_A1 * cos1 + fourier_B1 * sin1) +
                         (fourier_A2 * cos2 + fourier_B2 * sin2) +
                         (fourier_A4 * cos4 + fourier_B4 * sin4);

      float calib_rad = raw_rad - correction;
      while (calib_rad < 0.0f)
        calib_rad += _2PI;
      while (calib_rad >= _2PI)
        calib_rad -= _2PI;
      return calib_rad;
    }

    return raw_rad;
  }

  float getSingleTurnDeg() {
    float rad = angle_prev;
    float d = (rad / _2PI) * 360.0f - zero_offset_deg;
    while (d < 0.0f)
      d += 360.0f;
    while (d >= 360.0f)
      d -= 360.0f;
    return d;
  }

  void zeroCurrentPosition() {
    float rad = getMechanicalAngle();
    zero_offset_deg = (rad / _2PI) * 360.0f;
    full_rotations = 0;
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
bool ga_stream_active = false;
uint32_t last_ga_stream_time_ms = 0;
uint32_t ga_stream_interval_ms = 20; // 50 Hz default optimal rate for PID GA (20ms)

float calib_align_voltage = CALIBRATION_VOLTAGE;
float calib_hold_voltage = CALIBRATION_HOLD_VOLTAGE;
bool calib_user_aborted = false;
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
bool calibrateComprehensiveFOCSinglePass(float align_voltage);
bool calibrateComprehensiveFOC(float align_voltage);
bool calibrate256PointLUTSinglePass(float align_voltage);
bool calibrate256PointLUT(float align_voltage);
bool calibrateDynamicContinuousSinglePass(float calib_voltage);
bool calibrateDynamicContinuous(float calib_voltage);
void runBenchmarkComparison();
void printLUTMetrics();
float readMotorTemperatureC();
extern bool kv_test_running;

// ============================================================================
// ZERO-OVERHEAD REAL-TIME FOC TIMING & STUTTER PROFILER
// ============================================================================
#define FOC_SPIKE_EXEC_THRESHOLD_US     180 // Execution spike threshold (>180us, leaves healthy PID+SysTick unflagged)
#define FOC_SPIKE_INTERVAL_THRESHOLD_US 200 // Interval jitter threshold (>200us)

struct FOCProfilerStats {
  // Window metrics (reset every telemetry window ~10ms)
  uint32_t window_cycles_accum;
  uint32_t window_samples;
  uint32_t window_max_exec_cycles;
  uint32_t window_max_interval_cycles;

  // Session counters
  uint32_t total_cycles_executed;
  uint32_t exec_spikes_total;
  uint32_t interval_spikes_total;

  // Detailed breakdown of the single worst execution spike observed
  uint32_t worst_spike_exec_cycles;
  uint32_t worst_spike_zero_cycles;
  uint32_t worst_spike_comm_cycles;
  uint32_t worst_spike_move_cycles;
  uint32_t worst_spike_interval_cycles;
  uint32_t worst_spike_time_ms;

  // Breakdown of the most recent execution spike observed
  uint32_t latest_spike_exec_cycles;
  uint32_t latest_spike_zero_cycles;
  uint32_t latest_spike_comm_cycles;
  uint32_t latest_spike_move_cycles;
  uint32_t latest_spike_interval_cycles;

  // Current values
  float avg_exec_us;
  float max_exec_us;
  float max_interval_us;
};

volatile FOCProfilerStats g_foc_perf = {0};

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
  float avg_exec_us;
  float max_exec_us;
  float max_interval_us;
  uint32_t exec_spikes;
  uint32_t interval_spikes;
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

  const uint32_t cpu_freq_mhz = 240;
  const uint32_t spike_exec_thresh_cycles = FOC_SPIKE_EXEC_THRESHOLD_US * cpu_freq_mhz;
  const uint32_t spike_int_thresh_cycles = FOC_SPIKE_INTERVAL_THRESHOLD_US * cpu_freq_mhz;
  uint32_t last_loop_start_cycles = esp_cpu_get_ccount();

  while (true) {
    if (foc_paused) {
      vTaskDelay(pdMS_TO_TICKS(5));
      last_loop_start_cycles = esp_cpu_get_ccount();
      continue;
    }

    uint32_t loop_start_cycles = esp_cpu_get_ccount();
    uint32_t interval_cycles = loop_start_cycles - last_loop_start_cycles;
    last_loop_start_cycles = loop_start_cycles;

    if (is_enabled) {
      uint32_t t0 = loop_start_cycles;

      // 1. 11-Pole Continuous Zero Vector + Dynamic Sensor Lag Compensation
      float zero_elec = getInterpolatedElectricalZero(motor1.shaft_angle);
      if (sensor_lag_comp_us > 0.0f) {
        float lead_rad = motor1.shaft_velocity * (float)motor1.pole_pairs *
                         (sensor_lag_comp_us * 1e-6f);
        motor1.zero_electric_angle = _normalizeAngle(zero_elec - lead_rad);
      } else {
        motor1.zero_electric_angle = zero_elec;
      }
      uint32_t t1 = esp_cpu_get_ccount();

      // 2. High-Frequency Commutation Loop (includes SPI sensor read)
      motor1.loopFOC();
      uint32_t t2 = esp_cpu_get_ccount();

      // 3. Motion Control Move (PID / LPF downsampled)
      if (kv_test_running) {
        motor1.move(target_voltage);
      } else if (motor1.controller == MotionControlType::velocity) {
        float target_rad_s = target_velocity_rpm * 0.104719755f;
        motor1.move(target_rad_s);
      } else if (motor1.controller == MotionControlType::torque) {
        motor1.move(target_voltage);
      }
      uint32_t t3 = esp_cpu_get_ccount();

      uint32_t exec_cycles = t3 - t0;
      foc_cycle_count++;

      // Zero-lag profiling accumulators (pure 32-bit integer arithmetic)
      g_foc_perf.window_cycles_accum += exec_cycles;
      g_foc_perf.window_samples++;
      g_foc_perf.total_cycles_executed++;

      if (exec_cycles > g_foc_perf.window_max_exec_cycles) {
        g_foc_perf.window_max_exec_cycles = exec_cycles;
      }
      if (interval_cycles > g_foc_perf.window_max_interval_cycles && g_foc_perf.total_cycles_executed > 10) {
        g_foc_perf.window_max_interval_cycles = interval_cycles;
      }

      // Check for execution spike
      if (exec_cycles > spike_exec_thresh_cycles) {
        g_foc_perf.exec_spikes_total++;
        g_foc_perf.latest_spike_exec_cycles = exec_cycles;
        g_foc_perf.latest_spike_zero_cycles = t1 - t0;
        g_foc_perf.latest_spike_comm_cycles = t2 - t1;
        g_foc_perf.latest_spike_move_cycles = t3 - t2;
        g_foc_perf.latest_spike_interval_cycles = interval_cycles;
        if (exec_cycles > g_foc_perf.worst_spike_exec_cycles) {
          g_foc_perf.worst_spike_exec_cycles = exec_cycles;
          g_foc_perf.worst_spike_zero_cycles = t1 - t0;
          g_foc_perf.worst_spike_comm_cycles = t2 - t1;
          g_foc_perf.worst_spike_move_cycles = t3 - t2;
          g_foc_perf.worst_spike_interval_cycles = interval_cycles;
          g_foc_perf.worst_spike_time_ms = millis();
        }
      }

      // Check for interval jitter spike (preemption / delay)
      if (interval_cycles > spike_int_thresh_cycles && g_foc_perf.total_cycles_executed > 10) {
        g_foc_perf.interval_spikes_total++;
      }
    } else {
      // Motor disabled: update sensor and motion state for live GUI tracking, then idle briefly
      motor1.loopFOC();
      motor1.shaft_angle = motor1.shaftAngle();
      motor1.shaft_velocity = motor1.shaftVelocity();
      foc_cycle_count++;
      vTaskDelay(pdMS_TO_TICKS(1));
      last_loop_start_cycles = esp_cpu_get_ccount();
    }

    // Measure FOC loop frequency every 250ms
    uint32_t now_us = micros();
    if (now_us - last_hz_meas_us >= 250000) {
      current_foc_hz = (float)foc_cycle_count * 1000000.0f /
                       (float)(now_us - last_hz_meas_us);
      foc_cycle_count = 0;
      last_hz_meas_us = now_us;
    }

    // Update telemetry snapshot at ~100 Hz (every 10ms)
    if (now_us - last_snapshot_us >= 10000) {
      last_snapshot_us = now_us;

      // Extract single-turn mechanical degree using calibrated sensor angle directly [0, 360)
      float d = encoder1.getSingleTurnDeg();

      float avg_us = 0.0f;
      float max_us = (float)g_foc_perf.window_max_exec_cycles / (float)cpu_freq_mhz;
      float max_int_us = (float)g_foc_perf.window_max_interval_cycles / (float)cpu_freq_mhz;
      if (g_foc_perf.window_samples > 0) {
        avg_us = (float)g_foc_perf.window_cycles_accum / ((float)g_foc_perf.window_samples * (float)cpu_freq_mhz);
      }
      g_foc_perf.avg_exec_us = avg_us;
      g_foc_perf.max_exec_us = max_us;
      g_foc_perf.max_interval_us = max_int_us;

      // Reset window accumulators for next measurement window
      g_foc_perf.window_cycles_accum = 0;
      g_foc_perf.window_samples = 0;
      g_foc_perf.window_max_exec_cycles = 0;
      g_foc_perf.window_max_interval_cycles = 0;

      portENTER_CRITICAL(&telemSpinlock);
      g_telemetry.angle_deg = d;
      g_telemetry.actual_rpm = motor1.shaft_velocity * 9.54929658551f;
      g_telemetry.target_rpm = target_velocity_rpm;
      g_telemetry.target_voltage = target_voltage;
      g_telemetry.uq = is_enabled ? motor1.voltage.q : 0.0f;
      g_telemetry.ud = is_enabled ? motor1.voltage.d : 0.0f;
      g_telemetry.raw14 = encoder1.last_raw_14bit;
      g_telemetry.raw_h = encoder1.last_raw_h;
      g_telemetry.raw_l = encoder1.last_raw_l;
      g_telemetry.no_mag = encoder1.last_no_mag;
      g_telemetry.parity_ok = encoder1.last_parity_ok;
      g_telemetry.is_enabled = is_enabled;
      g_telemetry.kv_test = kv_test_running;
      g_telemetry.ctrl_mode = motor1.controller;
      g_telemetry.foc_hz = current_foc_hz;
      g_telemetry.avg_exec_us = avg_us;
      g_telemetry.max_exec_us = max_us;
      g_telemetry.max_interval_us = max_int_us;
      g_telemetry.exec_spikes = g_foc_perf.exec_spikes_total;
      g_telemetry.interval_spikes = g_foc_perf.interval_spikes_total;
      portEXIT_CRITICAL(&telemSpinlock);
    }
  }
}

// ============================================================================
// MOTOR TEMPERATURE SENSOR (ADC1_CH3 / GPIO 4)
// ============================================================================
float readMotorTemperatureC() {
  int raw = analogRead(PIN_THERM_M1);
  if (raw <= 0 || raw >= 4095)
    return 25.0f; // Default fallback if unpopulated/floating
  float v = (float)raw * (3.3f / 4095.0f);
  if (v <= 0.05f || v >= 3.25f)
    return 25.0f;
  float r_ntc = (10000.0f * v) / (3.3f - v);
  float steinhart = r_ntc / 10000.0f;
  steinhart = logf(steinhart);
  steinhart /= 3950.0f;
  steinhart += 1.0f / (25.0f + 273.15f);
  steinhart = 1.0f / steinhart;
  steinhart -= 273.15f;
  return steinhart;
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
  prefs.putUChar("corr_mode", encoder1.correction_mode);
  prefs.putFloat("f_a1", encoder1.fourier_A1);
  prefs.putFloat("f_b1", encoder1.fourier_B1);
  prefs.putFloat("f_a2", encoder1.fourier_A2);
  prefs.putFloat("f_b2", encoder1.fourier_B2);
  prefs.putFloat("f_a3", encoder1.fourier_A3);
  prefs.putFloat("f_b3", encoder1.fourier_B3);
  prefs.putFloat("f_a4", encoder1.fourier_A4);
  prefs.putFloat("f_b4", encoder1.fourier_B4);
  prefs.putFloat("h_r2", encoder1.harmonic_r_squared);
  prefs.putFloat("h_pk", encoder1.harmonic_pk_pk_deg);
  prefs.putFloat("h_rms", encoder1.harmonic_rms_deg);
  prefs.putFloat("h_respk", encoder1.harmonic_res_pk_pk_deg);
  prefs.putFloat("h_resrms", encoder1.harmonic_res_rms_deg);
  prefs.putFloat("h_mlag", encoder1.harmonic_mean_lag_deg);
  prefs.putFloat("h_lrip", encoder1.harmonic_lag_ripple_deg);
  prefs.putFloat("h_lpkang", encoder1.harmonic_max_drag_angle_deg);
  prefs.putBytes("lut_table", encoder1.lut_errors, sizeof(encoder1.lut_errors));
  prefs.putBytes("pole_zeros", calibrated_pole_zeros,
                 sizeof(calibrated_pole_zeros));
  prefs.putFloat("pid_p", motor1.PID_velocity.P);
  prefs.putFloat("pid_i", motor1.PID_velocity.I);
  prefs.putFloat("pid_d", motor1.PID_velocity.D);
  prefs.putFloat("lpf_tf", motor1.LPF_velocity.Tf);
  prefs.putUInt("spi_speed", encoder1.spi_speed);
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
                   "run calibration ('CALIB_DYN', 'w', or 'c').");
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
  encoder1.correction_mode =
      prefs.getUChar("corr_mode", encoder1.is_lut_calibrated ? 1 : 0);
  encoder1.fourier_A1 = prefs.getFloat("f_a1", 0.0f);
  encoder1.fourier_B1 = prefs.getFloat("f_b1", 0.0f);
  encoder1.fourier_A2 = prefs.getFloat("f_a2", 0.0f);
  encoder1.fourier_B2 = prefs.getFloat("f_b2", 0.0f);
  encoder1.fourier_A3 = prefs.getFloat("f_a3", 0.0f);
  encoder1.fourier_B3 = prefs.getFloat("f_b3", 0.0f);
  encoder1.fourier_A4 = prefs.getFloat("f_a4", 0.0f);
  encoder1.fourier_B4 = prefs.getFloat("f_b4", 0.0f);
  encoder1.harmonic_r_squared = prefs.getFloat("h_r2", 0.0f);
  encoder1.harmonic_pk_pk_deg = prefs.getFloat("h_pk", 0.0f);
  encoder1.harmonic_rms_deg = prefs.getFloat("h_rms", 0.0f);
  encoder1.harmonic_res_pk_pk_deg = prefs.getFloat("h_respk", 0.0f);
  encoder1.harmonic_res_rms_deg = prefs.getFloat("h_resrms", 0.0f);
  encoder1.harmonic_mean_lag_deg = prefs.getFloat("h_mlag", 0.0f);
  encoder1.harmonic_lag_ripple_deg = prefs.getFloat("h_lrip", 0.0f);
  encoder1.harmonic_max_drag_angle_deg = prefs.getFloat("h_lpkang", 0.0f);

  prefs.getBytes("lut_table", encoder1.lut_errors, sizeof(encoder1.lut_errors));
  prefs.getBytes("pole_zeros", calibrated_pole_zeros,
                 sizeof(calibrated_pole_zeros));

  motor1.PID_velocity.P = prefs.getFloat("pid_p", DEFAULT_PID_P);
  motor1.PID_velocity.I = prefs.getFloat("pid_i", DEFAULT_PID_I);
  motor1.PID_velocity.D = prefs.getFloat("pid_d", DEFAULT_PID_D);
  motor1.LPF_velocity.Tf =
      prefs.getFloat("lpf_tf", (DEFAULT_LPF_TF_MS * 0.001f));

  uint32_t saved_spi = prefs.getUInt("spi_speed", SPI_SPEED_DEFAULT_HZ);
  if (saved_spi >= 1000000 && saved_spi <= 20000000) {
    encoder1.setSPISpeed(saved_spi);
  }

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
  Serial.print("  Active Mode: ");
  if (encoder1.correction_mode == 2) {
    Serial.println("MODE 2 (CONTINUOUS ANALYTICAL HARMONICS)");
  } else if (encoder1.correction_mode == 1) {
    Serial.println("MODE 1 (128-PT STEPPED LUT)");
  } else {
    Serial.println("MODE 0 (RAW UNCALIBRATED)");
  }
  if (encoder1.is_lut_calibrated || encoder1.correction_mode != 0) {
    printLUTMetrics();
  }
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
  Serial.print(",\"correction_mode\":");
  Serial.print((int)encoder1.correction_mode);
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
  Serial.print(",\"fourier_A3\":");
  Serial.print(encoder1.fourier_A3, 6);
  Serial.print(",\"fourier_B3\":");
  Serial.print(encoder1.fourier_B3, 6);
  Serial.print(",\"fourier_A4\":");
  Serial.print(encoder1.fourier_A4, 6);
  Serial.print(",\"fourier_B4\":");
  Serial.print(encoder1.fourier_B4, 6);
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
  Serial.print("],\"spi_speed_hz\":");
  Serial.print(encoder1.spi_speed);
  Serial.print(",\"spi_speed_mhz\":");
  Serial.print((float)encoder1.spi_speed / 1000000.0f, 2);
  Serial.println("]}");
  Serial.println("[CONFIG_END]\n");
}

// KV Test State Machine
bool kv_test_running = false;
// 18-stage closed-loop KV sweep (9 positive, 9 negative) strictly within
// SpaceVectorPWM linear limit (12V / sqrt(3) ~= 6.93V) to prevent PWM clipping/saturation.
const float kv_voltages[] = {1.0f,  1.5f,  2.0f,  2.5f,  3.0f,  3.5f,  4.0f,  5.0f,  6.0f,
                            -1.0f, -1.5f, -2.0f, -2.5f, -3.0f, -3.5f, -4.0f, -5.0f, -6.0f};
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
      Serial.println(
          "\n-------------------------------------------------------------");
      Serial.printf(">>> 🚨 STALL DETECTED AT LAG %.1f us! Motor stopped.\n",
                    test_lag);
      Serial.printf("    Telemetry: Speed = %.1f RPM (target 300 RPM) | Demand "
                    "Uq = %.2f V | Phase Limit = %.2f V\n",
                    avg_rpm, avg_uq, motor1.voltage_limit);
      Serial.println("    ⚠️ ROOT CAUSE: Commutation lead angle over-advanced, "
                     "demagnetizing torque field,");
      Serial.println("       or insufficient supply voltage / current limit "
                     "for the chosen test speed.");
      Serial.println("       FIX: Ensure electrical zero is calibrated ('c') "
                     "and power supply voltage >= 10V.");
      Serial.println(
          "-------------------------------------------------------------\n");
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
// LUT METRICS DISPLAY FOR TROUBLESHOOTING & CALIBRATION VERIFICATION
// ============================================================================
void printLUTMetrics() {
  if (!encoder1.is_lut_calibrated && encoder1.correction_mode == 0) {
    Serial.println(
        "\n[LUT_METRICS] No active LUT calibration table available.");
    return;
  }
  float max_err = -100.0f, min_err = 100.0f;
  float sum_sq_err = 0.0f;
  int anomaly_count = 0;

  for (int i = 0; i < ENCODER_LUT_SIZE; i++) {
    float err = encoder1.lut_errors[i];
    if (err > max_err)
      max_err = err;
    if (err < min_err)
      min_err = err;
    sum_sq_err += err * err;
    if (fabsf(err) > LUT_ANOMALY_THRESHOLD_RAD) {
      anomaly_count++;
    }
  }

  float pk_pk_rad = max_err - min_err;
  float pk_pk_deg = pk_pk_rad * 57.2957795f;
  float rms_deg = sqrtf(sum_sq_err / (float)ENCODER_LUT_SIZE) * 57.2957795f;
  float h1_amp_deg = sqrtf(encoder1.fourier_A1 * encoder1.fourier_A1 +
                           encoder1.fourier_B1 * encoder1.fourier_B1) *
                     57.2957795f;
  float h2_amp_deg = sqrtf(encoder1.fourier_A2 * encoder1.fourier_A2 +
                           encoder1.fourier_B2 * encoder1.fourier_B2) *
                     57.2957795f;
  float h3_amp_deg = sqrtf(encoder1.fourier_A3 * encoder1.fourier_A3 +
                           encoder1.fourier_B3 * encoder1.fourier_B3) *
                     57.2957795f;
  float h4_amp_deg = sqrtf(encoder1.fourier_A4 * encoder1.fourier_A4 +
                           encoder1.fourier_B4 * encoder1.fourier_B4) *
                     57.2957795f;

  Serial.println("\n-----------------------------------------------------------"
                 "---------------------");
  Serial.println("📊 128-POINT ENCODER NON-LINEARITY & HARMONIC METRICS:");
  Serial.print("  Active Correction Mode        : ");
  if (encoder1.correction_mode == 2) {
    Serial.println("MODE 2 (CONTINUOUS ANALYTICAL HARMONICS)");
  } else if (encoder1.correction_mode == 1) {
    Serial.println("MODE 1 (128-POINT STEPPED LUT)");
  } else {
    Serial.println("MODE 0 (RAW UNCALIBRATED)");
  }
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
  Serial.print("  Harmonic 1 (Eccentricity/Run) : ±");
  Serial.print(h1_amp_deg, 3);
  Serial.println(" deg");
  Serial.print("  Harmonic 2 (Magnet Tilt/Oval) : ±");
  Serial.print(h2_amp_deg, 3);
  Serial.println(" deg");
  Serial.print("  Harmonic 3 (3-Point Clamping) : ±");
  Serial.print(h3_amp_deg, 3);
  Serial.println(" deg");
  Serial.print("  Harmonic 4 (Bridge/Saturation): ±");
  Serial.print(h4_amp_deg, 3);
  Serial.println(" deg");
  Serial.print("  Anomaly Points (> ");
  Serial.print(LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f, 1);
  Serial.print(" deg)     : ");
  Serial.println(anomaly_count);
  Serial.print("  Status                        : ");
  if (anomaly_count == 0 && pk_pk_rad <= LUT_PKPK_LIMIT_RAD) {
    Serial.println("✅ ACTIVE & VALID (All points within spec)");
  } else {
    Serial.println("⚠️ ANOMALIES DETECTED / OUT OF SPEC");
  }
  Serial.println("-------------------------------------------------------------"
                 "-------------------\n");
}

// ============================================================================
// COMPREHENSIVE HARMONIC & PHYSICAL FAULT DIAGNOSTIC REPORT
// ============================================================================
void printHarmonicReport() {
  float a1 = encoder1.fourier_A1, b1 = encoder1.fourier_B1;
  float a2 = encoder1.fourier_A2, b2 = encoder1.fourier_B2;
  float a3 = encoder1.fourier_A3, b3 = encoder1.fourier_B3;
  float a4 = encoder1.fourier_A4, b4 = encoder1.fourier_B4;

  float h1_amp = sqrtf(a1 * a1 + b1 * b1) * 57.2957795f;
  float h2_amp = sqrtf(a2 * a2 + b2 * b2) * 57.2957795f;
  float h3_amp = sqrtf(a3 * a3 + b3 * b3) * 57.2957795f;
  float h4_amp = sqrtf(a4 * a4 + b4 * b4) * 57.2957795f;

  float phi1 = atan2f(b1, a1) * 57.2957795f; if (phi1 < 0.0f) phi1 += 360.0f;
  float phi2 = atan2f(b2, a2) * 57.2957795f; if (phi2 < 0.0f) phi2 += 360.0f;
  float phi3 = atan2f(b3, a3) * 57.2957795f; if (phi3 < 0.0f) phi3 += 360.0f;
  float phi4 = atan2f(b4, a4) * 57.2957795f; if (phi4 < 0.0f) phi4 += 360.0f;

  float apex1 = phi1;
  float apex2 = phi2 * 0.5f;
  float apex3 = phi3 / 3.0f;
  float apex4 = phi4 * 0.25f;

  Serial.println("\n================================================================================");
  Serial.println("🔬 MT6816 HARMONIC CALIBRATION & PHYSICAL DIAGNOSTIC REPORT");
  Serial.println("================================================================================");
  Serial.printf("  Active Correction Mode        : %d (%s)\n",
                encoder1.correction_mode,
                encoder1.correction_mode == 2 ? "MODE 2: CONTINUOUS HARMONICS" :
                (encoder1.correction_mode == 1 ? "MODE 1: 128-PT STEPPED LUT" : "MODE 0: RAW UNCALIBRATED"));
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("  HARMONIC DECOMPOSITION & PHYSICAL APEX LOCALIZATION:");
  Serial.printf("  • Harmonic 1 (Runout/Eccentricity) : ±%.4f°  [A1=%+.6f, B1=%+.6f]\n", h1_amp, a1, b1);
  Serial.printf("    Phase φ1: %5.1f°  ==> Rotor Eccentricity Apex at: %5.1f° mech\n", phi1, apex1);
  Serial.printf("  • Harmonic 2 (Magnet Tilt/Ovality) : ±%.4f°  [A2=%+.6f, B2=%+.6f]\n", h2_amp, a2, b2);
  Serial.printf("    Phase φ2: %5.1f°  ==> Magnet Tilt Axis Apex at   : %5.1f° mech (and %5.1f°)\n", phi2, apex2, fmodf(apex2 + 180.0f, 360.0f));
  Serial.printf("  • Harmonic 3 (3-Point Chassis Pinch: ±%.4f°  [A3=%+.6f, B3=%+.6f]\n", h3_amp, a3, b3);
  Serial.printf("    Phase φ3: %5.1f°  ==> Primary Clamp Pinch Apex at: %5.1f° mech (every 120°)\n", phi3, apex3);
  Serial.printf("  • Harmonic 4 (Bridge/Flux Saturation): ±%.4f°  [A4=%+.6f, B4=%+.6f]\n", h4_amp, a4, b4);
  Serial.printf("    Phase φ4: %5.1f°  ==> Quad-Bridge Mismatch Axis  : %5.1f° mech (every 90°)\n", phi4, apex4);
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("  CALIBRATION QUALITY, RIPPLE REDUCTION & FIT:");
  Serial.printf("  • Raw Uncalibrated Error Span      : %.4f° mech (±%.4f°) | RMS: %.4f°\n",
                encoder1.harmonic_pk_pk_deg, encoder1.harmonic_pk_pk_deg * 0.5f, encoder1.harmonic_rms_deg);
  Serial.printf("  • Reconstructed Residual Ripple    : %.4f° mech (±%.4f°) | RMS: %.4f°\n",
                encoder1.harmonic_res_pk_pk_deg, encoder1.harmonic_res_pk_pk_deg * 0.5f, encoder1.harmonic_res_rms_deg);
  float ripple_reduc = (encoder1.harmonic_pk_pk_deg > 1e-4f) ?
                       ((1.0f - (encoder1.harmonic_res_pk_pk_deg / encoder1.harmonic_pk_pk_deg)) * 100.0f) : 0.0f;
  Serial.printf("  • Non-Linearity Ripple Reduction   : %.1f%%\n", ripple_reduc);
  Serial.printf("  • Harmonic Model Fit Quality (R²)  : %.2f%%\n", encoder1.harmonic_r_squared * 100.0f);
  if (encoder1.harmonic_r_squared >= 0.90f) {
    Serial.println("    Status: ✅ EXCELLENT FIT (Harmonics 1, 2, 4 explain >90% of encoder distortion)");
  } else if (encoder1.harmonic_r_squared >= 0.75f) {
    Serial.println("    Status: ℹ️ MODERATE FIT (Some unmodeled high-frequency ripple or noise present)");
  } else {
    Serial.println("    Status: ⚠️ POOR FIT (<75% explained). Check mechanical mounting, shaft wobble or magnet airgap!");
  }
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("  DYNAMIC MECHANICAL DRAG & BEARING UNIFORMITY (38 RPM):");
  Serial.printf("  • Mean Directional Rotor Lag       : %.4f° mech\n", encoder1.harmonic_mean_lag_deg);
  Serial.printf("  • Bearing Drag Non-Uniformity (Pk) : %.4f° mech\n", encoder1.harmonic_lag_ripple_deg);
  if (encoder1.harmonic_lag_ripple_deg > 0.08f) {
    Serial.printf("    ⚠️ WARNING: Friction varies significantly around rotation (Peak drag at %.1f° mech)!\n",
                  encoder1.harmonic_max_drag_angle_deg);
    Serial.println("    ACTION: Check for bearing notchiness, tight raceway, or mechanical rubbing.");
  } else {
    Serial.println("    Status: ✅ DRAG UNIFORM (Bearing rotation is smooth and isotropic)");
  }
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("  PHYSICAL FAULT DIAGNOSTICS:");
  bool h3_warn = (h1_amp > 0.05f && h3_amp > 0.20f * h1_amp);
  bool h4_warn = (h1_amp > 0.05f && h4_amp > 0.25f * h1_amp);
  if (h3_warn) {
    Serial.printf("  ⚠️ CHASSIS CLAMP ALERT: H3 (%.3f°) is %.1f%% of H1 (limit 20%%)!\n", h3_amp, (h3_amp / h1_amp) * 100.0f);
    Serial.println("     ROOT CAUSE: 3-bolt faceplate over-torqued or warped mounting flange.");
    Serial.printf("     ACTION: Loosen and re-torque mounting bolts near apex %.1f°.\n", apex3);
  } else {
    Serial.println("  • Chassis Clamping Distortion      : ✅ PASS (<20% of H1)");
  }
  if (h4_warn) {
    Serial.printf("  ⚠️ FLUX SATURATION ALERT: H4 (%.3f°) is %.1f%% of H1 (limit 25%%)!\n", h4_amp, (h4_amp / h1_amp) * 100.0f);
    Serial.println("     ROOT CAUSE: Magnet is too close to MT6816 (<0.8mm) causing quad-bridge flux saturation.");
    Serial.println("     ACTION: Increase magnet-to-sensor axial airgap to 1.0 - 1.5mm.");
  } else {
    Serial.println("  • Hall Quad-Bridge / Saturation    : ✅ PASS (<25% of H1)");
  }
  Serial.println("================================================================================\n");
}

// ============================================================================
// 128-POINT NON-LINEARITY LUT CALIBRATION
// ============================================================================
bool calibrate256PointLUTSinglePass(float align_voltage) {
  float hold_voltage = min(align_voltage, calib_hold_voltage);
  Serial.println("\n==========================================================="
                 "=====================");
  Serial.println(
      ">>> 🔬 STARTING 128-POINT DUAL-PASS NON-LINEARITY LUT CALIBRATION <<<");
  Serial.println("============================================================="
                 "===================");
  Serial.println("Target Motor       : GM3506 (11 Pole Pairs on ESP32-S3)");
  Serial.print("Step / Hold Voltage: ");
  Serial.print(align_voltage, 2);
  Serial.print(" V (move) / ");
  Serial.print(hold_voltage, 2);
  Serial.println(" V (hold & sample)");
  Serial.println(
      "Methodology        : 128-Step Dual-Pass Bidirectional Mechanical Sweep");
  Serial.printf("Oversampling       : %d Samples / Step with Micro-Dither "
                "(Stiction Cancellation)\n",
                CALIB_SWEEP_OVERSAMPLE_COUNT);
  Serial.print("Anomaly Guard      : Any |err| > ");
  Serial.print(LUT_ANOMALY_THRESHOLD_RAD, 3);
  Serial.print(" rad (");
  Serial.print(LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f, 1);
  Serial.println(" deg) -> FAIL (pole slip)");
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
  delay(100);
  motor1.setPhaseVoltage(hold_voltage, 0, _3PI_2);
  delay(300);
  float start_raw = encoder1.getRawMechanicalAngle();

  const int DIR_STEPS = 200;
  for (int step = 0; step <= DIR_STEPS; step++) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        calib_user_aborted = true;
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
        calib_user_aborted = true;
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

  float moved_deg = fabsf(d_ang) * 57.2957795f;
  float expected_deg = (360.0f / (float)MOTOR_POLE_PAIRS) * 0.40f;

  if (fabsf(d_ang) < 0.05f) {
    Serial.println("FAILED!");
    Serial.println(
        "  -------------------------------------------------------------");
    Serial.printf("  ❌ MOTION CHECK FAILED: Rotor moved only %.2f deg mech "
                  "(expected >= %.2f deg).\n",
                  moved_deg, expected_deg);
    Serial.printf("     Angles sampled : start = %.4f rad (%.1f deg), mid = "
                  "%.4f rad (%.1f deg)\n",
                  start_raw, start_raw * 57.2957795f, mid_raw,
                  mid_raw * 57.2957795f);
    Serial.printf("     Encoder Health : Parity Errors = %lu, Mag Faults = "
                  "%lu, Last Parity = %s\n",
                  encoder1.parity_errors, encoder1.mag_faults,
                  encoder1.last_parity_ok ? "OK" : "CORRUPT");
    if (!encoder1.last_parity_ok || encoder1.mag_faults > 0) {
      Serial.println("     ⚠️ ROOT CAUSE: Encoder bus signal integrity issue or "
                     "missing magnet.");
      Serial.println("        FIX: Check MT6816 SPI wiring, CS pullup, and "
                     "magnet airgap (1-2mm).");
    } else {
      Serial.println("     ⚠️ ROOT CAUSE: Insufficient torque to overcome "
                     "cogging/friction or unpowered inverter.");
      Serial.println("        FIX: Verify 12V DC bench supply is ON, driver EN "
                     "pin is HIGH, or increase voltage ('w 4.0').");
    }
    Serial.println(
        "  -------------------------------------------------------------\n");
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    return false;
  }

  Direction sensor_dir = (d_ang > 0.0f) ? Direction::CW : Direction::CCW;
  int dir_sign = (sensor_dir == Direction::CW) ? 1 : -1;
  Serial.printf("Detected %s (%+.2f deg mech) [OK]\n",
                sensor_dir == Direction::CW ? "CW (+1)" : "CCW (-1)",
                d_ang * 57.2957795f);

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
        calib_user_aborted = true;
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

    // Smooth micro-stepping transition from previous step (eliminates jerk and
    // pole slip)
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
    motor1.setPhaseVoltage(hold_voltage, 0, target_elec);
    delay(
        CALIB_SWEEP_DAMP_SETTLE_MS); // Conservative damp settle before sampling

    float sum_sin = 0.0f, sum_cos = 0.0f;
    for (int s = 0; s < CALIB_SWEEP_OVERSAMPLE_COUNT;
         s++) { // Static samples for noise rejection
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

  // --- DEDICATED THERMAL COOL-DOWN PAUSE (5s) ---
  Serial.printf("\n>>> ❄️ THERMAL COOL-DOWN PAUSE (%ds): De-energizing stator "
                 "coils to eliminate thermal drift...\n", CALIB_COOLDOWN_PAUSE_SEC);
  motor1.setPhaseVoltage(0, 0, 0);
  for (int sec = CALIB_COOLDOWN_PAUSE_SEC; sec > 0; sec--) {
    Serial.print("   Coil cool-down: ");
    Serial.print(sec);
    Serial.println("s remaining...");
    delay(1000);
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        calib_user_aborted = true;
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return false;
      }
    }
  }
  Serial.println(
      ">>> ❄️ Coils thermally stabilized. Resuming Reverse Sweep...\n");

  // Re-engage at end of mechanical travel before reverse sweep
  float rev_start_mech = (float)(TOTAL_STEPS - 1) * step_mech_rad;
  float rev_start_elec =
      _3PI_2 + (float)dir_sign * (float)MOTOR_POLE_PAIRS * rev_start_mech;
  motor1.setPhaseVoltage(align_voltage, 0, rev_start_elec);
  delay(100);
  motor1.setPhaseVoltage(hold_voltage, 0, rev_start_elec);
  delay(250);

  // PASS 2: REVERSE (CCW) SWEEP
  for (int i = TOTAL_STEPS - 1; i >= 0; i--) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        calib_user_aborted = true;
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
    motor1.setPhaseVoltage(hold_voltage, 0, target_elec);
    delay(CALIB_SWEEP_DAMP_SETTLE_MS); // Conservative damp settle

    float sum_sin = 0.0f, sum_cos = 0.0f;
    for (int s = 0; s < CALIB_SWEEP_OVERSAMPLE_COUNT; s++) { // Static samples
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
    float med = ((a >= b && a <= c) || (a <= b && a >= c))   ? a
                : ((b >= a && b <= c) || (b <= a && b >= c)) ? b
                                                             : c;

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
        float mech_pos_deg = ((float)i / (float)TOTAL_STEPS) * 360.0f;
        int nearest_pole =
            (int)(mech_pos_deg / (360.0f / (float)MOTOR_POLE_PAIRS)) + 1;
        Serial.printf("  ⚠️  ANOMALY idx[%3d] @ %5.1f deg mech (Pole #%2d) | "
                      "err = %+.4f rad (%+.2f deg) — POLE SLIP?\n",
                      i, mech_pos_deg, nearest_pole, filtered_err,
                      filtered_err * 57.2957795f);
      }
    }
  }

  // Fourier Harmonic Extraction (1st harmonic = eccentricity, 2nd harmonic =
  // tilt)
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
                           encoder1.fourier_B1 * encoder1.fourier_B1) *
                     57.2957795f;
  float h2_amp_deg = sqrtf(encoder1.fourier_A2 * encoder1.fourier_A2 +
                           encoder1.fourier_B2 * encoder1.fourier_B2) *
                     57.2957795f;

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
    Serial.println("-----------------------------------------------------------"
                   "---------------------");
    Serial.println(
        "❌ LUT CALIBRATION REJECTED — GEOMETRIC OR ROTOR STALL FAILURE");
    Serial.println("-----------------------------------------------------------"
                   "---------------------");

    if (anomaly_count > 0) {
      Serial.printf("  ⚠️ ROOT CAUSE: Rotor step stall / slip (%d points "
                    "exceeded ±%.3f rad / ±%.2f deg).\n",
                    anomaly_count, LUT_ANOMALY_THRESHOLD_RAD,
                    LUT_ANOMALY_THRESHOLD_RAD * 57.2957795f);
      Serial.println("     FIX: Motor stiction exceeded hold torque. Increase "
                     "sweep voltage ('w 4.2' or 'w 4.5').");
    }

    if (pk_pk_rad > LUT_PKPK_LIMIT_RAD) {
      Serial.printf("  ⚠️ ROOT CAUSE: Peak-to-Peak error span %.4f rad (%.2f "
                    "deg mech) exceeds %.3f rad (%.2f deg) limit.\n",
                    pk_pk_rad, pk_pk_deg, LUT_PKPK_LIMIT_RAD,
                    LUT_PKPK_LIMIT_RAD * 57.2957795f);

      if (h1_amp_deg > 1.2f) {
        Serial.printf("     DIAGNOSTIC: Harmonic 1 (Radial Eccentricity) is "
                      "high (±%.2f deg mech).\n",
                      h1_amp_deg);
        Serial.println("     FIX: Magnet is seated off-center (>0.15mm "
                       "runout). Re-seat magnet on motor shaft axis.");
      } else if (h2_amp_deg > 0.8f) {
        Serial.printf("     DIAGNOSTIC: Harmonic 2 (Magnet Tilt) is high "
                      "(±%.2f deg mech).\n",
                      h2_amp_deg);
        Serial.println("     FIX: Magnet is tilted or PCB is not parallel to "
                       "magnet face. Align sensor mounting plane.");
      } else {
        Serial.println("     DIAGNOSTIC: High-order harmonic distortion "
                       "(stator slot cogging or bearing binding).");
        Serial.println("     FIX: Check shaft freespin by hand and ensure "
                       "align voltage overcomes cogging detents.");
      }
    }

    Serial.printf(
        "  Encoder Bus Health: Parity Errors = %lu, Mag Faults = %lu\n",
        encoder1.parity_errors, encoder1.mag_faults);
    Serial.println("==========================================================="
                   "=====================\n");
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
  encoder1.correction_mode = 1;
  Serial.println("  Status                        : ✅ 128-Point LUT ACTIVE "
                 "(all points valid)");
  Serial.println("-------------------------------------------------------------"
                 "-------------------");

  // Thermal cool-down pause before 11-pole electrical alignment
  Serial.printf(
      "\n>>> ❄️ THERMAL COOL-DOWN PAUSE (%ds) before 11-Pole Alignment...\n", CALIB_COOLDOWN_PAUSE_SEC);
  motor1.setPhaseVoltage(0, 0, 0);
  for (int sec = CALIB_COOLDOWN_PAUSE_SEC; sec > 0; sec--) {
    Serial.print("   Coil cool-down: ");
    Serial.print(sec);
    Serial.println("s remaining...");
    delay(1000);
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        calib_user_aborted = true;
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

bool calibrate256PointLUT(float align_voltage) {
  calib_user_aborted = false;
  bool ok = calibrate256PointLUTSinglePass(align_voltage);
  if (!ok && align_voltage < CALIBRATION_BOOST_VOLTAGE && !calib_user_aborted) {
    Serial.println("\n================================================================================");
    Serial.printf("⚠️  STEPPED LUT CALIBRATION STALLED @ %.2fV!\n", align_voltage);
    Serial.printf("⚡ AUTOMATIC TORQUE BOOST: Ramping to %.2fV and retrying sweep...\n", CALIBRATION_BOOST_VOLTAGE);
    Serial.println("================================================================================\n");
    delay(400);
    ok = calibrate256PointLUTSinglePass(CALIBRATION_BOOST_VOLTAGE);
    if (ok) {
      calib_align_voltage = CALIBRATION_BOOST_VOLTAGE;
    }
  }
  return ok;
}

// ============================================================================
// CALIBRATION: FULL 360° BIDIRECTIONAL ROUND-TRIP SWEEP WITH DECAYING DITHER
// ============================================================================
bool calibrateComprehensiveFOCSinglePass(float align_voltage) {
  float hold_voltage = min(align_voltage, calib_hold_voltage);
  Serial.println("\n==========================================================="
                 "=====================");
  Serial.println(
      ">>> STARTING COMPREHENSIVE MULTI-POINT ELECTRICAL ZERO CALIBRATION <<<");
  Serial.println("============================================================="
                 "===================");
  Serial.println("Target Motor       : GM3506 (11 Pole Pairs on ESP32-S3)");
  Serial.print("Step / Hold Voltage: ");
  Serial.print(align_voltage, 2);
  Serial.print(" V (move) / ");
  Serial.print(hold_voltage, 2);
  Serial.println(" V (hold & sample)");
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
        calib_user_aborted = true;
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
        calib_user_aborted = true;
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
    Serial.println(
        "  -------------------------------------------------------------");
    Serial.printf("  ❌ MOTION CHECK FAILED: Rotor moved only %.2f deg mech "
                  "(expected >= %.2f deg).\n",
                  moved_rad * 57.2957795f, min_expected_rad * 57.2957795f);
    Serial.printf("     Angles sampled : start = %.4f rad (%.1f deg), mid = "
                  "%.4f rad (%.1f deg)\n",
                  start_angle, start_angle * 57.2957795f, mid_angle,
                  mid_angle * 57.2957795f);
    Serial.printf("     Encoder Health : Parity Errors = %lu, Mag Faults = "
                  "%lu, Last Parity = %s\n",
                  encoder1.parity_errors, encoder1.mag_faults,
                  encoder1.last_parity_ok ? "OK" : "CORRUPT");
    if (!encoder1.last_parity_ok || encoder1.mag_faults > 0) {
      Serial.println("     ⚠️ ROOT CAUSE: Encoder bus signal integrity issue or "
                     "missing magnet.");
      Serial.println("        FIX: Check MT6816 SPI wiring, CS pullup, and "
                     "magnet airgap (1-2mm).");
    } else {
      Serial.println("     ⚠️ ROOT CAUSE: Insufficient torque to overcome "
                     "cogging/friction or inverter unpowered.");
      Serial.println("        FIX: Verify 12V DC bench supply is ON, driver EN "
                     "pin is HIGH, or increase voltage ('c 4.2').");
    }
    Serial.println(
        "  -------------------------------------------------------------\n");
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
        calib_user_aborted = true;
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
    motor1.setPhaseVoltage(hold_voltage, 0, target_el);
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
        calib_user_aborted = true;
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
    motor1.setPhaseVoltage(hold_voltage, 0, target_el);
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

  // --- SYNTHESIS, ERROR DECOMPOSITION & ACTIONABLE DIAGNOSTICS ---
  float max_hyst_deg = 0.0f;
  float sum_hyst_deg = 0.0f;
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

    if (hyst_deg > max_hyst_deg)
      max_hyst_deg = hyst_deg;
    sum_hyst_deg += hyst_deg;

    // Strict per-pole validation: 10.0 deg elec (~0.9 deg mech)
    bool pole_pass = (hyst_deg <= 10.0f);
    if (!pole_pass)
      all_poles_ok = false;

    float pole_mech_target = ((float)p / (float)num_poles) * 360.0f;
    Serial.printf("  #%-2d | %5.1f deg | %6.4f rad | %6.4f rad | %6.4f rad | "
                  "±%4.2f deg | "
                  "%s\n",
                  p + 1, pole_mech_target, fwd_z, rev_z, true_z,
                  hyst_deg * 0.5f, pole_pass ? "[PASS]" : "[WARN-HYST]");
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
  float avg_hyst_deg = sum_hyst_deg / (float)num_poles;

  // Compute Standard Deviation of zero vector across all poles
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

  // Physical Commutation Degradation Metrics
  float max_elec_err_rad =
      fmaxf(std_dev_rad, max_hyst_deg * 0.01745329f * 0.5f);
  float torque_eff_pct = cosf(max_elec_err_rad) * 100.0f;
  float d_axis_leak_pct = sinf(max_elec_err_rad) * 100.0f;

  Serial.println("\n----------------- 🏆 CERTIFIED ELECTRICAL ALIGNMENT "
                 "RESULTS -----------------");
  Serial.printf("  ⭐ Mean Zero Electric Angle : %.5f rad (%.2f deg elec)\n",
                mean_zero_angle, mean_zero_angle * 57.2957795f);
  Serial.printf("  Commutation Direction       : %s\n",
                sensor_dir == Direction::CW ? "CW (+1)" : "CCW (-1)");
  Serial.printf("  Directional Hysteresis Span : ±%.2f deg elec (Avg: %.2f deg "
                "| Peak: %.2f deg)\n",
                global_hyst_deg * 0.5f, avg_hyst_deg, max_hyst_deg);
  Serial.printf("  Pole-to-Pole Dispersion     : ±%.2f deg elec (±%.4f rad)\n",
                std_dev_deg, std_dev_rad);
  Serial.printf("  Worst-Case Torque Output    : %.2f%% of theoretical Kt\n",
                torque_eff_pct);
  Serial.printf(
      "  Parasitic d-Axis Leakage    : %.2f%% (Current lost to heat)\n",
      d_axis_leak_pct);
  Serial.print("  Alignment Quality Score     : ");

  // Actionable Grading Hierarchy
  if (std_dev_deg > 10.0f || max_hyst_deg > 10.0f) {
    Serial.println("❌ GRADE F (REJECTED — Unusable for FOC)");
    Serial.println("  "
                   "-----------------------------------------------------------"
                   "------------------");
    if (max_hyst_deg > 10.0f) {
      Serial.printf("  ⚠️ ROOT CAUSE: Bearing stiction or detent pull "
                    "(Hysteresis Peak: %.1f deg).\n",
                    max_hyst_deg);
      Serial.println("     FIX: Increase alignment voltage ('y 4.2' or 'y "
                     "4.5') to overcome cogging.");
    }
    if (std_dev_deg > 10.0f) {
      Serial.printf("  ⚠️ ROOT CAUSE: Severe magnet runout or uncalibrated LUT "
                    "(StdDev: %.1f deg).\n",
                    std_dev_deg);
      Serial.println("     FIX: Re-run dual-pass LUT calibration ('w') first "
                     "before electrical zero.");
    }
    Serial.println("==========================================================="
                   "=====================\n");
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    is_calibrated = false;
    return false;
  }

  if (std_dev_deg < 2.0f && max_hyst_deg <= 2.0f) {
    Serial.println(
        "⭐ GRADE A (Exceptional Precision < 2.0 deg elec / < 0.18 deg mech)");
    Serial.println("     DIAGNOSTIC: Commutation vectors flawlessly decoupled; "
                   "maximum theoretical torque and zero drag.");
  } else if (std_dev_deg < 4.0f && max_hyst_deg <= 4.0f) {
    Serial.println(
        "✅ GRADE B (Excellent / Competition Grade < 4.0 deg elec / < 0.36 deg mech)");
    Serial.println("     DIAGNOSTIC: Superb alignment; negligible d-axis cross-talk "
                   "(<5%); maximum efficiency.");
  } else if (std_dev_deg < 6.0f && max_hyst_deg <= 6.0f) {
    Serial.println(
        "✨ GRADE C (Good / Standard < 6.0 deg elec / < 0.55 deg mech)");
    Serial.println("     DIAGNOSTIC: Solid closed-loop commutation; minimal "
                   "parasitic loss (<8%).");
  } else if (std_dev_deg < 8.0f && max_hyst_deg <= 8.0f) {
    Serial.println(
        "⚡ GRADE D (Fair / Minor Ripple < 8.0 deg elec / < 0.73 deg mech)");
    Serial.println("     DIAGNOSTIC: Mild hysteresis / cogging ripple detected; "
                   "minor thermal penalty under load.");
  } else {
    Serial.println("⚠️ GRADE E (Marginal / Usable — Moderate thermal penalty)");
    Serial.println("     DIAGNOSTIC: Safe to run, but expect 10-17% higher "
                   "coil heating under stall/load.");
    Serial.println("     RECOMMENDATION: Acceptable for testing; bump align "
                   "voltage or re-run LUT calibration for A-C grade.");
  }

  Serial.println("  "
                 "-------------------------------------------------------------"
                 "-----------------");
  Serial.println("  SimpleFOC Static Code Parameters:");
  Serial.printf("    motor.zero_electric_angle = %.5ff;\n", mean_zero_angle);
  Serial.printf("    motor.sensor_direction    = Direction::%s;\n",
                sensor_dir == Direction::CW ? "CW" : "CCW");
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

  if (!encoder1.is_lut_calibrated) {
    const float step_rad = _2PI / (float)ENCODER_LUT_SIZE;
    const float pole_span_rad = _2PI / (float)MOTOR_POLE_PAIRS;
    float f_a1 = 0.0f, f_b1 = 0.0f, f_a2 = 0.0f, f_b2 = 0.0f;
    for (int i = 0; i < ENCODER_LUT_SIZE; i++) {
      float rad = (float)i * step_rad;
      float pole_f = rad / pole_span_rad;
      int p0 = ((int)pole_f) % MOTOR_POLE_PAIRS;
      int p1 = (p0 + 1) % MOTOR_POLE_PAIRS;
      float frac = pole_f - (float)((int)pole_f);

      float err0 = calibrated_pole_zeros[p0] - mean_zero_angle;
      while (err0 > _PI) err0 -= _2PI;
      while (err0 < -_PI) err0 += _2PI;
      float mech_err0 = err0 / (float)MOTOR_POLE_PAIRS;

      float err1 = calibrated_pole_zeros[p1] - mean_zero_angle;
      while (err1 > _PI) err1 -= _2PI;
      while (err1 < -_PI) err1 += _2PI;
      float mech_err1 = err1 / (float)MOTOR_POLE_PAIRS;

      float interp_err = (1.0f - frac) * mech_err0 + frac * mech_err1;
      encoder1.lut_errors[i] = interp_err;
      f_a1 += interp_err * cosf(rad);
      f_b1 += interp_err * sinf(rad);
      f_a2 += interp_err * cosf(2.0f * rad);
      f_b2 += interp_err * sinf(2.0f * rad);
    }
    encoder1.fourier_A1 = 2.0f * f_a1 / (float)ENCODER_LUT_SIZE;
    encoder1.fourier_B1 = 2.0f * f_b1 / (float)ENCODER_LUT_SIZE;
    encoder1.fourier_A2 = 2.0f * f_a2 / (float)ENCODER_LUT_SIZE;
    encoder1.fourier_B2 = 2.0f * f_b2 / (float)ENCODER_LUT_SIZE;
    encoder1.is_lut_calibrated = true;
  }
  return true;
}

bool calibrateComprehensiveFOC(float align_voltage) {
  calib_user_aborted = false;
  bool ok = calibrateComprehensiveFOCSinglePass(align_voltage);
  if (!ok && align_voltage < CALIBRATION_BOOST_VOLTAGE && !calib_user_aborted) {
    Serial.println("\n================================================================================");
    Serial.printf("⚠️  ELECTRICAL ZERO ALIGNMENT STALLED @ %.2fV!\n", align_voltage);
    Serial.printf("⚡ AUTOMATIC TORQUE BOOST: Ramping to %.2fV and retrying multi-pole alignment...\n", CALIBRATION_BOOST_VOLTAGE);
    Serial.println("================================================================================\n");
    delay(400);
    ok = calibrateComprehensiveFOCSinglePass(CALIBRATION_BOOST_VOLTAGE);
    if (ok) {
      calib_align_voltage = CALIBRATION_BOOST_VOLTAGE;
    }
  }
  return ok;
}

// ============================================================================
// CONTINUOUS SPATIAL-BINNED HARMONIC CALIBRATION (128-SLOT SPATIAL GRID)
// ============================================================================
bool calibrateDynamicContinuousSinglePass(float calib_voltage) {
  float hold_voltage = min(calib_voltage, calib_hold_voltage);
  Serial.println("\n================================================================================");
  Serial.println(">>> 🚀 STARTING CONTINUOUS SPATIAL-BINNED HARMONIC CALIBRATION <<<");
  Serial.println("================================================================================");
  Serial.println("Target Motor       : GM3506 (11 Pole Pairs on ESP32-S3)");
  Serial.printf("Calib / Hold Volt  : %.2f V (sweep) / %.2f V (hold & settle)\n", calib_voltage, hold_voltage);
  Serial.println("Methodology        : 128-Slot Spatial Angular Grid with Direct DFT Extraction");
  Serial.println("Diagnostics        : Harmonics 1 (Eccentricity), 2 (Tilt), 3 (Clamp), 4 (Saturation)");
  Serial.println("Kinematics         : Soft DC Align -> CW Ramp -> 2-Turn CW -> Continuous Reversal -> 2-Turn CCW");
  Serial.println("Press 'x' to STOP anytime");
  Serial.println("--------------------------------------------------------------------------------");

  encoder1.correction_mode = 0; // Raw angle during sweep
  motor1.controller = MotionControlType::velocity_openloop;
  driver1.enable();
  is_enabled = true;

  // Phase 0: Soft DC Alignment
  Serial.print("Phase 0: Soft DC Alignment & Rotor Settling (200 ms)... ");
  float theta_0 = encoder1.getRawMechanicalAngle();
  float reference_elec = theta_0 * (float)MOTOR_POLE_PAIRS;
  motor1.setPhaseVoltage(hold_voltage, 0, reference_elec);
  delay(200);

  // Quick direction qualification
  motor1.setPhaseVoltage(calib_voltage, 0, reference_elec + 0.35f);
  delay(120);
  float theta_dir = encoder1.getRawMechanicalAngle();
  motor1.setPhaseVoltage(hold_voltage, 0, reference_elec);
  delay(80);

  float d_dir = theta_dir - theta_0;
  while (d_dir > _PI) d_dir -= _2PI;
  while (d_dir < -_PI) d_dir += _2PI;
  if (fabsf(d_dir) < 0.02f) {
    Serial.println("FAILED (No motion detected in direction qualification)!");
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
    return false;
  }
  int dir_sign = (d_dir >= 0.0f) ? 1 : -1;
  motor1.sensor_direction = (dir_sign == 1) ? Direction::CW : Direction::CCW;
  Serial.printf("Aligned at %.2f deg | Direction: %s\n", theta_0 * 57.2957795f, dir_sign == 1 ? "CW (+1)" : "CCW (-1)");

  // Spatial Bin Accumulators
  const int NUM_BINS = ENCODER_LUT_SIZE; // 128 bins (2.8125 deg each)
  const float bin_width_rad = _2PI / (float)NUM_BINS;
  double fwd_sum[NUM_BINS] = {0};
  uint32_t fwd_cnt[NUM_BINS] = {0};
  double rev_sum[NUM_BINS] = {0};
  uint32_t rev_cnt[NUM_BINS] = {0};

  const float max_omega = 4.0f; // ~38 RPM
  const float ramp_time_s = 0.8f; // Smooth 0.8s runway ramp (prevents sudden jerk/slip)
  const float cruise_time_s = (2.0f * _2PI) / max_omega; // 2 turns = 4*pi / 4.0 = pi ~ 3.1416 s

  const float max_slip_rad = _PI / (2.0f * (float)MOTOR_POLE_PAIRS); // ~0.1428 rad = 8.18 deg

  auto checkUserAbort = [&]() -> bool {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        calib_user_aborted = true;
        motor1.setPhaseVoltage(0, 0, 0);
        driver1.disable();
        is_enabled = false;
        Serial.println("\n>>> 🛑 CALIBRATION ABORTED BY USER <<<");
        return true;
      }
    }
    return false;
  };

  uint32_t last_step_us = micros();
  float current_omega = 0.0f;

  // Helper lambda to run kinematic trajectory phase
  auto runKinematicPhase = [&](float target_omega, float duration_s, bool is_accel, int sample_mode, const char* phase_name) -> bool {
    Serial.printf(">>> %s (%.1f s)...\n", phase_name, duration_s);
    uint32_t start_ms = millis();
    uint32_t total_dur_ms = (uint32_t)(duration_s * 1000.0f);
    uint32_t phase_last_log_ms = start_ms;
    float start_omega = current_omega;
    uint32_t slip_active_since_ms = 0;

    while (millis() - start_ms < total_dur_ms) {
      if (checkUserAbort()) return false;

      uint32_t now_us = micros();
      float dt = (now_us - last_step_us) * 1e-6f;
      if (dt < 0.0005f) { // ~2 kHz rate limit
        delayMicroseconds(100);
        continue;
      }
      last_step_us = now_us;
      if (dt > 0.01f) dt = 0.01f;

      float progress = (float)(millis() - start_ms) / (float)total_dur_ms;
      if (progress > 1.0f) progress = 1.0f;

      if (is_accel) {
        current_omega = start_omega + (target_omega - start_omega) * progress;
      } else {
        current_omega = target_omega;
      }

      // Step electrical angle continuously without wrap or reset
      reference_elec += (float)dir_sign * (float)MOTOR_POLE_PAIRS * current_omega * dt;
      motor1.setPhaseVoltage(calib_voltage, 0, reference_elec);

      // Mechanical reference unwrapping
      float ref_mech = reference_elec / (float)MOTOR_POLE_PAIRS;
      float ref_norm = fmodf(ref_mech, _2PI);
      if (ref_norm < 0.0f) ref_norm += _2PI;

      // Sample actual rotor angle
      float meas_angle = encoder1.getRawMechanicalAngle();

      // Instantaneous error
      float err = meas_angle - ref_norm;
      while (err > _PI) err -= _2PI;
      while (err < -_PI) err += _2PI;

      // Defensive Pole-Slip Guard with 100ms Confirmation Debounce
      if (fabsf(err) > max_slip_rad) {
        if (slip_active_since_ms == 0) {
          slip_active_since_ms = millis();
        } else if (millis() - slip_active_since_ms >= 100) {
          motor1.setPhaseVoltage(0, 0, 0);
          driver1.disable();
          is_enabled = false;
          Serial.println("\n--------------------------------------------------------------------------------");
          Serial.printf("❌ POLE-SLIP ABORT: |err| = %.4f rad (%.2f deg) exceeded torque pull-out limit %.4f rad (%.2f deg) sustained for >100ms!\n",
                        fabsf(err), fabsf(err) * 57.2957795f, max_slip_rad, max_slip_rad * 57.2957795f);
          Serial.println("   ROOT CAUSE: External torque / friction exceeded hold torque, or align voltage too low.");
          Serial.println("   FIX: Check motor free rotation or increase voltage (e.g. 'y 4.2').");
          Serial.println("--------------------------------------------------------------------------------\n");
          return false;
        }
      } else {
        slip_active_since_ms = 0;
      }

      // Spatial bin accumulation
      if (sample_mode == 1) { // Forward acquisition
        float norm_meas = meas_angle;
        while (norm_meas < 0.0f) norm_meas += _2PI;
        while (norm_meas >= _2PI) norm_meas -= _2PI;
        int bin = (int)(norm_meas / bin_width_rad);
        if (bin < 0) bin += NUM_BINS;
        bin %= NUM_BINS;
        fwd_sum[bin] += err;
        fwd_cnt[bin]++;
      } else if (sample_mode == 2) { // Reverse acquisition
        float norm_meas = meas_angle;
        while (norm_meas < 0.0f) norm_meas += _2PI;
        while (norm_meas >= _2PI) norm_meas -= _2PI;
        int bin = (int)(norm_meas / bin_width_rad);
        if (bin < 0) bin += NUM_BINS;
        bin %= NUM_BINS;
        rev_sum[bin] += err;
        rev_cnt[bin]++;
      }

      if (millis() - phase_last_log_ms >= 500) {
        phase_last_log_ms = millis();
        Serial.printf("   [%s] Progress: %3.0f%% | Speed: %+5.1f RPM | Angle: %5.1f deg\n",
                      phase_name, progress * 100.0f, current_omega * 9.54929658551f, meas_angle * 57.2957795f);
      }
    }
    return true;
  };

  // Phase 1: Forward Run-Up Runway (CW 0 -> +38 RPM over 1.0 turn)
  if (!runKinematicPhase(+max_omega, ramp_time_s, true, 0, "Phase 1: Forward Run-Up Runway (0 -> 38 RPM)")) return false;

  // Phase 2: Forward Spatial Acquisition (CW 2.0 full turns @ steady 38 RPM)
  if (!runKinematicPhase(+max_omega, cruise_time_s, false, 1, "Phase 2: Forward Spatial Acquisition (2.0 Turns CW)")) return false;

  // Phase 3: Phase-Continuous Reversal Runway
  // Decelerate CW from 38 to 0 RPM over 1.0 turn (preserve continuous phase coordinate without reset!)
  if (!runKinematicPhase(0.0f, ramp_time_s, true, 0, "Phase 3a: Forward Deceleration (38 -> 0 RPM)")) return false;

  // Dwell at hold voltage for 300 ms (no stator zero reset)
  Serial.println(">>> Phase 3b: Phase-Continuous Reversal Dwell (300 ms hold)...");
  motor1.setPhaseVoltage(hold_voltage, 0, reference_elec);
  delay(300);
  motor1.setPhaseVoltage(calib_voltage, 0, reference_elec);
  last_step_us = micros();

  // Accelerate CCW from 0 to -38 RPM over 1.0 turn
  if (!runKinematicPhase(-max_omega, ramp_time_s, true, 0, "Phase 3c: Reverse Run-Up Runway (0 -> -38 RPM)")) return false;

  // Phase 4: Reverse Spatial Acquisition (CCW 2.0 full turns @ steady -38 RPM)
  if (!runKinematicPhase(-max_omega, cruise_time_s, false, 2, "Phase 4: Reverse Spatial Acquisition (2.0 Turns CCW)")) return false;

  // Decelerate CCW to 0 RPM over 1.0 turn
  runKinematicPhase(0.0f, ramp_time_s, true, 0, "Phase 4b: Reverse Deceleration (-38 -> 0 RPM)");
  motor1.setPhaseVoltage(0, 0, 0);
  driver1.disable();
  is_enabled = false;

  // Phase 5: Point-by-Point Cancellation & Normalization
  Serial.println("\n>>> Phase 5: Point-by-Point Cancellation & Normalization...");
  float fwd_mean[NUM_BINS] = {0};
  float rev_mean[NUM_BINS] = {0};
  float drag_free[NUM_BINS] = {0};
  int missing_fwd = 0, missing_rev = 0;

  for (int i = 0; i < NUM_BINS; i++) {
    if (fwd_cnt[i] == 0) missing_fwd++;
    if (rev_cnt[i] == 0) missing_rev++;
  }

  // Circular interpolation for any empty bins (due to communication/interrupt jitter)
  for (int i = 0; i < NUM_BINS; i++) {
    if (fwd_cnt[i] > 0) {
      fwd_mean[i] = (float)(fwd_sum[i] / fwd_cnt[i]);
    } else {
      int l = (i - 1 + NUM_BINS) % NUM_BINS;
      while (fwd_cnt[l] == 0 && l != i) l = (l - 1 + NUM_BINS) % NUM_BINS;
      int r = (i + 1) % NUM_BINS;
      while (fwd_cnt[r] == 0 && r != i) r = (r + 1) % NUM_BINS;
      fwd_mean[i] = (fwd_cnt[l] > 0 && fwd_cnt[r] > 0) ? 0.5f * ((float)(fwd_sum[l] / fwd_cnt[l]) + (float)(fwd_sum[r] / fwd_cnt[r])) : 0.0f;
    }

    if (rev_cnt[i] > 0) {
      rev_mean[i] = (float)(rev_sum[i] / rev_cnt[i]);
    } else {
      int l = (i - 1 + NUM_BINS) % NUM_BINS;
      while (rev_cnt[l] == 0 && l != i) l = (l - 1 + NUM_BINS) % NUM_BINS;
      int r = (i + 1) % NUM_BINS;
      while (rev_cnt[r] == 0 && r != i) r = (r + 1) % NUM_BINS;
      rev_mean[i] = (rev_cnt[l] > 0 && rev_cnt[r] > 0) ? 0.5f * ((float)(rev_sum[l] / rev_cnt[l]) + (float)(rev_sum[r] / rev_cnt[r])) : 0.0f;
    }

    // Directional drag and rotor lag cancel out at the exact same physical bin!
    drag_free[i] = 0.5f * (fwd_mean[i] + rev_mean[i]);
  }

  // Directional Drag / Lag Analysis (Rotor lag at steady speed)
  float mean_lag_rad = 0.0f;
  float max_lag_rad = -100.0f, min_lag_rad = 100.0f;
  int max_lag_bin = 0;
  for (int i = 0; i < NUM_BINS; i++) {
    float lag = 0.5f * (fwd_mean[i] - rev_mean[i]);
    mean_lag_rad += lag;
    if (lag > max_lag_rad) { max_lag_rad = lag; max_lag_bin = i; }
    if (lag < min_lag_rad) { min_lag_rad = lag; }
  }
  mean_lag_rad /= (float)NUM_BINS;
  encoder1.harmonic_mean_lag_deg = fabsf(mean_lag_rad) * 57.2957795f;
  encoder1.harmonic_lag_ripple_deg = (max_lag_rad - min_lag_rad) * 57.2957795f;
  encoder1.harmonic_max_drag_angle_deg = ((float)max_lag_bin + 0.5f) * bin_width_rad * 57.2957795f;

  // DC Offset A0 Subtraction for zero-mean LUT & Harmonic Parity
  float A0 = 0.0f;
  for (int i = 0; i < NUM_BINS; i++) {
    A0 += drag_free[i];
  }
  A0 /= (float)NUM_BINS;
  for (int i = 0; i < NUM_BINS; i++) {
    drag_free[i] -= A0;
    encoder1.lut_errors[i] = drag_free[i]; // Store into Mode 1 LUT for direct parity!
  }
  encoder1.is_lut_calibrated = true;

  // Phase 6: Direct DFT Extraction & Diagnostics
  Serial.println(">>> Phase 6: Direct Discrete Fourier Transform Extraction (Orders 1, 2, 3, 4)...");
  float A[5] = {0};
  float B[5] = {0};
  for (int n = 1; n <= 4; n++) {
    float sum_c = 0.0f, sum_s = 0.0f;
    for (int i = 0; i < NUM_BINS; i++) {
      float theta_i = ((float)i + 0.5f) * bin_width_rad;
      sum_c += drag_free[i] * cosf((float)n * theta_i);
      sum_s += drag_free[i] * sinf((float)n * theta_i);
    }
    A[n] = (2.0f / (float)NUM_BINS) * sum_c;
    B[n] = (2.0f / (float)NUM_BINS) * sum_s;
  }

  encoder1.fourier_A1 = A[1]; encoder1.fourier_B1 = B[1];
  encoder1.fourier_A2 = A[2]; encoder1.fourier_B2 = B[2];
  encoder1.fourier_A3 = A[3]; encoder1.fourier_B3 = B[3];
  encoder1.fourier_A4 = A[4]; encoder1.fourier_B4 = B[4];

  float h1_amp_deg = sqrtf(A[1] * A[1] + B[1] * B[1]) * 57.2957795f;
  float h2_amp_deg = sqrtf(A[2] * A[2] + B[2] * B[2]) * 57.2957795f;
  float h3_amp_deg = sqrtf(A[3] * A[3] + B[3] * B[3]) * 57.2957795f;
  float h4_amp_deg = sqrtf(A[4] * A[4] + B[4] * B[4]) * 57.2957795f;

  float max_e = -100.0f, min_e = 100.0f, sum_sq = 0.0f;
  for (int i = 0; i < NUM_BINS; i++) {
    if (drag_free[i] > max_e) max_e = drag_free[i];
    if (drag_free[i] < min_e) min_e = drag_free[i];
    sum_sq += drag_free[i] * drag_free[i];
  }
  float pk_pk_deg = (max_e - min_e) * 57.2957795f;
  float rms_deg = sqrtf(sum_sq / (float)NUM_BINS) * 57.2957795f;
  encoder1.harmonic_pk_pk_deg = pk_pk_deg;
  encoder1.harmonic_rms_deg = rms_deg;

  // Fit Quality (R^2) & Residual Ripple after H1 + H2 + H4 compensation
  float ss_tot = 0.0f, ss_res = 0.0f;
  float res_max = -100.0f, res_min = 100.0f;
  for (int i = 0; i < NUM_BINS; i++) {
    float theta_i = ((float)i + 0.5f) * bin_width_rad;
    float s1 = sinf(theta_i), c1 = cosf(theta_i);
    float s2 = 2.0f * s1 * c1, c2 = c1 * c1 - s1 * s1;
    float s4 = 2.0f * s2 * c2, c4 = c2 * c2 - s2 * s2;
    float harm_val = (A[1] * c1 + B[1] * s1) + (A[2] * c2 + B[2] * s2) + (A[4] * c4 + B[4] * s4);
    float res_err = drag_free[i] - harm_val;
    ss_tot += drag_free[i] * drag_free[i];
    ss_res += res_err * res_err;
    if (res_err > res_max) res_max = res_err;
    if (res_err < res_min) res_min = res_err;
  }
  float r_squared = (ss_tot > 1e-12f) ? (1.0f - (ss_res / ss_tot)) : 1.0f;
  if (r_squared < 0.0f) r_squared = 0.0f;
  encoder1.harmonic_r_squared = r_squared;
  encoder1.harmonic_res_pk_pk_deg = (res_max - res_min) * 57.2957795f;
  encoder1.harmonic_res_rms_deg = sqrtf(ss_res / (float)NUM_BINS) * 57.2957795f;

  float phi1 = atan2f(B[1], A[1]) * 57.2957795f; if (phi1 < 0.0f) phi1 += 360.0f;
  float phi2 = atan2f(B[2], A[2]) * 57.2957795f; if (phi2 < 0.0f) phi2 += 360.0f;
  float phi3 = atan2f(B[3], A[3]) * 57.2957795f; if (phi3 < 0.0f) phi3 += 360.0f;
  float phi4 = atan2f(B[4], A[4]) * 57.2957795f; if (phi4 < 0.0f) phi4 += 360.0f;

  // Print Complete Diagnostic & Physical Localization Report
  printHarmonicReport();

  // Phase 7: NVS Flash Save & Runtime Reconstruction
  Serial.println(">>> Phase 7: NVS Flash Save & Runtime Reconstruction...");
  encoder1.correction_mode = 2; // Activate Continuous Analytical Harmonics Mode
  Serial.println(">>> Sensor Correction Mode activated: MODE 2 (CONTINUOUS ANALYTICAL HARMONICS)");

  // Structured lines for Python GUI
  Serial.print("[HARMONIC_RESULTS]: ");
  Serial.printf("A1=%.6f,B1=%.6f,A2=%.6f,B2=%.6f,A3=%.6f,B3=%.6f,A4=%.6f,B4=%.6f,H1=%.4f,H2=%.4f,H3=%.4f,H4=%.4f,PkPk=%.4f,RMS=%.4f,Phi1=%.2f,Phi2=%.2f,Phi3=%.2f,Phi4=%.2f,R2=%.4f,ResPkPk=%.4f,ResRMS=%.4f,LagMean=%.4f,LagRipple=%.4f,LagPeakAng=%.1f\n",
                A[1], B[1], A[2], B[2], A[3], B[3], A[4], B[4],
                h1_amp_deg, h2_amp_deg, h3_amp_deg, h4_amp_deg, pk_pk_deg, rms_deg,
                phi1, phi2, phi3, phi4, r_squared,
                encoder1.harmonic_res_pk_pk_deg, encoder1.harmonic_res_rms_deg,
                encoder1.harmonic_mean_lag_deg, encoder1.harmonic_lag_ripple_deg, encoder1.harmonic_max_drag_angle_deg);

  Serial.print("[HARMONIC_BINS]: ");
  for (int i = 0; i < NUM_BINS; i++) {
    Serial.print(drag_free[i], 6);
    if (i < NUM_BINS - 1) Serial.print(",");
  }
  Serial.println();

  // Run 11-pole electrical zero alignment on the linearized sensor
  Serial.println("\n>>> Aligning 11-pole electrical zero vector under Harmonic Correction...");
  calibrateComprehensiveFOC(calib_voltage);

  // Save full profile to NVS
  saveConfigToNVS();
  Serial.println("================================================================================\n");
  return true;
}

bool calibrateDynamicContinuous(float calib_voltage) {
  calib_user_aborted = false;
  bool ok = calibrateDynamicContinuousSinglePass(calib_voltage);
  if (!ok && calib_voltage < CALIBRATION_BOOST_VOLTAGE && !calib_user_aborted) {
    Serial.println("\n================================================================================");
    Serial.printf("⚠️  HARMONIC CALIBRATION STALL / POLE-SLIP DETECTED @ %.2fV!\n", calib_voltage);
    Serial.printf("⚡ AUTOMATIC TORQUE BOOST: Ramping to %.2fV and retrying continuous sweep...\n", CALIBRATION_BOOST_VOLTAGE);
    Serial.println("================================================================================\n");
    delay(400);
    ok = calibrateDynamicContinuousSinglePass(CALIBRATION_BOOST_VOLTAGE);
    if (ok) {
      calib_align_voltage = CALIBRATION_BOOST_VOLTAGE;
    }
  }
  return ok;
}

// ============================================================================
// AUTOMATED 3-WAY SENSOR MODE BENCHMARK COMPARISON WITH COMMUTATION PARITY
// ============================================================================
void runBenchmarkComparison() {
  Serial.println("\n================================================================================");
  Serial.println(">>> 🏁 COMMENCING 3-WAY SENSOR MODE BENCHMARK COMPARISON <<<");
  Serial.println("================================================================================");
  Serial.println("Target Motor       : GM3506 (11 Pole Pairs on ESP32-S3)");
  Serial.println("Test Velocity      : 30.0 rad/s (~286.5 RPM closed-loop velocity)");
  Serial.println("Parity Protocol    : Full 11-Pole Electrical Zero Realignment Per Mode");
  Serial.println("Evaluated Modes    : Mode 0 (Raw), Mode 1 (128-PT LUT), Mode 2 (Harmonics)");
  Serial.println("Press 'x' to STOP anytime");
  Serial.println("--------------------------------------------------------------------------------");

  uint8_t orig_mode = encoder1.correction_mode;

  struct BenchResult {
    const char* mode_name;
    uint8_t mode_id;
    float read_latency_us;
    float mean_speed_rad_s;
    float jitter_rad_s;
    float tracking_mae_rad_s;
    float temp_start_c;
    float temp_end_c;
    float delta_temp_c;
    float calibrated_zero_rad;
  } results[3];

  const char* mode_names[3] = {"Raw (Uncalibrated)", "128-Point Stepped LUT", "Continuous Harmonics"};

  for (int m = 0; m < 3; m++) {
    results[m].mode_name = mode_names[m];
    results[m].mode_id = (uint8_t)m;

    Serial.printf("\n>>> [BENCHMARK STAGE %d/3]: Testing %s <<<\n", m + 1, mode_names[m]);
    encoder1.correction_mode = (uint8_t)m;

    // Commutation Parity Protocol: 11-pole electrical zero alignment for THIS mode
    Serial.printf("  1. Re-aligning 11-pole electrical zero for %s...\n", mode_names[m]);
    calibrateComprehensiveFOC(calib_align_voltage);
    results[m].calibrated_zero_rad = motor1.zero_electric_angle;

    // Measure baseline temperature
    results[m].temp_start_c = readMotorTemperatureC();

    // Measure Sensor Read Latency (1000 iterations)
    uint32_t lat_start_us = micros();
    volatile float dummy_angle = 0.0f;
    for (int k = 0; k < 1000; k++) {
      dummy_angle = encoder1.getSensorAngle();
    }
    uint32_t lat_dur_us = micros() - lat_start_us;
    results[m].read_latency_us = (float)lat_dur_us / 1000.0f;

    // 2. Closed-Loop Velocity Spin at 30 rad/s (286.5 RPM)
    Serial.println("  2. Spinning Closed-Loop @ 30.0 rad/s (286.5 RPM) for 3.0s...");
    target_velocity_rpm = 30.0f * 9.54929658551f;
    motor1.controller = MotionControlType::velocity;
    driver1.enable();
    is_enabled = true;

    // Settle for 1.0s
    uint32_t settle_start_ms = millis();
    bool aborted = false;
    while (millis() - settle_start_ms < 1000) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'x' || c == 'X') { aborted = true; break; }
      }
      motor1.loopFOC();
      motor1.move(30.0f);
      delayMicroseconds(50);
    }
    if (aborted) {
      motor1.setPhaseVoltage(0, 0, 0); driver1.disable(); is_enabled = false;
      Serial.println("\n>>> 🛑 BENCHMARK ABORTED BY USER <<<");
      return;
    }

    // Sampling window: 3.0s (~300 samples)
    const int SAMPLE_CAP = 300;
    float vel_samples[SAMPLE_CAP];
    int sample_count = 0;
    uint32_t sample_start_ms = millis();
    uint32_t last_sample_us = micros();

    while (millis() - sample_start_ms < 3000 && sample_count < SAMPLE_CAP) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == 'x' || c == 'X') { aborted = true; break; }
      }
      motor1.loopFOC();
      motor1.move(30.0f);

      uint32_t now_us = micros();
      if (now_us - last_sample_us >= 10000) { // 100 Hz sampling
        last_sample_us = now_us;
        vel_samples[sample_count++] = motor1.shaft_velocity;
      }
    }

    // Stop motor
    target_velocity_rpm = 0.0f;
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;

    if (aborted) {
      Serial.println("\n>>> 🛑 BENCHMARK ABORTED BY USER <<<");
      return;
    }

    // End temperature
    results[m].temp_end_c = readMotorTemperatureC();
    results[m].delta_temp_c = results[m].temp_end_c - results[m].temp_start_c;

    // Compute statistics
    float sum_v = 0.0f;
    float sum_mae = 0.0f;
    for (int i = 0; i < sample_count; i++) {
      sum_v += vel_samples[i];
      sum_mae += fabsf(vel_samples[i] - 30.0f);
    }
    float mean_v = (sample_count > 0) ? (sum_v / (float)sample_count) : 0.0f;
    results[m].mean_speed_rad_s = mean_v;
    results[m].tracking_mae_rad_s = (sample_count > 0) ? (sum_mae / (float)sample_count) : 0.0f;

    float sum_var = 0.0f;
    for (int i = 0; i < sample_count; i++) {
      float diff = vel_samples[i] - mean_v;
      sum_var += diff * diff;
    }
    results[m].jitter_rad_s = (sample_count > 1) ? sqrtf(sum_var / (float)(sample_count - 1)) : 0.0f;

    Serial.printf("  Latency: %.2f us | Speed: %.2f rad/s | Jitter (σ): %.3f rad/s | MAE: %.3f rad/s | ΔTemp: %+.1f°C\n",
                  results[m].read_latency_us, results[m].mean_speed_rad_s,
                  results[m].jitter_rad_s, results[m].tracking_mae_rad_s, results[m].delta_temp_c);

    Serial.printf("[BENCH_COMPARE_RESULT]: Mode=%d,Name=%s,Latency_us=%.2f,Mean_rad_s=%.2f,Jitter_rad_s=%.4f,MAE_rad_s=%.4f,DeltaTemp_C=%.2f,Zero_rad=%.5f\n",
                  m, mode_names[m], results[m].read_latency_us, results[m].mean_speed_rad_s,
                  results[m].jitter_rad_s, results[m].tracking_mae_rad_s, results[m].delta_temp_c, results[m].calibrated_zero_rad);

    delay(500); // Inter-stage thermal cooldown
  }

  // Restore active mode to Mode 2 (Harmonic) or original mode
  encoder1.correction_mode = (encoder1.fourier_A1 != 0.0f || encoder1.fourier_B1 != 0.0f) ? 2 : orig_mode;

  Serial.println("\n================================================================================");
  Serial.println("🏆 HEAD-TO-HEAD BENCHMARK COMPARISON SUMMARY TABLE (GM3506 @ 30 rad/s):");
  Serial.println("================================================================================");
  Serial.println("| Mode | Description           | Latency (us) | Jitter σ (rad/s) | MAE (rad/s) | ΔTemp (°C) | Zero Angle (rad) |");
  Serial.println("|------|-----------------------|--------------|------------------|-------------|------------|------------------|");
  for (int m = 0; m < 3; m++) {
    Serial.printf("| %-4d | %-21s | %12.2f | %16.3f | %11.3f | %+10.1f | %16.5f |\n",
                  results[m].mode_id, results[m].mode_name, results[m].read_latency_us,
                  results[m].jitter_rad_s, results[m].tracking_mae_rad_s, results[m].delta_temp_c,
                  results[m].calibrated_zero_rad);
  }
  Serial.println("================================================================================\n");
}

// ============================================================================
// MT6816 HIGH-SPEED SPI BUS INTEGRITY & LATENCY BENCHMARK (4 - 16 MHz)
// ============================================================================
void runSPISpeedBenchmark(bool keep_best = false) {
  if (encoder1.use_3wire) {
    Serial.println("\n[ERROR] SPI Speed Benchmark requires 4-Wire Hardware SPI mode ('b' to toggle).");
    return;
  }

  Serial.println("\n[SPI_BENCH_START]");
  Serial.println("================================================================================");
  Serial.println("⚡ MT6816 SPI CLOCK FREQUENCY & BUS INTEGRITY BENCHMARK (4 - 16 MHz)");
  Serial.println("================================================================================");
  Serial.println(">>> Testing SPI bus stability, read latency, and parity error rates...");

  bool prev_foc_paused = foc_paused;
  bool was_enabled = is_enabled;
  if (is_enabled) {
    motor1.setPhaseVoltage(0, 0, 0);
    driver1.disable();
    is_enabled = false;
  }
  foc_paused = true;
  vTaskDelay(pdMS_TO_TICKS(20));

  uint32_t orig_speed = encoder1.spi_speed;
  const uint32_t freqs[] = {4000000, 6000000, 8000000, 10000000, 12000000, 14000000, 16000000};
  const int n_freqs = sizeof(freqs) / sizeof(freqs[0]);

  struct SPITestRow {
    float freq_mhz;
    float latency_us;
    uint32_t parity_errors;
    uint32_t mag_faults;
    uint32_t reads;
    float err_rate_pct;
    bool stable;
  } rows[7];

  const uint32_t TEST_READS = 2500;
  uint32_t best_stable_speed = 4000000;
  float base_latency_us = 0.0f;

  for (int i = 0; i < n_freqs; i++) {
    uint32_t cur_freq = freqs[i];
    float mhz = (float)cur_freq / 1000000.0f;
    encoder1.setSPISpeed(cur_freq);
    delayMicroseconds(500);

    // Warm-up flush
    for (int w = 0; w < 50; w++) {
      uint8_t dummy_h = 0, dummy_l = 0;
      encoder1.read4WireHW(encoder1.cs_pin, dummy_h, dummy_l);
    }

    uint32_t parity_err = 0;
    uint32_t mag_err = 0;

    // Latency & stability measurement using existing benchmark timing pattern
    uint32_t lat_start_us = micros();
    for (uint32_t k = 0; k < TEST_READS; k++) {
      uint8_t h = 0, l = 0;
      encoder1.read4WireHW(encoder1.cs_pin, h, l);
      uint16_t angle_data = ((uint16_t)h << 8) | l;
      if (!MT6816Sensor::parityCheck(angle_data)) {
        parity_err++;
      }
      if ((l & 0x02) != 0) {
        mag_err++;
      }
    }
    uint32_t lat_dur_us = micros() - lat_start_us;

    float lat_us = (float)lat_dur_us / (float)TEST_READS;
    float err_pct = ((float)parity_err / (float)TEST_READS) * 100.0f;
    bool is_stable = (parity_err == 0 && mag_err == 0);

    if (i == 0) {
      base_latency_us = lat_us;
    }

    rows[i].freq_mhz = mhz;
    rows[i].latency_us = lat_us;
    rows[i].parity_errors = parity_err;
    rows[i].mag_faults = mag_err;
    rows[i].reads = TEST_READS;
    rows[i].err_rate_pct = err_pct;
    rows[i].stable = is_stable;

    if (is_stable && cur_freq >= best_stable_speed) {
      best_stable_speed = cur_freq;
    }

    // Machine-readable telemetry line
    Serial.printf("[SPI_BENCH_RESULT]: Freq_MHz=%.1f,Latency_us=%.2f,Parity_Errors=%u,Mag_Faults=%u,Reads=%u,Err_Rate_Pct=%.3f,Stable=%d\n",
                  mhz, lat_us, parity_err, mag_err, TEST_READS, err_pct, is_stable ? 1 : 0);

    Serial.printf("  Stage %d/%d: %4.1f MHz -> Latency: %5.2f us | Parity Errors: %u | Mag Faults: %u | Status: %s\n",
                  i + 1, n_freqs, mhz, lat_us, parity_err, mag_err, is_stable ? "✅ STABLE" : "❌ UNSTABLE");
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Summary Table
  Serial.println("\n================================================================================");
  Serial.println("🏆 SPI SPEED STABILITY & READ LATENCY LEADERBOARD (MT6816 4-WIRE SPI)");
  Serial.println("================================================================================");
  Serial.println("| Freq (MHz) | Latency (us) | Speedup vs 4M | Parity Err | Mag Faults | Bus Status     |");
  Serial.println("|------------|--------------|---------------|------------|------------|----------------|");
  for (int i = 0; i < n_freqs; i++) {
    float speedup = (rows[i].latency_us > 0.01f && base_latency_us > 0.01f) ? (base_latency_us / rows[i].latency_us) : 1.0f;
    Serial.printf("|   %4.1f     |    %6.2f    |     %4.2fx    |  %8u  |  %8u  | %-14s |\n",
                  rows[i].freq_mhz, rows[i].latency_us, speedup,
                  rows[i].parity_errors, rows[i].mag_faults,
                  rows[i].stable ? "✅ 100% STABLE" : "❌ DATA ERRORS");
  }
  Serial.println("================================================================================");

  float best_mhz = (float)best_stable_speed / 1000000.0f;
  if (keep_best) {
    encoder1.setSPISpeed(best_stable_speed);
    Serial.printf(">>> [APPLIED] Highest stable SPI speed retained: %.1f MHz\n", best_mhz);
  } else {
    encoder1.setSPISpeed(orig_speed);
    Serial.printf(">>> [RESTORED] Original SPI speed restored: %.1f MHz (Use 'spi_speed %.1f' or 'spi_test keep' to set)\n",
                  (float)orig_speed / 1000000.0f, best_mhz);
  }
  Serial.println("[SPI_BENCH_END]\n");

  foc_paused = prev_foc_paused;
  if (was_enabled) {
    driver1.enable();
    is_enabled = true;
  }
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

  float fwd_phase_kv = 0, fwd_v0 = 0, fwd_r2 = 0;
  float rev_phase_kv = 0, rev_v0 = 0, rev_r2 = 0;
  bool fwd_ok = calculateLinearRegression(fwd_v, fwd_rpm, fwd_count,
                                          fwd_phase_kv, fwd_v0, fwd_r2);
  bool rev_ok = calculateLinearRegression(rev_v, rev_rpm, rev_count,
                                          rev_phase_kv, rev_v0, rev_r2);

  float true_phase_kv = 0.0f, friction_v0 = 0.0f, best_r2 = 0.0f;
  if (fwd_ok && rev_ok) {
    true_phase_kv = (fwd_phase_kv + rev_phase_kv) * 0.5f;
    friction_v0 = fabsf(fwd_v0 / fwd_phase_kv + rev_v0 / rev_phase_kv) * 0.5f;
    best_r2 = (fwd_r2 + rev_r2) * 0.5f;
  } else if (fwd_ok) {
    true_phase_kv = fwd_phase_kv;
    friction_v0 = fabsf(fwd_v0 / fwd_phase_kv);
    best_r2 = fwd_r2;
  } else if (rev_ok) {
    true_phase_kv = rev_phase_kv;
    friction_v0 = fabsf(rev_v0 / rev_phase_kv);
    best_r2 = rev_r2;
  }

  // Standard BLDC Motor KV rating (RPM per Volt Line-to-Line / DC Bus):
  // SimpleFOC defines motor.KV_rating = RPM / (Uq * sqrt(3)) = Phase_KV / sqrt(3)
  float true_kv = (true_phase_kv > 0.1f) ? (true_phase_kv / _SQRT3) : 0.0f;
  float fwd_kv = (fwd_phase_kv > 0.1f) ? (fwd_phase_kv / _SQRT3) : 0.0f;
  float rev_kv = (rev_phase_kv > 0.1f) ? (rev_phase_kv / _SQRT3) : 0.0f;

  float ke_v_s_rad = (true_kv > 0.1f) ? (60.0f / (_2PI * true_kv)) : 0.0f;
  float kt_nm_a = ke_v_s_rad;
  // Maximum achievable linear speed @ 12V PSU (SpaceVectorPWM limit Uq_max = 12V / sqrt(3)):
  float max_rpm_12v =
      true_phase_kv * ((POWER_SUPPLY_VOLTAGE / _SQRT3) - friction_v0);
  if (max_rpm_12v < 0.0f) max_rpm_12v = 0.0f;
  float symmetry_pct = (fwd_kv > 1.0f && rev_kv > 1.0f)
                           ? (1.0f - fabsf(fwd_kv - rev_kv) / true_kv) * 100.0f
                           : 100.0f;

  motor_kv_rating = true_kv;
  motor1.KV_rating = true_kv;

  Serial.print("[KV_RESULT]: KV=");
  Serial.print(true_kv, 2);
  Serial.print(", PhaseKV=");
  Serial.print(true_phase_kv, 2);
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

void printFOCPerfReport() {
  const uint32_t cpu_mhz = 240;
  float cur_avg = g_foc_perf.avg_exec_us;
  float cur_max = g_foc_perf.max_exec_us;
  float worst_spk_us = (float)g_foc_perf.worst_spike_exec_cycles / (float)cpu_mhz;
  float worst_zero_us = (float)g_foc_perf.worst_spike_zero_cycles / (float)cpu_mhz;
  float worst_comm_us = (float)g_foc_perf.worst_spike_comm_cycles / (float)cpu_mhz;
  float worst_move_us = (float)g_foc_perf.worst_spike_move_cycles / (float)cpu_mhz;
  float worst_int_us = (float)g_foc_perf.worst_spike_interval_cycles / (float)cpu_mhz;

  Serial.println("\n========================================================");
  Serial.println("  ESP32-S3 FOC REAL-TIME LATENCY & JITTER PROFILER");
  Serial.println("========================================================");
  Serial.printf("  Loop Commutation Rate    : %.1f kHz (Period: ~%.1f us)\n", 
                g_telemetry.foc_hz * 0.001f, 
                g_telemetry.foc_hz > 0 ? (1000000.0f / g_telemetry.foc_hz) : 0.0f);
  Serial.printf("  Recent Avg Exec Duration : %.2f us (Active FOC Duty: ~%.1f%%)\n", 
                cur_avg, 
                g_telemetry.foc_hz > 0 ? (cur_avg * g_telemetry.foc_hz * 0.0001f) : 0.0f);
  Serial.printf("  Recent Peak Exec (Window): %.2f us\n", cur_max);
  Serial.printf("  Peak Interval (Jitter)   : %.2f us\n", g_telemetry.max_interval_us);
  Serial.printf("  Total Loops Executed     : %u\n", g_foc_perf.total_cycles_executed);
  Serial.println("--------------------------------------------------------");
  Serial.printf("  Execution Spikes (>%dus) : %u\n", FOC_SPIKE_EXEC_THRESHOLD_US, g_foc_perf.exec_spikes_total);
  Serial.printf("  Interval Spikes (>%dus)  : %u\n", FOC_SPIKE_INTERVAL_THRESHOLD_US, g_foc_perf.interval_spikes_total);
  if (g_foc_perf.worst_spike_exec_cycles > 0) {
    Serial.println("  [Worst Execution Spike Breakdown]:");
    Serial.printf("    Total Spike Duration   : %.2f us (observed at T+%lu ms)\n", worst_spk_us, g_foc_perf.worst_spike_time_ms);
    Serial.printf("    -- Zero/Lag Transform  : %.2f us\n", worst_zero_us);
    Serial.printf("    -- loopFOC (Comm+SPI)  : %.2f us\n", worst_comm_us);
    Serial.printf("    -- move (PID+LPF)      : %.2f us\n", worst_move_us);
    Serial.printf("    -- Loop Interval       : %.2f us\n", worst_int_us);
  } else {
    Serial.println("  [Worst Execution Spike]  : None detected (Clean real-time execution).");
  }
  Serial.println("--------------------------------------------------------");
  Serial.printf("  MT6816 Total Reads       : %u\n", encoder1.total_reads);
  Serial.printf("  MT6816 SPI Master Clock  : %.1f MHz (%s)\n",
                (float)encoder1.spi_speed * 0.000001f,
                encoder1.use_3wire ? "3-Wire Bitbang" : "4-Wire Hardware SPI");
  Serial.printf("  MT6816 Parity Retries    : %u (SPI noise/timing retry)\n", encoder1.parity_retries);
  Serial.printf("  MT6816 Parity Failures   : %u\n", encoder1.parity_errors);
  Serial.printf("  MT6816 Mag Faults        : %u\n", encoder1.mag_faults);
  Serial.println("========================================================\n");
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

  if (cmd.equalsIgnoreCase("perf") || cmd.equalsIgnoreCase("diag") || cmd.equalsIgnoreCase("foc_perf") || cmd.equalsIgnoreCase("profile")) {
    printFOCPerfReport();
    return;
  }
  if (cmd.equalsIgnoreCase("perf_reset") || cmd.equalsIgnoreCase("diag_reset")) {
    g_foc_perf.exec_spikes_total = 0;
    g_foc_perf.interval_spikes_total = 0;
    g_foc_perf.worst_spike_exec_cycles = 0;
    g_foc_perf.worst_spike_zero_cycles = 0;
    g_foc_perf.worst_spike_comm_cycles = 0;
    g_foc_perf.worst_spike_move_cycles = 0;
    g_foc_perf.worst_spike_interval_cycles = 0;
    g_foc_perf.latest_spike_exec_cycles = 0;
    g_foc_perf.latest_spike_zero_cycles = 0;
    g_foc_perf.latest_spike_comm_cycles = 0;
    g_foc_perf.latest_spike_move_cycles = 0;
    g_foc_perf.latest_spike_interval_cycles = 0;
    encoder1.parity_retries = 0;
    encoder1.parity_errors = 0;
    encoder1.mag_faults = 0;
    Serial.println(">>> FOC Latency Profiler counters reset.");
    return;
  }
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
  if (cmd.startsWith("CALIB_DYN") || cmd.startsWith("calib_dyn")) {
    float v = calib_align_voltage;
    String arg_v = cmd.substring(9);
    arg_v.trim();
    if (arg_v.length() > 0 && arg_v.toFloat() > 0.0f) {
      v = arg_v.toFloat();
    }
    if (kv_test_running)
      abortKVTest("Interrupted by Dynamic Calibration");
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(15));
    calibrateDynamicContinuous(v);
    foc_paused = false;
    return;
  }
  if (cmd.startsWith("CALIB_LUT") || cmd.startsWith("calib_lut")) {
    float v = calib_align_voltage;
    String arg_v = cmd.substring(9);
    arg_v.trim();
    if (arg_v.length() > 0 && arg_v.toFloat() > 0.0f) {
      v = arg_v.toFloat();
    }
    if (kv_test_running)
      abortKVTest("Interrupted by LUT Calibration");
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(15));
    calibrate256PointLUT(v);
    foc_paused = false;
    return;
  }
  if (cmd.startsWith("SET_MODE") || cmd.startsWith("set_mode")) {
    String arg_m = cmd.substring(8);
    arg_m.trim();
    int m_val = arg_m.toInt();
    if (m_val >= 0 && m_val <= 2) {
      encoder1.correction_mode = (uint8_t)m_val;
      Serial.printf(">>> Sensor Correction Mode set to: %d (%s)\n",
                    m_val,
                    m_val == 0 ? "RAW (Uncalibrated)" : (m_val == 1 ? "128-PT LUT" : "CONTINUOUS HARMONICS"));
    } else {
      Serial.println(">>> Invalid mode: use 0 (Raw), 1 (LUT), or 2 (Harmonics)");
    }
    return;
  }
  if (cmd.equalsIgnoreCase("BENCH_COMPARE") || cmd.equalsIgnoreCase("bench_compare")) {
    if (kv_test_running)
      abortKVTest("Interrupted by Benchmark");
    foc_paused = true;
    vTaskDelay(pdMS_TO_TICKS(15));
    runBenchmarkComparison();
    foc_paused = false;
    return;
  }
  if (cmd.startsWith("SPI_TEST") || cmd.startsWith("spi_test") || cmd.startsWith("TEST_SPI") || cmd.startsWith("test_spi") || cmd.startsWith("SPI_BENCH") || cmd.startsWith("spi_bench")) {
    if (kv_test_running)
      abortKVTest("Interrupted by SPI Benchmark");
    bool keep = (cmd.indexOf("keep") >= 0 || cmd.indexOf("KEEP") >= 0);
    runSPISpeedBenchmark(keep);
    return;
  }
  if (cmd.startsWith("SPI_SPEED") || cmd.startsWith("spi_speed")) {
    String spd_str = cmd.substring(9);
    spd_str.trim();
    if (spd_str.length() == 0) {
      Serial.printf(">>> Active MT6816 SPI Master Clock: %.2f MHz (%u Hz) | Bus: %s\n",
                    (float)encoder1.spi_speed / 1000000.0f, encoder1.spi_speed,
                    encoder1.use_3wire ? "3-Wire Bitbang" : "4-Wire Hardware SPI");
    } else {
      float val = spd_str.toFloat();
      if (val > 0.0f) {
        uint32_t hz = (val <= 50.0f) ? (uint32_t)(val * 1000000.0f) : (uint32_t)val;
        if (hz < 1000000) hz = 1000000;
        if (hz > 20000000) hz = 20000000;
        encoder1.setSPISpeed(hz);
        
        // Measure sensor read latency (1000 iterations) using existing latency check pattern
        uint32_t lat_start_us = micros();
        volatile float dummy_angle = 0.0f;
        for (int k = 0; k < 1000; k++) {
          dummy_angle = encoder1.getSensorAngle();
        }
        uint32_t lat_dur_us = micros() - lat_start_us;
        float read_latency_us = (float)lat_dur_us / 1000.0f;
        
        Serial.printf(">>> Active MT6816 SPI Clock set to %.2f MHz (%u Hz) | Read Latency: %.2f us\n",
                      (float)encoder1.spi_speed / 1000000.0f, encoder1.spi_speed, read_latency_us);
      }
    }
    return;
  }
  if (cmd.equalsIgnoreCase("HARM_INFO") || cmd.equalsIgnoreCase("harm_info") || cmd.equalsIgnoreCase("HARM") || cmd.equalsIgnoreCase("harm") || cmd.equalsIgnoreCase("HARM_REPORT")) {
    printHarmonicReport();
    return;
  }
  if (cmd.startsWith("GA_STREAM") || cmd.startsWith("ga_stream")) {
    String arg_s = cmd.substring(9);
    arg_s.trim();
    if (arg_s.equalsIgnoreCase("OFF") || arg_s.equals("0")) {
      ga_stream_active = false;
      Serial.println(">>> GA Dedicated Telemetry Stream: DISABLED");
    } else {
      int val = arg_s.toInt();
      if (val >= 10 && val <= 200) {
        ga_stream_interval_ms = 1000 / val;
      } else {
        ga_stream_interval_ms = 20; // 50 Hz default optimal rate
      }
      ga_stream_active = true;
      Serial.printf(">>> GA Dedicated Telemetry Stream: ENABLED @ %lu Hz (%lu ms)\n",
                    1000 / ga_stream_interval_ms, ga_stream_interval_ms);
    }
    return;
  }
  if (cmd.startsWith("PID_EVAL") || cmd.startsWith("pid_eval")) {
    float p = 0, i = 0, d = 0, tf = 0, tgt = 0;
    int parsed = sscanf(cmd.c_str() + 8, "%f %f %f %f %f", &p, &i, &d, &tf, &tgt);
    if (parsed >= 4) {
      motor1.PID_velocity.P = p;
      motor1.PID_velocity.I = i;
      motor1.PID_velocity.D = d;
      if (tf > 0.0f) motor1.LPF_velocity.Tf = tf;
      if (parsed >= 5) {
        target_velocity_rpm = tgt;
        motor1.controller = MotionControlType::velocity;
        driver1.enable();
        is_enabled = true;
      }
      Serial.printf(">>> PID_EVAL: P=%.4f I=%.4f D=%.5f Tf=%.5fs Target=%.1f RPM\n",
                    p, i, d, tf, tgt);
    }
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
      // Check if harmonic coefficients exist
      float harm_mag = fabsf(encoder1.fourier_A1) + fabsf(encoder1.fourier_B1) +
                       fabsf(encoder1.fourier_A2) + fabsf(encoder1.fourier_B2) +
                       fabsf(encoder1.fourier_A4) + fabsf(encoder1.fourier_B4);
      if (harm_mag > 1e-5f) {
        const float step_rad = _2PI / (float)ENCODER_LUT_SIZE;
        for (int i = 0; i < ENCODER_LUT_SIZE; i++) {
          float rad = ((float)i + 0.5f) * step_rad;
          float s1 = sinf(rad), c1 = cosf(rad);
          float s2 = 2.0f * s1 * c1, c2 = c1 * c1 - s1 * s1;
          float s4 = 2.0f * s2 * c2, c4 = c2 * c2 - s2 * s2;
          encoder1.lut_errors[i] = (encoder1.fourier_A1 * c1 + encoder1.fourier_B1 * s1) +
                                   (encoder1.fourier_A2 * c2 + encoder1.fourier_B2 * s2) +
                                   (encoder1.fourier_A4 * c4 + encoder1.fourier_B4 * s4);
        }
        encoder1.is_lut_calibrated = true;
      } else if (is_calibrated) {
        // Synthesize from 11-pole zeros
        const float step_rad = _2PI / (float)ENCODER_LUT_SIZE;
        const float pole_span_rad = _2PI / (float)MOTOR_POLE_PAIRS;
        for (int i = 0; i < ENCODER_LUT_SIZE; i++) {
          float rad = (float)i * step_rad;
          float pole_f = rad / pole_span_rad;
          int p0 = ((int)pole_f) % MOTOR_POLE_PAIRS;
          int p1 = (p0 + 1) % MOTOR_POLE_PAIRS;
          float frac = pole_f - (float)((int)pole_f);

          float err0 = calibrated_pole_zeros[p0] - base_zero_electric_angle;
          while (err0 > _PI) err0 -= _2PI;
          while (err0 < -_PI) err0 += _2PI;
          float mech_err0 = err0 / (float)MOTOR_POLE_PAIRS;

          float err1 = calibrated_pole_zeros[p1] - base_zero_electric_angle;
          while (err1 > _PI) err1 -= _2PI;
          while (err1 < -_PI) err1 += _2PI;
          float mech_err1 = err1 / (float)MOTOR_POLE_PAIRS;

          encoder1.lut_errors[i] = (1.0f - frac) * mech_err0 + frac * mech_err1;
        }
        encoder1.is_lut_calibrated = true;
      }
    }
    if (!encoder1.is_lut_calibrated) {
      Serial.println("[LUT_DUMP] No active calibration found — run 'CALIB_DYN', 'w', or 'c' first.");
      break;
    }
    printLUTMetrics();
    Serial.println(
        "// --- 128-POINT NON-LINEARITY CORRECTION LUT (RADIANS) ---");
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
    ga_stream_active = false;
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
    Serial.println("  CALIB_DYN     : 🚀 Continuous Spatial-Binned Harmonic Calibration");
    Serial.println("  CALIB_LUT     : 🔬 Stepped 128-Point Dual-Pass LUT Calibration");
    Serial.println("  SET_MODE <0-2>: Set Mode: 0=Raw, 1=128-PT LUT, 2=Harmonics");
    Serial.println("  BENCH_COMPARE : 🏁 Run 3-Way Mode Comparison with Commutation Parity");
    Serial.println("  SPI_TEST [keep]: ⚡ Benchmark SPI 4-16 MHz Latency & Bus Stability");
    Serial.println("  SPI_SPEED <mhz>: ⚡ Set MT6816 SPI Clock Frequency (e.g. 4, 8, 12, 16)");
    Serial.println("  HARM_INFO     : 🔬 Print Full Physical Harmonic Diagnostic & Apex Report");
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
    Serial.println("  u             : View LUT Non-Linearity Metrics & Dump "
                   "128-Point Table");
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
    Serial.println("  GA_STREAM [hz]: 🧬 Dedicated High-Rate PID GA Stream (default 50 Hz, 0=OFF)");
    Serial.println("  PID_EVAL p i d tf [tgt]: ⚡ Set all PID parameters + target atomically");
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
  driver1.voltage_limit = POWER_SUPPLY_VOLTAGE; // Inverter bus rail limit (12.0V)
  driver1.init();

  // Initialize MT6816 SPI Sensor
  encoder1.init();
  motor1.linkSensor(&encoder1);

  // Link Driver & Configure SimpleFOC Motor
  motor1.linkDriver(&driver1);
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.controller = MotionControlType::velocity;

  motor1.voltage_limit = DEFAULT_VOLTAGE_LIMIT; // 6.93V linear SpaceVectorPWM phase voltage limit
  motor1.KV_rating = motor_kv_rating;
  motor1.velocity_limit = (DEFAULT_VELOCITY_LIMIT_RPM * 0.104719755f);
  motor1.PID_velocity.P = DEFAULT_PID_P;
  motor1.PID_velocity.I = DEFAULT_PID_I;
  motor1.PID_velocity.D = DEFAULT_PID_D;
  motor1.PID_velocity.output_ramp = DEFAULT_OUTPUT_RAMP_V_PER_S;
  motor1.LPF_velocity.Tf = (DEFAULT_LPF_TF_MS * 0.001f);
  motor1.motion_downsample =
      15; // Decouple velocity PID (~1.5 kHz) from FOC commutation (~25 kHz)

  motor1.init();

  motor1.zero_electric_angle = base_zero_electric_angle;
  motor1.sensor_direction = Direction::CCW;

  driver1.disable();
  is_enabled = false;

  // Attempt Auto-Loading from Flash
  loadConfigFromNVS();

  // Launch Dedicated Real-Time FOC Task on Core 0 (Priority 24: maximum system priority)
  xTaskCreatePinnedToCore(focTaskLoop, "FOC_Task", 8192, NULL, 24,
                          &focTaskHandle, 0);

  Serial.println("  System Ready. FOC Loop running on Core 0 @ high rate.");
  Serial.println("  Type 'w' for Master Fourier Calibration, 'c' for Fast "
                 "Align, 'n' for Current Trim.");
  Serial.println("==================================================\n");

  // Clear boot initialization transients so startup jitter does not pollute runtime stats
  delay(150);
  g_foc_perf.exec_spikes_total = 0;
  g_foc_perf.interval_spikes_total = 0;
  g_foc_perf.worst_spike_exec_cycles = 0;
  g_foc_perf.worst_spike_zero_cycles = 0;
  g_foc_perf.worst_spike_comm_cycles = 0;
  g_foc_perf.worst_spike_move_cycles = 0;
  g_foc_perf.worst_spike_interval_cycles = 0;
  g_foc_perf.latest_spike_exec_cycles = 0;
  g_foc_perf.latest_spike_zero_cycles = 0;
  g_foc_perf.latest_spike_comm_cycles = 0;
  g_foc_perf.latest_spike_move_cycles = 0;
  g_foc_perf.latest_spike_interval_cycles = 0;
}

void loop() {
  // 1. Process KV State Machine (Timing & Stage Transitions on Core 1)
  if (kv_test_running) {
    processKVStateMachine();
  }

  // 2. Non-blocking Serial Command Processor
  processSerialCommand();

  // 3. High-Rate Dedicated GA Telemetry Stream (Optimal Hz, compact ~18 bytes)
  uint32_t now_ms = millis();
  if (ga_stream_active &&
      (now_ms - last_ga_stream_time_ms >= ga_stream_interval_ms)) {
    last_ga_stream_time_ms = now_ms;

    TelemetrySnapshot snap;
    portENTER_CRITICAL(&telemSpinlock);
    snap = g_telemetry;
    portEXIT_CRITICAL(&telemSpinlock);

    char ga_buf[48];
    snprintf(ga_buf, sizeof(ga_buf), "[GA]:%+6.1f,%+5.2f\r\n", snap.actual_rpm, snap.uq);
    Serial.write((const uint8_t *)ga_buf, strlen(ga_buf));
  }
  // 4. Periodic Full Telemetry Stream (20 Hz, active only when GA stream is inactive)
  else if (show_telemetry && !ga_stream_active &&
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

    char line_buf[320];
    char target_buf[32];
    if (snap.kv_test || snap.ctrl_mode == MotionControlType::torque) {
      snprintf(target_buf, sizeof(target_buf), "%+5.2f V ",
               snap.target_voltage);
    } else {
      snprintf(target_buf, sizeof(target_buf), "%+6.1f RPM", snap.target_rpm);
    }

    char mode_buf[32];
    if (encoder1.correction_mode == 2) {
      snprintf(mode_buf, sizeof(mode_buf), "%s(HARMONIC)",
               encoder1.use_3wire ? "3-Wire" : "4-Wire");
    } else if (encoder1.correction_mode == 1) {
      snprintf(mode_buf, sizeof(mode_buf), "%s(FOURIER:128)",
               encoder1.use_3wire ? "3-Wire" : "4-Wire");
    } else {
      snprintf(mode_buf, sizeof(mode_buf), "%s(RAW)",
               encoder1.use_3wire ? "3-Wire" : "4-Wire");
    }

    snprintf(
        line_buf, sizeof(line_buf),
        "Ctrl: %-5s | Target: %s | Actual: %+6.1f RPM | Angle: %6.2f deg | "
        "Uq: %+5.2fV | Ud: %+5.2fV | Raw: %5u/16383 | Mode: %s | Mag: %s | "
        "Parity: %s %s | FOC: %.1fkHz | Lat: %.0f/%.0fus | Spk: %u\r\n",
        ctrl_str, target_buf, snap.actual_rpm, snap.angle_deg, snap.uq, snap.ud,
        snap.raw14, mode_buf, mag_str, parity_str, dial_segments[seg],
        snap.foc_hz * 0.001f, snap.avg_exec_us, snap.max_exec_us, snap.exec_spikes);

    Serial.write((const uint8_t *)line_buf, strlen(line_buf));

    // Asynchronous spike alert notification (runs on Core 1 without stalling Core 0)
    static uint32_t last_alerted_spikes = 0;
    if (snap.exec_spikes > last_alerted_spikes) {
      last_alerted_spikes = snap.exec_spikes;
      float spk_us = (float)g_foc_perf.latest_spike_exec_cycles / 240.0f;
      float comm_us = (float)g_foc_perf.latest_spike_comm_cycles / 240.0f;
      float move_us = (float)g_foc_perf.latest_spike_move_cycles / 240.0f;
      float zero_us = (float)g_foc_perf.latest_spike_zero_cycles / 240.0f;
      float worst_us = (float)g_foc_perf.worst_spike_exec_cycles / 240.0f;
      char alert_buf[240];
      snprintf(alert_buf, sizeof(alert_buf),
               "\n>>> [FOC-SPIKE-ALERT] Recent Spike: %.1fus (comm/SPI: %.1fus, move: %.1fus, zero: %.1fus | Worst: %.1fus) | Total Spikes: %u (Period Jitter Spikes: %u) <<<\r\n",
               spk_us, comm_us, move_us, zero_us, worst_us, snap.exec_spikes, snap.interval_spikes);
      Serial.write((const uint8_t *)alert_buf, strlen(alert_buf));
    }
  }

  // Yield to FreeRTOS on Core 1
  delay(1);
}
