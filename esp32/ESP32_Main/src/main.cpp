#include "config.h"
#include "electronic_diff.h"
#include "imu_bno08x.h"
#include "mcu_telemetry_protocol.h"
#include "sensor_fusion.h"
#include "servo_controller.h"
#include "spsc_triple_buffer.h"
#include "stm32_link.h"
#include <Arduino.h>
#include <Preferences.h>
#include <esp_cpu.h>
#include <esp_timer.h>

// =============================================================================
// Inter-Core Shared Memory Structures (Allocated in Internal SRAM)
// =============================================================================
struct SharedSensorInputs {
  float target_v;             // Commanded forward velocity (m/s)
  float target_steer_rad;     // Commanded center steering angle (rad)
  float gyro_z;               // IMU body yaw rate (rad/s)
  uint32_t gyro_seq;          // Fresh IMU gyro report counter
  uint64_t gyro_timestamp_us; // IMU gyro report timestamp (us)
  float accel_fwd;            // IMU forward linear acceleration (m/s^2)
  float accel_lat;            // IMU lateral acceleration (m/s^2)
  float roll_deg;             // IMU Euler roll
  float pitch_deg;            // IMU Euler pitch
  float yaw_deg;              // IMU Euler yaw
  float fl_rpm;               // Front-Left motor actual RPM
  float fr_rpm;               // Front-Right motor actual RPM
  float rl_rpm;               // Rear-Left motor actual RPM
  float rr_rpm;               // Rear-Right motor actual RPM
  bool override_active;       // Manual override flag
  bool fake_data_mode;        // Synthetic benchmark mode
};

struct SharedControlOutputs {
  // Actuator Setpoints
  float fl_target_rpm;
  float fr_target_rpm;
  float rl_target_rpm;
  float rr_target_rpm;
  float steer_rad;
  float steer_deg;
  float driver_steer_deg;
  float servo_pulse_us;

  // Kinematics & States
  float rate_limited_v;
  float v_FL, v_FR, v_RL, v_RR;
  float omega_cmd;
  float scale_factor;
  VehicleSlipState slip_state;
  float yaw_error;

  // Torque Envelope & Rear TV Diagnostics
  float delta_omega_tv;
  float delta_vtrq_tv;
  float vtrq_max_FL;
  float vtrq_max_RL;
  float vtrq_max_RR;
  float vtrq_ff_FL;
  float vtrq_ff_RL;
  float vtrq_ff_RR;
  bool is_countersteering;

  // Fused Odometry
  float v_est;
  float v_wheels;
  float yaw_est;
  float total_distance_m;
  float slip_ratio;

  // 400 Hz Real-Time Profiler Metrics
  uint32_t cycle_count;
  float exec_us;
  float exec_us_min;
  float exec_us_max;
  float exec_us_avg;
  float jitter_us;
  uint32_t overrun_count;
};

// Lock-Free Triple Buffers
static SPSCTripleBuffer<SharedSensorInputs> s_sensorInputs;
static SPSCTripleBuffer<SharedControlOutputs> s_controlOutputs;

// Core 1 Control Task & Timer Handles
static TaskHandle_t s_controlTaskHandle = nullptr;
static esp_timer_handle_t s_controlTimer = nullptr;

// Peripherals & Subsystems (Instantiated in SRAM)
static ElectronicDifferential4WD s_ediff;
static ServoController s_servo;
static IMUEncoderFusion s_fusion;
static STM32DualLink s_stm32Link;
static BNO08xIMU s_imu;
#define s_piSerial                                                             \
  Serial // Telemetry & Motion Commands routed directly over USB Serial cable

// Global State
static volatile bool s_fake_data_active =
    false; // Default to live sensors (manual toggle only)
static volatile float s_manual_v = 0.0f;
static volatile float s_manual_steer = 0.0f;
static volatile bool s_manual_override = false;
static volatile bool s_cli_override_active = false;
static volatile bool s_manual_servo_direct = false;
static volatile float s_direct_servo_pulse = SERVO_PULSE_NEUTRAL_US;
static volatile float s_servo_trim_us = 0.0f;
static volatile bool s_estop_latched = false;
static volatile bool s_drive_stopped = true;
static volatile bool s_imu_usb_stream = false;

static inline float servoPulseToAngle(float pulse_us) {
  if (pulse_us <= SERVO_PULSE_NEUTRAL_US) {
    if (pulse_us < SERVO_PULSE_MIN_US)
      pulse_us = SERVO_PULSE_MIN_US;
    return ((pulse_us - SERVO_PULSE_MIN_US) /
            (SERVO_PULSE_NEUTRAL_US - SERVO_PULSE_MIN_US)) *
           90.0f;
  } else {
    if (pulse_us > SERVO_PULSE_MAX_US)
      pulse_us = SERVO_PULSE_MAX_US;
    return 90.0f + ((pulse_us - SERVO_PULSE_NEUTRAL_US) /
                    (SERVO_PULSE_MAX_US - SERVO_PULSE_NEUTRAL_US)) *
                       90.0f;
  }
}

static inline float servoAngleToPulse(float angle_deg) {
  if (angle_deg < 0.0f)
    angle_deg = 0.0f;
  if (angle_deg > 180.0f)
    angle_deg = 180.0f;
  if (angle_deg <= 90.0f) {
    return SERVO_PULSE_MIN_US +
           (angle_deg / 90.0f) * (SERVO_PULSE_NEUTRAL_US - SERVO_PULSE_MIN_US);
  } else {
    return SERVO_PULSE_NEUTRAL_US +
           ((angle_deg - 90.0f) / 90.0f) *
               (SERVO_PULSE_MAX_US - SERVO_PULSE_NEUTRAL_US);
  }
}

static void sendServoTelemetry() {
  float cur_pulse = s_manual_servo_direct
                        ? fmaxf(SERVO_HARD_MIN_US,
                                fminf(SERVO_HARD_MAX_US,
                                      s_direct_servo_pulse + s_servo_trim_us))
                        : s_servo.getCurrentPulseUs();
  float angle = servoPulseToAngle(cur_pulse);
  float period_us = 1000000.0f / (float)SERVO_PWM_FREQ_HZ;
  Serial.printf("{\"pin\":%d,\"pulse_us\":%d,\"angle\":%.2f,\"freq_hz\":%d,"
                "\"period_us\":%.1f,\"enabled\":%s}\n",
                SERVO_STEER_PWM_PIN, (int)cur_pulse, angle,
                (int)SERVO_PWM_FREQ_HZ, period_us,
                s_servo.isEnabled() ? "true" : "false");
}

static inline STM32WheelCmd mapWheelToSTM32(float rpm,
                                            float v_min,
                                            float v_max,
                                            float v_ff,
                                            float sign) {
  if (sign >= 0.0f) {
    return {rpm, v_min, v_max, v_ff};
  }
  return {-rpm, -v_max, -v_min, -v_ff};
}

static inline float estimateWheelVtrq(float omega_cmd,
                                      float v_min,
                                      float v_max,
                                      float v_ff,
                                      float omega_meas) {
  if (v_min > v_max) {
    float mid = 0.5f * (v_min + v_max);
    v_min = mid;
    v_max = mid;
  }
  float est = v_ff + 0.10f * (omega_cmd - omega_meas);
  return fmaxf(v_min, fminf(v_max, est));
}

// =============================================================================
// Hardware Timer ISR Callback (Fires periodically every 2500 us / 400 Hz)
// =============================================================================
static void IRAM_ATTR onControlTimerAlarm(void *arg) {
  if (s_controlTaskHandle != nullptr) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(s_controlTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// =============================================================================
// CORE 1: Real-Time Deterministic Control Task (400 Hz / Priority 24)
// =============================================================================
static void IRAM_ATTR realTimeControlTask(void *pvParameters) {
  s_controlTaskHandle = xTaskGetCurrentTaskHandle();

  // Start 400 Hz High-Resolution Hardware Timer (2500 microseconds)
  const esp_timer_create_args_t timer_args = {.callback = &onControlTimerAlarm,
                                              .arg = nullptr,
                                              .dispatch_method = ESP_TIMER_TASK,
                                              .name = "ctrl_400hz"};
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_controlTimer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(s_controlTimer, CONTROL_LOOP_PERIOD_US));

  uint32_t cycle_counter = 0;
  uint32_t overrun_counter = 0;
  float min_exec_us = 99999.0f;
  float max_exec_us = 0.0f;
  float avg_exec_us = 0.0f;
  int64_t last_tick_time_us = esp_timer_get_time();
  float max_jitter_us = 0.0f;
  static ElectronicDiffOutputs s_prev_diff = {};
  static bool s_prev_diff_valid = false;

  for (;;) {
    // Wait for deterministic hardware timer notification
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    int64_t now_us = esp_timer_get_time();
    int64_t period_us = now_us - last_tick_time_us;
    last_tick_time_us = now_us;

    float jitter = fabsf((float)period_us - (float)CONTROL_LOOP_PERIOD_US);
    if (cycle_counter > 50 && jitter > max_jitter_us) {
      max_jitter_us = jitter;
    }

    uint32_t t_start_cycles = esp_cpu_get_ccount();

    // 1. Read latest snapshot from Core 0 via lock-free triple buffer
    s_sensorInputs.updateRead();
    const SharedSensorInputs &inputs = s_sensorInputs.getReadBuffer();

    // 1b. Drain STM32 feedback inside the deterministic task so the UART link
    // is never delayed by IMU SPI waits / USB shell work on Core 0.
    s_stm32Link.update();

    // Working variables
    float target_v;
    float target_steer;
    float gyro_z;
    uint32_t gyro_seq = 0;
    uint64_t gyro_ts_us = 0;
    float accel_fwd;
    float fl_rpm, fr_rpm, rl_rpm, rr_rpm;

    if (inputs.fake_data_mode) {
      // -----------------------------------------------------------------
      // SYNTHETIC BENCHMARK SIGNAL GENERATOR (Fake Data Mode)
      // -----------------------------------------------------------------
      float t_sec = (float)cycle_counter * CONTROL_LOOP_DT_S;
      target_v = 1.5f + 3.0f * sinf(t_sec * 0.5f);
      target_steer = 0.45f * sinf(t_sec * 1.2f);
      float clean_omega = (target_v * tanf(target_steer)) / VEHICLE_WHEELBASE_M;
      gyro_z = clean_omega + 0.03f * sinf(t_sec * 10.0f);
      gyro_seq = cycle_counter + 1U;
      gyro_ts_us = (uint64_t)now_us;
      accel_fwd =
          (3.0f * 0.5f * cosf(t_sec * 0.5f)) + 0.1f * sinf(t_sec * 15.0f);

      float rad_to_rpm = 60.0f / (2.0f * 3.14159265f);
      float sim_rpm = (target_v / VEHICLE_WHEEL_RADIUS_M) * rad_to_rpm *
                      (VEHICLE_MOTOR_DIR);
      fl_rpm = sim_rpm;
      fr_rpm = sim_rpm;
      rl_rpm = sim_rpm;
      rr_rpm = sim_rpm;
    } else {
      // Live Sensor Inputs from Core 0
      target_v = inputs.override_active ? s_manual_v : inputs.target_v;
      target_steer =
          inputs.override_active ? s_manual_steer : inputs.target_steer_rad;
      gyro_z = inputs.gyro_z;
      gyro_seq = inputs.gyro_seq;
      gyro_ts_us = inputs.gyro_timestamp_us;
      accel_fwd = inputs.accel_fwd;
      fl_rpm = WHEEL_SIGN_FL * s_stm32Link.getFL_RPM();
      fr_rpm = WHEEL_SIGN_FR * s_stm32Link.getFR_RPM();
      rl_rpm = WHEEL_SIGN_RL * s_stm32Link.getRL_RPM();
      rr_rpm = WHEEL_SIGN_RR * s_stm32Link.getRR_RPM();

      // Hardware Ramp Speed Governor (Safety speed ceiling on incline/pitch)
      const float ramp_pitch_threshold_deg = 8.0f;
      const float ramp_speed_ceiling_mps = 2.0f;
      if (fabsf(inputs.pitch_deg) >= ramp_pitch_threshold_deg) {
        if (target_v > 0.0f) {
          target_v = fminf(target_v, ramp_speed_ceiling_mps);
        } else if (target_v < 0.0f) {
          target_v = fmaxf(target_v, -ramp_speed_ceiling_mps);
        }
      }
    }

    // 2. Estimate axle torque effort from previous tick's commanded V_trq bounds & FF
    float front_vtrq_avg = 0.0f;
    float rear_vtrq_avg = 0.0f;
    if (s_prev_diff_valid && !s_estop_latched && !s_drive_stopped) {
      constexpr float rpm_to_rad_s = (2.0f * 3.14159265f) / 60.0f;
      float vtrq_fl = estimateWheelVtrq(s_prev_diff.omega_FL, s_prev_diff.vtrq_min_FL,
                                        s_prev_diff.vtrq_max_FL, s_prev_diff.vtrq_ff_FL,
                                        fl_rpm * rpm_to_rad_s);
      float vtrq_fr = estimateWheelVtrq(s_prev_diff.omega_FR, s_prev_diff.vtrq_min_FR,
                                        s_prev_diff.vtrq_max_FR, s_prev_diff.vtrq_ff_FR,
                                        fr_rpm * rpm_to_rad_s);
      float vtrq_rl = estimateWheelVtrq(s_prev_diff.omega_RL, s_prev_diff.vtrq_min_RL,
                                        s_prev_diff.vtrq_max_RL, s_prev_diff.vtrq_ff_RL,
                                        rl_rpm * rpm_to_rad_s);
      float vtrq_rr = estimateWheelVtrq(s_prev_diff.omega_RR, s_prev_diff.vtrq_min_RR,
                                        s_prev_diff.vtrq_max_RR, s_prev_diff.vtrq_ff_RR,
                                        rr_rpm * rpm_to_rad_s);
#if TEMP_3WD_FR_DEAD
      (void)vtrq_fr;
      front_vtrq_avg = vtrq_fl;
#else
      front_vtrq_avg = 0.5f * (vtrq_fl + vtrq_fr);
#endif
      rear_vtrq_avg = 0.5f * (vtrq_rl + vtrq_rr);
    }

    // 3. Execute 4WD Electronic Differential, Axle Envelopes & Rear TV
    static float s_last_v_est = 0.0f;
    ElectronicDiffOutputs diff =
        s_ediff.update(target_v, target_steer, gyro_z, CONTROL_LOOP_DT_S,
                       DEFAULT_ROAD_MU, s_last_v_est, gyro_seq, gyro_ts_us);

    // 4. Command Single Steering Servo on LEDC PWM (GPIO 21 @ 333 Hz)
    float servo_pulse_cmd =
        (s_manual_servo_direct ? s_direct_servo_pulse : diff.servo_pulse_us) +
        s_servo_trim_us;
    servo_pulse_cmd =
        fmaxf(SERVO_HARD_MIN_US, fminf(SERVO_HARD_MAX_US, servo_pulse_cmd));
    s_servo.setPulseUs(servo_pulse_cmd);

    // 5. Execute Torque-Gated Reference Odometry + IMU Complementary Sensor Fusion
    FusionOutputs fusion =
        s_fusion.update(fl_rpm, fr_rpm, rl_rpm, rr_rpm, gyro_z, accel_fwd,
                        diff.steer_rad, CONTROL_LOOP_DT_S, front_vtrq_avg,
                        rear_vtrq_avg);
    s_last_v_est = fusion.v_est;

    // 6. Measure Compute Latency (Cycles -> Microseconds @ 240 MHz)
    uint32_t t_exec_cycles = esp_cpu_get_ccount() - t_start_cycles;
    float exec_us = (float)t_exec_cycles * (1.0f / 240.0f);

    if (exec_us > (float)CONTROL_LOOP_PERIOD_US) {
      overrun_counter++;
    }

    if (cycle_counter > 50) {
      if (exec_us < min_exec_us)
        min_exec_us = exec_us;
      if (exec_us > max_exec_us)
        max_exec_us = exec_us;
      avg_exec_us = 0.995f * avg_exec_us + 0.005f * exec_us;
    } else {
      avg_exec_us = exec_us;
    }
    cycle_counter++;

    // 6b. Dispatch velocity + torque-voltage command to STM32s at jitter-free 400 Hz
    if (s_estop_latched || s_drive_stopped) {
      s_stm32Link.emergencyStop();
      s_prev_diff_valid = false;
    } else {
      STM32WheelCmd cmd_fl = mapWheelToSTM32(diff.rpm_FL, diff.vtrq_min_FL,
                                             diff.vtrq_max_FL, diff.vtrq_ff_FL,
                                             WHEEL_SIGN_FL);
#if TEMP_3WD_FR_DEAD
      STM32WheelCmd cmd_fr = {0.0f, 0.0f, 0.0f, 0.0f};
#else
      STM32WheelCmd cmd_fr = mapWheelToSTM32(diff.rpm_FR, diff.vtrq_min_FR,
                                             diff.vtrq_max_FR, diff.vtrq_ff_FR,
                                             WHEEL_SIGN_FR);
#endif
      STM32WheelCmd cmd_rl = mapWheelToSTM32(diff.rpm_RL, diff.vtrq_min_RL,
                                             diff.vtrq_max_RL, diff.vtrq_ff_RL,
                                             WHEEL_SIGN_RL);
      STM32WheelCmd cmd_rr = mapWheelToSTM32(diff.rpm_RR, diff.vtrq_min_RR,
                                             diff.vtrq_max_RR, diff.vtrq_ff_RR,
                                             WHEEL_SIGN_RR);
      s_stm32Link.sendDriveTorque(cmd_fl, cmd_fr, cmd_rl, cmd_rr);
      s_prev_diff = diff;
      s_prev_diff_valid = true;
    }

    // 7. Commit outputs to Core 0 via lock-free triple buffer
    SharedControlOutputs &out = s_controlOutputs.getWriteBuffer();
    out.fl_target_rpm = diff.rpm_FL;
    out.fr_target_rpm = diff.rpm_FR;
    out.rl_target_rpm = diff.rpm_RL;
    out.rr_target_rpm = diff.rpm_RR;
    out.steer_rad = diff.steer_rad;
    out.steer_deg = diff.steer_deg;
    out.driver_steer_deg = diff.driver_steer_deg;
    out.servo_pulse_us = servo_pulse_cmd;
    out.rate_limited_v = diff.rate_limited_v;
    out.v_FL = diff.v_FL;
    out.v_FR = diff.v_FR;
    out.v_RL = diff.v_RL;
    out.v_RR = diff.v_RR;
    out.omega_cmd = diff.omega_cmd;
    out.scale_factor = diff.scale_factor;
    out.slip_state = diff.slip_state;
    out.yaw_error = diff.yaw_error;

    out.delta_omega_tv = diff.delta_omega_tv;
    out.delta_vtrq_tv = diff.delta_vtrq_tv;
    out.vtrq_max_FL = diff.vtrq_max_FL;
    out.vtrq_max_RL = diff.vtrq_max_RL;
    out.vtrq_max_RR = diff.vtrq_max_RR;
    out.vtrq_ff_FL = diff.vtrq_ff_FL;
    out.vtrq_ff_RL = diff.vtrq_ff_RL;
    out.vtrq_ff_RR = diff.vtrq_ff_RR;
    out.is_countersteering = diff.is_countersteering;

    out.v_est = fusion.v_est;
    out.v_wheels = fusion.v_wheels;
    out.yaw_est = fusion.yaw_est;
    out.total_distance_m = fusion.total_distance_m;
    out.slip_ratio = fusion.slip_ratio;

    out.cycle_count = cycle_counter;
    out.exec_us = exec_us;
    out.exec_us_min = min_exec_us;
    out.exec_us_max = max_exec_us;
    out.exec_us_avg = avg_exec_us;
    out.jitter_us = jitter;
    out.overrun_count = overrun_counter;

    s_controlOutputs.commitWrite();
  }
}

// =============================================================================
// Non-Blocking ASCII Command Handler (Zero Stream Timeout Stalls)
// =============================================================================
static void handleAsciiCommand(const char *raw_line) {
  String line(raw_line);
  line.trim();
  if (line.length() == 0)
    return;

  if (line.equalsIgnoreCase("PING")) {
    Serial.println("PONG");
  } else if (line.equalsIgnoreCase("D") || line.equalsIgnoreCase("DISABLE")) {
    s_manual_v = 0.0f;
    s_manual_override = false;
    s_cli_override_active = false;
    s_manual_servo_direct = true;
    s_servo.disable();
    sendServoTelemetry();
  } else if (line.equalsIgnoreCase("E") || line.equalsIgnoreCase("ENABLE")) {
    s_servo.enable();
    sendServoTelemetry();
  } else if (line.charAt(0) == 'P' || line.charAt(0) == 'p') {
    String valStr = line.substring(1);
    valStr.trim();
    float us = valStr.toFloat();
    if (us >= SERVO_PULSE_MIN_US && us <= SERVO_PULSE_MAX_US) {
      s_direct_servo_pulse = us;
      s_manual_servo_direct = true;
      s_servo.enable();
      float trimmed_us = fmaxf(
          SERVO_HARD_MIN_US,
          fminf(SERVO_HARD_MAX_US, s_direct_servo_pulse + s_servo_trim_us));
      s_servo.setPulseUs(trimmed_us);
      // Map 1294..1836 us (1294=Left +MAX_STEER, 1565=Center 0, 1836=Right
      // -MAX_STEER)
      if (us <= SERVO_PULSE_NEUTRAL_US) {
        s_manual_steer = ((SERVO_PULSE_NEUTRAL_US - us) /
                          (SERVO_PULSE_NEUTRAL_US - SERVO_PULSE_MIN_US)) *
                         VEHICLE_MAX_STEER_RAD;
      } else {
        s_manual_steer = -((us - SERVO_PULSE_NEUTRAL_US) /
                           (SERVO_PULSE_MAX_US - SERVO_PULSE_NEUTRAL_US)) *
                         VEHICLE_MAX_STEER_RAD;
      }
      s_cli_override_active = true;
      s_manual_override = true;
      sendServoTelemetry();
    } else {
      Serial.printf(
          "{\"error\":\"Pulse out of bounds (%.0f-%.0f us): %.1f\"}\n",
          SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US, us);
    }
  } else if (line.equalsIgnoreCase("BENCH") || line.equalsIgnoreCase("FAKE")) {
    s_fake_data_active = !s_fake_data_active;
    s_manual_servo_direct = false;
    if (s_fake_data_active) {
      s_cli_override_active = false;
      s_manual_override = false;
      if (!s_estop_latched && s_drive_stopped) {
        s_stm32Link.enableDrivers();
        s_drive_stopped = false;
      }
    } else {
      s_drive_stopped = true;
      s_stm32Link.emergencyStop();
    }
    Serial.printf("\n>>> [MODE] Synthetic 400Hz Benchmark Mode: %s\n",
                  s_fake_data_active ? "ENABLED (Fake Data)"
                                     : "DISABLED (Live Sensors)");
  } else if (line.equalsIgnoreCase("STATUS") || line.equalsIgnoreCase("S")) {
    sendServoTelemetry();
    const SharedControlOutputs &c = s_controlOutputs.getReadBuffer();
    Serial.println(
        F("\n========================================================"));
    Serial.println(
        F(" FOLKRACE ESP32-S3 REAL-TIME DUAL-CORE STATUS REPORT    "));
    Serial.println(
        F("========================================================"));
    Serial.printf(
        " Mode: %s | Core 1 Control Loop: 400.0 Hz (Period: 2500 us)\n",
        s_fake_data_active ? "BENCHMARK (SYNTHETIC)" : "LIVE SENSORS");
    Serial.printf(" Compute Latency: Current: %.2f us | Min: %.2f us | Max: "
                  "%.2f us | Avg: %.2f us\n",
                  c.exec_us, c.exec_us_min, c.exec_us_max, c.exec_us_avg);
    Serial.printf(" Core 1 CPU Utilization: %.2f%% (Headroom: %.2f%%)\n",
                  (c.exec_us_avg / 2500.0f) * 100.0f,
                  100.0f - (c.exec_us_avg / 2500.0f) * 100.0f);
    Serial.printf(" Jitter: %.2f us | Deadline Overruns: %u / %u cycles\n",
                  c.jitter_us, c.overrun_count, c.cycle_count);
    Serial.printf(
        " Steering: %.2f deg (%.4f rad) | Servo PWM: %.1f us | Trim: %+.1f us (GPIO %d)\n",
        c.steer_deg, c.steer_rad, c.servo_pulse_us, s_servo_trim_us, SERVO_STEER_PWM_PIN);
    Serial.printf(
        " Speed Cmd: %.2f m/s (Rate-Limited: %.2f m/s) | Scale: %.2f\n",
        s_manual_v, c.rate_limited_v, c.scale_factor);
    Serial.printf(
        " Motor Targets (RPM): FL: %+.1f | FR: %+.1f | RL: %+.1f | RR: %+.1f\n",
        c.fl_target_rpm, c.fr_target_rpm, c.rl_target_rpm, c.rr_target_rpm);
    Serial.printf(" Fused Odometry: v_est = %.2f m/s | v_wheels = %.2f m/s | "
                  "Dist = %.2f m\n",
                  c.v_est, c.v_wheels, c.total_distance_m);
    Serial.printf(
        " Slip State: %s (Slip Ratio: %.2f, Yaw Error: %+.2f rad/s, Countersteer: %s)\n",
        c.slip_state == SLIP_OVERSTEER
            ? "OVERSTEER"
            : (c.slip_state == SLIP_UNDERSTEER ? "UNDERSTEER" : "NORMAL"),
        c.slip_ratio, c.yaw_error, c.is_countersteering ? "YES" : "NO");
    Serial.printf(
        " Traction Ctrl: GYRO=%d TV=%d FENV=%d OS_MU=%.2f | TV(Kp=%.2f Kd=%.3f Vff=%.2f) | dOmegaTV=%+.2f rad/s dVff=%+.2f V\n",
        s_ediff.isGyroAssistEnabled() ? 1 : 0,
        s_ediff.isRearTVEnabled() ? 1 : 0,
        s_ediff.isFrontEnvelopeEnabled() ? 1 : 0,
        s_ediff.getOversteerMuCap(),
        s_ediff.getTvKpPsi(),
        s_ediff.getTvKdPsi(),
        s_ediff.getTvMaxVff(),
        c.delta_omega_tv,
        c.delta_vtrq_tv);
    Serial.printf(
        " Vtrq Envelope: FL(max=%.2f ff=%+.2f) RL(max=%.2f ff=%+.2f) RR(max=%.2f ff=%+.2f)\n",
        c.vtrq_max_FL, c.vtrq_ff_FL,
        c.vtrq_max_RL, c.vtrq_ff_RL,
        c.vtrq_max_RR, c.vtrq_ff_RR);
    Serial.printf(" STM32 Front (FL/FR): %s | Actual RPM: %+.1f / %+.1f | "
                  "Pkts: %u (Err: %u) | En: %d/%d | Prog: %u%%\n",
                  s_stm32Link.isFrontConnected() ? "ONLINE" : "OFFLINE",
                  s_stm32Link.getFL_RPM(), s_stm32Link.getFR_RPM(),
                  s_stm32Link.getFront().getValidPacketCount(),
                  s_stm32Link.getFront().getChecksumErrorCount(),
                  s_stm32Link.getFront().isM1Enabled(),
                  s_stm32Link.getFront().isM2Enabled(),
                  s_stm32Link.getFront().getCalibProgress());
    Serial.printf(" STM32 Rear  (RL/RR): %s | Actual RPM: %+.1f / %+.1f | "
                  "Pkts: %u (Err: %u) | En: %d/%d | Prog: %u%%\n",
                  s_stm32Link.isRearConnected() ? "ONLINE" : "OFFLINE",
                  s_stm32Link.getRL_RPM(), s_stm32Link.getRR_RPM(),
                  s_stm32Link.getRear().getValidPacketCount(),
                  s_stm32Link.getRear().getChecksumErrorCount(),
                  s_stm32Link.getRear().isM1Enabled(),
                  s_stm32Link.getRear().isM2Enabled(),
                  s_stm32Link.getRear().getCalibProgress());
    Serial.println(
        F("========================================================\n"));
  } else if (line.equalsIgnoreCase("ESTOP") || line.equalsIgnoreCase("STOP") ||
             line.equalsIgnoreCase("X")) {
    s_manual_v = 0.0f;
    s_manual_override = false;
    s_cli_override_active = false;
    s_fake_data_active = false;
    s_estop_latched = true;
    s_drive_stopped = true;
    s_stm32Link.emergencyStop();
    Serial.println(F(">>> 🛑 EMERGENCY STOP BROADCAST TO ALL STM32s!"));
  } else if (line.equalsIgnoreCase("MOTORS") ||
             line.equalsIgnoreCase("START")) {
    s_estop_latched = false;
    s_drive_stopped = false;
    s_stm32Link.enableDrivers();
    Serial.println(F(">>> ⚡ MOTORS ENABLED ON ALL STM32s."));
  } else if (line.equalsIgnoreCase("DISABLE")) {
    s_manual_v = 0.0f;
    s_manual_override = false;
    s_cli_override_active = false;
    s_fake_data_active = false;
    s_drive_stopped = true;
    s_stm32Link.emergencyStop();
    Serial.println(F(">>> 🔌 MOTOR DRIVERS DISABLED ON ALL STM32s."));
  } else if (line.startsWith("PID ") || line.startsWith("pid ")) {
    float kp = 0.1f, ki = 1.5f;
    sscanf(line.c_str() + 4, "%f %f", &kp, &ki);
    s_stm32Link.setPID(kp, ki);
    Serial.printf(">>> Velocity PID Dispatched to STM32s: Kp=%.3f, Ki=%.3f\n",
                  kp, ki);
  } else if (line.equalsIgnoreCase("SAVE")) {
    s_stm32Link.saveFlash();
    Serial.println(F(">>> 💾 FLASH SAVE DISPATCHED TO ALL STM32s."));
  } else if (line.equalsIgnoreCase("DFU")) {
    s_stm32Link.enterDFU(true, true);
    Serial.println(F(">>> 🔄 DFU BOOTLOADER REBOOT DISPATCHED TO ALL STM32s."));
  } else if (line.startsWith("CALIB ") || line.startsWith("calib ")) {
    int axle = 0, motor = 1;
    float v = 4.0f;
    sscanf(line.c_str() + 6, "%d %d %f", &axle, &motor, &v);
    s_stm32Link.startCalibration(axle, motor, v);
    Serial.printf(">>> Calibration Dispatched: Axle %d, Motor %d @ %.1fV\n",
                  axle, motor, v);
  } else if (line.startsWith("V ") || line.startsWith("v ")) {
    s_manual_v = line.substring(2).toFloat();
    s_cli_override_active = (fabsf(s_manual_v) > 0.001f);
    s_manual_override = s_cli_override_active;
    s_fake_data_active = false;
    if (!s_cli_override_active) {
      s_drive_stopped = true;
      s_stm32Link.emergencyStop();
    } else if (!s_estop_latched && s_drive_stopped) {
      s_stm32Link.enableDrivers();
      s_drive_stopped = false;
    }
    Serial.printf(">>> Manual Speed Setpoint: %.2f m/s\n", s_manual_v);
  } else if (line.equalsIgnoreCase("BRAKE")) {
    s_manual_v = 0.0f;
    s_cli_override_active = true;
    s_manual_override = true;
    s_fake_data_active = false;
    if (!s_estop_latched && s_drive_stopped) {
      s_stm32Link.enableDrivers();
      s_drive_stopped = false;
    }
    Serial.println(F(">>> Active Brake Hold (0.00 m/s, drivers enabled)"));
  } else if (line.charAt(0) == 'A' || line.charAt(0) == 'a') {
    String valStr = line.substring(1);
    valStr.trim();
    float deg = valStr.toFloat();
    if (deg >= 0.0f && deg <= 180.0f) {
      s_direct_servo_pulse = servoAngleToPulse(deg);
      s_manual_servo_direct = true;
      s_servo.enable();
      float trimmed_us = fmaxf(
          SERVO_HARD_MIN_US,
          fminf(SERVO_HARD_MAX_US, s_direct_servo_pulse + s_servo_trim_us));
      s_servo.setPulseUs(trimmed_us);
      // Map 0..180 deg dial (0=Left 1294us -> +MAX_STEER, 90=Center 1565us ->
      // 0, 180=Right 1836us -> -MAX_STEER)
      s_manual_steer = ((90.0f - deg) / 90.0f) * VEHICLE_MAX_STEER_RAD;
      s_cli_override_active = true;
      s_manual_override = true;
      sendServoTelemetry();
    } else {
      Serial.printf("{\"error\":\"Angle out of bounds (0-180 deg): %.1f\"}\n",
                    deg);
    }
  } else if (line.startsWith("TRIM ") || line.startsWith("trim ")) {
    float trim_us = line.substring(5).toFloat();
    trim_us = fmaxf(-500.0f, fminf(500.0f, trim_us));
    s_servo_trim_us = trim_us;
    Serial.printf(
        ">>> Servo neutral trim: %+.1f us (effective neutral: %.1f us)\n",
        s_servo_trim_us, SERVO_PULSE_NEUTRAL_US + s_servo_trim_us);
  } else if (line.equalsIgnoreCase("TRIM_SAVE")) {
    Preferences prefs;
    prefs.begin("servo_cfg", false);
    prefs.putFloat("trim_us", s_servo_trim_us);
    prefs.end();
    Serial.printf(
        ">>> Servo neutral trim saved to NVS: %+.1f us (effective neutral: %.1f us)\n",
        s_servo_trim_us, SERVO_PULSE_NEUTRAL_US + s_servo_trim_us);
  } else if (line.equalsIgnoreCase("TRIM_GET")) {
    Serial.printf(
        "{\"servo_trim_us\":%.1f,\"servo_neutral_us\":%.1f,\"servo_center_us\":%.1f}\n",
        s_servo_trim_us, SERVO_PULSE_NEUTRAL_US,
        SERVO_PULSE_NEUTRAL_US + s_servo_trim_us);
  } else if (line.equalsIgnoreCase("RESET")) {
    s_manual_v = 0.0f;
    s_manual_steer = 0.0f;
    s_manual_override = false;
    s_cli_override_active = false;
    s_manual_servo_direct = false;
    s_drive_stopped = true;
    s_stm32Link.emergencyStop();
    s_ediff.reset();
    s_fusion.reset();
    s_servo.setPulseUs(fmaxf(
        SERVO_HARD_MIN_US,
        fminf(SERVO_HARD_MAX_US, SERVO_PULSE_NEUTRAL_US + s_servo_trim_us)));
    Serial.println(F(">>> Controller & Sensor Fusion State Reset."));
  } else if (line.length() == 1 && strchr("tTrR12345+-0mM", line[0])) {
    s_imu.handleSerialCommand(line[0], Serial);
  } else if (line.equalsIgnoreCase("IMU") || line.equalsIgnoreCase("I")) {
    s_imu.printCurrentReadings(Serial);
  } else if (line.equalsIgnoreCase("IMU_STREAM 1") ||
             line.equalsIgnoreCase("IMU_STREAM ON")) {
    s_imu_usb_stream = true;
    Serial.println(F("{\"imu_stream\":true}"));
  } else if (line.equalsIgnoreCase("IMU_STREAM 0") ||
             line.equalsIgnoreCase("IMU_STREAM OFF")) {
    s_imu_usb_stream = false;
    Serial.println(F("{\"imu_stream\":false}"));
  } else if (line.equalsIgnoreCase("IMU_TARE_ALL")) {
    s_imu.tareAllAxes(Serial);
  } else if (line.equalsIgnoreCase("IMU_TARE_YAW")) {
    s_imu.tareYawOnly(Serial);
  } else if (line.equalsIgnoreCase("IMU_SAVE")) {
    s_imu.saveTare(Serial);
  } else if (line.equalsIgnoreCase("IMU_CLEAR")) {
    s_imu.clearTare(Serial);
  } else if (line.startsWith("GYRO ") || line.startsWith("gyro ")) {
    bool en = (line.substring(5).toInt() != 0);
    s_ediff.setGyroAssistEnabled(en);
    Serial.printf(">>> Servo Gyro Countersteer: %s\n", en ? "ENABLED" : "DISABLED");
  } else if (line.startsWith("TV ") || line.startsWith("tv ")) {
    bool en = (line.substring(3).toInt() != 0);
    s_ediff.setRearTVEnabled(en);
    Serial.printf(">>> Rear Torque Vectoring: %s\n", en ? "ENABLED" : "DISABLED");
  } else if (line.startsWith("FENV ") || line.startsWith("fenv ")) {
    bool en = (line.substring(5).toInt() != 0);
    s_ediff.setFrontEnvelopeEnabled(en);
    Serial.printf(">>> Front Steering Torque Envelope: %s\n", en ? "ENABLED" : "DISABLED");
  } else if (line.startsWith("TV_GAINS ") || line.startsWith("tv_gains ")) {
    float kp = s_ediff.getTvKpPsi();
    float kd = s_ediff.getTvKdPsi();
    float max_vff = s_ediff.getTvMaxVff();
    int n = sscanf(line.c_str() + 9, "%f %f %f", &kp, &kd, &max_vff);
    if (n >= 2) {
      s_ediff.setRearTVGains(kp, kd, max_vff);
      Serial.printf(">>> Rear TV Gains: Kp=%.3f, Kd=%.4f, MaxVff=%.2fV\n",
                    s_ediff.getTvKpPsi(), s_ediff.getTvKdPsi(), s_ediff.getTvMaxVff());
    }
  } else if (line.startsWith("OS_MU ") || line.startsWith("os_mu ")) {
    float mu_os = line.substring(6).toFloat();
    if (mu_os >= 0.40f && mu_os <= 2.50f) {
      s_ediff.setOversteerMuCap(mu_os);
      Serial.printf(">>> Oversteer Mu Cap: %.2f\n", s_ediff.getOversteerMuCap());
    }
  }
}

// =============================================================================
// CORE 0: Communications & Peripheral Task (Priority 15)
// =============================================================================
static void ioTask(void *pvParameters) {
  uint32_t last_report_ms = millis();
  static uint32_t s_last_cmd_time_ms = 0;
  static uint8_t s_rx_buf[256];
  static size_t s_rx_len = 0;
  static char s_ascii_buf[96];
  static size_t s_ascii_len = 0;

  for (;;) {
    uint32_t now = millis();

    // 1. Non-Blocking Ingestion: Binary Motion Commands (Pi 4) & ASCII Shell
    // Commands
    int avail = Serial.available();
    if (avail > 0) {
      size_t space = sizeof(s_rx_buf) - s_rx_len;
      if (space == 0) {
        // Buffer full of unparseable data; drop oldest half
        memmove(s_rx_buf, s_rx_buf + 128, sizeof(s_rx_buf) - 128);
        s_rx_len -= 128;
        space = 128;
      }
      size_t to_read = ((size_t)avail < space) ? (size_t)avail : space;
      for (size_t i = 0; i < to_read; i++) {
        int c = Serial.read();
        if (c < 0)
          break;
        s_rx_buf[s_rx_len++] = (uint8_t)c;
      }

      size_t pos = 0;
      while (pos < s_rx_len) {
        uint8_t b0 = s_rx_buf[pos];
        if (b0 == PI_UART_SYNC_1) {
          size_t rem = s_rx_len - pos;
          if (rem < 2) {
            break; // Need at least 2 bytes to verify 0x55 0xAA vs ASCII 'U'
          }
          if (s_rx_buf[pos + 1] == PI_UART_SYNC_2) {
            if (rem < 3) {
              break; // Wait for msg_type byte
            }
            if (s_rx_buf[pos + 2] == MSG_ID_MOTION_CMD) {
              if (rem < sizeof(MotionCommandPacket)) {
                break; // Wait for remaining bytes of 13-byte frame
              }
              const uint8_t *frame = &s_rx_buf[pos];
              uint8_t chk =
                  mcu_calc_checksum(frame, sizeof(MotionCommandPacket) - 1);
              if (chk == frame[sizeof(MotionCommandPacket) - 1]) {
                const MotionCommandPacket *cmd =
                    (const MotionCommandPacket *)frame;
                s_last_cmd_time_ms = millis();
                if (cmd->run_mode == 2) {
                  // Emergency brake
                  s_manual_v = 0.0f;
                  s_manual_override = false;
                  s_cli_override_active = false;
                  s_estop_latched = true;
                  s_drive_stopped = true;
                  s_stm32Link.emergencyStop();
                } else if (!s_cli_override_active) {
                  if (cmd->run_mode == 1 || cmd->run_mode == 3) {
                    s_manual_v = (float)cmd->target_v_mm_s / 1000.0f;
                    s_manual_steer = (float)cmd->target_steer_mrad / 1000.0f;
                    s_manual_override = true;
                    s_manual_servo_direct = false;
                    s_fake_data_active = false;
                    if (cmd->target_v_mm_s != 0 && !s_estop_latched &&
                        s_drive_stopped) {
                      s_stm32Link.enableDrivers();
                      s_drive_stopped = false;
                    }
                  } else {
                    s_manual_v = 0.0f;
                    s_manual_steer = 0.0f;
                    s_manual_override = false;
                    s_drive_stopped = true;
                    if (s_stm32Link.getFront().isM1Enabled() ||
                        s_stm32Link.getFront().isM2Enabled() ||
                        s_stm32Link.getRear().isM1Enabled() ||
                        s_stm32Link.getRear().isM2Enabled()) {
                      s_stm32Link.emergencyStop();
                    }
                  }
                }
                pos += sizeof(MotionCommandPacket);
                s_ascii_len = 0;
                continue;
              } else {
                // Checksum mismatch: advance past 0x55 0xAA so nested sync
                // isn't missed
                pos += 2;
                continue;
              }
            } else {
              pos += 2;
              continue;
            }
          }
          // 0x55 not followed by 0xAA -> treat as ASCII 'U' (e.g. in "STATUS"
          // or "IMU")
        }

        // Non-binary byte processing for ASCII commands
        pos++;
        if (b0 == '\n' || b0 == '\r') {
          if (s_ascii_len > 0) {
            s_ascii_buf[s_ascii_len] = '\0';
            handleAsciiCommand(s_ascii_buf);
            s_ascii_len = 0;
          }
        } else if (b0 >= 32 && b0 <= 126) {
          if (s_ascii_len < sizeof(s_ascii_buf) - 1) {
            s_ascii_buf[s_ascii_len++] = (char)b0;
          } else {
            s_ascii_len = 0;
          }
        } else {
          // Non-printable byte: clear ASCII accumulator
          s_ascii_len = 0;
        }
      }

      if (pos > 0) {
        if (pos < s_rx_len) {
          memmove(s_rx_buf, s_rx_buf + pos, s_rx_len - pos);
          s_rx_len -= pos;
        } else {
          s_rx_len = 0;
        }
      }
    }

    // 2. Process BNO086 IMU over SPI2 (if not fake data)
    if (!s_fake_data_active && s_imu.isInitialized()) {
      s_imu.update();
    }

    // 3. STM32 UART RX/TX now runs in the 400 Hz control task (Core 1).

    // Command Loss Watchdog (Dead-Man Switch @ 150 ms timeout for autonomous
    // stream)
    if (s_manual_override && !s_cli_override_active &&
        (millis() - s_last_cmd_time_ms > 150)) {
      s_manual_v = 0.0f;
      s_manual_steer = 0.0f;
      s_manual_override = false;
      s_drive_stopped = true;
      if (s_stm32Link.getFront().isM1Enabled() ||
          s_stm32Link.getFront().isM2Enabled() ||
          s_stm32Link.getRear().isM1Enabled() ||
          s_stm32Link.getRear().isM2Enabled()) {
        s_stm32Link.emergencyStop();
      }
    }

    // 4. Update Shared Sensor Inputs for Core 1
    SharedSensorInputs &inputs = s_sensorInputs.getWriteBuffer();
    inputs.fake_data_mode = s_fake_data_active;
    inputs.override_active = s_manual_override;
    inputs.target_v = s_manual_v;
    inputs.target_steer_rad = s_manual_steer;

    if (!s_fake_data_active) {
      if (s_imu.isInitialized()) {
        const auto &imu_data = s_imu.getData();
        inputs.gyro_z = imu_data.gyro_z;
        inputs.gyro_seq = imu_data.gyro_seq;
        inputs.gyro_timestamp_us = imu_data.gyro_timestamp_us;
        inputs.accel_fwd = imu_data.lin_accel_x; // local body forward
        inputs.accel_lat = imu_data.lin_accel_y;
        inputs.roll_deg = imu_data.roll_deg;
        inputs.pitch_deg = imu_data.pitch_deg;
        inputs.yaw_deg = imu_data.yaw_deg;
      } else {
        inputs.gyro_z = 0.0f;
        inputs.gyro_seq = 0;
        inputs.gyro_timestamp_us = 0;
        inputs.accel_fwd = 0.0f;
        inputs.accel_lat = 0.0f;
        inputs.roll_deg = 0.0f;
        inputs.pitch_deg = 0.0f;
        inputs.yaw_deg = 0.0f;
      }
    }

    inputs.fl_rpm = WHEEL_SIGN_FL * s_stm32Link.getFL_RPM();
    inputs.fr_rpm = WHEEL_SIGN_FR * s_stm32Link.getFR_RPM();
    inputs.rl_rpm = WHEEL_SIGN_RL * s_stm32Link.getRL_RPM();
    inputs.rr_rpm = WHEEL_SIGN_RR * s_stm32Link.getRR_RPM();
    s_sensorInputs.commitWrite();

    // 5. Receive latest control outputs from Core 1 and dispatch to STM32s @
    // 400 Hz
    s_controlOutputs.updateRead();

    // 6. Periodic Binary Telemetry Dispatch to Pi 4 @ 100 Hz (10 ms period)
    static uint32_t s_last_pi_tx_ms = 0;
    static uint32_t s_telem_seq = 0;
    if (now - s_last_pi_tx_ms >= 10) {
      s_last_pi_tx_ms = now;
      const SharedControlOutputs &c = s_controlOutputs.getReadBuffer();
      const SharedSensorInputs &in = s_sensorInputs.getReadBuffer();

      ChassisTelemetryPacket pkt;
      pkt.sync[0] = MCU_UART_SYNC_1;
      pkt.sync[1] = MCU_UART_SYNC_2;
      pkt.msg_type = MSG_ID_CHASSIS_TELEM;
      pkt.payload_len = sizeof(pkt) - 5;
      pkt.seq = s_telem_seq++;
      pkt.timestamp_us = (uint32_t)esp_timer_get_time();

      pkt.yaw_deg_x10 = (int16_t)(in.yaw_deg * 10.0f);
      pkt.pitch_deg_x10 = (int16_t)(in.pitch_deg * 10.0f);
      pkt.roll_deg_x10 = (int16_t)(in.roll_deg * 10.0f);
      pkt.gyro_z_mrad_s = (int16_t)(in.gyro_z * 1000.0f);
      pkt.accel_fwd_cm_s2 = (int16_t)(in.accel_fwd * 100.0f);
      pkt.accel_lat_cm_s2 = (int16_t)(in.accel_lat * 100.0f);

      pkt.steer_cmd_deg_x10 = (int16_t)(c.driver_steer_deg * 10.0f);
      pkt.steer_act_deg_x10 = (int16_t)(c.steer_deg * 10.0f);
      pkt.servo_pulse_us = (uint16_t)c.servo_pulse_us;
      pkt.v_cmd_mm_s = (int16_t)(c.rate_limited_v * 1000.0f);
      pkt.v_est_mm_s = (int16_t)(c.v_est * 1000.0f);

      pkt.fl_rpm_cmd = (int16_t)c.fl_target_rpm;
      pkt.fr_rpm_cmd = (int16_t)c.fr_target_rpm;
      pkt.rl_rpm_cmd = (int16_t)c.rl_target_rpm;
      pkt.rr_rpm_cmd = (int16_t)c.rr_target_rpm;
      pkt.fl_rpm_act = (int16_t)in.fl_rpm;
      pkt.fr_rpm_act = (int16_t)in.fr_rpm;
      pkt.rl_rpm_act = (int16_t)in.rl_rpm;
      pkt.rr_rpm_act = (int16_t)in.rr_rpm;

      pkt.slip_state = (uint8_t)c.slip_state;
      pkt.slip_ratio_pct = (int8_t)(c.slip_ratio * 100.0f);
      pkt.flags =
          (in.override_active ? FLAG_OVERRIDE_ACTIVE : 0) |
          (s_fake_data_active ? FLAG_BENCH_FAKE_DATA : 0) |
          (s_imu.isInitialized() ? FLAG_IMU_ONLINE : 0) |
          (s_stm32Link.isFrontConnected() ? FLAG_STM32_FRONT_ONLINE : 0) |
          (s_stm32Link.isRearConnected() ? FLAG_STM32_REAR_ONLINE : 0);

      pkt.checksum = mcu_calc_checksum((const uint8_t *)&pkt, sizeof(pkt) - 1);
      if (s_piSerial.availableForWrite() >= (int)sizeof(pkt)) {
        s_piSerial.write((const uint8_t *)&pkt, sizeof(pkt));
      }
    }

    // 7. Periodic USB IMU JSON Stream (50 Hz when enabled via IMU_STREAM 1)
    static uint32_t s_last_imu_stream_ms = 0;
    if (s_imu_usb_stream && (now - s_last_imu_stream_ms >= 20)) {
      s_last_imu_stream_ms = now;
      if (s_imu.isInitialized()) {
        const auto &d = s_imu.getData();
        Serial.printf(
            "{\"imu\":1,\"online\":1,\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,"
            "\"qz\":%.4f,\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"ax\":%.2f,"
            "\"ay\":%.2f,\"az\":%.2f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
            "\"lax\":%.2f,\"lay\":%.2f,\"acc\":%u}\n",
            d.quat_w, d.quat_x, d.quat_y, d.quat_z, d.roll_deg, d.pitch_deg,
            d.yaw_deg, d.accel_x, d.accel_y, d.accel_z, d.gyro_x, d.gyro_y,
            d.gyro_z, d.lin_accel_x, d.lin_accel_y, d.quat_accuracy);
      } else {
        Serial.println(F("{\"imu\":1,\"online\":0}"));
      }
    }

    // 8. Periodic Benchmark Telemetry to Serial (every 500 ms = 2 Hz, bench
    // mode only)
    if (now - last_report_ms >= 500) {
      last_report_ms = now;
      if (s_fake_data_active && !s_manual_override) {
        const SharedControlOutputs &c = s_controlOutputs.getReadBuffer();
        float cpu_pct = (c.exec_us_avg / 2500.0f) * 100.0f;

        Serial.printf(
            "[400Hz BENCH] Cycles: %6u | T_exec: %5.2fus (min: %5.2f, max: "
            "%5.2f, avg: %5.2f) | CPU: %4.1f%% | Overruns: %u | v_est: "
            "%+.2fm/s | Steer: %+.1fdeg | PWM: %4.0fus\n",
            c.cycle_count, c.exec_us, c.exec_us_min, c.exec_us_max,
            c.exec_us_avg, cpu_pct, c.overrun_count, c.v_est, c.steer_deg,
            c.servo_pulse_us);
      }
    }

    // Low-latency 1ms polling cadence for dual STM32 UART link and Pi USB
    // telemetry
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
// Arduino Setup & Core Assignment
// =============================================================================
void setup() {
  // 1. Initialize Single Ackermann Steering Servo immediately (GPIO 21 @ 333 Hz LEDC)
  //    Load saved NVS trim first so the very first PWM pulse is already trimmed.
  {
    Preferences prefs;
    prefs.begin("servo_cfg", true);
    float saved_trim = prefs.getFloat("trim_us", 0.0f);
    prefs.end();
    if (!isnan(saved_trim)) {
      s_servo_trim_us = fmaxf(-500.0f, fminf(500.0f, saved_trim));
    }
  }
  s_servo.begin(SERVO_PULSE_NEUTRAL_US + s_servo_trim_us);

  Serial.setRxBufferSize(2048);
  Serial.setTxBufferSize(2048);
  Serial.begin(921600);
  Serial.setTimeout(0);
  delay(200);

  Serial.println(
      F("\n========================================================"));
  Serial.println(F("  FOLKRACE ESP32-S3 HIGH-PERFORMANCE MASTER CONTROLLER  "));
  Serial.println(F("  Deterministic 400Hz (2500us) Math + Dual-Core RTOS    "));
  Serial.println(F("  4WD Electronic Diff + 333Hz Servo + IMU Sensor Fusion "));
  Serial.println(F("========================================================"));
  Serial.printf(
      "[+] Ackermann Steering Servo Initialized on GPIO 21 (333 Hz, trim: %+.1f us).\n",
      s_servo_trim_us);

  // 2. Initialize Dual STM32 UART Links (UART1 Front, UART2 Rear @ 921,600
  // Baud)
  s_stm32Link.begin(STM32_1_UART_BAUD, STM32_1_UART_RX_PIN, STM32_1_UART_TX_PIN,
                    STM32_2_UART_RX_PIN, STM32_2_UART_TX_PIN);
  Serial.println(
      F("[+] Dual STM32 FOC UART Links Initialized (UART1 & UART2 @ 921.6k)."));

  // 2b. Raspberry Pi 4 Direct USB Serial Link
  Serial.println(
      F("[+] Raspberry Pi 4 Direct USB Link Online (Serial @ 921.6k)."));

  // 3. Initialize BNO086 IMU over SPI2 (CS=10, INT=9, SCK=12, MOSI=11, MISO=13,
  // RST=14)
  if (s_imu.begin(IMU_SPI_CS_PIN, IMU_SPI_MISO_PIN, IMU_SPI_MOSI_PIN,
                  IMU_SPI_SCK_PIN, IMU_INT_PIN, IMU_RST_PIN)) {
    Serial.println(F("[+] BNO086 9-DOF IMU Connected over SPI2 Master."));
  } else {
    Serial.println(
        F("[!] BNO086 IMU not detected. Operating in live mode without IMU."));
  }

  // 4. Pin Core 1 Task: Deterministic 400 Hz Real-Time Control & Kinematics
  // (Priority 24)
  xTaskCreatePinnedToCore(realTimeControlTask, "RealTimeControl", 8192, nullptr,
                          24, // High priority (preempts background tasks)
                          &s_controlTaskHandle,
                          1 // Core 1 (APP CPU)
  );
  Serial.println(F("[+] Core 1 Control Task Launched (Priority 24, 400 Hz)."));

  // 5. Pin Core 0 Task: Asynchronous Communications, Sensor Polling & Telemetry
  // (Priority 15)
  xTaskCreatePinnedToCore(ioTask, "CommIO", 8192, nullptr,
                          15, // Medium priority
                          nullptr,
                          0 // Core 0 (PRO CPU)
  );
  Serial.println(
      F("[+] Core 0 I/O Task Launched (Priority 15, Asynchronous).\n"));
}

void loop() {
  // Main Arduino loop sleeps; tasks are handled by FreeRTOS on Core 0 and Core
  // 1
  vTaskDelay(pdMS_TO_TICKS(1000));
}
