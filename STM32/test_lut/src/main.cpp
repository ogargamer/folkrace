#include "config.h"
#include "motor_axis.h"
#include "safety.h"
#include <Arduino.h>

#include "HardwareTimerEncoder.h"

#define LUT_SIZE 128
float encoder_lut[LUT_SIZE] = {0};
bool is_lut_calibrated = false;

// 1. Instantiate 100% Hardware Timer Encoder on TIM3 (PB4=TIM3_CH1, PB5=TIM3_CH2)
// Zero CPU overhead, hardware glitch filter enabled
HardwareTimerEncoder encoder(TIM3, 1024);

// 2. Instantiate Motor Axis with hardware encoder
MotorAxis axis(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_EN, &encoder);

// Control Mode state
MotionControlType control_mode = MotionControlType::velocity;
bool is_calibrated = false;
bool is_enabled = false;
bool show_telemetry = false;

SafetyManager safety;
float m1_target = 0.0f;
uint32_t last_control_time_us = 0;

bool is_calibrating_lut = false;
float calib_target_rpm = 0.0f;
uint32_t calib_start_ms = 0;
float calib_start_angle = 0.0f;
int calib_samples[LUT_SIZE] = {0};
float calib_error_sum[LUT_SIZE] = {0};

void startLUTCalibration(float target_rpm) {
  Serial.println("\n==================================================");
  Serial.println("  Starting Non-Blocking Encoder LUT Calibration   ");
  Serial.println("==================================================");
  Serial.print("Target Speed: "); Serial.print(target_rpm, 1); Serial.println(" RPM");
  Serial.println("Motor running via main loop open-loop motion...");
  Serial.println("Press 'x' to ABORT.");

  memset(calib_samples, 0, sizeof(calib_samples));
  memset(calib_error_sum, 0, sizeof(calib_error_sum));
  memset(encoder_lut, 0, sizeof(encoder_lut));

  calib_target_rpm = target_rpm;
  m1_target = target_rpm;

  // Set motor to open loop using exact same sequence as 'o' key
  axis.disable();
  axis.setControlMode(MotionControlType::velocity_openloop);
  control_mode = MotionControlType::velocity_openloop;
  is_enabled = true;
  axis.enable();

  calib_start_angle = encoder.getMechanicalAngle();
  calib_start_ms = millis();
  is_calibrating_lut = true;
}

void printHelp() {
  Serial.println("\n==================================================");
  Serial.println("  STM32 Motor Test Tool + Encoder LUT Calibration ");
  Serial.println("==================================================");
  Serial.println("Actions:");
  Serial.println("  l             : Run Encoder LUT Calibration (Magnet Eccentricity)");
  Serial.println("  c             : Start FOC closed-loop calibration");
  Serial.println("  o             : Switch motor to Open-Loop mode");
  Serial.println("  e             : Toggle Enable/Disable motor driver");
  Serial.println("  x             : Disable motor and stop");
  Serial.println("  g             : Toggle telemetry status logging ON/OFF");
  Serial.println("  h or ?        : Show this help menu");
  Serial.println("Presets:");
  Serial.println("  0, 1, 2, 3, 4 : Preset speeds (0, 50, 100, 200, 500 RPM)");
  Serial.println("  + / -         : Increase / decrease target speed by 10 RPM");
  Serial.println("--------------------------------------------------");
  Serial.print("LUT Calibrated   : ");
  Serial.println(is_lut_calibrated ? "YES (128-point Active)" : "NO (Uncalibrated)");
  Serial.print("Motor Status     : ");
  Serial.println(is_enabled ? "ENABLED" : "DISABLED");
  Serial.print("Target Speed     : ");
  Serial.print(m1_target, 1);
  Serial.println(" RPM");
  Serial.println("==================================================\n");
}

void printStatus() {
  Serial.print("Ctrl: ");
  if (control_mode == MotionControlType::velocity_openloop) {
    Serial.print("OPEN  ");
  } else {
    Serial.print("CLOSE ");
  }
  Serial.print("| Target: ");
  Serial.print(m1_target, 1);
  Serial.print(" RPM | Actual: ");
  Serial.print(axis.getRPM(), 1);
  Serial.print(" RPM | Angle: ");
  Serial.print(encoder.getAngle(), 3);
  Serial.print(" rad | Uq: ");
  Serial.print(axis.getMotor().voltage.q, 2);
  Serial.print(" V | Ud: ");
  Serial.print(axis.getMotor().voltage.d, 2);
  Serial.print(" V | Vbus: ");
  if (axis.getMotor().driver) {
    Serial.print(axis.getMotor().driver->voltage_power_supply, 1);
  } else {
    Serial.print("N/A");
  }
  Serial.println(" V");
}

void setup() {
  pinMode(PA15, INPUT);
  pinMode(PB3, INPUT);
  pinMode(PC14, INPUT);
  pinMode(PF0, INPUT);
  pinMode(PF1, INPUT);

  safety.init();

  // Initialize TIM3 Hardware Encoder
  encoder.init();

  axis.init();
  axis.disable();

  Serial.begin(921600);
  Serial.setTimeout(30);
  SimpleFOCDebug::enable(&Serial);
  uint32_t start_time = millis();
  while (!Serial && (millis() - start_time < 3000)) {
    digitalWrite(STATUS_LED, (millis() % 200 < 100) ? LOW : HIGH);
  }
  digitalWrite(STATUS_LED, HIGH);

  printHelp();
  last_control_time_us = micros();
}

void loop() {
  uint32_t now_us = micros();
  if (now_us - last_control_time_us >= 50) {
    last_control_time_us += 50;

    encoder.update();

    if (is_enabled) {
      axis.loopFOC();
    }

    if (Serial.available() > 0) {
      char cmd = Serial.read();
      switch (cmd) {
      case '0': m1_target = 0.0f; Serial.println("Target: 0 RPM"); break;
      case '1': m1_target = 50.0f; Serial.println("Target: 50 RPM"); break;
      case '2': m1_target = 100.0f; Serial.println("Target: 100 RPM"); break;
      case '3': m1_target = 200.0f; Serial.println("Target: 200 RPM"); break;
      case '4': m1_target = 500.0f; Serial.println("Target: 500 RPM"); break;
      case '+': m1_target += 10.0f; Serial.print("Target: "); Serial.print(m1_target, 1); Serial.println(" RPM"); break;
      case '-': m1_target -= 10.0f; Serial.print("Target: "); Serial.print(m1_target, 1); Serial.println(" RPM"); break;

      case 'j':
      case 'J':
        startLUTCalibration(10.0f);
        break;

      case 'k':
      case 'K':
        startLUTCalibration(30.0f);
        break;

      case 'l':
      case 'L':
        startLUTCalibration(60.0f);
        break;

      case 'e':
      case 'E':
        if (is_enabled) {
          Serial.println("\n>>> Disabling motor...");
          is_enabled = false;
          is_calibrating_lut = false;
          axis.disable();
        } else {
          Serial.println("\n>>> Enabling motor...");
          is_enabled = true;
          axis.enable();
        }
        break;

      case 'c':
      case 'C':
        is_calibrating_lut = false;
        Serial.println("\n>>> Starting FOC calibration via Serial...");
        axis.setControlMode(MotionControlType::velocity);
        control_mode = MotionControlType::velocity;
        axis.enable();

        if (axis.initFOC()) {
          is_calibrated = true;
          is_enabled = true;
          Serial.println(">>> Calibration SUCCESS! Closed-loop FOC is active.");
        } else {
          is_calibrated = false;
          is_enabled = false;
          axis.disable();
          Serial.println(">>> Calibration FAILED. Motor disabled for safety.");
        }
        break;

      case 'o':
      case 'O':
        is_calibrating_lut = false;
        Serial.println("\n>>> Activating OPEN-LOOP mode...");
        axis.disable();
        axis.setControlMode(MotionControlType::velocity_openloop);
        control_mode = MotionControlType::velocity_openloop;
        is_enabled = true;
        axis.enable();
        break;

      case 'x':
      case 'X':
        Serial.println("\n>>> Emergency STOP / Disable Motor");
        m1_target = 0.0f;
        is_enabled = false;
        is_calibrating_lut = false;
        axis.disable();
        break;

      case 'g':
      case 'G':
        show_telemetry = !show_telemetry;
        Serial.print("Telemetry logging toggled: ");
        Serial.println(show_telemetry ? "ON" : "OFF");
        break;

      case 'h':
      case '?':
        printHelp();
        break;

      case 't': m1_target = Serial.parseFloat(); Serial.print("Target RPM set to: "); Serial.println(m1_target, 1); break;
      case 'p': axis.getMotor().PID_velocity.P = Serial.parseFloat(); Serial.print("P set to: "); Serial.println(axis.getMotor().PID_velocity.P, 4); break;
      case 'i': axis.getMotor().PID_velocity.I = Serial.parseFloat(); Serial.print("I set to: "); Serial.println(axis.getMotor().PID_velocity.I, 4); break;
      case 'd': axis.getMotor().PID_velocity.D = Serial.parseFloat(); Serial.print("D set to: "); Serial.println(axis.getMotor().PID_velocity.D, 5); break;
      }
    }

    if (is_enabled) {
      axis.move(m1_target);
    }

    // Non-blocking LUT calibration sample collection
    if (is_calibrating_lut) {
      uint32_t elapsed = millis() - calib_start_ms;
      
      if (elapsed > 1000 && elapsed < 6000) {
        // Collect sample during open loop motion driven by loop()
        float open_loop_speed = calib_target_rpm * 0.104719755f;
        float elapsed_s = (elapsed - 1000) / 1000.0f;
        float expected_mech = fmod(calib_start_angle + elapsed_s * open_loop_speed, _2PI);
        if (expected_mech < 0) expected_mech += _2PI;

        float actual_mech = encoder.getMechanicalAngle();
        float norm_mech = fmod(actual_mech, _2PI);
        if (norm_mech < 0) norm_mech += _2PI;

        float err = expected_mech - norm_mech;
        while (err > _PI) err -= _2PI;
        while (err < -_PI) err += _2PI;

        int bin = (int)((norm_mech / _2PI) * LUT_SIZE);
        if (bin >= LUT_SIZE) bin = LUT_SIZE - 1;
        if (bin >= 0 && bin < LUT_SIZE) {
          calib_error_sum[bin] += err;
          calib_samples[bin]++;
        }
      } else if (elapsed >= 6000) {
        is_calibrating_lut = false;
        axis.move(0);
        axis.disable();
        is_enabled = false;

        // 1. Calculate raw average error per bin
        float raw_lut[LUT_SIZE] = {0};
        int valid_bins = 0;
        int total_samples_recorded = 0;
        for (int i = 0; i < LUT_SIZE; i++) {
          total_samples_recorded += calib_samples[i];
          if (calib_samples[i] > 0) {
            raw_lut[i] = calib_error_sum[i] / calib_samples[i];
            valid_bins++;
          }
        }

        // 2. Extract pure 1st-harmonic (A*sin(theta) + B*cos(theta)) for true physical magnet eccentricity
        float sum_sin = 0.0f;
        float sum_cos = 0.0f;
        int count = 0;

        for (int i = 0; i < LUT_SIZE; i++) {
          if (calib_samples[i] > 0) {
            float theta = (i / (float)LUT_SIZE) * _2PI;
            sum_sin += raw_lut[i] * sinf(theta);
            sum_cos += raw_lut[i] * cosf(theta);
            count++;
          }
        }

        // Fourier 1st-harmonic coefficients
        float A = (2.0f / count) * sum_sin;
        float B = (2.0f / count) * sum_cos;
        float amplitude = sqrtf(A * A + B * B);
        float phase_angle = atan2f(B, A);

        // Populate smooth distortion-free sine LUT
        float min_err = 100.0f, max_err = -100.0f;
        for (int i = 0; i < LUT_SIZE; i++) {
          float theta = (i / (float)LUT_SIZE) * _2PI;
          encoder_lut[i] = A * sinf(theta) + B * cosf(theta);

          if (encoder_lut[i] < min_err) min_err = encoder_lut[i];
          if (encoder_lut[i] > max_err) max_err = encoder_lut[i];
        }

        Serial.print("\n>>> Total Samples Captured: ");
        Serial.println(total_samples_recorded);

        if (valid_bins > (LUT_SIZE / 2)) {
          is_lut_calibrated = true;
          Serial.println(">>> 1st-Harmonic Fourier Calibration SUCCESS!");
          Serial.print("Magnet Eccentricity Wobble Amplitude: ");
          Serial.print(amplitude, 4);
          Serial.print(" rad (");
          Serial.print(amplitude * 57.2958f, 2);
          Serial.println(" deg)");
          Serial.print("Correction Range: ["); Serial.print(min_err, 4);
          Serial.print(" rad, "); Serial.print(max_err, 4); Serial.println(" rad]");
        } else {
          is_lut_calibrated = false;
          Serial.print(">>> LUT Calibration FAILED: Only ");
          Serial.print(valid_bins);
          Serial.println("/128 bins received samples.");
        }
      }
    }

    static uint32_t last_telemetry_ms = 0;
    if (show_telemetry && (millis() - last_telemetry_ms >= 100)) {
      last_telemetry_ms = millis();
      printStatus();
    }
  }

  safety.update(millis(), m1_target, axis.isSensorFault());
}
