#include "config.h"
#include <Arduino.h>
#include <SimpleFOC.h>

// =========================================================================
// Hardware Pinout Configuration
// =========================================================================
// 1. Motor Phase PWM Pins:   PA0 (A), PA1 (B), PA2 (C)
// 2. Driver Enable Pin:      PA5 (EN)
// 3. Encoder PWM Input Pin:  PA8 (PWM) -- Previously used for I2C SDA
//    NOTE: Move your SDA wire from the encoder over to the encoder's PWM pin.
//    No rewiring is required on the STM32 board side.
// =========================================================================
#define ENCODER_PWM_PIN PA9

// 1. Instantiate the MagneticSensorPWM class.
// For MT6701, the PWM pulse ranges from 1 microsecond (0 rad) to 1023
// microseconds (2pi rad). PWM frequency is 976.56Hz (~1ms period).
MagneticSensorPWM sensor(ENCODER_PWM_PIN, 1, 1023);

volatile uint32_t isr_count = 0;
// Interrupt Service Routine callback for PWM signal pulse measurement
void doPWM() {
  sensor.handlePWM();
  isr_count++;
}

// 2. Instantiate Motor and Driver
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_EN);

// Telemetry & Control variables
bool is_calibrated = false;
bool is_enabled = false;
bool show_telemetry = false; // Silent by default to prevent console flooding
float m1_target = 0.0f;      // Target speed (RPM)

// Heartbeat & status LED tracking
uint32_t last_led_toggle = 0;
bool led_state = false;
uint32_t last_control_time_us = 0;

void printHelp() {
  Serial.println("\n==================================================");
  Serial.println("  STM32 PWM Encoder (MT6701) Closed Loop Tool     ");
  Serial.println("==================================================");
  Serial.println("Wiring Check:");
  Serial.println("  1. Connect MT6701 PWM output pin directly to PA8.");
  Serial.println("  2. Driver PWM: PA0, PA1, PA2. Driver EN: PA5.");
  Serial.println("--------------------------------------------------");
  Serial.println("Actions (No Enter key required!):");
  Serial.println("  c             : Start FOC closed-loop calibration");
  Serial.println("  o             : Switch motor to Open-Loop mode");
  Serial.println("  e             : Toggle Enable/Disable motor driver");
  Serial.println("  d             : Disable motor and stop");
  Serial.println("  g             : Toggle telemetry status logging ON/OFF");
  Serial.println("  h or ?        : Show this help menu");
  Serial.println("Presets:");
  Serial.println("  0, 1, 2, 3, 4 : Preset speeds (0, 50, 100, 200, 500 RPM)");
  Serial.println(
      "  + / -         : Increase / decrease target speed by 10 RPM");
  Serial.println(
      "Live Parameter Tuning (Prefix + Value, no Enter key needed):");
  Serial.println("  t<float>      : Set target speed in RPM (e.g. 't120')");
  Serial.println("  p<float>      : Set velocity P gain (e.g. 'p0.15')");
  Serial.println("  i<float>      : Set velocity I gain (e.g. 'i3.0')");
  Serial.println(
      "  f<float>      : Set velocity LPF Tf in seconds (e.g. 'f0.02')");
  Serial.println(
      "  r<float>      : Set phase resistance in ohms (e.g. 'r6.0')");
  Serial.println("  k<float>      : Set motor Kv rating (e.g. 'k130')");
  Serial.println("--------------------------------------------------");
  Serial.print("Motor Status     : ");
  Serial.println(is_enabled ? "ENABLED" : "DISABLED");
  Serial.print("Target Speed     : ");
  Serial.print(m1_target, 1);
  Serial.println(" RPM");
  Serial.print("Telemetry Logging: ");
  Serial.println(show_telemetry ? "ON" : "OFF");
  Serial.print("Current P Gain   : ");
  Serial.println(motor.PID_velocity.P, 4);
  Serial.print("Current I Gain   : ");
  Serial.println(motor.PID_velocity.I, 4);
  Serial.print("Current Tf LPF   : ");
  Serial.println(motor.LPF_velocity.Tf, 4);
  Serial.print("Current Resistance: ");
  Serial.print(motor.phase_resistance, 2);
  Serial.println(" Ohms");
  Serial.print("Current Kv Rating : ");
  Serial.print(motor.KV_rating, 2);
  Serial.println(" RPM/V");
  Serial.println("==================================================\n");
}

void printStatus() {
  Serial.print("Ctrl: ");
  if (motor.controller == MotionControlType::velocity_openloop) {
    Serial.print("OPEN  ");
  } else {
    Serial.print("CLOSE ");
  }
  Serial.print("| Target: ");
  Serial.print(m1_target, 1);
  Serial.print(" RPM | Encoder: ");
  Serial.print(sensor.getAngle(), 3);
  // RPM calculation: rad/s * 60 / (2*PI)
  float motor_rpm = motor.shaft_velocity * 9.549296585f;
  Serial.print(" rad | Actual: ");
  Serial.print(motor_rpm, 1);
  Serial.print(" RPM | ISR Count: ");
  Serial.println(isr_count);
}

void updateLED() {
  uint32_t now = millis();
  uint32_t interval = 1000;

  float abs_speed = abs(m1_target);
  if (abs_speed > 500.0f) {
    interval = 100;
  } else if (abs_speed > 100.0f) {
    interval = 250;
  } else {
    interval = 1000;
  }

  // Extremely fast blink if driver is disabled but target is commanded
  if (m1_target != 0.0f && !is_enabled) {
    interval = 50;
  }

  if (now - last_led_toggle >= interval) {
    led_state = !led_state;
    last_led_toggle = now;
    digitalWrite(STATUS_LED, led_state ? LOW : HIGH);
  }
}

void setup() {
  // 1. Configure status LED pin
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH); // Off by default (Active-Low)

  // 2. Reclaim PA15, PB3, PC14, PF0, PF1 oscillator/debug pins as GPIO inputs
  pinMode(PA15, INPUT);
  pinMode(PB3, INPUT);
  pinMode(PC14, INPUT);
  pinMode(PF0, INPUT);
  pinMode(PF1, INPUT);

  // 3. Initialize USB Serial
  Serial.begin(921600);
  Serial.setTimeout(30); // 30ms timeout so parseFloat doesn't block loops
  SimpleFOCDebug::enable(&Serial);

  uint32_t start_time = millis();
  while (!Serial && (millis() - start_time < 3000)) {
    digitalWrite(STATUS_LED, (millis() % 200 < 100) ? LOW : HIGH);
  }
  digitalWrite(STATUS_LED, HIGH);

  // 4. Initialize Magnetic Sensor PWM
  sensor.init();
  sensor.enableInterrupt(doPWM);

  // 5. Initialize Driver
  driver.voltage_power_supply = DRIVER_VOLTAGE_LIMIT;
  driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  driver.init();
  motor.linkDriver(&driver);

  // 6. Link Sensor to Motor
  motor.linkSensor(&sensor);

  // 7. Configure Motor Parameters
  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.current_limit = MOTOR_CURRENT_LIMIT;

  // GM3506 Feed-Forward configuration
  motor.phase_resistance = 6.0f; // Measured 6.0 Ohms
  motor.KV_rating = 130.0f;      // Manufacturer specification

  // Closed loop velocity tuning parameters
  motor.PID_velocity.P = 0.15f;
  motor.PID_velocity.I = 3.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f;
  motor.LPF_velocity.Tf = 0.02f; // Low pass filter time constant

  motor.voltage_sensor_align = 2.0f; // Limit alignment current

  // Initialize motor hardware (will not calibrate until 'c' is typed)
  motor.init();
  motor.disable();

  printHelp();

  last_control_time_us = micros();
}

void loop() {
  // ---- Main Control Loop (20kHz / 50µs) ----
  // Since PWM uses hardware interrupts, sensor.update() is virtually free
  // and does not block on standard synchronous transfers. This allows a true
  // 20kHz loop.
  uint32_t now_us = micros();
  if (now_us - last_control_time_us >= 50) {
    last_control_time_us += 50;

    // 0. Run FOC core commutation logic
    if (is_enabled) {
      motor.loopFOC();
    }

    // 1. Process Serial Commands (Instant keyboard interaction)
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      switch (cmd) {
      case '0':
        m1_target = 0.0f;
        Serial.println("Target: 0 RPM");
        break;
      case '1':
        m1_target = 50.0f;
        Serial.println("Target: 50 RPM");
        break;
      case '2':
        m1_target = 100.0f;
        Serial.println("Target: 100 RPM");
        break;
      case '3':
        m1_target = 200.0f;
        Serial.println("Target: 200 RPM");
        break;
      case '4':
        m1_target = 500.0f;
        Serial.println("Target: 500 RPM");
        break;

      case '+':
        m1_target += 10.0f;
        Serial.print("Target: ");
        Serial.print(m1_target, 1);
        Serial.println(" RPM");
        break;
      case '-':
        m1_target -= 10.0f;
        Serial.print("Target: ");
        Serial.print(m1_target, 1);
        Serial.println(" RPM");
        break;

      case 'e':
      case 'E':
        if (is_enabled) {
          Serial.println("\n>>> Disabling motor...");
          is_enabled = false;
          motor.disable();
        } else {
          Serial.println("\n>>> Enabling motor...");
          is_enabled = true;
          motor.enable();
        }
        break;

      case 'c':
      case 'C':
        Serial.println("\n>>> Starting FOC calibration via PWM...");
        motor.controller = MotionControlType::velocity;
        motor.enable();

        if (motor.initFOC()) {
          is_calibrated = true;
          is_enabled = true;
          Serial.println(">>> Calibration SUCCESS! Closed-loop FOC is active.");
        } else {
          is_calibrated = false;
          is_enabled = false;
          motor.disable();
          Serial.println(">>> Calibration FAILED. Motor disabled.");
        }
        break;

      case 'o':
      case 'O':
        Serial.println("\n>>> Activating OPEN-LOOP mode...");
        motor.disable();
        motor.controller = MotionControlType::velocity_openloop;
        is_enabled = true;
        motor.enable();
        Serial.println(">>> Open-loop velocity mode active.");
        break;

      case 'd':
      case 'D':
        Serial.println("\n>>> Motor disabled.");
        is_enabled = false;
        motor.disable();
        m1_target = 0.0f;
        break;

      case 'g':
      case 'G':
        show_telemetry = !show_telemetry;
        Serial.print(">>> Telemetry status logging: ");
        Serial.println(show_telemetry ? "ON" : "OFF");
        break;

      case 'h':
      case 'H':
      case '?':
        printHelp();
        break;

      // Fast float parameters parsing on trigger
      case 'p':
      case 'P':
        motor.PID_velocity.P = Serial.parseFloat();
        Serial.print(">>> Updated Velocity P: ");
        Serial.println(motor.PID_velocity.P, 4);
        break;
      case 'i':
      case 'I':
        motor.PID_velocity.I = Serial.parseFloat();
        Serial.print(">>> Updated Velocity I: ");
        Serial.println(motor.PID_velocity.I, 4);
        break;
      case 'f':
      case 'F':
        motor.LPF_velocity.Tf = Serial.parseFloat();
        Serial.print(">>> Updated Velocity Tf: ");
        Serial.println(motor.LPF_velocity.Tf, 4);
        break;
      case 'r':
      case 'R':
        motor.phase_resistance = Serial.parseFloat();
        Serial.print(">>> Updated Phase Resistance: ");
        Serial.println(motor.phase_resistance, 2);
        break;
      case 'k':
      case 'K':
        motor.KV_rating = Serial.parseFloat();
        Serial.print(">>> Updated Kv Rating: ");
        Serial.println(motor.KV_rating, 2);
        break;
      case 't':
      case 'T':
        m1_target = Serial.parseFloat();
        Serial.print(">>> Updated Target Speed: ");
        Serial.print(m1_target, 1);
        Serial.println(" RPM");
        break;
      }
    }

    // 2. Refresh accumulated sensor calculations (non-blocking)
    sensor.update();

    // 3. Command target speed (converted from RPM to rad/s)
    if (is_enabled) {
      motor.move(m1_target * 0.104719755f);
    }

    // 4. Update Status Heartbeat LED indicator
    updateLED();

    // 5. Throttled Telemetry status logging (runs every 100ms: 2000 loops *
    // 50µs = 100ms)
    if (show_telemetry) {
      static uint32_t print_count = 0;
      print_count++;
      if (print_count >= 2000) {
        print_count = 0;
        printStatus();
      }
    }
  }
}
