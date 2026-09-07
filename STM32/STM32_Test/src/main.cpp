#include "config.h"
#include "motor_axis.h"
#include "safety.h"
#include <Arduino.h>
// 1. Instantiate hardware Encoder class (A/B)
// MT6701 AB mode outputs 1024 PPR = 4096 CPR (Counts Per Revolution).
Encoder encoder(ENC_A, ENC_B, 4096);

// Interrupt service routines (ISRs) for encoder pulses
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

// 2. Instantiate the Motor Axis
MotorAxis axis(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_EN, &encoder);

// Control Mode state
MotionControlType control_mode = MotionControlType::velocity;
bool is_calibrated = false;
bool is_enabled = false;
bool show_telemetry = false; // Silent by default to prevent screen flooding

// Instantiate Safety Watchdog
SafetyManager safety;

// Global target velocity (RPM)
float m1_target = 0.0f;

// 10kHz Control & Kinematics execution timer
uint32_t last_control_time_us = 0;

void printHelp() {
  Serial.println("\n==================================================");
  Serial.println("     STM32 Single Motor & Encoder Test Tool       ");
  Serial.println("==================================================");
  Serial.println("Actions (No Enter key required!):");
  Serial.println("  c             : Start FOC closed-loop calibration");
  Serial.println("  o             : Switch motor to Open-Loop mode");
  Serial.println("  e             : Toggle Enable/Disable motor driver");
  Serial.println("  x             : Disable motor and stop");
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
  Serial.println("  d<float>      : Set velocity D gain (e.g. 'd0.001')");
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
  Serial.println(axis.getMotor().PID_velocity.P, 4);
  Serial.print("Current I Gain   : ");
  Serial.println(axis.getMotor().PID_velocity.I, 4);
  Serial.print("Current D Gain   : ");
  Serial.println(axis.getMotor().PID_velocity.D, 5);
  Serial.print("Current Tf LPF   : ");
  Serial.println(axis.getMotor().LPF_velocity.Tf, 4);
  Serial.print("Current Resistance: ");
  Serial.print(axis.getMotor().phase_resistance, 2);
  Serial.println(" Ohms");
  Serial.print("Current Kv Rating : ");
  Serial.print(axis.getMotor().KV_rating, 2);
  Serial.println(" RPM/V");
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
  // 1. Reclaim PB3, PB4, PB5 and PA15 from default JTAG control
  pinMode(PA15, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP);
  pinMode(PB4, INPUT_PULLUP);
  pinMode(PB5, INPUT_PULLUP);

  // 2. Reclaim PC14, PF0, PF1 (Oscillator pins) as digital inputs
  pinMode(PC14, INPUT);
  pinMode(PF0, INPUT);
  pinMode(PF1, INPUT);

  // 3. Initialize Safety Watchdog
  safety.init();

  // 4. Initialize Quadrature Encoder
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doA, doB);

  // 7. Initialize motor axis (hardware level only, no calibration yet)
  axis.init();

  // Ensure motor is disabled on boot for startup safety
  axis.disable();

  // 8. Initialize USB CDC Serial (with a short parseFloat timeout)
  Serial.begin(921600);
  Serial.setTimeout(30); // 30ms timeout so parsing doesn't block FOC
  SimpleFOCDebug::enable(&Serial);
  uint32_t start_time = millis();
  while (!Serial && (millis() - start_time < 3000)) {
    digitalWrite(STATUS_LED, (millis() % 200 < 100) ? LOW : HIGH);
  }
  digitalWrite(STATUS_LED, HIGH); // Turn off LED (Active-low)

  printHelp();

  last_control_time_us = micros();
}

void loop() {
  // ---- Main Control Loop (20kHz / 50µs) ----
  // Now using a high-speed quadrature encoder with interrupts, reading 
  // is nearly instantaneous. We can run the FOC loop at a fast 20kHz rate.
  uint32_t now_us = micros();
  if (now_us - last_control_time_us >= 50) {
    last_control_time_us += 50;

    // Update encoder state (mandatory for Encoder class to update cached angle)
    encoder.update();

    // 0. Run FOC commutation (includes sensor read)
    if (is_enabled) {
      axis.loopFOC();
    }

    // 1. Process Serial Commands
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
          axis.disable();
        } else {
          Serial.println("\n>>> Enabling motor...");
          is_enabled = true;
          axis.enable();
        }
        break;

      case 's':
      case 'S':
        m1_target = (m1_target == 0.0f) ? 50.0f : 0.0f;
        Serial.print("Serial speed toggle. Target: ");
        Serial.print(m1_target, 1);
        Serial.println(" RPM");
        break;

      case 'c':
      case 'C':
        Serial.println("\n>>> Starting FOC calibration via Serial...");
        axis.setControlMode(MotionControlType::velocity);
        control_mode = MotionControlType::velocity;
        axis.enable(); // Explicitly enable driver before calibration

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
        Serial.println("\n>>> Activating OPEN-LOOP mode...");
        axis.disable();
        axis.setControlMode(MotionControlType::velocity_openloop);
        control_mode = MotionControlType::velocity_openloop;
        is_enabled = true;
        axis.enable();
        Serial.println(
            ">>> Open-loop velocity mode is active. (No sensor required)");
        break;

      case 'x':
      case 'X':
        Serial.println("\n>>> Motor disabled.");
        is_enabled = false;
        axis.disable();
        m1_target = 0.0f;
        break;

      case 'd':
      case 'D': {
        float val = Serial.parseFloat();
        axis.getMotor().PID_velocity.D = val;
        Serial.print(">>> Updated Velocity D: ");
        Serial.println(axis.getMotor().PID_velocity.D, 5);
        break;
      }

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

      // Live Parameter Tuning (Prefix + Float parsing)
      case 'p':
      case 'P': {
        float val = Serial.parseFloat();
        axis.getMotor().PID_velocity.P = val;
        Serial.print(">>> Updated Velocity P: ");
        Serial.println(axis.getMotor().PID_velocity.P, 4);
        break;
      }
      case 'i':
      case 'I': {
        float val = Serial.parseFloat();
        axis.getMotor().PID_velocity.I = val;
        Serial.print(">>> Updated Velocity I: ");
        Serial.println(axis.getMotor().PID_velocity.I, 4);
        break;
      }
      case 'f':
      case 'F': {
        float val = Serial.parseFloat();
        axis.getMotor().LPF_velocity.Tf = val;
        Serial.print(">>> Updated Velocity Tf (LPF): ");
        Serial.println(axis.getMotor().LPF_velocity.Tf, 4);
        break;
      }
      case 'r':
      case 'R': {
        float val = Serial.parseFloat();
        axis.getMotor().phase_resistance = val;
        Serial.print(">>> Updated Phase Resistance: ");
        Serial.println(axis.getMotor().phase_resistance, 2);
        break;
      }
      case 'k':
      case 'K': {
        float val = Serial.parseFloat();
        axis.getMotor().KV_rating = val;
        Serial.print(">>> Updated Kv Rating: ");
        Serial.println(axis.getMotor().KV_rating, 2);
        break;
      }
      case 't':
      case 'T': {
        float val = Serial.parseFloat();
        m1_target = val;
        Serial.print(">>> Updated Target Speed: ");
        Serial.print(m1_target, 1);
        Serial.println(" RPM");
        break;
      }
      }
    }

    // Note: encoder.update() is called once at the top of the loop (line 156).
    // Do NOT call it again here — double-updating corrupts velocity calculation.

    // 3. Command target speed to motor
    if (is_enabled) {
      axis.move(m1_target);
    }

    // 4. Evaluate Safety limits
    bool has_sensor_fault = axis.isSensorFault();
    safety.update(millis(), m1_target, has_sensor_fault);

    // 4b. Evaluate Stall detection and trigger safety shutdown
    static uint32_t stall_timer = 0;
    bool is_stalled = false;

    if (is_enabled && control_mode == MotionControlType::velocity) {
      float vel_error = fabsf(m1_target - axis.getRPM());
      float applied_u = fabsf(axis.getMotor().voltage.q);
      
      // If error is high and applied voltage is substantial (implying motor is trying but failing/stuck)
      if (vel_error > 35.0f && applied_u > 2.5f) {
        if (stall_timer == 0) {
          stall_timer = millis();
        } else if (millis() - stall_timer > 1200) { // 1.2 seconds continuous
          is_stalled = true;
        }
      } else {
        stall_timer = 0;
      }
    } else {
      stall_timer = 0;
    }

    if (is_stalled) {
      Serial.println("\n>>> SAFETY SHUTDOWN: Motor stall/overload detected! <<<");
      Serial.print(">>> Target: "); Serial.print(m1_target, 1);
      Serial.print(" RPM | Actual: "); Serial.print(axis.getRPM(), 1);
      Serial.print(" RPM | Uq: "); Serial.print(axis.getMotor().voltage.q, 2);
      Serial.println(" V");
      Serial.println(">>> Motor disabled. Check mechanics and PIDs.");
      Serial.println(">>> Type 'e' to re-enable when safe.");
      
      is_enabled = false;
      axis.disable();
      m1_target = 0.0f;
      is_stalled = false;
      stall_timer = 0;
    }

    // 5. Print telemetry every 100ms if toggled ON
    if (show_telemetry) {
      static uint32_t print_count = 0;
      print_count++;
      if (print_count >= 500) {
        print_count = 0;
        printStatus();
      }
    }
  }
}
