#include "config_i2c.h"
#include "motor_axis_i2c.h"
#include "safety.h"
#include <Arduino.h>
#include <Wire.h>

// ---- Hardware I2C1 Encoder with Register Diagnostic ----
class FilteredMagneticSensorI2C : public MagneticSensorI2C {
public:
    FilteredMagneticSensorI2C(MagneticSensorI2CConfig_s config)
        : MagneticSensorI2C(config), last_valid_angle(0.0f), last_valid_time_us(0),
          estimated_velocity(0.0f), initialized(false), glitch_count(0) {}

    float getSensorAngle() override {
        uint32_t now_us = micros();
        float raw_angle = MagneticSensorI2C::getSensorAngle();

        if (!initialized) {
            last_valid_angle = raw_angle;
            last_valid_time_us = now_us;
            initialized = true;
            return raw_angle;
        }

        float dt = (now_us - last_valid_time_us) * 1.0e-6f;
        if (dt <= 0.0f) dt = 0.0005f;

        float delta = raw_angle - last_valid_angle;
        while (delta > _PI)  delta -= _2PI;
        while (delta < -_PI) delta += _2PI;

        if (fabsf(delta) > 0.5f) {
            glitch_count++;
            last_valid_angle += estimated_velocity * dt;
            while (last_valid_angle >= _2PI) last_valid_angle -= _2PI;
            while (last_valid_angle < 0.0f)  last_valid_angle += _2PI;
            last_valid_time_us = now_us;
            return last_valid_angle;
        }

        float current_vel = delta / dt;
        estimated_velocity = 0.85f * estimated_velocity + 0.15f * current_vel;

        last_valid_angle = raw_angle;
        last_valid_time_us = now_us;
        return raw_angle;
    }

    uint32_t getGlitchCount() const { return glitch_count; }

private:
    float last_valid_angle;
    uint32_t last_valid_time_us;
    float estimated_velocity;
    bool initialized;
    uint32_t glitch_count;
};

FilteredMagneticSensorI2C sensor(MT6701_I2C);

bool checkSensorResponding() {
    Wire.beginTransmission(0x06);
    if (Wire.endTransmission() == 0) return true;
    Wire.beginTransmission(0x46);
    return (Wire.endTransmission() == 0);
}

MotorAxisI2C axis(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_EN, &sensor);

MotionControlType control_mode = MotionControlType::velocity;
bool is_calibrated = false;
bool is_enabled    = false;
bool show_telemetry = false;

SafetyManager safety;
float m1_target = 0.0f;
uint32_t last_control_time_us = 0;

// ---- MT6701 Full EEPROM / Register Dump ----
void dumpMT6701Registers() {
    Serial.println("\n========================================");
    Serial.println("   MT6701 EEPROM / Register Diagnostic  ");
    Serial.println("========================================");

    Wire.beginTransmission(0x06);
    Wire.write(0x00);
    byte err = Wire.endTransmission();

    if (err != 0) {
        Serial.println("Error: MT6701 (0x06) not responding on I2C bus!");
        Serial.println("Ensure SDA=PB9, SCL=PB8 are connected.");
        return;
    }

    Wire.requestFrom(0x06, 16);
    byte regs[16];
    for (int i = 0; i < 16; i++) {
        if (Wire.available()) {
            regs[i] = Wire.read();
        } else {
            regs[i] = 0xFF;
        }
    }

    for (int i = 0; i < 16; i++) {
        Serial.print(" Reg 0x");
        if (i < 10) Serial.print("0");
        Serial.print(i, HEX);
        Serial.print(" : 0x");
        if (regs[i] < 16) Serial.print("0");
        Serial.print(regs[i], HEX);
        Serial.print(" (BIN: ");
        for (int b = 7; b >= 0; b--) {
            Serial.print((regs[i] >> b) & 1);
        }
        Serial.println(")");
    }

    uint16_t angle_raw = ((regs[3] & 0xFF) << 6) | ((regs[4] & 0xFC) >> 2);
    Serial.print("\nRaw 14-bit Angle : "); Serial.print(angle_raw); Serial.print(" / 16384 (");
    Serial.print((angle_raw / 16384.0f) * 360.0f, 2); Serial.println(" deg)");

    Serial.print("ABZ Resolution   : Reg 0x05 = 0x"); Serial.println(regs[5], HEX);
    Serial.print("Mode / Config    : Reg 0x06 = 0x"); Serial.println(regs[6], HEX);
    Serial.println("========================================\n");
}

void scanI2CBus() {
    dumpMT6701Registers();
}

void printHelp() {
    Serial.println("\n==================================================");
    Serial.println("  STM32 Hardware I2C Motor Test (MT6701 Diagnostic) ");
    Serial.println("==================================================");
    Serial.println("Actions:");
    Serial.println("  c     : Start FOC closed-loop calibration");
    Serial.println("  o     : Switch to Open-Loop mode");
    Serial.println("  e     : Toggle Enable/Disable motor driver");
    Serial.println("  x     : Disable motor and stop");
    Serial.println("  g     : Toggle telemetry ON/OFF");
    Serial.println("  b     : Dump MT6701 EEPROM Registers");
    Serial.println("  h / ? : Show this menu");
    Serial.println("Presets:");
    Serial.println("  0, 1, 2, 3, 4 : 0, 50, 100, 200, 500 RPM");
    Serial.println("  + / -         : +/- 10 RPM");
    Serial.println("--------------------------------------------------");
    Serial.print("Sensor Status : ");
    Serial.println(checkSensorResponding() ? "OK (MT6701 Responding)" : "NO RESPONSE / DISCONNECTED");
    Serial.println("Hardware Pins : SDA=PB9 | SCL=PB8 (I2C1 @ 200kHz)");
    Serial.print("Sensor Angle  : "); Serial.print(sensor.getAngle(), 3); Serial.println(" rad");
    Serial.print("Glitches Cut  : "); Serial.println(sensor.getGlitchCount());
    Serial.print("Motor Status  : "); Serial.println(is_enabled ? "ENABLED" : "DISABLED");
    Serial.print("Target Speed  : "); Serial.print(m1_target, 1); Serial.println(" RPM");
    Serial.print("Telemetry     : "); Serial.println(show_telemetry ? "ON" : "OFF");
    Serial.println("==================================================\n");
}

void printStatus() {
    bool ok = checkSensorResponding();
    Serial.print("Ctrl: ");
    Serial.print(control_mode == MotionControlType::velocity_openloop ? "OPEN  " : "CLOSE ");
    Serial.print("| Target: "); Serial.print(m1_target, 1);
    Serial.print(" RPM | Actual: "); Serial.print(axis.getRPM(), 1);
    Serial.print(" RPM | Angle: "); Serial.print(sensor.getAngle(), 3);
    Serial.print(" rad | Sensor: "); Serial.print(ok ? "OK " : "ERR");
    Serial.print(" | Glitches: "); Serial.print(sensor.getGlitchCount());
    Serial.print(" | Uq: "); Serial.print(axis.getMotor().voltage.q, 2);
    Serial.print(" V | Ud: "); Serial.print(axis.getMotor().voltage.d, 2);
    Serial.println(" V");
}

void setup() {
    pinMode(PA15, INPUT_PULLUP); pinMode(PB3, INPUT_PULLUP);
    pinMode(PC14, INPUT_PULLUP); pinMode(PF0, INPUT_PULLUP); pinMode(PF1, INPUT_PULLUP);

    pinMode(ENC_SDA, INPUT_PULLUP);
    pinMode(ENC_SCL, INPUT_PULLUP);

    Wire.setSDA(ENC_SDA);
    Wire.setSCL(ENC_SCL);
    Wire.begin();
    Wire.setClock(200000);

    safety.init();

    Serial.begin(921600);
    Serial.setTimeout(30);
    SimpleFOCDebug::enable(&Serial);
    uint32_t start_time = millis();
    while (!Serial && (millis() - start_time < 3000)) {
        digitalWrite(STATUS_LED, (millis() % 200 < 100) ? LOW : HIGH);
    }
    digitalWrite(STATUS_LED, HIGH);

    scanI2CBus();

    sensor.init(&Wire);
    axis.init();
    axis.disable();

    printHelp();
    last_control_time_us = micros();
}

void loop() {
    uint32_t now_us = micros();
    if (now_us - last_control_time_us >= 500) {
        if (now_us - last_control_time_us > 5000) {
            last_control_time_us = now_us;
        } else {
            last_control_time_us += 500;
        }

        sensor.update();
        if (is_enabled) { axis.loopFOC(); }

        if (Serial.available() > 0) {
            char cmd = Serial.read();
            switch (cmd) {
            case '0': m1_target = 0.0f;   Serial.println("Target: 0 RPM"); break;
            case '1': m1_target = 50.0f;  Serial.println("Target: 50 RPM"); break;
            case '2': m1_target = 100.0f; Serial.println("Target: 100 RPM"); break;
            case '3': m1_target = 200.0f; Serial.println("Target: 200 RPM"); break;
            case '4': m1_target = 500.0f; Serial.println("Target: 500 RPM"); break;
            case '+': m1_target += 10.0f; Serial.print("Target: "); Serial.print(m1_target,1); Serial.println(" RPM"); break;
            case '-': m1_target -= 10.0f; Serial.print("Target: "); Serial.print(m1_target,1); Serial.println(" RPM"); break;

            case 'e': case 'E':
                if (is_enabled) { Serial.println("\n>>> Disabling..."); is_enabled = false; axis.disable(); }
                else             { Serial.println("\n>>> Enabling...");  is_enabled = true;  axis.enable();  }
                break;

            case 'x': case 'X':
                Serial.println("\n>>> Motor disabled.");
                is_enabled = false; axis.disable(); m1_target = 0.0f;
                break;

            case 'b': case 'B':
                dumpMT6701Registers();
                break;

            case 'c': case 'C':
                Serial.println("\n>>> Starting FOC calibration (Hardware I2C 200kHz)...");
                if (!checkSensorResponding()) {
                    Serial.println(">>> ERROR: Cannot calibrate - MT6701 I2C sensor is NOT responding!");
                    break;
                }
                axis.setControlMode(MotionControlType::velocity);
                control_mode = MotionControlType::velocity;
                axis.enable();
                if (axis.initFOC()) {
                    is_calibrated = true; is_enabled = true;
                    Serial.println(">>> Calibration SUCCESS!");
                } else {
                    is_calibrated = false; is_enabled = false; axis.disable();
                    Serial.println(">>> Calibration completed / failed.");
                }
                break;

            case 'o': case 'O':
                Serial.println("\n>>> Open-loop mode...");
                axis.disable();
                axis.setControlMode(MotionControlType::velocity_openloop);
                control_mode = MotionControlType::velocity_openloop;
                is_enabled = true; axis.enable();
                Serial.println(">>> Open-loop active.");
                break;

            case 'g': case 'G':
                show_telemetry = !show_telemetry;
                Serial.print(">>> Telemetry: "); Serial.println(show_telemetry ? "ON" : "OFF");
                break;

            case 'h': case 'H': case '?': printHelp(); break;
            }
        }

        if (is_enabled) { axis.move(m1_target); }

        bool has_fault = axis.isSensorFault();
        safety.update(millis(), m1_target, has_fault);

        if (show_telemetry) {
            static uint32_t cnt = 0;
            if (++cnt >= 200) { cnt = 0; printStatus(); }
        }
    }
}
