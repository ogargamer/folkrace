#include <Arduino.h>
#include "servo_driver.h"

constexpr uint8_t SERVO_PIN = 21;

// Use ESP32-S3 Native Dedicated MCPWM Hardware Peripheral
ESP32MCPWMDriver servo(SERVO_PIN, ES08MDII_MIN_US, ES08MDII_MAX_US);

// Motion State
bool sweepMode = true;
float currentAngle = 90.0f; // Start at center (1500us)
float sweepDirection = 1.0f;
float sweepSpeedDegPerSec = 120.0f; // 120 degrees per second smooth movement

unsigned long lastUpdateUs = 0;
unsigned long lastTelemetryMs = 0;

void printMenu() {
    Serial.println(F("\n======================================================================="));
    Serial.println(F(" ES08MDII SERVO NATIVE ESP32-S3 MCPWM DRIVER (GPIO 21)                 "));
    Serial.println(F(" Dedicated Silicon Motor Control Peripheral (Direct 1.0 us Tick Compare)"));
    Serial.println(F("======================================================================="));
    Serial.println(F(" Mandatory Wiring Checklist for External PSU:"));
    Serial.println(F("   [!] EXTERNAL PSU GND must be connected to ESP32 GND (Shared Ground!) "));
    Serial.println(F("   [!] EXTERNAL PSU 5V connected to Servo RED Wire (+V)                "));
    Serial.println(F("   [!] GPIO 21 connected to Servo ORANGE/YELLOW Signal Wire            "));
    Serial.println(F("-----------------------------------------------------------------------"));
    Serial.println(F(" Commands:"));
    Serial.println(F("   s - Toggle smooth continuous sweep ON / OFF"));
    Serial.println(F("   0 - Move servo to 0 deg (500 us)"));
    Serial.println(F("   4 - Move servo to 45 deg (1000 us)"));
    Serial.println(F("   9 - Move servo to 90 deg / Neutral (1500 us)"));
    Serial.println(F("   3 - Move servo to 135 deg (2000 us)"));
    Serial.println(F("   1 - Move servo to 180 deg (2500 us)"));
    Serial.println(F("   + - Increment (+10 deg)"));
    Serial.println(F("   - - Decrement (-10 deg)"));
    Serial.println(F("=======================================================================\n"));
}

void setup() {
    Serial.begin(921600);
    delay(1000);

    Serial.println(F("\n[INIT] Initializing Native ESP32-S3 MCPWM Hardware Engine on GPIO 21..."));

    if (servo.begin()) {
        Serial.println(F("[SUCCESS] Dedicated MCPWM Hardware attached to GPIO 21 successfully."));
    } else {
        Serial.println(F("[FAIL] Could not attach MCPWM to GPIO 21!"));
    }

    printMenu();

    // Calibration Sweep: Neutral -> 0 -> 180 -> 90
    Serial.println(F("[SWEEP] Executing MCPWM calibration movement..."));
    servo.setAngle(90.0f); delay(500);
    servo.setAngle(0.0f);  delay(500);
    servo.setAngle(180.0f); delay(500);
    servo.setAngle(90.0f); delay(500);

    lastUpdateUs = micros();
    lastTelemetryMs = millis();
}

void loop() {
    unsigned long nowUs = micros();
    unsigned long nowMs = millis();

    // Process Serial Input Commands
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 's' || c == 'S') {
            sweepMode = !sweepMode;
            Serial.printf("[CMD] Continuous Sweep Mode: %s\n", sweepMode ? "ACTIVE" : "PAUSED");
        } else if (c == '0') {
            sweepMode = false;
            currentAngle = 0.0f;
            servo.setAngle(currentAngle);
            Serial.printf("[CMD] Set target 0.0 deg (500 us)\n");
        } else if (c == '4') {
            sweepMode = false;
            currentAngle = 45.0f;
            servo.setAngle(currentAngle);
            Serial.printf("[CMD] Set target 45.0 deg (1000 us)\n");
        } else if (c == '9') {
            sweepMode = false;
            currentAngle = 90.0f;
            servo.setAngle(currentAngle);
            Serial.printf("[CMD] Set target 90.0 deg (1500 us)\n");
        } else if (c == '3') {
            sweepMode = false;
            currentAngle = 135.0f;
            servo.setAngle(currentAngle);
            Serial.printf("[CMD] Set target 135.0 deg (2000 us)\n");
        } else if (c == '1') {
            sweepMode = false;
            currentAngle = 180.0f;
            servo.setAngle(currentAngle);
            Serial.printf("[CMD] Set target 180.0 deg (2500 us)\n");
        } else if (c == '+' || c == '=') {
            sweepMode = false;
            currentAngle = min(180.0f, currentAngle + 10.0f);
            servo.setAngle(currentAngle);
            Serial.printf("[CMD] Incremented to %.1f deg (%u us)\n", currentAngle, servo.getSteeringPWM());
        } else if (c == '-' || c == '_') {
            sweepMode = false;
            currentAngle = max(0.0f, currentAngle - 10.0f);
            servo.setAngle(currentAngle);
            Serial.printf("[CMD] Decremented to %.1f deg (%u us)\n", currentAngle, servo.getSteeringPWM());
        }
    }

    // Continuous Sweep Update
    float dt = (nowUs - lastUpdateUs) / 1000000.0f;
    lastUpdateUs = nowUs;

    if (sweepMode) {
        currentAngle += sweepDirection * sweepSpeedDegPerSec * dt;
        if (currentAngle >= 180.0f) {
            currentAngle = 180.0f;
            sweepDirection = -1.0f;
        } else if (currentAngle <= 0.0f) {
            currentAngle = 0.0f;
            sweepDirection = 1.0f;
        }
        servo.setAngle(currentAngle);
    }

    // Telemetry Stream (200ms)
    if (nowMs - lastTelemetryMs >= 200) {
        lastTelemetryMs = nowMs;
        Serial.printf("[GPIO 21 MCPWM] Angle: %5.1f deg | Pulse: %4u us | Status: %s\n",
                      currentAngle,
                      servo.getSteeringPWM(),
                      sweepMode ? "SWEEPING..." : "HOLD");
    }

    delay(2);
}
