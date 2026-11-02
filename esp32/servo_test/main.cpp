/*
 * =========================================================================================
 * FOLKRACE — ESP32-S3 HIGH-RATE (333 Hz) DIGITAL SERVO FIRMWARE
 * =========================================================================================
 * Target: ESP32-S3 (ESP32-S3-WROOM-1 / DevKitC-1 N8R8 / N16R8)
 *
 * SERVO SPECIFICATIONS:
 *   - Modulation:       Digital
 *   - Rotational Range: 180°
 *   - Refresh Rate:     333 Hz (Period = 3003.0 µs ≈ 3.0 ms)
 *   - Pulse Width:      500 µs to 2500 µs
 *   - Neutral Position: 1520 µs (Center / 90°)
 *   - Dead Band:        2 µs
 *   - Control System:   PWM (LEDC Peripheral, 14-bit resolution = 0.183 µs/tick)
 *
 * RECOMMENDED PWM PIN FOR ESP32-S3:
 *   - BEST PIN: GPIO 21 (Matches "FOLKRACE" master SERVO_FRONT_PWM allocation).
 *     Why: Free of internal Octal PSRAM/Flash (GPIO 26-37), no strapping roles,
 *          full push-pull drive (unlike input-only GPIO 46), glitch-free boot,
 *          and physically routed to the outer header.
 *   - Alternatives: GPIO 38, GPIO 47, GPIO 1, GPIO 2.
 *   - NEVER USE ON S3:
 *       * GPIO 22-25: DO NOT EXIST in ESP32-S3 silicon!
 *       * GPIO 26-37: Reserved for Octal Flash & PSRAM (immediate fatal crash).
 *       * GPIO 46: INPUT-ONLY (GPI, lacks output transistors).
 *       * GPIO 48: DevKit onboard WS2812 RGB LED.
 *       * GPIO 43/44: UART0 console TX/RX.
 *       * GPIO 19/20: Native USB D-/D+.
 *
 * SERIAL COMMAND PROTOCOL (115200 Baud, LF / \n terminated):
 *   - P<us>        : Set pulse width in microseconds (e.g. "P1520", "P500", "P2500")
 *   - A<deg>       : Set angle in degrees (e.g. "A90.0", "A0", "A180")
 *   - D or DISABLE : Cut PWM output (0% duty) so servo motor rests without holding torque
 *   - E or ENABLE  : Re-enable PWM output at the last target pulse width
 *   - STATUS       : Query current telemetry / state
 *   - PING         : Responds with "PONG"
 * =========================================================================================
 */

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Pin and Hardware Configuration
// -----------------------------------------------------------------------------
#define SERVO_PIN         21        // Recommended PWM pin for ESP32-S3 (FOLKRACE SERVO_FRONT_PWM)
#define PWM_FREQ_HZ       333       // 333 Hz refresh rate (3.0 ms cycle)
#define PWM_CHANNEL       0         // LEDC channel (0-7 on ESP32-S3)
#define PWM_RESOLUTION    14        // 14-bit (0-16383), ~0.183 us per count

// Servo Mechanical & Electrical Limits
#define PULSE_MIN_US      500       // 0° endpoint
#define PULSE_NEUTRAL_US  1520      // 90° neutral position
#define PULSE_MAX_US      2500      // 180° endpoint
#define DEAD_BAND_US      2         // Minimum movement threshold

// Math Constants for LEDC PWM
// At 333 Hz, period = 1,000,000 / 333 = 3003.003 us
const float PWM_PERIOD_US = 1000000.0f / (float)PWM_FREQ_HZ;
const uint32_t PWM_MAX_DUTY = (1UL << PWM_RESOLUTION) - 1; // 16383

// -----------------------------------------------------------------------------
// Global State Variables
// -----------------------------------------------------------------------------
volatile uint16_t currentPulseUs = PULSE_NEUTRAL_US;
volatile float currentAngleDeg = 90.0f;
volatile bool isPwmEnabled = true;

// -----------------------------------------------------------------------------
// Helper: Map Angle (0-180°) to Pulse Width (500-2500 µs with 1520 µs Center)
// -----------------------------------------------------------------------------
uint16_t angleToPulseUs(float angleDeg) {
    if (angleDeg < 0.0f) angleDeg = 0.0f;
    if (angleDeg > 180.0f) angleDeg = 180.0f;

    // Piecewise linear mapping respecting exact 1520 µs digital neutral at 90°
    if (angleDeg <= 90.0f) {
        return (uint16_t)(PULSE_MIN_US + (angleDeg / 90.0f) * (PULSE_NEUTRAL_US - PULSE_MIN_US) + 0.5f);
    } else {
        return (uint16_t)(PULSE_NEUTRAL_US + ((angleDeg - 90.0f) / 90.0f) * (PULSE_MAX_US - PULSE_NEUTRAL_US) + 0.5f);
    }
}

// -----------------------------------------------------------------------------
// Helper: Map Pulse Width (500-2500 µs) back to Angle (0-180°)
// -----------------------------------------------------------------------------
float pulseUsToAngle(uint16_t pulseUs) {
    if (pulseUs <= PULSE_NEUTRAL_US) {
        if (pulseUs < PULSE_MIN_US) pulseUs = PULSE_MIN_US;
        return ((float)(pulseUs - PULSE_MIN_US) / (float)(PULSE_NEUTRAL_US - PULSE_MIN_US)) * 90.0f;
    } else {
        if (pulseUs > PULSE_MAX_US) pulseUs = PULSE_MAX_US;
        return 90.0f + ((float)(pulseUs - PULSE_NEUTRAL_US) / (float)(PULSE_MAX_US - PULSE_NEUTRAL_US)) * 90.0f;
    }
}

// -----------------------------------------------------------------------------
// Hardware PWM Output Control
// -----------------------------------------------------------------------------
void writeHardwarePwm(uint16_t pulseUs) {
    if (!isPwmEnabled) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
        ledcWrite(SERVO_PIN, 0);
#else
        ledcWrite(PWM_CHANNEL, 0);
#endif
        return;
    }

    if (pulseUs < PULSE_MIN_US) pulseUs = PULSE_MIN_US;
    if (pulseUs > PULSE_MAX_US) pulseUs = PULSE_MAX_US;

    // Calculate duty cycle: duty = (pulse_us / period_us) * MAX_DUTY
    uint32_t duty = (uint32_t)(((float)pulseUs / PWM_PERIOD_US) * (float)PWM_MAX_DUTY + 0.5f);
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(SERVO_PIN, duty);
#else
    ledcWrite(PWM_CHANNEL, duty);
#endif
}

void setPulseWidth(uint16_t pulseUs) {
    if (pulseUs < PULSE_MIN_US) pulseUs = PULSE_MIN_US;
    if (pulseUs > PULSE_MAX_US) pulseUs = PULSE_MAX_US;

    currentPulseUs = pulseUs;
    currentAngleDeg = pulseUsToAngle(pulseUs);
    writeHardwarePwm(currentPulseUs);
}

void setAngle(float angleDeg) {
    if (angleDeg < 0.0f) angleDeg = 0.0f;
    if (angleDeg > 180.0f) angleDeg = 180.0f;

    currentAngleDeg = angleDeg;
    currentPulseUs = angleToPulseUs(angleDeg);
    writeHardwarePwm(currentPulseUs);
}

void setPwmEnabled(bool enable) {
    isPwmEnabled = enable;
    if (isPwmEnabled) {
        writeHardwarePwm(currentPulseUs);
    } else {
        writeHardwarePwm(0);
    }
}

// -----------------------------------------------------------------------------
// Telemetry & Feedback
// -----------------------------------------------------------------------------
void sendTelemetry() {
    Serial.print("{\"pin\":");
    Serial.print(SERVO_PIN);
    Serial.print(",\"pulse_us\":");
    Serial.print(currentPulseUs);
    Serial.print(",\"angle\":");
    Serial.print(currentAngleDeg, 2);
    Serial.print(",\"freq_hz\":");
    Serial.print(PWM_FREQ_HZ);
    Serial.print(",\"period_us\":");
    Serial.print(PWM_PERIOD_US, 1);
    Serial.print(",\"enabled\":");
    Serial.print(isPwmEnabled ? "true" : "false");
    Serial.println("}");
}

// -----------------------------------------------------------------------------
// Command Parser
// -----------------------------------------------------------------------------
void processCommand(String line) {
    line.trim();
    if (line.length() == 0) return;

    if (line.equalsIgnoreCase("PING")) {
        Serial.println("PONG");
    }
    else if (line.equalsIgnoreCase("STATUS")) {
        sendTelemetry();
    }
    else if (line.equalsIgnoreCase("D") || line.equalsIgnoreCase("DISABLE")) {
        setPwmEnabled(false);
        sendTelemetry();
    }
    else if (line.equalsIgnoreCase("E") || line.equalsIgnoreCase("ENABLE")) {
        setPwmEnabled(true);
        sendTelemetry();
    }
    else if (line.charAt(0) == 'P' || line.charAt(0) == 'p') {
        // Direct pulse width command, e.g. "P1520"
        long us = line.substring(1).toInt();
        if (us >= PULSE_MIN_US && us <= PULSE_MAX_US) {
            setPwmEnabled(true);
            setPulseWidth((uint16_t)us);
            sendTelemetry();
        } else {
            Serial.print("{\"error\":\"Pulse out of bounds (500-2500 us): ");
            Serial.print(us);
            Serial.println("\"}");
        }
    }
    else if (line.charAt(0) == 'A' || line.charAt(0) == 'a') {
        // Angle command, e.g. "A90.0"
        float deg = line.substring(1).toFloat();
        if (deg >= 0.0f && deg <= 180.0f) {
            setPwmEnabled(true);
            setAngle(deg);
            sendTelemetry();
        } else {
            Serial.print("{\"error\":\"Angle out of bounds (0-180 deg): ");
            Serial.print(deg);
            Serial.println("\"}");
        }
    }
    else {
        Serial.print("{\"error\":\"Unknown command: ");
        Serial.print(line);
        Serial.println("\"}");
    }
}

// -----------------------------------------------------------------------------
// Setup & Loop
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n--- ESP32-S3 333Hz Digital Servo Controller Initialized ---");
    Serial.printf("Config: Pin=%d, Freq=%dHz, Resolution=%d bits (0-%d)\n",
                  SERVO_PIN, PWM_FREQ_HZ, PWM_RESOLUTION, PWM_MAX_DUTY);
    Serial.printf("Limits: Min=%d us (0°), Neutral=%d us (90°), Max=%d us (180°)\n",
                  PULSE_MIN_US, PULSE_NEUTRAL_US, PULSE_MAX_US);

    // Initialize LEDC Hardware Timer & Channel
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // Arduino ESP32 Core v3.0+ API
    ledcAttach(SERVO_PIN, PWM_FREQ_HZ, PWM_RESOLUTION);
#else
    // Arduino ESP32 Core v2.x API
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION);
    ledcAttachPin(SERVO_PIN, PWM_CHANNEL);
#endif

    // Move to 1520 us neutral position immediately
    setPulseWidth(PULSE_NEUTRAL_US);
    sendTelemetry();
}

void loop() {
    static String rxBuffer = "";

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (rxBuffer.length() > 0) {
                processCommand(rxBuffer);
                rxBuffer = "";
            }
        } else {
            if (rxBuffer.length() < 64) {
                rxBuffer += c;
            }
        }
    }
}
