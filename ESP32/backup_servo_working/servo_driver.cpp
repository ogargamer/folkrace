#include "servo_driver.h"
#include <esp32-hal-ledc.h>

// ============================================================================
// 1. ESP32-S3 DEDICATED SILICON MCPWM DRIVER
// ============================================================================
ESP32MCPWMDriver::ESP32MCPWMDriver(uint8_t servo_pin, uint16_t min_pulse_us, uint16_t max_pulse_us)
    : pin(servo_pin),
      minUs(min_pulse_us),
      maxUs(max_pulse_us),
      currentSteeringUs(ES08MDII_NEUTRAL_US),
      currentSuspensionUs(ES08MDII_NEUTRAL_US),
      isAttached(false) {
}

ESP32MCPWMDriver::~ESP32MCPWMDriver() {
    stop();
}

bool ESP32MCPWMDriver::begin() {
    // 1. Route GPIO Pin to MCPWM0A unit
    esp_err_t err = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pin);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] Failed to map MCPWM GPIO on pin %u\n", pin);
        return false;
    }

    // 2. Configure MCPWM Timer for 50Hz Servo Frequency
    mcpwm_config_t pwm_config = {};
    pwm_config.frequency = 50; // 50 Hz = 20ms period
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    err = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] Failed to initialize MCPWM unit 0 timer 0!\n");
        return false;
    }

    // 3. Set initial pulse width in microseconds directly
    setSteeringPWM(ES08MDII_NEUTRAL_US);

    isAttached = true;
    Serial.printf("[MCPWM] Dedicated Silicon MCPWM Engine Active on GPIO %u (50Hz, Direct Microsecond Target)\n", pin);
    return true;
}

void ESP32MCPWMDriver::stop() {
    if (isAttached) {
        mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        isAttached = false;
        Serial.printf("[MCPWM] Stopped MCPWM peripheral on GPIO %u\n", pin);
    }
}

void ESP32MCPWMDriver::setSteeringPWM(uint16_t pulse_width_us) {
    if (pulse_width_us < minUs) pulse_width_us = minUs;
    if (pulse_width_us > maxUs) pulse_width_us = maxUs;

    currentSteeringUs = pulse_width_us;

    if (isAttached) {
        // Direct microsecond pulse setting in hardware (no duty cycle math needed!)
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, currentSteeringUs);
    }
}

void ESP32MCPWMDriver::setSuspensionPWM(uint16_t pulse_width_us) {
    currentSuspensionUs = pulse_width_us;
}

void ESP32MCPWMDriver::setAngle(float degrees) {
    if (degrees < 0.0f) degrees = 0.0f;
    if (degrees > 180.0f) degrees = 180.0f;

    uint16_t us = static_cast<uint16_t>(minUs + (degrees / 180.0f) * (maxUs - minUs));
    setSteeringPWM(us);
}

void ESP32MCPWMDriver::setSteeringAngle(float degrees) {
    if (degrees < -45.0f) degrees = -45.0f;
    if (degrees > 45.0f) degrees = 45.0f;

    uint16_t us = static_cast<uint16_t>(ES08MDII_NEUTRAL_US + (degrees / 45.0f) * 500.0f);
    setSteeringPWM(us);
}


// ============================================================================
// 2. ESP32 LEDC PWM DRIVER (LEGACY / FALLBACK)
// ============================================================================
ESP32ServoDriver::ESP32ServoDriver(uint8_t servo_pin, uint16_t min_pulse_us, uint16_t max_pulse_us)
    : pin(servo_pin),
      minUs(min_pulse_us),
      maxUs(max_pulse_us),
      currentSteeringUs(ES08MDII_NEUTRAL_US),
      currentSuspensionUs(ES08MDII_NEUTRAL_US),
      isAttached(false) {
}

ESP32ServoDriver::~ESP32ServoDriver() {
    stop();
}

bool ESP32ServoDriver::begin() {
    pinMode(pin, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    isAttached = ledcAttach(pin, 50, 14);
#else
    ledcSetup(0, 50, 14);
    ledcAttachPin(pin, 0);
    isAttached = true;
#endif
    if (isAttached) {
        setSteeringPWM(ES08MDII_NEUTRAL_US);
    }
    return isAttached;
}

void ESP32ServoDriver::stop() {
    if (isAttached) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcDetach(pin);
#else
        ledcDetachPin(pin);
#endif
        isAttached = false;
    }
}

void ESP32ServoDriver::setSteeringPWM(uint16_t pulse_width_us) {
    if (pulse_width_us < minUs) pulse_width_us = minUs;
    if (pulse_width_us > maxUs) pulse_width_us = maxUs;

    currentSteeringUs = pulse_width_us;

    if (isAttached) {
        uint32_t duty = (static_cast<uint32_t>(currentSteeringUs) * 16383U) / 20000U;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(pin, duty);
#else
        ledcWrite(0, duty);
#endif
    }
}

void ESP32ServoDriver::setSuspensionPWM(uint16_t pulse_width_us) {
    currentSuspensionUs = pulse_width_us;
}

void ESP32ServoDriver::setAngle(float degrees) {
    if (degrees < 0.0f) degrees = 0.0f;
    if (degrees > 180.0f) degrees = 180.0f;
    uint16_t us = static_cast<uint16_t>(minUs + (degrees / 180.0f) * (maxUs - minUs));
    setSteeringPWM(us);
}

void ESP32ServoDriver::setSteeringAngle(float degrees) {
    if (degrees < -45.0f) degrees = -45.0f;
    if (degrees > 45.0f) degrees = 45.0f;
    uint16_t us = static_cast<uint16_t>(ES08MDII_NEUTRAL_US + (degrees / 45.0f) * 500.0f);
    setSteeringPWM(us);
}
