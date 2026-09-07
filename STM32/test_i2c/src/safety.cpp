#include "safety.h"
#include "config_i2c.h"

SafetyManager::SafetyManager()
    : _watchdog_active(false), _last_led_toggle(0), _led_state(false), _last_decel_update(0) {}

void SafetyManager::init() {
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH); // Off by default (Active-Low)
    _last_led_toggle = millis();
    _last_decel_update = millis();
}

void SafetyManager::update(uint32_t last_packet_time, float &m1_target, bool has_sensor_fault) {
    // In standalone test mode, bypass the UART communication watchdog
    _watchdog_active = false;
    _last_decel_update = millis();

    update_led(_watchdog_active || has_sensor_fault);
}

void SafetyManager::decelerate_target(float &m1) {
    uint32_t now = millis();
    float dt = (now - _last_decel_update) / 1000.0f; // dt in seconds
    _last_decel_update = now;

    if (dt <= 0.0f) return;

    float decel_limit = 1000.0f * dt;

    if (m1 > 0.0f) {
        m1 -= decel_limit;
        if (m1 < 0.0f) m1 = 0.0f;
    } else if (m1 < 0.0f) {
        m1 += decel_limit;
        if (m1 > 0.0f) m1 = 0.0f;
    }
}

extern float m1_target;

void SafetyManager::update_led(bool has_fault) {
    uint32_t now = millis();
    uint32_t interval = 1000; // default slow blink when stopped

    float abs_speed = abs(m1_target);
    if (abs_speed > 1000.0f) {
        interval = 50;  // Fast blink for > 1000 RPM
    } else if (abs_speed > 500.0f) {
        interval = 125; // Medium blink for > 500 RPM
    } else if (abs_speed > 100.0f) {
        interval = 250; // Slow blink for > 100 RPM
    } else {
        interval = 1000; // Very slow blink when stopped
    }

    if (has_fault) {
        interval = 25; // Extremely fast blink if sensor fault is present
    }

    if (now - _last_led_toggle >= interval) {
        _led_state = !_led_state;
        _last_led_toggle = now;
        
        digitalWrite(STATUS_LED, _led_state ? LOW : HIGH);
    }
}
