#include "safety.h"
#include "config.h"

SafetyManager::SafetyManager()
    : _watchdog_active(false), _last_led_toggle(0), _led_state(false), _last_decel_update(0) {}

void SafetyManager::init() {
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, HIGH); // Off by default (Active-Low)
    _last_led_toggle = millis();
    _last_decel_update = millis();
}

void SafetyManager::update(uint32_t last_packet_time, float &m1_target, float &m2_target,
                            float &m3_target, float &m4_target, bool has_sensor_fault) {
    uint32_t now = millis();
    
    if (now - last_packet_time > 500) {
        _watchdog_active = true;
    } else {
        _watchdog_active = false;
    }

    if (_watchdog_active) {
        decelerate_targets(m1_target, m2_target, m3_target, m4_target);
    } else {
        _last_decel_update = now;
    }

    update_led(_watchdog_active || has_sensor_fault);
}

void SafetyManager::decelerate_targets(float &m1, float &m2, float &m3, float &m4) {
    uint32_t now = millis();
    float dt = (now - _last_decel_update) / 1000.0f; // dt in seconds
    _last_decel_update = now;

    if (dt <= 0.0f) return;

    float decel_limit = 1000.0f * dt;

    auto apply_decel = [decel_limit](float &target) {
        if (target > 0.0f) {
            target -= decel_limit;
            if (target < 0.0f) target = 0.0f;
        } else if (target < 0.0f) {
            target += decel_limit;
            if (target > 0.0f) target = 0.0f;
        }
    };

    apply_decel(m1);
    apply_decel(m2);
    apply_decel(m3);
    apply_decel(m4);
}

void SafetyManager::update_led(bool has_fault) {
    uint32_t now = millis();
    uint32_t interval = has_fault ? 50 : 250;

    if (now - _last_led_toggle >= interval) {
        _led_state = !_led_state;
        _last_led_toggle = now;
        
        digitalWrite(STATUS_LED, _led_state ? LOW : HIGH);
    }
}
