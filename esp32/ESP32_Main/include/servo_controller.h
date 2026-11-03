#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

class ServoController {
public:
    ServoController(
        uint8_t pin = SERVO_STEER_PWM_PIN,
        uint8_t channel = SERVO_PWM_CHANNEL,
        uint32_t freq_hz = SERVO_PWM_FREQ_HZ,
        uint8_t resolution = SERVO_PWM_RESOLUTION
    );

    bool begin();
    void setPulseUs(float pulse_us);
    void disable();
    void enable();

    float getCurrentPulseUs() const { return current_pulse_us_; }
    bool isEnabled() const { return enabled_; }

private:
    uint8_t pin_;
    uint8_t channel_;
    uint32_t freq_hz_;
    uint8_t resolution_;
    float period_us_;
    uint32_t max_duty_;
    float current_pulse_us_;
    bool enabled_;
};

#endif // SERVO_CONTROLLER_H

