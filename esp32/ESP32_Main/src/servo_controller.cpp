#include "servo_controller.h"

ServoController::ServoController(
    uint8_t pin,
    uint8_t channel,
    uint32_t freq_hz,
    uint8_t resolution
) : pin_(pin),
    channel_(channel),
    freq_hz_(freq_hz),
    resolution_(resolution),
    current_pulse_us_(SERVO_PULSE_NEUTRAL_US),
    enabled_(false) {
    period_us_ = 1000000.0f / (float)freq_hz_;
    max_duty_ = (1UL << resolution_) - 1;
}

bool ServoController::begin() {
    ledcSetup(channel_, freq_hz_, resolution_);
    ledcAttachPin(pin_, channel_);
    enable();
    setPulseUs(SERVO_PULSE_NEUTRAL_US);
    return true;
}

void IRAM_ATTR ServoController::setPulseUs(float pulse_us) {
    pulse_us = fmaxf(SERVO_PULSE_MIN_US, fminf(SERVO_PULSE_MAX_US, pulse_us));
    current_pulse_us_ = pulse_us;

    if (!enabled_) return;

    // Convert microseconds to 14-bit duty cycles
    uint32_t duty = (uint32_t)((pulse_us / period_us_) * (float)max_duty_ + 0.5f);
    if (duty > max_duty_) duty = max_duty_;
    ledcWrite(channel_, duty);
}

void ServoController::disable() {
    enabled_ = false;
    ledcWrite(channel_, 0);
}

void ServoController::enable() {
    enabled_ = true;
    setPulseUs(current_pulse_us_);
}

