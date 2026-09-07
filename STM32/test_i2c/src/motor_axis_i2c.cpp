#include "motor_axis_i2c.h"
#include "config_i2c.h"

MotorAxisI2C::MotorAxisI2C(int pinA, int pinB, int pinC, int pinEN, Sensor* sensor)
    : motor(MOTOR_POLE_PAIRS),
      driver(pinA, pinB, pinC, pinEN),
      _sensor(sensor),
      _pinA(pinA), _pinB(pinB), _pinC(pinC), _pinEN(pinEN) {}

void MotorAxisI2C::init() {
    // 1. Initialize Driver
    driver.voltage_power_supply = DRIVER_VOLTAGE_LIMIT;
    driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    driver.init();
    motor.linkDriver(&driver);

    // 2. Link Sensor (if provided)
    if (_sensor) { motor.linkSensor(_sensor); }

    // 3. Configure Motor Control Parameters
    motor.controller = MotionControlType::velocity;
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor.current_limit = MOTOR_CURRENT_LIMIT;

    // Feed-Forward disabled: PID outputs voltage directly for full authority
    motor.phase_resistance = 0.0f;
    motor.KV_rating = 0.0f;

    // Velocity Loop PID Tuning tailored for I2C feedback
    // Lower P & I gains prevent runaway amplification on discrete I2C reads
    motor.PID_velocity.P = 0.08f;  // Smooth voltage response
    motor.PID_velocity.I = 0.80f;  // Steady-state correction without overshoot
    motor.PID_velocity.D = 0.00f;  // Disabled to prevent amplifying I2C noise
    motor.PID_velocity.output_ramp = 100.0f; // Soft voltage slew rate (V/s)
    motor.PID_velocity.limit = 3.0f;         // Soft current/voltage limit

    // Velocity Low-Pass Filter - 25ms filter smooths discrete I2C velocity steps
    motor.LPF_velocity.Tf = 0.025f; // 25ms filter time constant

    // 4. Set alignment voltage and initialize Motor Hardware
    motor.voltage_sensor_align = 2.0f;
    motor.init();
}

bool MotorAxisI2C::initFOC() {
    if (!_sensor) {
        Serial.println("Error: Cannot run initFOC without a sensor linked!");
        return false;
    }
    return motor.initFOC();
}

void MotorAxisI2C::loopFOC() { motor.loopFOC(); }

void MotorAxisI2C::move(float target_rpm) {
    float target_rad_s = target_rpm * 0.104719755f;
    motor.move(target_rad_s);
}

void MotorAxisI2C::setControlMode(MotionControlType mode) { motor.controller = mode; }

float MotorAxisI2C::getRPM() {
    if (_sensor) {
        return _sensor->getVelocity() * 9.54929659f;
    }
    return motor.shaft_velocity * 9.54929659f;
}

float MotorAxisI2C::getAngle() {
    return _sensor ? _sensor->getAngle() : 0.0f;
}

bool MotorAxisI2C::isSensorFault() { return false; }

void MotorAxisI2C::disable() { motor.disable(); }
void MotorAxisI2C::enable()  { motor.enable();  }
