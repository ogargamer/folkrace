#include "motor_axis.h"
#include "config.h"

MotorAxis::MotorAxis(int pinA, int pinB, int pinC, int pinEN, Sensor* sensor)
    : motor(MOTOR_POLE_PAIRS),
      driver(pinA, pinB, pinC, pinEN),
      _sensor(sensor),
      _pinA(pinA), _pinB(pinB), _pinC(pinC), _pinEN(pinEN) {}

void MotorAxis::init() {
    // 1. Initialize Driver
    driver.voltage_power_supply = DRIVER_VOLTAGE_LIMIT;
    driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    driver.init();
    motor.linkDriver(&driver);

    // 2. Link Sensor (if provided)
    if (_sensor) {
        motor.linkSensor(_sensor);
    }

    // 3. Configure Motor Control Parameters
    motor.controller = MotionControlType::velocity;
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor.current_limit = MOTOR_CURRENT_LIMIT;
    
    // Feed-Forward disabled: PID outputs voltage directly for full authority
    motor.phase_resistance = 0.0f;
    motor.KV_rating = 0.0f;

    // Velocity Loop PID Tuning (Pure Voltage Mode - Tuned for smooth speed tracking)
    motor.PID_velocity.P = 0.08f;  // Gentle P gain to eliminate 6-step commutation torque oscillation
    motor.PID_velocity.I = 1.0f;   // Smooth steady-state I gain
    motor.PID_velocity.D = 0.0f;   // Disabled to prevent noise amplification
    motor.PID_velocity.output_ramp = 50.0f;  // Smooth voltage slew rate (50V/s)
    motor.PID_velocity.limit = 4.0f;         // Cap PID output at 4V
    
    // Velocity Low-Pass Filter (60ms filter for smooth velocity estimation)
    motor.LPF_velocity.Tf = 0.06f; 
    
    // Downsample velocity loop execution to 2 kHz (20 kHz / 10)
    motor.motion_downsample = 10;

    // 4. Set alignment voltage and initialize Motor Hardware
    motor.voltage_sensor_align = 2.0f;
    motor.init();
}

bool MotorAxis::initFOC() {
    if (!_sensor) {
        Serial.println("Error: Cannot run initFOC without a sensor linked!");
        return false;
    }
    return motor.initFOC();
}

void MotorAxis::loopFOC() {
    motor.loopFOC();
}

void MotorAxis::move(float target_rpm) {
    // SimpleFOC expects target in rad/s: rad/s = RPM * (2 * PI) / 60
    float target_rad_s = target_rpm * 0.104719755f;
    motor.move(target_rad_s);
}

void MotorAxis::setControlMode(MotionControlType mode) {
    motor.controller = mode;
}

float MotorAxis::getRPM() {
    if (_sensor) {
        return _sensor->getVelocity() * 9.54929659f;
    }
    return motor.shaft_velocity * 9.54929659f;
}

float MotorAxis::getAngle() {
    if (_sensor) {
        return _sensor->getAngle();
    }
    return 0.0f;
}

bool MotorAxis::isSensorFault() {
    // Custom fault logic can be added here if needed
    return false;
}

void MotorAxis::disable() {
    motor.disable();
}

void MotorAxis::enable() {
    motor.enable();
}
