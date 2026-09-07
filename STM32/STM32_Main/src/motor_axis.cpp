#include "motor_axis.h"
#include "config.h"

MotorAxis::MotorAxis(int pinA, int pinB, int pinC, int pinEN, int encA, int encB, void (*interruptA)(), void (*interruptB)())
    : motor(MOTOR_POLE_PAIRS),
      driver(pinA, pinB, pinC, pinEN),
      encoder(encA, encB, 512), // 512 PPR = 2048 CPR
      _pinA(pinA), _pinB(pinB), _pinC(pinC), _pinEN(pinEN),
      _encA(encA), _encB(encB),
      _interruptA(interruptA), _interruptB(interruptB) {}

void MotorAxis::init() {
    // 1. Initialize Encoder
    encoder.init();
    encoder.enableInterrupts(_interruptA, _interruptB);
    motor.linkSensor(&encoder);

    // 2. Initialize Driver
    driver.voltage_power_supply = DRIVER_VOLTAGE_LIMIT;
    driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    driver.init();
    motor.linkDriver(&driver);

    // 3. Configure Motor Control Parameters
    motor.controller = MotionControlType::velocity;
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor.current_limit = MOTOR_CURRENT_LIMIT;

    // Velocity Loop PID Tuning (GM3506 Sane Defaults)
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 5.0f;
    motor.PID_velocity.D = 0.001f;
    motor.PID_velocity.output_ramp = 1000.0f; // Max voltage change rate
    
    // Velocity Low-Pass Filter
    motor.LPF_velocity.Tf = 0.01f; // 10ms time constant

    // 4. Initialize FOC
    motor.init();
    motor.initFOC();
}

void MotorAxis::loopFOC() {
    motor.loopFOC();
}

void MotorAxis::move(float target_rpm) {
    // SimpleFOC expects target in rad/s: rad/s = RPM * (2 * PI) / 60
    float target_rad_s = target_rpm * 0.104719755f;
    motor.move(target_rad_s);
}

float MotorAxis::getRPM() {
    // actual velocity is in rad/s, convert back to RPM
    return motor.shaft_velocity * 9.54929659f;
}

bool MotorAxis::isSensorFault() {
    return false;
}

void MotorAxis::disable() {
    motor.disable();
}

void MotorAxis::enable() {
    motor.enable();
}
