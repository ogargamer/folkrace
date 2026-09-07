#ifndef MOTOR_AXIS_I2C_H
#define MOTOR_AXIS_I2C_H

#include <SimpleFOC.h>

class MotorAxisI2C {
public:
    MotorAxisI2C(int pinA, int pinB, int pinC, int pinEN, Sensor* sensor = nullptr);

    void init();
    bool initFOC();
    void loopFOC();
    void move(float target_rpm);

    void setControlMode(MotionControlType mode);
    float getRPM();
    float getAngle();
    bool isSensorFault();
    void disable();
    void enable();

    BLDCMotor& getMotor() { return motor; }

private:
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    Sensor* _sensor;
    int _pinA, _pinB, _pinC, _pinEN;
};

#endif // MOTOR_AXIS_I2C_H
