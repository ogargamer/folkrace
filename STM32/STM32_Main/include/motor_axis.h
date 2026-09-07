#ifndef MOTOR_AXIS_H
#define MOTOR_AXIS_H

#include <SimpleFOC.h>

class MotorAxis {
public:
    MotorAxis(int pinA, int pinB, int pinC, int pinEN, int encA, int encB, void (*interruptA)(), void (*interruptB)());

    void init();
    void loopFOC();
    void move(float target_rpm);
    
    float getRPM();
    bool isSensorFault();
    void disable();
    void enable();

    void handleA() { encoder.handleA(); }
    void handleB() { encoder.handleB(); }

private:
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    Encoder encoder;
    
    int _pinA, _pinB, _pinC, _pinEN;
    int _encA, _encB;
    void (*_interruptA)();
    void (*_interruptB)();
};

#endif // MOTOR_AXIS_H
