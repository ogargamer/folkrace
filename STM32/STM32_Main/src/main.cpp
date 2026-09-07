#include <Arduino.h>
#include "config.h"
#include "motor_axis.h"
#include "esp32_interface.h"
#include "safety.h"

// Forward declaration of interrupt callback functions
void doA1(); void doB1();
void doA2(); void doB2();
void doA3(); void doB3();
void doA4(); void doB4();

// Instantiate 4 Motor Axis objects
MotorAxis axis1(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN, ENC1_A, ENC1_B, doA1, doB1);
MotorAxis axis2(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN, ENC2_A, ENC2_B, doA2, doB2);
MotorAxis axis3(M3_PWM_A, M3_PWM_B, M3_PWM_C, M3_EN, ENC3_A, ENC3_B, doA3, doB3);
MotorAxis axis4(M4_PWM_A, M4_PWM_B, M4_PWM_C, M4_EN, ENC4_A, ENC4_B, doA4, doB4);

// Instantiate Master Interface & Safety Watchdog
ESP32Interface esp32;
SafetyManager safety;

// Global target velocities (RPM)
float m1_target = 0.0f;
float m2_target = 0.0f;
float m3_target = 0.0f;
float m4_target = 0.0f;

// 1kHz Kinematics execution timer
uint32_t last_1khz_time_us = 0;

void setup() {
    // 1. Reclaim PB3, PB4, and PA15 from default JTAG control
    pinMode(PA15, INPUT);
    pinMode(PB3, INPUT);
    pinMode(PB4, INPUT);

    // 2. Reclaim PC14 (Oscillator pin) as digital input
    pinMode(PC14, INPUT);

    // 3. Initialize Safety & ESP32 Communications UART (USART1)
    safety.init();
    esp32.init();

    // 4. Initialize Motor Axes
    axis1.init();
    axis2.init();
    axis3.init();
    axis4.init();

    last_1khz_time_us = micros();
}

void loop() {
    // ---- Unconstrained Commutation Loop (>10kHz) ----
    axis1.loopFOC();
    axis2.loopFOC();
    axis3.loopFOC();
    axis4.loopFOC();

    // ---- Throttled Kinematic & Communication Loop (1kHz) ----
    uint32_t now_us = micros();
    if (now_us - last_1khz_time_us >= 1000) {
        last_1khz_time_us += 1000;

        // Fetch actual motor feedback speeds (RPM)
        float m1_rpm = axis1.getRPM();
        float m2_rpm = axis2.getRPM();
        float m3_rpm = axis3.getRPM();
        float m4_rpm = axis4.getRPM();

        // 1. Process Master UART commands and immediately trigger query-response telemetry
        esp32.update(
            m1_target, m2_target, m3_target, m4_target,
            m1_rpm, m2_rpm, m3_rpm, m4_rpm
        );

        // 2. Evaluate Safety limits and communication watchdog timeout
        bool has_sensor_fault = axis1.isSensorFault() || axis2.isSensorFault() || 
                               axis3.isSensorFault() || axis4.isSensorFault();
        
        safety.update(
            esp32.getLastPacketTime(),
            m1_target, m2_target, m3_target, m4_target,
            has_sensor_fault
        );

        // 3. Command updated targets to SimpleFOC velocity loops
        axis1.move(m1_target);
        axis2.move(m2_target);
        axis3.move(m3_target);
        axis4.move(m4_target);
    }
}

// ---- Global Interrupt Service Routine (ISR) Routing ----
void doA1() { axis1.handleA(); }
void doB1() { axis1.handleB(); }
void doA2() { axis2.handleA(); }
void doB2() { axis2.handleB(); }
void doA3() { axis3.handleA(); }
void doB3() { axis3.handleB(); }
void doA4() { axis4.handleA(); }
void doB4() { axis4.handleB(); }
