#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <Arduino.h>
#include "hal_interfaces.h"
#include "driver/mcpwm.h"

// Default ES08MDII Pin and Pulse Constants
constexpr uint8_t DEFAULT_SERVO_PIN = 21;
constexpr uint16_t ES08MDII_MIN_US = 500;   // 0 degrees
constexpr uint16_t ES08MDII_MAX_US = 2500;  // 180 degrees
constexpr uint16_t ES08MDII_NEUTRAL_US = 1500; // 90 degrees

/**
 * @brief Dedicated ESP32-S3 MCPWM (Motor Control PWM) Hardware Servo Driver
 * Uses Espressif's native MCPWM silicon peripheral with direct microsecond pulse control.
 */
class ESP32MCPWMDriver : public IServoDriver {
private:
    uint8_t pin;
    uint16_t minUs;
    uint16_t maxUs;
    uint16_t currentSteeringUs;
    uint16_t currentSuspensionUs;
    bool isAttached;

public:
    ESP32MCPWMDriver(uint8_t servo_pin = DEFAULT_SERVO_PIN, 
                      uint16_t min_pulse_us = ES08MDII_MIN_US, 
                      uint16_t max_pulse_us = ES08MDII_MAX_US);
    ~ESP32MCPWMDriver() override;

    /**
     * @brief Initialize native Espressif MCPWM hardware timer on target pin
     */
    bool begin();

    /**
     * @brief Detach MCPWM hardware generator
     */
    void stop();

    /**
     * @brief IServoDriver Interface: Set steering pulse width in microseconds (500us - 2500us)
     */
    void setSteeringPWM(uint16_t pulse_width_us) override;

    /**
     * @brief IServoDriver Interface: Set suspension pulse width in microseconds
     */
    void setSuspensionPWM(uint16_t pulse_width_us) override;

    /**
     * @brief Helper: Set servo position directly in degrees (0.0 to 180.0)
     */
    void setAngle(float degrees);

    /**
     * @brief Helper: Set servo steering angle in degrees (-45.0 to +45.0)
     */
    void setSteeringAngle(float degrees);

    /**
     * @brief Get current active microsecond pulse width
     */
    uint16_t getSteeringPWM() const { return currentSteeringUs; }
    
    /**
     * @brief Check if MCPWM is active
     */
    bool attached() const { return isAttached; }
    
    /**
     * @brief Get configured pin number
     */
    uint8_t getPin() const { return pin; }
};

/**
 * @brief Legacy/Alternative LEDC PWM Hardware Servo Driver
 */
class ESP32ServoDriver : public IServoDriver {
private:
    uint8_t pin;
    uint16_t minUs;
    uint16_t maxUs;
    uint16_t currentSteeringUs;
    uint16_t currentSuspensionUs;
    bool isAttached;

public:
    ESP32ServoDriver(uint8_t servo_pin = DEFAULT_SERVO_PIN, 
                     uint16_t min_pulse_us = ES08MDII_MIN_US, 
                     uint16_t max_pulse_us = ES08MDII_MAX_US);
    ~ESP32ServoDriver() override;

    bool begin();
    void stop();
    void setSteeringPWM(uint16_t pulse_width_us) override;
    void setSuspensionPWM(uint16_t pulse_width_us) override;
    void setAngle(float degrees);
    void setSteeringAngle(float degrees);
    uint16_t getSteeringPWM() const { return currentSteeringUs; }
    bool attached() const { return isAttached; }
    uint8_t getPin() const { return pin; }
};

#endif // SERVO_DRIVER_H
