#ifndef HAL_INTERFACES_H
#define HAL_INTERFACES_H

#include <cstdint>
#include <cstddef>

/**
 * @brief Hardware Abstraction Layer for Servo Driver (Steering & Active Suspension)
 */
class IServoDriver {
public:
    virtual ~IServoDriver() = default;
    virtual void setSteeringPWM(uint16_t pulse_width_us) = 0;
    virtual void setSuspensionPWM(uint16_t pulse_width_us) = 0;
};

/**
 * @brief Hardware Abstraction Layer for STM32 Motor Driver UART Link
 */
class IUARTDriver {
public:
    virtual ~IUARTDriver() = default;
    virtual void transmitMotorCommand(const float target_rpms[4]) = 0;
    virtual bool readMotorEncoderFeedback(float measured_rpms[4]) = 0;
};

/**
 * @brief Hardware Abstraction Layer for SPI Bus Communications (BNO08x IMU & Host Pi 5)
 */
class ISPIDriver {
public:
    virtual ~ISPIDriver() = default;
    virtual bool transferIMUData(void* rx_buf, size_t len) = 0;
    virtual bool transferHostData(const void* tx_buf, void* rx_buf, size_t len) = 0;
};

/**
 * @brief Mock implementation of IServoDriver for native unit testing
 */
class MockServoDriver : public IServoDriver {
public:
    uint16_t last_steering_pwm{1500};
    uint16_t last_suspension_pwm{1500};

    void setSteeringPWM(uint16_t pulse_width_us) override {
        last_steering_pwm = pulse_width_us;
    }
    void setSuspensionPWM(uint16_t pulse_width_us) override {
        last_suspension_pwm = pulse_width_us;
    }
};

/**
 * @brief Mock implementation of IUARTDriver for native unit testing
 */
class MockUARTDriver : public IUARTDriver {
public:
    float last_target_rpms[4]{0.0f, 0.0f, 0.0f, 0.0f};
    float simulated_measured_rpms[4]{0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t transmit_count{0};

    void transmitMotorCommand(const float target_rpms[4]) override {
        for (int i = 0; i < 4; ++i) {
            last_target_rpms[i] = target_rpms[i];
        }
        transmit_count++;
    }

    bool readMotorEncoderFeedback(float measured_rpms[4]) override {
        for (int i = 0; i < 4; ++i) {
            measured_rpms[i] = simulated_measured_rpms[i];
        }
        return true;
    }
};

#endif // HAL_INTERFACES_H
