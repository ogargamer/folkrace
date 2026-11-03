#ifndef STM32_LINK_H
#define STM32_LINK_H

#include <Arduino.h>
#include "config.h"

class STM32Channel {
public:
    STM32Channel();

    void begin(HardwareSerial *serial, uint32_t baud, int rxPin, int txPin);
    void sendCommand(float m1_rpm, float m2_rpm);
    bool update();

    float getM1RPM() const { return m1_actual_rpm_; }
    float getM2RPM() const { return m2_actual_rpm_; }
    uint32_t getValidPacketCount() const { return valid_packet_count_; }
    uint32_t getChecksumErrorCount() const { return checksum_error_count_; }
    bool isConnected() const;

private:
    HardwareSerial *serial_;
    uint8_t rx_buffer_[32];
    uint8_t rx_index_;
    bool sync_found_;

    float m1_actual_rpm_;
    float m2_actual_rpm_;
    uint32_t last_packet_ms_;
    uint32_t valid_packet_count_;
    uint32_t checksum_error_count_;
};

class STM32DualLink {
public:
    STM32DualLink();

    void begin(
        uint32_t baud = STM32_1_UART_BAUD,
        int front_rx = STM32_1_UART_RX_PIN,
        int front_tx = STM32_1_UART_TX_PIN,
        int rear_rx  = STM32_2_UART_RX_PIN,
        int rear_tx  = STM32_2_UART_TX_PIN
    );

    void sendCommands(float fl_rpm, float fr_rpm, float rl_rpm, float rr_rpm);
    bool update();

    float getFL_RPM() const { return front_.getM1RPM(); }
    float getFR_RPM() const { return front_.getM2RPM(); }
    float getRL_RPM() const { return rear_.getM1RPM(); }
    float getRR_RPM() const { return rear_.getM2RPM(); }

    bool isFrontConnected() const { return front_.isConnected(); }
    bool isRearConnected() const { return rear_.isConnected(); }

    const STM32Channel& getFront() const { return front_; }
    const STM32Channel& getRear() const { return rear_; }

private:
    HardwareSerial serial_front_;
    HardwareSerial serial_rear_;
    STM32Channel front_;
    STM32Channel rear_;
};

#endif // STM32_LINK_H
