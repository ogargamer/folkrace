#ifndef STM32_LINK_H
#define STM32_LINK_H

#include "config.h"

class STM32Link {
public:
    STM32Link();

    void begin(uint32_t baud = STM32_UART_BAUD, int rxPin = STM32_UART_RX_PIN, int txPin = STM32_UART_TX_PIN);
    
    // Sends a 400Hz MotorCommandPacket to STM32 over hardware serial
    void sendCommand(float m1_rpm, float m2_rpm, float m3_rpm, float m4_rpm);

    // Processes incoming bytes from STM32 hardware serial, updates telemetry if valid packet received
    bool update();

    // Telemetry getters
    float getM1RPM() const { return _latest_feedback.m1_rpm; }
    float getM2RPM() const { return _latest_feedback.m2_rpm; }
    float getM3RPM() const { return _latest_feedback.m3_rpm; }
    float getM4RPM() const { return _latest_feedback.m4_rpm; }

    uint32_t getLastPacketTime() const { return _last_valid_packet_time; }
    uint32_t getValidPacketCount() const { return _valid_packet_count; }
    uint32_t getChecksumErrorCount() const { return _checksum_error_count; }

private:
    HardwareSerial _serial;
    MotorFeedbackPacket _latest_feedback;
    
    // RX State Machine
    uint8_t _rx_buffer[sizeof(MotorFeedbackPacket) - 1]; // 17 bytes (4 floats + 1 byte checksum)
    int _rx_index;
    bool _sync_found;

    uint32_t _last_valid_packet_time;
    uint32_t _valid_packet_count;
    uint32_t _checksum_error_count;

    uint8_t calculateChecksum(const uint8_t *data, size_t len);
};

#endif // STM32_LINK_H
