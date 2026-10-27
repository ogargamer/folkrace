#ifndef STM32_LINK_H
#define STM32_LINK_H

#include <Arduino.h>
#include "config.h"

class STM32Link {
public:
    STM32Link();

    void begin(uint32_t baud, int rxPin, int txPin);
    void sendCommand(float m1_rpm, float m2_rpm, float m3_rpm, float m4_rpm);
    bool update();

    const MotorFeedbackPacket& getLatestFeedback() const { return _latest_feedback; }
    unsigned long getLastValidPacketTime() const { return _last_valid_packet_time; }
    uint32_t getValidPacketCount() const { return _valid_packet_count; }
    uint32_t getChecksumErrorCount() const { return _checksum_error_count; }

private:
    uint8_t calculateChecksum(const uint8_t *data, size_t len);

    HardwareSerial _serial;
    uint8_t _rx_buffer[17];
    int _rx_index;
    bool _sync_found;

    MotorFeedbackPacket _latest_feedback;
    unsigned long _last_valid_packet_time;
    uint32_t _valid_packet_count;
    uint32_t _checksum_error_count;
};

#endif // STM32_LINK_H
