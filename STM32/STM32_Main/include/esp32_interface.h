#ifndef ESP32_INTERFACE_H
#define ESP32_INTERFACE_H

#include <Arduino.h>
#include "config.h"

class ESP32Interface {
public:
    ESP32Interface();

    void init();
    
    // Checks for incoming serial data, parses packets, updates targets,
    // and sends feedback immediately upon valid command receipt.
    bool update(float &m1_target, float &m2_target, float &m3_target, float &m4_target,
                float m1_rpm, float m2_rpm, float m3_rpm, float m4_rpm);

    uint32_t getLastPacketTime() const { return _last_packet_time; }

private:
    uint32_t _last_packet_time;
    
    // Local receive state machine
    uint8_t _rx_buffer[sizeof(MotorCommandPacket) - 1]; // Payload + Checksum (17 bytes)
    int _rx_index;
    bool _sync_found;

    uint8_t calculate_checksum(const uint8_t *data, size_t len);
    void send_feedback(float m1_rpm, float m2_rpm, float m3_rpm, float m4_rpm);
};

#endif // ESP32_INTERFACE_H
