#include "esp32_interface.h"

#if defined(ARDUINO_ARCH_STM32)
HardwareSerial Serial1(USART1);
#endif

ESP32Interface::ESP32Interface()
    : _last_packet_time(0), _rx_index(0), _sync_found(false) {}

void ESP32Interface::init() {
    Serial1.setTx(PB6);
    Serial1.setRx(PB7);
    Serial1.begin(USART_BAUD_RATE);
    _last_packet_time = millis();
}

bool ESP32Interface::update(float &m1_target, float &m2_target, float &m3_target, float &m4_target,
                            float m1_rpm, float m2_rpm, float m3_rpm, float m4_rpm) {
    bool got_packet = false;
    
    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        
        if (!_sync_found) {
            if (c == 0xAA) {
                _sync_found = true;
                _rx_index = 0;
            }
        } else {
            _rx_buffer[_rx_index++] = c;
            
            // Payload (16 bytes floats) + Checksum (1 byte) = 17 bytes
            if (_rx_index >= 17) {
                _sync_found = false; // Reset for next sync
                
                // Compute XOR checksum on payload
                uint8_t calc_cs = calculate_checksum(_rx_buffer, 16);
                uint8_t rx_cs = _rx_buffer[16];
                
                if (calc_cs == rx_cs) {
                    // Valid packet! Copy target velocities
                    memcpy(&m1_target, &_rx_buffer[0], sizeof(float));
                    memcpy(&m2_target, &_rx_buffer[4], sizeof(float));
                    memcpy(&m3_target, &_rx_buffer[8], sizeof(float));
                    memcpy(&m4_target, &_rx_buffer[12], sizeof(float));
                    
                    _last_packet_time = millis();
                    got_packet = true;
                    
                    // Symmetrical Query-Response: Send immediate feedback response
                    send_feedback(m1_rpm, m2_rpm, m3_rpm, m4_rpm);
                }
            }
        }
    }
    
    return got_packet;
}

uint8_t ESP32Interface::calculate_checksum(const uint8_t *data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; ++i) {
        cs ^= data[i];
    }
    return cs;
}

void ESP32Interface::send_feedback(float m1_rpm, float m2_rpm, float m3_rpm, float m4_rpm) {
    MotorFeedbackPacket packet;
    packet.sync = 0x55;
    packet.m1_rpm = m1_rpm;
    packet.m2_rpm = m2_rpm;
    packet.m3_rpm = m3_rpm;
    packet.m4_rpm = m4_rpm;
    
    // Checksum: XOR of the 16 payload float bytes
    uint8_t *payload_ptr = reinterpret_cast<uint8_t*>(&packet.m1_rpm);
    packet.checksum = calculate_checksum(payload_ptr, 16);
    
    // Write packet to Serial1
    Serial1.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(MotorFeedbackPacket));
}
