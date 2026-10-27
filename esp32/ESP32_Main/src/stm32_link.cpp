#include "stm32_link.h"

STM32Link::STM32Link()
    : _serial(1),
      _rx_index(0),
      _sync_found(false),
      _last_valid_packet_time(0),
      _valid_packet_count(0),
      _checksum_error_count(0) {
    memset(&_latest_feedback, 0, sizeof(_latest_feedback));
}

void STM32Link::begin(uint32_t baud, int rxPin, int txPin) {
    _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
}

uint8_t STM32Link::calculateChecksum(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

void STM32Link::sendCommand(float m1_rpm, float m2_rpm, float m3_rpm, float m4_rpm) {
    // Non-blocking guard: Never stall the main CPU loop if STM32 is disconnected or TX buffer full
    if (_serial.availableForWrite() < static_cast<int>(sizeof(MotorCommandPacket) + 1)) {
        return;
    }

    MotorCommandPacket packet;
    packet.m1_target = m1_rpm;
    packet.m2_target = m2_rpm;
    packet.m3_target = m3_rpm;
    packet.m4_target = m4_rpm;
    packet.checksum = calculateChecksum(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - 1);

    // Send leading sync byte 0xAA followed by packed payload & checksum (18 bytes total)
    uint8_t sync = COMMAND_SYNC_BYTE;
    _serial.write(&sync, 1);
    _serial.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

bool STM32Link::update() {
    bool newPacket = false;
    
    while (_serial.available() > 0) {
        uint8_t byte = _serial.read();

        if (!_sync_found) {
            if (byte == FEEDBACK_SYNC_BYTE) { // 0x55
                _sync_found = true;
                _rx_index = 0;
            }
        } else {
            _rx_buffer[_rx_index++] = byte;

            if (_rx_index >= static_cast<int>(sizeof(_rx_buffer))) {
                // Buffer filled with 17 bytes: 16 payload bytes (4 floats) + 1 checksum byte
                uint8_t expectedChecksum = _rx_buffer[16];
                uint8_t computedChecksum = calculateChecksum(_rx_buffer, 16);

                if (expectedChecksum == computedChecksum) {
                    _latest_feedback.sync = FEEDBACK_SYNC_BYTE;
                    memcpy(&_latest_feedback.m1_rpm, _rx_buffer, 16);
                    _latest_feedback.checksum = expectedChecksum;

                    _last_valid_packet_time = millis();
                    _valid_packet_count++;
                    newPacket = true;
                } else {
                    _checksum_error_count++;
                }

                // Reset state machine for next sync byte
                _sync_found = false;
                _rx_index = 0;
            }
        }
    }

    return newPacket;
}
