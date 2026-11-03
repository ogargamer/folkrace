#include "stm32_link.h"

STM32Channel::STM32Channel()
    : serial_(nullptr),
      rx_index_(0),
      sync_found_(false),
      m1_actual_rpm_(0.0f),
      m2_actual_rpm_(0.0f),
      last_packet_ms_(0),
      valid_packet_count_(0),
      checksum_error_count_(0) {}

void STM32Channel::begin(HardwareSerial *serial, uint32_t baud, int rxPin, int txPin) {
    serial_ = serial;
    if (!serial_) return;
    serial_->setRxBufferSize(512);
    serial_->begin(baud, SERIAL_8N1, rxPin, txPin);
}

void STM32Channel::sendCommand(float m1_rpm, float m2_rpm) {
    if (!serial_) return;

    // Non-blocking guard
    if (serial_->availableForWrite() < (int)sizeof(STM32MotorCommandPacket)) {
        return;
    }

    STM32MotorCommandPacket pkt;
    pkt.sync = COMMAND_SYNC_BYTE;
    pkt.m1_rpm = m1_rpm;
    pkt.m2_rpm = m2_rpm;

    uint8_t chk = 0;
    const uint8_t *p = (const uint8_t *)&pkt;
    for (size_t i = 1; i < sizeof(pkt) - 1; i++) {
        chk ^= p[i];
    }
    pkt.checksum = chk;

    serial_->write((const uint8_t *)&pkt, sizeof(pkt));
}

bool STM32Channel::update() {
    if (!serial_) return false;

    bool new_packet = false;
    while (serial_->available() > 0) {
        uint8_t b = serial_->read();

        if (!sync_found_) {
            if (b == 0xAA) {
                rx_buffer_[0] = 0xAA;
                rx_index_ = 1;
            } else if (rx_index_ == 1 && b == 0x55) {
                rx_buffer_[1] = 0x55;
                rx_index_ = 2;
                sync_found_ = true;
            } else {
                rx_index_ = 0;
            }
        } else {
            rx_buffer_[rx_index_++] = b;

            // 26-Byte Telemetry Frame
            if (rx_index_ >= 26) {
                sync_found_ = false;
                rx_index_ = 0;

                // Validate XOR checksum over bytes [2 .. 24]
                uint8_t expected_chk = rx_buffer_[25];
                uint8_t computed_chk = 0;
                for (int i = 2; i < 25; i++) {
                    computed_chk ^= rx_buffer_[i];
                }

                if (computed_chk == expected_chk) {
                    const STM32TelemetryPacket *pkt = (const STM32TelemetryPacket *)rx_buffer_;
                    m1_actual_rpm_ = (float)pkt->m1_actual_rpm * 0.1f;
                    m2_actual_rpm_ = (float)pkt->m2_actual_rpm * 0.1f;
                    valid_packet_count_++;
                    last_packet_ms_ = millis();
                    new_packet = true;
                } else {
                    checksum_error_count_++;
                }
            }
        }
    }
    return new_packet;
}

bool STM32Channel::isConnected() const {
    return (last_packet_ms_ > 0) && (millis() - last_packet_ms_ < 200);
}

// =============================================================================
// STM32 Dual Link Implementation
// =============================================================================
STM32DualLink::STM32DualLink()
    : serial_front_(STM32_1_UART_NUM),
      serial_rear_(STM32_2_UART_NUM) {}

void STM32DualLink::begin(uint32_t baud, int front_rx, int front_tx, int rear_rx, int rear_tx) {
    front_.begin(&serial_front_, baud, front_rx, front_tx);
    rear_.begin(&serial_rear_, baud, rear_rx, rear_tx);
}

void STM32DualLink::sendCommands(float fl_rpm, float fr_rpm, float rl_rpm, float rr_rpm) {
    front_.sendCommand(fl_rpm, fr_rpm);
    rear_.sendCommand(rl_rpm, rr_rpm);
}

bool STM32DualLink::update() {
    bool f_ok = front_.update();
    bool r_ok = rear_.update();
    return f_ok || r_ok;
}
