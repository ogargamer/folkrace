#include "stm32_link.h"

STM32Channel::STM32Channel()
    : serial_(nullptr),
      rx_index_(0),
      sync_found_(false),
      m1_actual_rpm_(0.0f),
      m2_actual_rpm_(0.0f),
      status_flags_(0),
      calib_progress_(0),
      last_packet_ms_(0),
      valid_packet_count_(0),
      checksum_error_count_(0) {}

void STM32Channel::begin(HardwareSerial *serial, uint32_t baud, int rxPin, int txPin) {
    serial_ = serial;
    if (!serial_) return;
    serial_->setRxBufferSize(512);
    serial_->setTxBufferSize(512);
    serial_->begin(baud, SERIAL_8N1, rxPin, txPin);
}

void STM32Channel::sendPacket(uint8_t mode, float param1, float param2) {
    if (!serial_) return;

    // Non-blocking guard
    if (serial_->availableForWrite() < (int)sizeof(STM32MotorCommandPacket)) {
        return;
    }

    STM32MotorCommandPacket pkt;
    pkt.header[0] = STM32_CMD_SYNC_1;
    pkt.header[1] = STM32_CMD_SYNC_2;
    pkt.cmd_mode = mode;
    pkt.param1 = param1;
    pkt.param2 = param2;

    uint8_t chk = 0;
    const uint8_t *p = (const uint8_t *)&pkt;
    for (size_t i = 2; i < sizeof(pkt) - 1; i++) {
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
            if (b == STM32_FEEDBACK_SYNC_1) {
                rx_buffer_[0] = STM32_FEEDBACK_SYNC_1;
                rx_index_ = 1;
            } else if (rx_index_ == 1 && b == STM32_FEEDBACK_SYNC_2) {
                rx_buffer_[1] = STM32_FEEDBACK_SYNC_2;
                rx_index_ = 2;
                sync_found_ = true;
            } else {
                rx_index_ = 0;
            }
        } else {
            rx_buffer_[rx_index_++] = b;

            // 13-Byte High-Speed 400 Hz Feedback Packet
            if (rx_index_ >= sizeof(STM32FeedbackPacket)) {
                sync_found_ = false;
                rx_index_ = 0;

                // Validate XOR checksum over payload bytes [2 .. 11]
                uint8_t expected_chk = rx_buffer_[sizeof(STM32FeedbackPacket) - 1];
                uint8_t computed_chk = 0;
                for (size_t i = 2; i < sizeof(STM32FeedbackPacket) - 1; i++) {
                    computed_chk ^= rx_buffer_[i];
                }

                if (computed_chk == expected_chk) {
                    const STM32FeedbackPacket *pkt = (const STM32FeedbackPacket *)rx_buffer_;
                    m1_actual_rpm_ = pkt->m1_actual_rpm;
                    m2_actual_rpm_ = pkt->m2_actual_rpm;
                    status_flags_  = pkt->status_flags;
                    calib_progress_= pkt->calib_progress;

                    valid_packet_count_++;
                    last_packet_ms_ = millis();
                    new_packet = true;
                } else {
                    checksum_error_count_++;
                    // Rescan buffered bytes [1..12] for a real 0xAA 0x55 header
                    for (size_t shift = 1; shift < sizeof(STM32FeedbackPacket); shift++) {
                        if (rx_buffer_[shift] == STM32_FEEDBACK_SYNC_1) {
                            if (shift + 1 < sizeof(STM32FeedbackPacket)) {
                                if (rx_buffer_[shift + 1] == STM32_FEEDBACK_SYNC_2) {
                                    size_t rem = sizeof(STM32FeedbackPacket) - shift;
                                    memmove(rx_buffer_, &rx_buffer_[shift], rem);
                                    rx_index_ = (uint8_t)rem;
                                    sync_found_ = true;
                                    break;
                                }
                            } else {
                                rx_buffer_[0] = STM32_FEEDBACK_SYNC_1;
                                rx_index_ = 1;
                                sync_found_ = false;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    return new_packet;
}

bool STM32Channel::isConnected() const {
    return (last_packet_ms_ > 0) && (millis() - last_packet_ms_ < 250);
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

void STM32DualLink::sendDrive(float fl_rpm, float fr_rpm, float rl_rpm, float rr_rpm) {
    front_.sendDrive(fl_rpm, fr_rpm);
    rear_.sendDrive(rl_rpm, rr_rpm);
}

void STM32DualLink::emergencyStop() {
    front_.emergencyStop();
    rear_.emergencyStop();
}

void STM32DualLink::enableDrivers() {
    front_.enableDrivers();
    rear_.enableDrivers();
}

void STM32DualLink::setPID(float kp, float ki) {
    front_.setPID(kp, ki);
    rear_.setPID(kp, ki);
}

void STM32DualLink::saveFlash() {
    front_.saveFlash();
    rear_.saveFlash();
}

void STM32DualLink::enterDFU(bool front, bool rear) {
    if (front) front_.enterDFU();
    if (rear)  rear_.enterDFU();
}

void STM32DualLink::startCalibration(uint8_t axle, int motor_id, float align_voltage) {
    if (axle == 0 || axle == 1) { // 0=both/front, 1=front
        front_.startCalibration(motor_id, align_voltage);
    }
    if (axle == 0 || axle == 2) { // 0=both/rear, 2=rear
        rear_.startCalibration(motor_id, align_voltage);
    }
}

bool STM32DualLink::update() {
    bool f_ok = front_.update();
    bool r_ok = rear_.update();
    return f_ok || r_ok;
}
