#ifndef STM32_LINK_H
#define STM32_LINK_H

#include <Arduino.h>
#include "config.h"

struct STM32WheelCmd {
    float rpm;
    float vtrq_min;
    float vtrq_max;
    float vtrq_ff;

    STM32WheelCmd(float r = 0.0f, float vmin = 0.0f, float vmax = 0.0f, float vff = 0.0f)
        : rpm(r), vtrq_min(vmin), vtrq_max(vmax), vtrq_ff(vff) {}
};

class STM32Channel {
public:
    STM32Channel();

    void begin(HardwareSerial *serial, uint32_t baud, int rxPin, int txPin);
    
    // Command Dispatch
    void sendPacket(uint8_t mode,
                    float param1 = 0.0f,
                    float param2 = 0.0f,
                    int16_t m1_vmin_mv = -12000,
                    int16_t m1_vmax_mv = 12000,
                    int16_t m1_vff_mv = 0,
                    int16_t m2_vmin_mv = -12000,
                    int16_t m2_vmax_mv = 12000,
                    int16_t m2_vff_mv = 0);
    void sendDrive(float m1_rpm, float m2_rpm) { sendPacket(STM32_CMD_DRIVE, m1_rpm, m2_rpm); }
    void sendDriveTorque(const STM32WheelCmd &m1, const STM32WheelCmd &m2);
    void sendCommand(float m1_rpm, float m2_rpm) { sendDrive(m1_rpm, m2_rpm); } // backwards compatibility
    void emergencyStop() { sendPacket(STM32_CMD_ESTOP); }
    void enableDrivers() { sendPacket(STM32_CMD_ENABLE); }
    void disableDrivers() { sendPacket(STM32_CMD_DISABLE); }
    void setPID(float kp, float ki) { sendPacket(STM32_CMD_SET_PID, kp, ki); }
    void startCalibration(int motor_id, float align_voltage = 4.0f) {
        sendPacket(motor_id == 1 ? STM32_CMD_CALIB_M1 : STM32_CMD_CALIB_M2, align_voltage, 0.0f);
    }
    void saveFlash() { sendPacket(STM32_CMD_SAVE_FLASH); }
    void enterDFU() { sendPacket(STM32_CMD_ENTER_DFU); }

    // High-Speed 400 Hz Feedback Ingestion
    bool update();

    float getM1RPM() const { return isConnected() ? m1_actual_rpm_ : 0.0f; }
    float getM2RPM() const { return isConnected() ? m2_actual_rpm_ : 0.0f; }
    uint8_t getStatusFlags() const { return isConnected() ? status_flags_ : 0; }
    uint8_t getCalibProgress() const { return isConnected() ? calib_progress_ : 0; }

    bool isM1Enabled() const { return (status_flags_ & STM32_STATUS_M1_ENABLED) != 0; }
    bool isM2Enabled() const { return (status_flags_ & STM32_STATUS_M2_ENABLED) != 0; }
    bool isCalibActive() const { return (status_flags_ & STM32_STATUS_CALIB_ACTIVE) != 0; }
    bool isStalled() const { return (status_flags_ & STM32_STATUS_STALL_WARNING) != 0; }
    bool hasFault() const { return (status_flags_ & STM32_STATUS_LATCHED_FAULT) != 0; }

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
    uint8_t status_flags_;
    uint8_t calib_progress_;
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

    // 400 Hz Synchronized Drive Dispatch
    void sendDrive(float fl_rpm, float fr_rpm, float rl_rpm, float rr_rpm);
    void sendDriveTorque(const STM32WheelCmd &fl,
                         const STM32WheelCmd &fr,
                         const STM32WheelCmd &rl,
                         const STM32WheelCmd &rr);
    void sendCommands(float fl_rpm, float fr_rpm, float rl_rpm, float rr_rpm) { // backwards compatibility
        sendDrive(fl_rpm, fr_rpm, rl_rpm, rr_rpm);
    }

    // Safety & Operational Broadcasts
    void emergencyStop();
    void enableDrivers();
    void disableDrivers();
    void setPID(float kp, float ki);
    void saveFlash();
    void enterDFU(bool front = true, bool rear = true);
    void startCalibration(uint8_t axle, int motor_id, float align_voltage = 4.0f);

    bool update();

    float getFL_RPM() const { return front_.getM1RPM(); }
    float getFR_RPM() const { return front_.getM2RPM(); }
    float getRL_RPM() const { return rear_.getM1RPM(); }
    float getRR_RPM() const { return rear_.getM2RPM(); }

    bool isFrontConnected() const { return front_.isConnected(); }
    bool isRearConnected() const { return rear_.isConnected(); }

    STM32Channel& getFront() { return front_; }
    STM32Channel& getRear() { return rear_; }
    const STM32Channel& getFront() const { return front_; }
    const STM32Channel& getRear() const { return rear_; }

private:
    HardwareSerial serial_front_;
    HardwareSerial serial_rear_;
    STM32Channel front_;
    STM32Channel rear_;
};

#endif // STM32_LINK_H
