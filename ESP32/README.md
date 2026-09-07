# ESP32 Master Controller Project

This folder contains the master controller code for the ESP32 which coordinates target velocities and reads real-time telemetry from the STM32G474CBT6 4-axis BLDC controller.

## Communications Configuration
*   **Interface**: Bidirectional Hardware UART
*   **Baud Rate**: 921600 baud
*   **Update Cadence**: 400Hz control loop (sends `MotorCommandPacket`, then blocks briefly to wait for the immediate query-response `MotorFeedbackPacket`).

## Communication Protocols

Both controllers communicate using packed, fixed-size structures with XOR checksums.

### ESP32 -> STM32 Command Packet (400Hz Query)
```cpp
struct __attribute__((__packed__)) MotorCommandPacket {
    float m1_target; // Target RPM
    float m2_target; // Target RPM
    float m3_target; // Target RPM
    float m4_target; // Target RPM
    uint8_t checksum; // XOR of the 16 preceding payload bytes
};
// Sent with a leading sync byte: [0xAA Sync Byte] [16 bytes Payload] [1 byte Checksum] (18 bytes total)
```

### STM32 -> ESP32 Telemetry Packet (Immediate Response)
```cpp
struct __attribute__((__packed__)) MotorFeedbackPacket {
    uint8_t sync; // 0x55 Sync Byte
    float m1_rpm; // Actual shaft RPM
    float m2_rpm;
    float m3_rpm;
    float m4_rpm;
    uint8_t checksum; // XOR of the 16 preceding payload float bytes
}; // 18 bytes total
```
