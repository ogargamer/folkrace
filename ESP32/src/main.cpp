#include <Arduino.h>
#include "config.h"
#include "stm32_link.h"
#include "imu_bno08x.h"

// Global Link and IMU Instances
STM32Link stm32Link;
BNO08xIMU imu;
HardwareSerial piSerial(2);

// Timing Control
unsigned long lastLoopTimeMicros = 0;
unsigned long lastTelemetryReportMs = 0;
unsigned long lastIMURetryMs = 0;

// Target Speeds (RPM)
float targetM1 = 0.0f;
float targetM2 = 0.0f;
float targetM3 = 0.0f;
float targetM4 = 0.0f;

void setup() {
    // USB Serial Debug & Control Interface (921600 Baud)
    Serial.begin(921600);
    delay(1000);
    Serial.println(F("\n========================================================"));
    Serial.println(F(" ESP32-S3 Master Controller & IMU Firmware Initialized  "));
    Serial.println(F(" SPI BNO08x IMU Interface + 400Hz STM32 BLDC UART Link "));
    Serial.println(F("========================================================\n"));

    // 1. Initialize UART1 Link to STM32 (921600 Baud, GPIO 18 RX, GPIO 17 TX)
    stm32Link.begin(STM32_UART_BAUD, STM32_UART_RX_PIN, STM32_UART_TX_PIN);

    // 2. Initialize High-Speed UART2 Link to Raspberry Pi 5 (921600 Baud, GPIO 21 RX, GPIO 47 TX)
    piSerial.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX_PIN, PI_UART_TX_PIN);

    // 3. Initialize GY-BNO08X IMU over SPI (CS=10, MISO=13, MOSI=11, SCK=12, INT=9, RST=-1)
    if (imu.begin(DEFAULT_IMU_SPI_CS, DEFAULT_IMU_SPI_MISO, DEFAULT_IMU_SPI_MOSI, DEFAULT_IMU_SPI_SCK, DEFAULT_IMU_SPI_INT, DEFAULT_IMU_SPI_RST)) {
        imu.printMenu();
    } else {
        Serial.println(F("[WARNING] BNO08x IMU hardware scan failed. Will auto-retry in loop..."));
    }

    lastLoopTimeMicros = micros();
    lastTelemetryReportMs = millis();
    lastIMURetryMs = millis();
}

void loop() {
    unsigned long nowMicros = micros();
    unsigned long nowMs = millis();

    // 1. Handle incoming user commands over USB Serial or Pi 5 UART2
    if (Serial.available()) {
        char cmd = Serial.read();
        imu.handleSerialCommand(cmd, Serial);
    }
    if (piSerial.available()) {
        char cmd = piSerial.read();
        imu.handleSerialCommand(cmd, Serial);
    }

    // 2. Auto-retry IMU initialization every 3 seconds if not yet established
    if (!imu.isInitialized()) {
        if (nowMs - lastIMURetryMs >= 3000) {
            lastIMURetryMs = nowMs;
            if (imu.begin(DEFAULT_IMU_SPI_CS, DEFAULT_IMU_SPI_MISO, DEFAULT_IMU_SPI_MOSI, DEFAULT_IMU_SPI_SCK, DEFAULT_IMU_SPI_INT, DEFAULT_IMU_SPI_RST)) {
                imu.printMenu();
            }
        }
    } else {
        // Poll and service GY-BNO08X IMU SHTP events
        imu.update();
    }

    // 3. Precise 400Hz Control Loop Execution (every 2500 us)
    if (nowMicros - lastLoopTimeMicros >= CONTROL_LOOP_PERIOD_US) {
        lastLoopTimeMicros += CONTROL_LOOP_PERIOD_US;

        // Send Motor Command Packet to STM32
        stm32Link.sendCommand(targetM1, targetM2, targetM3, targetM4);
    }

    // 4. Continually poll and parse incoming serial feedback from STM32
    if (stm32Link.update()) {
        // Instant feedback received from STM32
    }

    // 5. High-Speed IMU Telemetry Stream to Raspberry Pi 5 over UART2 (every 10ms = 100Hz)
    static unsigned long lastPiStreamUs = 0;
    if (nowMicros - lastPiStreamUs >= 10000) { // 100 Hz Rock-Solid
        lastPiStreamUs = nowMicros;
        if (imu.isInitialized()) {
            const auto& data = imu.getData();
            // 42-byte Complete High-Precision Binary Frame: 0xAA 0x55 + 10 floats (Quat + Accel + Gyro)
            uint8_t binBuf[42];
            binBuf[0] = 0xAA;
            binBuf[1] = 0x55;
            memcpy(&binBuf[2],  &data.quat_w, 4);
            memcpy(&binBuf[6],  &data.quat_x, 4);
            memcpy(&binBuf[10], &data.quat_y, 4);
            memcpy(&binBuf[14], &data.quat_z, 4);
            memcpy(&binBuf[18], &data.accel_x, 4);
            memcpy(&binBuf[22], &data.accel_y, 4);
            memcpy(&binBuf[26], &data.accel_z, 4);
            memcpy(&binBuf[30], &data.gyro_x,  4);
            memcpy(&binBuf[34], &data.gyro_y,  4);
            memcpy(&binBuf[38], &data.gyro_z,  4);
            piSerial.write(binBuf, 42);
        }
    }

    // 6. Periodic IMU & Telemetry Report to USB Serial Monitor (every 200ms = 5Hz debug stream)
    if (nowMs - lastTelemetryReportMs >= 200) {
        lastTelemetryReportMs = nowMs;

        if (imu.isInitialized()) {
            imu.printCurrentReadings(Serial);
        }
    }
}
