#include <Arduino.h>
#include "imu_bno08x.h"
#include "stm32_link.h"
#include "robot_config.h"

// Hardware Component Instances
BNO08xIMU imu;
STM32Link stm32Link(Serial1);

// Motor Target Duty Cycles (-1.0f to +1.0f)
float targetM1 = 0.0f;
float targetM2 = 0.0f;
float targetM3 = 0.0f;
float targetM4 = 0.0f;

// Loop Timing Control (400Hz Main Loop)
constexpr unsigned long CONTROL_LOOP_FREQ_HZ = 400;
constexpr unsigned long CONTROL_LOOP_PERIOD_US = 1000000 / CONTROL_LOOP_FREQ_HZ;
unsigned long lastLoopTimeMicros = 0;

// Serial Telemetry Reporting (5Hz Periodic Output)
unsigned long lastTelemetryReportMs = 0;
unsigned long lastIMURetryMs = 0;

void setup() {
    // 1. Initialize USB Serial Monitor for High-Speed Debugging (921600 Baud)
    Serial.begin(921600);
    delay(500);

    Serial.println(F("\n========================================================"));
    Serial.println(F(" ESP32-S3 Master Controller & IMU Firmware Initialized  "));
    Serial.println(F(" SPI BNO08x IMU Interface + 400Hz STM32 BLDC UART Link "));
    Serial.println(F("========================================================\n"));

    // 2. Initialize 400Hz STM32 UART Communication Link (GPIO 17=TX, GPIO 18=RX)
    stm32Link.begin(STM32_UART_BAUD, STM32_UART_TX_PIN, STM32_UART_RX_PIN);

    // 3. Initialize GY-BNO08X IMU over Hardware SPI
    if (imu.begin(DEFAULT_IMU_SPI_CS, DEFAULT_IMU_SPI_MISO, DEFAULT_IMU_SPI_MOSI, DEFAULT_IMU_SPI_SCK, DEFAULT_IMU_SPI_INT, DEFAULT_IMU_SPI_RST)) {
        imu.printMenu();
    } else {
        Serial.println(F("[WARNING] BNO08x IMU hardware scan failed. Will auto-retry in loop...\n"));
    }

    lastLoopTimeMicros = micros();
}

void loop() {
    unsigned long nowMicros = micros();
    unsigned long nowMs = millis();

    // 1. Handle incoming user commands over USB Serial (for mode switching and rate config)
    if (Serial.available()) {
        char cmd = Serial.read();
        imu.handleSerialCommand(cmd);
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

    // 5. Periodic IMU & Telemetry Report to USB Serial Monitor (every 200ms = 5Hz output stream)
    if (nowMs - lastTelemetryReportMs >= 200) {
        lastTelemetryReportMs = nowMs;

        if (imu.isInitialized()) {
            imu.printCurrentReadings();
        }
    }
}
