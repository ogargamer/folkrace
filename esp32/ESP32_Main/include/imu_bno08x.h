#ifndef IMU_BNO08X_H
#define IMU_BNO08X_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

// Default SPI Pinout (CS=GPIO 10, INT=GPIO 9, MOSI=11, SCK=12, MISO=13, RST=14)
constexpr int DEFAULT_IMU_SPI_CS   = 10; // GPIO 10 (Pin 15)
constexpr int DEFAULT_IMU_SPI_MOSI = 11; // GPIO 11 (Pin 16)
constexpr int DEFAULT_IMU_SPI_SCK  = 12; // GPIO 12 (Pin 17)
constexpr int DEFAULT_IMU_SPI_MISO = 13; // GPIO 13 (Pin 18)
constexpr int DEFAULT_IMU_SPI_INT  = 9;  // GPIO 9  (Pin 14 / ADC1)
constexpr int DEFAULT_IMU_SPI_RST  = 14; // GPIO 14 (Pin 19 / ADC2)

enum ReportMode {
    REPORT_MODE_FUSION_GAME = 1, // 6-DOF Game Rotation Vector (Quaternion + YPR)
    REPORT_MODE_FUSION_9AXIS,    // 9-DOF Rotation Vector (Mag + Accel + Gyro)
    REPORT_MODE_FILTERED_IMU,   // Filtered Accel, Gyro, Linear Accel, Gravity
    REPORT_MODE_RAW_MEMS,        // Raw MEMS Accelerometer, Gyroscope, Magnetometer
    REPORT_MODE_ALL              // All major reports active
};

struct IMUData {
    // Sensor Fusion / Orientation
    float quat_w = 1.0f;
    float quat_x = 0.0f;
    float quat_y = 0.0f;
    float quat_z = 0.0f;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    uint8_t quat_accuracy = 0;

    // Filtered Kinematics (m/s^2, rad/s, uT)
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
    float lin_accel_x = 0.0f, lin_accel_y = 0.0f, lin_accel_z = 0.0f;
    float gravity_x = 0.0f, gravity_y = 0.0f, gravity_z = 0.0f;

    // Raw MEMS Counts
    int16_t raw_accel_x = 0, raw_accel_y = 0, raw_accel_z = 0;
    int16_t raw_gyro_x = 0, raw_gyro_y = 0, raw_gyro_z = 0;
    int16_t raw_mag_x = 0, raw_mag_y = 0, raw_mag_z = 0;

    uint32_t last_update_ms = 0;
    bool updated = false;
};

class BNO08xIMU {
public:
    BNO08xIMU();
    ~BNO08xIMU();

    bool begin(int csPin = DEFAULT_IMU_SPI_CS,
               int misoPin = DEFAULT_IMU_SPI_MISO,
               int mosiPin = DEFAULT_IMU_SPI_MOSI,
               int sckPin = DEFAULT_IMU_SPI_SCK,
               int intPin = DEFAULT_IMU_SPI_INT,
               int rstPin = DEFAULT_IMU_SPI_RST);

    bool update();
    void setReportMode(ReportMode mode, uint32_t intervalMs = 10);
    void disableAllReports();

    const IMUData& getData() const { return data_; }
    ReportMode getCurrentReportMode() const { return current_mode_; }
    bool isInitialized() const { return initialized_; }

    void printMenu(Stream &out = Serial);
    void printCurrentReadings(Stream &out = Serial);
    void handleSerialCommand(char cmd, Stream &out = Serial);

    void tareAllAxes(Stream &out = Serial);
    void tareYawOnly(Stream &out = Serial);
    void saveTare(Stream &out = Serial);
    void clearTare(Stream &out = Serial);

private:
    void convertQuaternionToEuler(float w, float x, float y, float z, float &roll, float &pitch, float &yaw);

    BNO080 bno08x_;
    IMUData data_;
    ReportMode current_mode_ = REPORT_MODE_FUSION_GAME;
    uint32_t report_interval_ms_ = 10;
    bool initialized_ = false;
};

#endif // IMU_BNO08X_H
