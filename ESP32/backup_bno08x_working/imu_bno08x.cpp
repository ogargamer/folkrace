#include "imu_bno08x.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

BNO08xIMU::BNO08xIMU() {
}

BNO08xIMU::~BNO08xIMU() {
}

bool BNO08xIMU::begin(int csPin, int misoPin, int mosiPin, int sckPin, int intPin, int rstPin) {
    Serial.println(F("\n========================================================"));
    Serial.println(F(" [BNO08x IMU] SparkFun Hardware SPI Driver Initialization"));
    Serial.println(F("========================================================"));
    Serial.printf(" [Config] CS=%d, INT=%d, RST=%d, SCK=%d, MISO=%d, MOSI=%d\n",
                  csPin, intPin, rstPin, sckPin, misoPin, mosiPin);

    // Initialize ESP32-S3 Hardware SPI bus
    SPI.begin(sckPin, misoPin, mosiPin, csPin);

    // Attempt 1: Standard Pin Assignment (CS=10, INT=9)
    Serial.printf(" [SPI Attempt 1/2] Connecting with CS=%d, INT=%d, RST=%d...\n", csPin, intPin, rstPin);
    if (bno08x_.beginSPI(csPin, 255, intPin, rstPin, 3000000, SPI)) {
        Serial.println(F(" [SUCCESS] SparkFun BNO08x Connected over SPI (CS=10, INT=9)!"));
        initialized_ = true;
        delay(100);
        setReportMode(REPORT_MODE_FUSION_GAME, 10);
        return true;
    }

    // Attempt 2: Swapped Pin Assignment (CS=9, INT=10)
    Serial.printf(" [SPI Attempt 2/2] Swapping pins CS=%d, INT=%d, RST=%d...\n", intPin, csPin, rstPin);
    if (bno08x_.beginSPI(intPin, 255, csPin, rstPin, 3000000, SPI)) {
        Serial.println(F(" [SUCCESS] SparkFun BNO08x Connected over SPI (Swapped Pins CS=9, INT=10)!"));
        initialized_ = true;
        delay(100);
        setReportMode(REPORT_MODE_FUSION_GAME, 10);
        return true;
    }

    Serial.println(F(" [ERROR] Unable to establish SPI link with BNO08x IMU over CS=10/INT=9 or CS=9/INT=10."));
    initialized_ = false;
    return false;
}

void BNO08xIMU::disableAllReports() {
    if (!initialized_) return;
}

void BNO08xIMU::setReportMode(ReportMode mode, uint32_t intervalMs) {
    if (!initialized_) return;

    current_mode_ = mode;
    report_interval_ms_ = intervalMs;

    Serial.printf("[BNO08x IMU] Enabling Report Mode %d (Interval: %u ms / %u Hz)...\n",
                  (int)mode, intervalMs, 1000 / intervalMs);

    switch (mode) {
        case REPORT_MODE_FUSION_GAME:
            bno08x_.enableGameRotationVector(intervalMs);
            break;

        case REPORT_MODE_FUSION_9AXIS:
            bno08x_.enableRotationVector(intervalMs);
            break;

        case REPORT_MODE_FILTERED_IMU:
            bno08x_.enableAccelerometer(intervalMs);
            bno08x_.enableGyro(intervalMs);
            bno08x_.enableLinearAccelerometer(intervalMs);
            bno08x_.enableGravity(intervalMs);
            break;

        case REPORT_MODE_RAW_MEMS:
            bno08x_.enableUncalibratedGyro(intervalMs);
            bno08x_.enableMagnetometer(intervalMs);
            break;

        case REPORT_MODE_ALL:
            bno08x_.enableGameRotationVector(intervalMs);
            bno08x_.enableRotationVector(intervalMs);
            bno08x_.enableAccelerometer(intervalMs);
            bno08x_.enableGyro(intervalMs);
            bno08x_.enableMagnetometer(intervalMs);
            bno08x_.enableLinearAccelerometer(intervalMs);
            bno08x_.enableGravity(intervalMs);
            break;
    }
}

bool BNO08xIMU::update() {
    if (!initialized_) return false;

    // Check if BNO08x experienced a reset event
    if (bno08x_.hasReset()) {
        Serial.println(F("[BNO08x IMU] WARNING: BNO08x Sensor Was Reset! Re-enabling reports..."));
        delay(100);
        setReportMode(current_mode_, report_interval_ms_);
    }

    bool got_new_data = false;

    // Drain all pending SHTP packets available from SparkFun BNO08x driver
    while (bno08x_.dataAvailable()) {
        got_new_data = true;
        data_.last_update_ms = millis();
        data_.updated = true;

        data_.quat_w = bno08x_.getQuatReal();
        data_.quat_x = bno08x_.getQuatI();
        data_.quat_y = bno08x_.getQuatJ();
        data_.quat_z = bno08x_.getQuatK();
        data_.roll_deg = bno08x_.getRoll() * (180.0f / M_PI);
        data_.pitch_deg = bno08x_.getPitch() * (180.0f / M_PI);
        data_.yaw_deg = bno08x_.getYaw() * (180.0f / M_PI);
        data_.quat_accuracy = bno08x_.getQuatRadianAccuracy();

        data_.accel_x = bno08x_.getAccelX();
        data_.accel_y = bno08x_.getAccelY();
        data_.accel_z = bno08x_.getAccelZ();

        data_.gyro_x = bno08x_.getGyroX();
        data_.gyro_y = bno08x_.getGyroY();
        data_.gyro_z = bno08x_.getGyroZ();

        data_.mag_x = bno08x_.getMagX();
        data_.mag_y = bno08x_.getMagY();
        data_.mag_z = bno08x_.getMagZ();

        data_.lin_accel_x = bno08x_.getLinAccelX();
        data_.lin_accel_y = bno08x_.getLinAccelY();
        data_.lin_accel_z = bno08x_.getLinAccelZ();

        data_.gravity_x = bno08x_.getGravityX();
        data_.gravity_y = bno08x_.getGravityY();
        data_.gravity_z = bno08x_.getGravityZ();
    }

    return got_new_data;
}

void BNO08xIMU::convertQuaternionToEuler(float w, float x, float y, float z, float &roll, float &pitch, float &yaw) {
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    float sinp = 2.0f * (w * y - z * x);
    if (fabs(sinp) >= 1.0f)
        pitch = copysign(90.0f, sinp);
    else
        pitch = asin(sinp) * (180.0f / M_PI);

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp) * (180.0f / M_PI);
}

void BNO08xIMU::printMenu(Stream &out) {
    out.println(F("\n=============================================="));
    out.println(F("     BNO08x IMU Interactive Report Menu       "));
    out.println(F("=============================================="));
    out.println(F(" [1] Mode 1: 6-DOF Game Rotation Vector (Quaternion + Roll/Pitch/Yaw)"));
    out.println(F(" [2] Mode 2: 9-DOF Rotation Vector (Mag Integrated)"));
    out.println(F(" [3] Mode 3: Filtered IMU (Accel, Gyro, LinAccel, Gravity)"));
    out.println(F(" [4] Mode 4: Raw MEMS Counts (Raw Accel, Gyro, Mag)"));
    out.println(F(" [5] Mode 5: ALL Reports Active (Full Telemetry)"));
    out.println(F(" [+] Increase Rate (100Hz -> 200Hz)"));
    out.println(F(" [-] Decrease Rate (100Hz -> 50Hz)"));
    out.println(F(" [m] Display Menu"));
    out.println(F("==============================================\n"));
}

void BNO08xIMU::printCurrentReadings(Stream &out) {
    if (!initialized_) {
        out.println(F("[IMU] Hardware Not Initialized! Retry in progress..."));
        return;
    }

    switch (current_mode_) {
        case REPORT_MODE_FUSION_GAME:
        case REPORT_MODE_FUSION_9AXIS:
            out.printf("[FUSION %s] Quat (W:%.3f X:%.3f Y:%.3f Z:%.3f) | R: %6.2f°  P: %6.2f°  Y: %6.2f°\n",
                       (current_mode_ == REPORT_MODE_FUSION_GAME) ? "6DOF" : "9DOF",
                       data_.quat_w, data_.quat_x, data_.quat_y, data_.quat_z,
                       data_.roll_deg, data_.pitch_deg, data_.yaw_deg);
            break;

        case REPORT_MODE_FILTERED_IMU:
            out.printf("[FILTERED] Accel(m/s²): [%6.2f %6.2f %6.2f] | Gyro(rad/s): [%6.2f %6.2f %6.2f] | LinAcc: [%6.2f %6.2f %6.2f]\n",
                       data_.accel_x, data_.accel_y, data_.accel_z,
                       data_.gyro_x, data_.gyro_y, data_.gyro_z,
                       data_.lin_accel_x, data_.lin_accel_y, data_.lin_accel_z);
            break;

        case REPORT_MODE_RAW_MEMS:
            out.printf("[RAW MEMS] RawAccel: [%6d %6d %6d] | RawGyro: [%6d %6d %6d] | RawMag: [%6d %6d %6d]\n",
                       data_.raw_accel_x, data_.raw_accel_y, data_.raw_accel_z,
                       data_.raw_gyro_x, data_.raw_gyro_y, data_.raw_gyro_z,
                       data_.raw_mag_x, data_.raw_mag_y, data_.raw_mag_z);
            break;

        case REPORT_MODE_ALL:
            out.printf("[ALL TELEMETRY] R:%6.1f° P:%6.1f° Y:%6.1f° | Accel:[%.1f %.1f %.1f] | Gyro:[%.2f %.2f %.2f]\n",
                       data_.roll_deg, data_.pitch_deg, data_.yaw_deg,
                       data_.accel_x, data_.accel_y, data_.accel_z,
                       data_.gyro_x, data_.gyro_y, data_.gyro_z);
            break;
    }
}

void BNO08xIMU::handleSerialCommand(char cmd, Stream &out) {
    switch (cmd) {
        case '1':
            setReportMode(REPORT_MODE_FUSION_GAME, report_interval_ms_);
            out.println(F("[BNO08x] Switched to Mode 1: 6-DOF Game Rotation Vector."));
            break;
        case '2':
            setReportMode(REPORT_MODE_FUSION_9AXIS, report_interval_ms_);
            out.println(F("[BNO08x] Switched to Mode 2: 9-DOF Rotation Vector."));
            break;
        case '3':
            setReportMode(REPORT_MODE_FILTERED_IMU, report_interval_ms_);
            out.println(F("[BNO08x] Switched to Mode 3: Filtered IMU (Accel/Gyro/LinAccel)."));
            break;
        case '4':
            setReportMode(REPORT_MODE_RAW_MEMS, report_interval_ms_);
            out.println(F("[BNO08x] Switched to Mode 4: Raw MEMS Counts."));
            break;
        case '5':
            setReportMode(REPORT_MODE_ALL, report_interval_ms_);
            out.println(F("[BNO08x] Switched to Mode 5: ALL Reports Active."));
            break;
        case '+':
            report_interval_ms_ = 5; // 200Hz
            setReportMode(current_mode_, report_interval_ms_);
            out.println(F("[BNO08x] Rate updated to 200Hz (5 ms)."));
            break;
        case '-':
            report_interval_ms_ = 20; // 50Hz
            setReportMode(current_mode_, report_interval_ms_);
            out.println(F("[BNO08x] Rate updated to 50Hz (20 ms)."));
            break;
        case 'm':
        case 'M':
            printMenu(out);
            break;
        default:
            break;
    }
}
