#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cstring>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

#define SHTP_HEADER_LEN 4
#define CHANNEL_COMMAND 0
#define CHANNEL_EXE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3

#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_SET_FEATURE 0xFD
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB

#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08

class BNO08xLinuxSPI {
public:
    BNO08xLinuxSPI(int cs_pin = 16, int int_pin = 26, int rst_pin = 21, const char* spidev_path = "/dev/spidev0.0")
        : cs_pin_(cs_pin), int_pin_(int_pin), rst_pin_(rst_pin), spidev_path_(spidev_path) {}

    ~BNO08xLinuxSPI() {
        close_hardware();
    }

    bool begin() {
        std::cout << "========================================================\n";
        std::cout << "  BNO08X NATIVE C++ SPIDEV DRIVER (RASPBERRY PI 5)\n";
        std::cout << "========================================================\n";

        // 1. Open SPI device
        spi_fd_ = open(spidev_path_, O_RDWR);
        if (spi_fd_ < 0) {
            std::cerr << "Failed to open SPI device: " << spidev_path_ << "\n";
            return false;
        }

        uint8_t mode = SPI_MODE_3;
        uint8_t bits = 8;
        uint32_t speed = 3000000;

        if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
            ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
            ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
            std::cerr << "Failed to configure SPI parameters\n";
            return false;
        }

        // 2. Open GPIO lines on chip 4 (RP1)
        chip_ = gpiod_chip_open("/dev/gpiochip4");
        if (!chip_) {
            std::cerr << "Failed to open /dev/gpiochip4\n";
            return false;
        }

        cs_line_ = gpiod_chip_get_line(chip_, cs_pin_);
        int_line_ = gpiod_chip_get_line(chip_, int_pin_);
        rst_line_ = gpiod_chip_get_line(chip_, rst_pin_);

        if (!cs_line_ || !int_line_ || !rst_line_) {
            std::cerr << "Failed to get GPIO lines\n";
            return false;
        }

        gpiod_line_request_output(cs_line_, "bno08x_cs", 1);
        gpiod_line_request_output(rst_line_, "bno08x_rst", 1);
        gpiod_line_request_input_flags(int_line_, "bno08x_int", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);

        // 3. Hardware Reset Pulse
        std::cout << "[Step 1] Hardware Resetting BNO08X...\n";
        gpiod_line_set_value(cs_line_, 1);
        gpiod_line_set_value(rst_line_, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        gpiod_line_set_value(rst_line_, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Wait for first assertion of INT
        if (!wait_for_int(2000)) {
            std::cerr << "Timeout waiting for INT assertion after reset!\n";
            return false;
        }
        std::cout << " -> INT asserted by BNO08X!\n";

        // 4. Drain startup advertisement & initialization packets
        std::cout << "[Step 2] Draining SHTP Bootloader Packets...\n";
        for (int i = 0; i < 5; ++i) {
            if (wait_for_int(100)) {
                std::vector<uint8_t> payload;
                uint8_t chan = 0, seq = 0;
                uint16_t plen = 0;
                if (receive_packet(chan, seq, plen, payload)) {
                    std::cout << "   -> Boot Packet #" << (i+1) << ": Chan=" << (int)chan 
                              << ", Len=" << plen << ", Seq=" << (int)seq << "\n";
                }
            } else {
                break;
            }
        }

        // 5. Send Product ID Request
        std::cout << "[Step 3] Requesting Product ID (Report 0xF9)...\n";
        uint8_t prod_id_req[2] = {SHTP_REPORT_PRODUCT_ID_REQUEST, 0x00};
        send_packet(CHANNEL_CONTROL, prod_id_req, 2);

        // 6. Wait for Product ID Response
        std::cout << "[Step 4] Waiting for Product ID Response...\n";
        for (int i = 0; i < 5; ++i) {
            if (wait_for_int(200)) {
                std::vector<uint8_t> payload;
                uint8_t chan = 0, seq = 0;
                uint16_t plen = 0;
                if (receive_packet(chan, seq, plen, payload)) {
                    if (!payload.empty() && payload[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
                        uint32_t part_num = ((uint32_t)payload[7] << 24) | ((uint32_t)payload[6] << 16) | 
                                            ((uint32_t)payload[5] << 8) | payload[4];
                        std::cout << "========================================================\n";
                        std::cout << "  BNO08X VERIFIED! Part Number: " << part_num << "\n";
                        std::cout << "  Firmware Version: v" << (int)payload[2] << "." << (int)payload[3] << "\n";
                        std::cout << "========================================================\n";
                        break;
                    }
                }
            }
        }

        // 7. Enable Game Rotation Vector (0x08) & Accelerometer (0x01) @ 100Hz (10,000 us)
        std::cout << "[Step 5] Enabling 100Hz Game Rotation Vector & Accelerometer Reports...\n";
        enable_feature(SENSOR_REPORTID_GAME_ROTATION_VECTOR, 10000);
        enable_feature(SENSOR_REPORTID_ACCELEROMETER, 10000);

        return true;
    }

    bool update() {
        if (!is_int_asserted()) return false;

        std::vector<uint8_t> payload;
        uint8_t chan = 0, seq = 0;
        uint16_t plen = 0;

        if (!receive_packet(chan, seq, plen, payload)) return false;

        if (chan == CHANNEL_REPORTS && payload.size() >= 5) {
            size_t idx = 0;
            while (idx + 5 <= payload.size()) {
                uint8_t rep_id = payload[idx + 4];

                if (rep_id == SENSOR_REPORTID_GAME_ROTATION_VECTOR && idx + 19 <= payload.size()) {
                    int16_t qi = (int16_t)(payload[idx + 7] | (payload[idx + 8] << 8));
                    int16_t qj = (int16_t)(payload[idx + 9] | (payload[idx + 10] << 8));
                    int16_t qk = (int16_t)(payload[idx + 11] | (payload[idx + 12] << 8));
                    int16_t qr = (int16_t)(payload[idx + 13] | (payload[idx + 14] << 8));

                    float scale = 1.0f / (1 << 14);
                    quat_x_ = qi * scale;
                    quat_y_ = qj * scale;
                    quat_z_ = qk * scale;
                    quat_w_ = qr * scale;

                    // Convert to Roll, Pitch, Yaw in degrees
                    float sinr_cosp = 2.0f * (quat_w_ * quat_x_ + quat_y_ * quat_z_);
                    float cosr_cosp = 1.0f - 2.0f * (quat_x_ * quat_x_ + quat_y_ * quat_y_);
                    roll_deg_ = std::atan2(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

                    float sinp = 2.0f * (quat_w_ * quat_y_ - quat_z_ * quat_x_);
                    pitch_deg_ = std::asin(std::max(-1.0f, std::min(1.0f, sinp))) * (180.0f / M_PI);

                    float siny_cosp = 2.0f * (quat_w_ * quat_z_ + quat_x_ * quat_y_);
                    float cosy_cosp = 1.0f - 2.0f * (quat_y_ * quat_y_ + quat_z_ * quat_z_);
                    yaw_deg_ = std::atan2(siny_cosp, cosy_cosp) * (180.0f / M_PI);

                    idx += 19;
                } else if (rep_id == SENSOR_REPORTID_ACCELEROMETER && idx + 15 <= payload.size()) {
                    int16_t ax = (int16_t)(payload[idx + 7] | (payload[idx + 8] << 8));
                    int16_t ay = (int16_t)(payload[idx + 9] | (payload[idx + 10] << 8));
                    int16_t az = (int16_t)(payload[idx + 11] | (payload[idx + 12] << 8));

                    float scale = 1.0f / (1 << 8);
                    accel_x_ = ax * scale;
                    accel_y_ = ay * scale;
                    accel_z_ = az * scale;

                    idx += 15;
                } else {
                    idx += 1;
                }
            }
            return true;
        }
        return false;
    }

    void enable_feature(uint8_t feature_id, uint32_t interval_us) {
        uint8_t cmd[17] = {0};
        cmd[0] = SHTP_REPORT_SET_FEATURE;
        cmd[1] = feature_id;
        cmd[5] = interval_us & 0xFF;
        cmd[6] = (interval_us >> 8) & 0xFF;
        cmd[7] = (interval_us >> 16) & 0xFF;
        cmd[8] = (interval_us >> 24) & 0xFF;

        send_packet(CHANNEL_CONTROL, cmd, 17);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Telemetry getters
    float roll() const { return roll_deg_; }
    float pitch() const { return pitch_deg_; }
    float yaw() const { return yaw_deg_; }
    float qw() const { return quat_w_; }
    float qx() const { return quat_x_; }
    float qy() const { return quat_y_; }
    float qz() const { return quat_z_; }
    float ax() const { return accel_x_; }
    float ay() const { return accel_y_; }
    float az() const { return accel_z_; }

private:
    bool is_int_asserted() {
        return gpiod_line_get_value(int_line_) == 0; // Active LOW
    }

    bool wait_for_int(int timeout_ms) {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout_ms) {
            if (is_int_asserted()) return true;
            usleep(100);
        }
        return false;
    }

    bool receive_packet(uint8_t& chan, uint8_t& seq, uint16_t& plen, std::vector<uint8_t>& payload) {
        if (!is_int_asserted()) return false;

        gpiod_line_set_value(cs_line_, 0); // Assert CS LOW
        usleep(10);

        uint8_t hdr_tx[4] = {0};
        uint8_t hdr_rx[4] = {0};

        struct spi_ioc_transfer tr_hdr = {0};
        tr_hdr.tx_buf = (unsigned long)hdr_tx;
        tr_hdr.rx_buf = (unsigned long)hdr_rx;
        tr_hdr.len = 4;
        tr_hdr.speed_hz = 3000000;
        tr_hdr.bits_per_word = 8;

        if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr_hdr) < 0) {
            gpiod_line_set_value(cs_line_, 1);
            return false;
        }

        plen = (hdr_rx[0] | (hdr_rx[1] << 8)) & ~0x8000;
        chan = hdr_rx[2];
        seq = hdr_rx[3];

        payload.clear();
        if (plen > 4 && plen < 1024) {
            payload.resize(plen - 4);
            std::vector<uint8_t> dummy_tx(plen - 4, 0);

            struct spi_ioc_transfer tr_data = {0};
            tr_data.tx_buf = (unsigned long)dummy_tx.data();
            tr_data.rx_buf = (unsigned long)payload.data();
            tr_data.len = plen - 4;
            tr_data.speed_hz = 3000000;
            tr_data.bits_per_word = 8;

            ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr_data);
        }

        gpiod_line_set_value(cs_line_, 1); // Deassert CS HIGH
        usleep(10);
        return true;
    }

    void send_packet(uint8_t channel, const uint8_t* data, size_t data_len) {
        uint16_t packet_len = data_len + 4;
        uint8_t seq = seq_tx_[channel]++;

        std::vector<uint8_t> tx(packet_len);
        std::vector<uint8_t> rx(packet_len, 0);
        tx[0] = packet_len & 0xFF;
        tx[1] = (packet_len >> 8) & 0xFF;
        tx[2] = channel;
        tx[3] = seq;
        std::memcpy(tx.data() + 4, data, data_len);

        gpiod_line_set_value(cs_line_, 0); // CS LOW
        usleep(100);

        struct spi_ioc_transfer tr = {0};
        tr.tx_buf = (unsigned long)tx.data();
        tr.rx_buf = (unsigned long)rx.data();
        tr.len = packet_len;
        tr.speed_hz = 1000000;
        tr.bits_per_word = 8;

        int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 0) {
            std::cerr << "ioctl SPI_IOC_MESSAGE error\n";
        }

        gpiod_line_set_value(cs_line_, 1); // CS HIGH
        usleep(1000);
    }

    void close_hardware() {
        if (cs_line_) gpiod_line_release(cs_line_);
        if (int_line_) gpiod_line_release(int_line_);
        if (rst_line_) gpiod_line_release(rst_line_);
        if (chip_) gpiod_chip_close(chip_);
        if (spi_fd_ >= 0) close(spi_fd_);
    }

    int cs_pin_, int_pin_, rst_pin_;
    const char* spidev_path_;
    int spi_fd_ = -1;
    struct gpiod_chip* chip_ = nullptr;
    struct gpiod_line* cs_line_ = nullptr;
    struct gpiod_line* int_line_ = nullptr;
    struct gpiod_line* rst_line_ = nullptr;

    uint8_t seq_tx_[6] = {0};
    float roll_deg_ = 0.0f, pitch_deg_ = 0.0f, yaw_deg_ = 0.0f;
    float quat_w_ = 1.0f, quat_x_ = 0.0f, quat_y_ = 0.0f, quat_z_ = 0.0f;
    float accel_x_ = 0.0f, accel_y_ = 0.0f, accel_z_ = 0.0f;
};

int main() {
    BNO08xLinuxSPI imu(16, 26, 21, "/dev/spidev0.0");

    if (!imu.begin()) {
        std::cerr << "Failed to initialize BNO08X IMU!\n";
        return 1;
    }

    std::cout << "\n[Step 6] Streaming 100Hz IMU Telemetry to Terminal (5 seconds):\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    auto start_time = std::chrono::steady_clock::now();
    int count = 0;

    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5) {
        if (imu.update()) {
            count++;
            if (count % 5 == 0) {
                printf("[%04d] ROLL:%+6.2f° | PITCH:%+6.2f° | YAW:%+6.2f° | Quat:(%+.3f,%+.3f,%+.3f,%+.3f) | Accel:(%+5.2f,%+5.2f,%+5.2f)\n",
                       count, imu.roll(), imu.pitch(), imu.yaw(),
                       imu.qw(), imu.qx(), imu.qy(), imu.qz(),
                       imu.ax(), imu.ay(), imu.az());
            }
        }
        usleep(100);
    }

    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << ">> BNO08X NATIVE DRIVER TEST COMPLETE! <<\n";
    return 0;
}
