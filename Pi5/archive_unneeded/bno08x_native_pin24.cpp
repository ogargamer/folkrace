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

#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_SET_FEATURE 0xFD

#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08

int main() {
    std::cout << "========================================================\n";
    std::cout << "  BNO08X NATIVE SPIDEV (CS=PIN 24 / GPIO 8)\n";
    std::cout << "========================================================\n";

    int spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) {
        std::cerr << "Failed to open /dev/spidev0.0\n";
        return 1;
    }

    uint8_t mode = SPI_MODE_3;
    uint8_t bits = 8;
    uint32_t speed = 3000000;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    struct gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip4");
    struct gpiod_line* int_line = gpiod_chip_get_line(chip, 26); // Pin 37
    struct gpiod_line* rst = gpiod_chip_get_line(chip, 21);      // Pin 40

    gpiod_line_request_output(rst, "rst", 1);
    gpiod_line_request_input_flags(int_line, "int", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);

    auto is_int = [&]() { return gpiod_line_get_value(int_line) == 0; };

    // Reset
    std::cout << "Step 1: Hardware Resetting BNO08X...\n";
    gpiod_line_set_value(rst, 0);
    usleep(20000);
    gpiod_line_set_value(rst, 1);
    usleep(50000);

    // Wait for INT
    int wait_count = 1000;
    while (!is_int() && wait_count-- > 0) usleep(1000);
    if (is_int()) {
        std::cout << " -> INT asserted by BNO08X!\n";
    }

    auto spi_transfer = [&](const uint8_t* tx, uint8_t* rx, size_t len) {
        struct spi_ioc_transfer tr = {0};
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = len;
        tr.speed_hz = 3000000;
        tr.bits_per_word = 8;
        return ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    };

    // Helper to read a complete packet from BNO08X
    auto read_full_packet = [&](uint8_t& chan, uint8_t& seq, std::vector<uint8_t>& payload) -> bool {
        if (!is_int()) return false;

        uint8_t hdr_tx[4] = {0};
        uint8_t hdr_rx[4] = {0};
        spi_transfer(hdr_tx, hdr_rx, 4);

        uint16_t plen = (hdr_rx[0] | (hdr_rx[1] << 8)) & ~0x8000;
        chan = hdr_rx[2];
        seq = hdr_rx[3];

        payload.clear();
        if (plen > 4 && plen < 1024) {
            payload.resize(plen - 4);
            std::vector<uint8_t> dtx(plen - 4, 0);
            spi_transfer(dtx.data(), payload.data(), plen - 4);
        }
        return true;
    };

    // Step 2: Explicitly clock out all startup advertisement and boot phases
    std::cout << "Step 2: Draining SHTP Bootloader Sequence (Advertisements, Reset, Executable)...\n";
    int count = 0;
    for (int iter = 0; iter < 5; ++iter) {
        uint8_t chan = 0, seq = 0;
        std::vector<uint8_t> data;
        
        // Wait up to 100ms for INT or clock a header
        int timeout = 200;
        while (!is_int() && timeout-- > 0) usleep(500);

        uint8_t hdr_tx[4] = {0};
        uint8_t hdr_rx[4] = {0};
        spi_transfer(hdr_tx, hdr_rx, 4);

        uint16_t plen = (hdr_rx[0] | (hdr_rx[1] << 8)) & ~0x8000;
        chan = hdr_rx[2];
        seq = hdr_rx[3];

        if (plen > 4 && plen < 1024) {
            count++;
            data.resize(plen - 4);
            std::vector<uint8_t> dtx(plen - 4, 0);
            spi_transfer(dtx.data(), data.data(), plen - 4);
            std::cout << "   -> Boot Packet #" << count << ": Chan=" << (int)chan << ", Len=" << plen 
                      << ", RepID=0x" << std::hex << (data.empty() ? 0 : (int)data[0]) << std::dec << "\n";
        }
        usleep(5000);
    }

    std::cout << "   -> Bootloader sequence fully consumed (" << count << " packets)!\n";

    // Step 3: Send Product ID Request (Report 0xF9 on Channel 2)
    std::cout << "\nStep 3: Sending Product ID Request (Report 0xF9 on Channel 2)...\n";
    uint8_t prod_req[6] = {0x06, 0x00, 0x02, 0x00, SHTP_REPORT_PRODUCT_ID_REQUEST, 0x00};
    uint8_t prod_rx[6] = {0};
    spi_transfer(prod_req, prod_rx, 6);
    usleep(5000);

    // Step 4: Wait for Product ID Response
    std::cout << "Step 4: Waiting for Product ID Response...\n";
    for (int i = 0; i < 5; ++i) {
        int timeout = 400;
        while (!is_int() && timeout-- > 0) usleep(500);
        if (is_int()) {
            uint8_t chan = 0, seq = 0;
            std::vector<uint8_t> data;
            if (read_full_packet(chan, seq, data)) {
                std::cout << "   -> Response #" << (i+1) << ": Chan=" << (int)chan << ", Len=" << (data.size() + 4)
                          << ", RepID=0x" << std::hex << (data.empty() ? 0 : (int)data[0]) << std::dec << "\n";
                if (!data.empty() && data[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
                    uint32_t part = ((uint32_t)data[7] << 24) | ((uint32_t)data[6] << 16) | ((uint32_t)data[5] << 8) | data[4];
                    std::cout << "========================================================\n";
                    std::cout << ">> SUCCESS: BNO08X VERIFIED! Part #" << part << ", FW v" << (int)data[2] << "." << (int)data[3] << " <<\n";
                    std::cout << "========================================================\n";
                    break;
                }
            }
        }
    }

    // Step 5: Enable Game Rotation Vector (0x08) @ 100Hz (10,000 us = 0x2710)
    std::cout << "Step 5: Enabling 100Hz Game Rotation Vector & Accelerometer...\n";
    uint8_t cmd_rot[21] = {0x15, 0x00, 0x02, 0x01, SHTP_REPORT_SET_FEATURE, SENSOR_REPORTID_GAME_ROTATION_VECTOR, 0, 0, 0, 0x10, 0x27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t cmd_rx[21] = {0};
    spi_transfer(cmd_rot, cmd_rx, 21);
    usleep(5000);

    uint8_t cmd_acc[21] = {0x15, 0x00, 0x02, 0x02, SHTP_REPORT_SET_FEATURE, SENSOR_REPORTID_ACCELEROMETER, 0, 0, 0, 0x10, 0x27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    spi_transfer(cmd_acc, cmd_rx, 21);
    usleep(5000);

    // Step 6: Stream Telemetry
    std::cout << "\nStep 6: Streaming Live 9-DOF IMU Telemetry (5 seconds):\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    auto start_t = std::chrono::steady_clock::now();
    int sample_count = 0;
    float roll = 0, pitch = 0, yaw = 0, qw = 1, qx = 0, qy = 0, qz = 0, ax = 0, ay = 0, az = 0;

    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_t).count() < 5) {
        if (is_int()) {
            uint8_t chan = 0, seq = 0;
            std::vector<uint8_t> drx;
            if (read_full_packet(chan, seq, drx)) {
                if (chan == 3 && drx.size() >= 5) { // Channel 3 = Sensor Reports
                    size_t idx = 0;
                    while (idx + 5 <= drx.size()) {
                        uint8_t rep = drx[idx + 4];
                        if (rep == SENSOR_REPORTID_GAME_ROTATION_VECTOR && idx + 19 <= drx.size()) {
                            int16_t qi = (int16_t)(drx[idx + 7] | (drx[idx + 8] << 8));
                            int16_t qj = (int16_t)(drx[idx + 9] | (drx[idx + 10] << 8));
                            int16_t qk = (int16_t)(drx[idx + 11] | (drx[idx + 12] << 8));
                            int16_t qr = (int16_t)(drx[idx + 13] | (drx[idx + 14] << 8));
                            float s = 1.0f / (1 << 14);
                            qx = qi * s; qy = qj * s; qz = qk * s; qw = qr * s;

                            float sinr_cosp = 2.0f * (qw * qx + qy * qz);
                            float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
                            roll = std::atan2(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

                            float sinp = 2.0f * (qw * qy - qz * qx);
                            pitch = std::asin(std::max(-1.0f, std::min(1.0f, sinp))) * (180.0f / M_PI);

                            float siny_cosp = 2.0f * (qw * qz + qx * qy);
                            float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
                            yaw = std::atan2(siny_cosp, cosy_cosp) * (180.0f / M_PI);

                            sample_count++;
                            printf("[%04d] ROLL:%+6.2f° | PITCH:%+6.2f° | YAW:%+6.2f° | Quat:(%+.3f,%+.3f,%+.3f,%+.3f) | Accel:(%+5.2f,%+5.2f,%+5.2f)\n",
                                   sample_count, roll, pitch, yaw, qw, qx, qy, qz, ax, ay, az);
                            idx += 19;
                        } else if (rep == SENSOR_REPORTID_ACCELEROMETER && idx + 15 <= drx.size()) {
                            int16_t raw_ax = (int16_t)(drx[idx + 7] | (drx[idx + 8] << 8));
                            int16_t raw_ay = (int16_t)(drx[idx + 9] | (drx[idx + 10] << 8));
                            int16_t raw_az = (int16_t)(drx[idx + 11] | (drx[idx + 12] << 8));
                            float s = 1.0f / (1 << 8);
                            ax = raw_ax * s; ay = raw_ay * s; az = raw_az * s;
                            idx += 15;
                        } else {
                            idx += 1;
                        }
                    }
                }
            }
        }
        usleep(500);
    }

    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << ">> TEST COMPLETE: " << sample_count << " SAMPLES PROCESSED! <<\n";

    gpiod_line_release(int_line);
    gpiod_line_release(rst);
    gpiod_chip_close(chip);
    close(spi_fd);
    return 0;
}
