#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>

int main() {
    int spi_fd = open("/dev/spidev0.0", O_RDWR);
    uint8_t mode = SPI_MODE_3;
    uint8_t bits = 8;
    uint32_t speed = 1000000;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    struct gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip4");
    struct gpiod_line* cs = gpiod_chip_get_line(chip, 16);
    struct gpiod_line* int_line = gpiod_chip_get_line(chip, 26);
    struct gpiod_line* rst = gpiod_chip_get_line(chip, 21);

    gpiod_line_request_output(cs, "cs", 1);
    gpiod_line_request_output(rst, "rst", 1);
    gpiod_line_request_input_flags(int_line, "int", GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);

    // Reset
    gpiod_line_set_value(rst, 0);
    usleep(20000);
    gpiod_line_set_value(rst, 1);
    usleep(50000);

    auto is_int = [&]() { return gpiod_line_get_value(int_line) == 0; };

    // Drain
    for (int i = 0; i < 5; ++i) {
        int timeout = 1000;
        while (!is_int() && timeout-- > 0) usleep(1000);
        if (is_int()) {
            gpiod_line_set_value(cs, 0);
            usleep(50);
            uint8_t rx_buf[512] = {0};
            uint8_t tx_buf[512] = {0};
            
            struct spi_ioc_transfer tr = {0};
            tr.tx_buf = (unsigned long)tx_buf;
            tr.rx_buf = (unsigned long)rx_buf;
            tr.len = 4;
            tr.speed_hz = 1000000;
            ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

            uint16_t plen = (rx_buf[0] | (rx_buf[1] << 8)) & ~0x8000;
            if (plen > 4 && plen < 512) {
                tr.len = plen - 4;
                tr.rx_buf = (unsigned long)(rx_buf + 4);
                ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
            }
            gpiod_line_set_value(cs, 1);
            usleep(100);
            std::cout << "Drain #" << i+1 << ": Chan=" << (int)rx_buf[2] << ", Len=" << plen 
                      << ", Rep=" << std::hex << (int)rx_buf[4] << std::dec << "\n";
        }
    }

    // Now test sending Set Feature Command for Rotation Vector (0x05) @ 50Hz (20000 us)
    // SHTP Header: Len 21 (0x15, 0x00), Chan 2, Seq 0
    // Payload: 0xFD, 0x05, 0x00, 0x00, 0x00, 0x20, 0x4E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    uint8_t cmd[21] = {
        0x15, 0x00, 0x02, 0x00,
        0xFD, 0x05, 0x00, 0x00, 0x00,
        0x20, 0x4E, 0x00, 0x00, // 20000 us (50Hz)
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    };
    uint8_t cmd_rx[21] = {0};

    std::cout << "Sending Set Feature Command (Rotation Vector @ 50Hz)...\n";
    gpiod_line_set_value(cs, 0);
    usleep(100);

    struct spi_ioc_transfer tr_cmd = {0};
    tr_cmd.tx_buf = (unsigned long)cmd;
    tr_cmd.rx_buf = (unsigned long)cmd_rx;
    tr_cmd.len = 21;
    tr_cmd.speed_hz = 1000000;
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr_cmd);

    gpiod_line_set_value(cs, 1);
    usleep(2000);

    std::cout << "Listening for incoming sensor reports on Channel 3 for 3 seconds...\n";
    auto start = std::chrono::steady_clock::now();
    int reports_received = 0;

    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < 3) {
        if (is_int()) {
            gpiod_line_set_value(cs, 0);
            usleep(20);
            uint8_t buf[256] = {0};
            struct spi_ioc_transfer tr = {0};
            tr.rx_buf = (unsigned long)buf;
            tr.len = 4;
            tr.speed_hz = 1000000;
            ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

            uint16_t plen = (buf[0] | (buf[1] << 8)) & ~0x8000;
            if (plen > 4 && plen < 256) {
                tr.len = plen - 4;
                tr.rx_buf = (unsigned long)(buf + 4);
                ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
            }
            gpiod_line_set_value(cs, 1);
            usleep(50);

            reports_received++;
            std::cout << "Report #" << reports_received << ": Chan=" << (int)buf[2] << ", Len=" << plen 
                      << ", RepID=0x" << std::hex << (int)buf[4] << std::dec << "\n";
        }
        usleep(500);
    }

    std::cout << "Total reports received: " << reports_received << "\n";

    gpiod_line_release(cs);
    gpiod_line_release(int_line);
    gpiod_line_release(rst);
    gpiod_chip_close(chip);
    close(spi_fd);
    return 0;
}
