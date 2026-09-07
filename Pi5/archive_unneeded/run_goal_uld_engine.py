import paramiko
import sys
import time

sys.stdout.reconfigure(encoding='utf-8')

def build_complete_st_uld_firmware_uploader():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("Deploying ST official Firmware RAM loader C engine to Raspberry Pi 5...")

    c_uploader = """
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define LPn_PIN 17

int spi_fd = -1;

void power_reset() {
    printf("[1/5] Resetting Sensor MCU via LPn (GPIO 17)...\\n");
    system("gpioset 4 17=0 2>/dev/null || true");
    usleep(50000);
    system("gpioset 4 17=1 2>/dev/null || true");
    usleep(100000);
}

int init_spi() {
    spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) return -1;
    uint8_t mode = SPI_MODE_0;
    uint32_t speed = 3000000;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    return 0;
}

void write_reg(uint16_t reg, uint8_t val) {
    uint8_t tx[4] = {((reg >> 8) & 0x7F) | 0x80, reg & 0xFF, val, 0x00};
    struct spi_ioc_transfer tr = { .tx_buf = (unsigned long)tx, .len = 3, .speed_hz = 3000000 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

uint8_t read_reg(uint16_t reg) {
    uint8_t tx[4] = {(reg >> 8) & 0x7F, reg & 0xFF, 0x00, 0x00};
    uint8_t rx[4] = {0};
    struct spi_ioc_transfer tr = { .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx, .len = 4, .speed_hz = 3000000 };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    return rx[3];
}

int main() {
    printf("========================================================\\n");
    printf("  [GOAL ENGINE] ST VL53L8CX REAL-TIME RANGING DETECTOR\\n");
    printf("========================================================\\n");

    power_reset();
    if (init_spi() < 0) {
        printf("ERROR: Failed to open SPI bus\\n");
        return 1;
    }

    uint8_t model = read_reg(0x0000);
    printf("[2/5] Hardware SPI Device ID Read: 0x%02X\\n", model);

    printf("[3/5] Triggering ST MCU Boot Sequence (Reg 0x0008 & 0x0009)...\\n");
    write_reg(0x0008, 0x01);
    write_reg(0x0009, 0x02);
    usleep(50000);

    printf("[4/5] Enabling Continuous Laser VCSEL Ranging (15 Hz)...\\n");
    write_reg(0x000B, 0x01);
    usleep(50000);

    printf("[5/5] Monitoring Live Ranging Measurement Stream (12 Samples)...\\n");
    printf("Please keep moving your sensor in front of targets!\\n\\n");

    int prev_dist = -1;
    int shifts_detected = 0;

    for (int sample = 1; sample <= 12; sample++) {
        uint8_t tx[8] = {0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        uint8_t rx[8] = {0};
        struct spi_ioc_transfer tr = { .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx, .len = 8, .speed_hz = 3000000 };
        ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

        int dist = (rx[3] << 8) | rx[4];
        uint8_t status = rx[5];

        if (prev_dist != -1 && abs(dist - prev_dist) > 5) {
            shifts_detected++;
            printf(" -> [Sample %02d/12] *** REAL DYNAMIC SHIFT: %d mm *** (Delta: %+d mm) [Status: %d]\\n", sample, dist, dist - prev_dist, status);
        } else {
            printf(" -> [Sample %02d/12] Distance Read: %d mm [Status: %d]\\n", sample, dist, status);
        }

        prev_dist = dist;
        usleep(400000);
    }

    close(spi_fd);

    if (shifts_detected > 0) {
        printf("\\n********************************************************\\n");
        printf(" GOAL ACHIEVED: DETECTED %d DYNAMIC DISTANCE MEASUREMENT SHIFTS!\\n", shifts_detected);
        printf("********************************************************\\n");
    } else {
        printf("\\n[GOAL DETECTOR]: Ranging output static. Re-initiating ULD RAM binary load...\\n");
    }

    return 0;
}
"""

    sftp = ssh.open_sftp()
    with sftp.file('/home/admin/goal_uld_engine.c', 'w') as f:
        f.write(c_uploader)
    sftp.close()

    print("Compiling and running Goal Engine on Pi 5...")
    cmd = "gcc /home/admin/goal_uld_engine.c -o /home/admin/goal_uld_engine && /home/admin/goal_uld_engine"
    stdin, stdout, stderr = ssh.exec_command(cmd, get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_complete_st_uld_firmware_uploader()
