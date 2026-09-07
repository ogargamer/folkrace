import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def build_spi_write_verify():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect("10.43.8.42", port=22, username="admin", password="l", timeout=10)

    print("======================================================================")
    print("  SPI WRITE-VERIFY TEST: Write pattern to sensor RAM, read it back")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e

cat << 'EOF' > /tmp/spi_write_verify.c
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPEED 3000000
#define SPIDEV "/dev/spidev0.0"

int fd;

void spi_write(uint16_t reg, uint8_t *data, int len) {
    uint8_t *tx = malloc(len + 2);
    tx[0] = ((reg >> 8) & 0x7F) | 0x80;
    tx[1] = reg & 0xFF;
    memcpy(tx + 2, data, len);
    struct spi_ioc_transfer tr = {.tx_buf=(unsigned long)tx, .len=len+2, .speed_hz=SPEED};
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) fprintf(stderr, "WRITE ERROR: reg=0x%04X ret=%d\n", reg, ret);
    free(tx);
}

void spi_read(uint16_t reg, uint8_t *data, int len) {
    uint8_t *tx = calloc(len + 3, 1);
    uint8_t *rx = calloc(len + 3, 1);
    tx[0] = (reg >> 8) & 0x7F;
    tx[1] = reg & 0xFF;
    struct spi_ioc_transfer tr = {.tx_buf=(unsigned long)tx, .rx_buf=(unsigned long)rx, .len=len+3, .speed_hz=SPEED};
    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) fprintf(stderr, "READ ERROR: reg=0x%04X ret=%d\n", reg, ret);
    // try rx[3] as data offset
    for (int i = 0; i < len; i++) data[i] = rx[3 + i];
    free(tx);
    free(rx);
}

uint8_t spi_read_byte(uint16_t reg) {
    uint8_t v;
    spi_read(reg, &v, 1);
    return v;
}

void spi_write_byte(uint16_t reg, uint8_t val) {
    spi_write(reg, &val, 1);
}

int main() {
    fd = open(SPIDEV, O_RDWR);
    uint8_t mode = SPI_MODE_0;
    uint32_t speed = SPEED;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    printf("=== SPI Write-Verify Test for VL53L8CX ===\n\n");

    // Step 1: Verify reads - chip model ID at reg 0x0000
    printf("--- Step 1: Chip ID Read (reg 0x0000) ---\n");
    // read 8 bytes and show all offsets
    {
        uint8_t tx[10] = {0};
        uint8_t rx[10] = {0};
        struct spi_ioc_transfer tr = {.tx_buf=(unsigned long)tx,.rx_buf=(unsigned long)rx,.len=10,.speed_hz=SPEED};
        ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        printf("rx bytes: ");
        for (int i=0; i<10; i++) printf("[%d]=0x%02X ", i, rx[i]);
        printf("\n");
        printf("If data@rx[2]: ID=0x%02X%02X | If data@rx[3]: ID=0x%02X%02X\n",
               rx[2], rx[3], rx[3], rx[4]);
    }

    // Step 2: Reset sensor via GPIO 17
    printf("\n--- Step 2: Hardware Reset (GPIO 17) ---\n");
    system("gpioset 4 17=0 2>/dev/null || true");
    usleep(100000);
    system("gpioset 4 17=1 2>/dev/null || true");
    usleep(500000); // 500ms after reset

    // Step 3: Poll reg=0x0006 to check bootloader ready, show raw bytes
    printf("\n--- Step 3: Poll reg=0x0006 (bootloader ready) ---\n");
    for (int i = 0; i < 10; i++) {
        uint8_t tx[8] = {0x00, 0x06, 0, 0, 0, 0, 0, 0};
        uint8_t rx[8] = {0};
        struct spi_ioc_transfer tr = {.tx_buf=(unsigned long)tx,.rx_buf=(unsigned long)rx,.len=8,.speed_hz=SPEED};
        ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        printf("  poll[%d] rx: [0]=0x%02X [1]=0x%02X [2]=0x%02X [3]=0x%02X [4]=0x%02X\n",
               i, rx[0], rx[1], rx[2], rx[3], rx[4]);
        if (rx[3] == 0x03 || rx[2] == 0x03) { printf("  BOOTLOADER READY! (0x03)\n"); break; }
        usleep(10000);
    }

    // Step 4: Write known pattern to a RAM area and read back
    printf("\n--- Step 4: Write-Verify Test ---\n");

    // Set page to 0x00 first
    spi_write_byte(0x7FFF, 0x00);
    usleep(1000);

    // Read 0x7FFF to verify page selection
    uint8_t page_val = spi_read_byte(0x7FFF);
    printf("Read 0x7FFF (page select) = 0x%02X (expect 0x00)\n", page_val);

    // Write page to 0x02 (firmware RAM page)
    spi_write_byte(0x7FFF, 0x02);
    usleep(1000);

    // Read 0x7FFF to verify
    page_val = spi_read_byte(0x7FFF);
    printf("Read 0x7FFF after write 0x02 = 0x%02X (expect 0x02 if writes work)\n", page_val);

    // Write pattern to RAM: 16 bytes starting at 0x2400
    uint8_t pattern[16] = {0xAA, 0xBB, 0xCC, 0xDD, 0x11, 0x22, 0x33, 0x44,
                           0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0xEE, 0xFF};
    spi_write(0x2400, pattern, 16);
    usleep(1000);

    // Read back from 0x2400
    uint8_t readback[16] = {0};
    spi_read(0x2400, readback, 16);
    printf("\nWritten  to 0x2400: ");
    for (int i = 0; i < 16; i++) printf("%02X ", pattern[i]);
    printf("\nRead back from 0x2400: ");
    for (int i = 0; i < 16; i++) printf("%02X ", readback[i]);
    printf("\n");

    int match = memcmp(pattern, readback, 16) == 0;
    printf("\nWrite-Verify: %s\n", match ? "PASS - SPI writes ARE working!" : "FAIL - SPI writes NOT reaching sensor");

    // Also try rx[2] offset for readback
    {
        uint8_t tx[18] = {0};
        uint8_t rx[18] = {0};
        tx[0] = 0x24; tx[1] = 0x00;  // read 0x2400, no write bit
        struct spi_ioc_transfer tr = {.tx_buf=(unsigned long)tx,.rx_buf=(unsigned long)rx,.len=18,.speed_hz=SPEED};
        ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
        printf("\nRaw read 0x2400 (18 bytes): ");
        for (int i=0; i<18; i++) printf("[%d]=%02X ", i, rx[i]);
        printf("\nData@rx[2]: ");
        for (int i=0; i<16; i++) printf("%02X ", rx[2+i]);
        printf("\nData@rx[3]: ");
        for (int i=0; i<16; i++) printf("%02X ", rx[3+i]);
        printf("\n");
    }

    return 0;
}
EOF

gcc /tmp/spi_write_verify.c -o /tmp/spi_write_verify
/tmp/spi_write_verify
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_spi_write_verify.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_spi_write_verify.sh && /tmp/run_spi_write_verify.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    build_spi_write_verify()
