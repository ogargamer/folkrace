import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

def trace_exact_init_step():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ip = "192.168.0.13"
    ssh.connect(ip, port=22, username="admin", password="l", timeout=5)

    print("======================================================================")
    print("  TRACING EXACT LINE IN vl53l8cx_init WHERE STATUS != 0")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > trace_init_exact.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);

    uint8_t status = 0, tmp = 0;

    printf("Step 1: Soft reset sequence...\n");
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0009, 0x04);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x000F, 0x40);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x000A, 0x03);
    status |= VL53L8CX_RdByte(&(Dev.platform), 0x7FFF, &tmp);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x000C, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0101, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0102, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x010A, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x4002, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x4002, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x010A, 0x03);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0103, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x000C, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x000F, 0x43);
    status |= VL53L8CX_WaitMs(&(Dev.platform), 1);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x000F, 0x40);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x000A, 0x01);
    status |= VL53L8CX_WaitMs(&(Dev.platform), 100);
    printf("  Status after reset: %d\n", status);

    printf("Step 2: Polling bootloader ready (0x06 == 1)...\n");
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x00);
    uint8_t boot_ready = 0;
    for (int i = 0; i < 50; i++) {
        VL53L8CX_RdByte(&(Dev.platform), 0x06, &boot_ready);
        if (boot_ready == 1) {
            printf("  Bootloader ready on poll %d!\n", i);
            break;
        }
        VL53L8CX_WaitMs(&(Dev.platform), 5);
    }
    printf("  boot_ready = %d\n", boot_ready);

    printf("Step 3: Enable FW access...\n");
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x000E, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x02);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x06, 0x01);

    uint8_t fw_ready = 0;
    for (int i = 0; i < 50; i++) {
        VL53L8CX_RdByte(&(Dev.platform), 0x21, &fw_ready);
        if (fw_ready == 0x04) {
            printf("  FW access ready (0x04) on poll %d!\n", i);
            break;
        }
        VL53L8CX_WaitMs(&(Dev.platform), 5);
    }
    printf("  fw_ready = 0x%02X\n", fw_ready);

    printf("Step 4: Uploading 84KB Firmware in chunks...\n");
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x00);
    status |= VL53L8CX_RdByte(&(Dev.platform), 0x7fff, &tmp);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0C, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x101, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x102, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x010A, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x4002, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x4002, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x010A, 0x03);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x103, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x400F, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x21A, 0x43);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x21A, 0x03);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x21A, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x21A, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x219, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x21B, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x00);
    status |= VL53L8CX_RdByte(&(Dev.platform), 0x7fff, &tmp);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x01);

    extern const uint8_t VL53L8CX_FIRMWARE[];
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x09);
    status |= VL53L8CX_WrMulti(&(Dev.platform), 0, (uint8_t*)&VL53L8CX_FIRMWARE[0], 0x8000);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x0a);
    status |= VL53L8CX_WrMulti(&(Dev.platform), 0, (uint8_t*)&VL53L8CX_FIRMWARE[0x8000], 0x8000);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x0b);
    status |= VL53L8CX_WrMulti(&(Dev.platform), 0, (uint8_t*)&VL53L8CX_FIRMWARE[0x10000], 0x5000);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x01);

    printf("  Status after 84KB write: %d\n", status);

    printf("Step 5: Check FW CRC and Boot MCU...\n");
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x01);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x06, 0x03);
    VL53L8CX_WaitMs(&(Dev.platform), 5);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7fff, 0x00);
    status |= VL53L8CX_RdByte(&(Dev.platform), 0x7fff, &tmp);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0C, 0x01);

    status |= VL53L8CX_WrByte(&(Dev.platform), 0x7FFF, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x114, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x115, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x116, 0x42);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x117, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0B, 0x00);
    status |= VL53L8CX_RdByte(&(Dev.platform), 0x7fff, &tmp);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0C, 0x00);
    status |= VL53L8CX_WrByte(&(Dev.platform), 0x0B, 0x01);

    uint8_t mcu_boot0 = 0, mcu_boot1 = 0;
    for (int i = 0; i < 500; i++) {
        VL53L8CX_RdByte(&(Dev.platform), 0x06, &mcu_boot0);
        if (mcu_boot0 & 0x80) {
            VL53L8CX_RdByte(&(Dev.platform), 0x07, &mcu_boot1);
            if (mcu_boot1 & 0x01) {
                printf("  MCU BOOTED SUCCESSFULLY on poll %d!\n", i);
                break;
            }
        }
        if (mcu_boot0 & 0x01) {
            printf("  MCU booted with status0 = 0x%02X!\n", mcu_boot0);
            break;
        }
        VL53L8CX_WaitMs(&(Dev.platform), 1);
    }
    printf("  mcu_boot0 = 0x%02X, mcu_boot1 = 0x%02X\n", mcu_boot0, mcu_boot1);

    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o trace_init_exact trace_init_exact.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./trace_init_exact
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_exact_steps.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_exact_steps.sh && /tmp/run_exact_steps.sh", get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    trace_exact_init_step()
