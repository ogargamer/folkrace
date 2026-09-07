import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def dump_full_236_byte_packet():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  DUMPING COMPLETE 236-BYTE SENSOR PACKET & EXTRACTING ALL 16 ZONES")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > dump_packet.c
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
    uint8_t isAlive = 0;
    vl53l8cx_is_alive(&Dev, &isAlive);
    vl53l8cx_init(&Dev);

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("Waiting for 1 good frame...\n");
    uint8_t isReady = 0;
    for (int p = 0; p < 100; p++) {
        VL53L8CX_WaitMs(&Dev.platform, 5);
        vl53l8cx_check_data_ready(&Dev, &isReady);
        if (isReady) break;
    }

    printf("Reading full packet (size = %u)...\n", Dev.data_read_size);
    VL53L8CX_RdMulti(&Dev.platform, 0x0, Dev.temp_buffer, Dev.data_read_size);

    printf("\n--- COMPLETE PACKET HEX DUMP (%u bytes) ---\n", Dev.data_read_size);
    for (uint32_t b = 0; b < Dev.data_read_size; b++) {
        if (b % 16 == 0) printf("\n0x%04X: ", b);
        printf("%02X ", Dev.temp_buffer[b]);
    }
    printf("\n\n");

    // Also test without SwapBuffer vs with SwapBuffer
    printf("Block headers found in packet:\n");
    for (uint32_t i = 0; i < Dev.data_read_size - 4; i += 4) {
        uint32_t w = *(uint32_t*)&Dev.temp_buffer[i];
        uint32_t w_swap = (Dev.temp_buffer[i]<<24) | (Dev.temp_buffer[i+1]<<16) | (Dev.temp_buffer[i+2]<<8) | Dev.temp_buffer[i+3];

        uint16_t idx = (w >> 16) & 0xFFFF;
        uint16_t idx_s = (w_swap >> 16) & 0xFFFF;

        if (idx == 0x5440 || idx_s == 0x5440 || (idx >= 0x0001 && idx <= 0x0020) || (idx_s >= 0x0001 && idx_s <= 0x0020)) {
            printf("  Match at offset 0x%04X: Raw=0x%08X (idx=0x%04X), Swapped=0x%08X (idx=0x%04X)\n",
                   i, w, idx, w_swap, idx_s);
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o dump_packet dump_packet.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./dump_packet
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_dump.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_dump.sh && /tmp/run_dump.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    dump_full_236_byte_packet()
