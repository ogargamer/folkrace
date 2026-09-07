import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "192.168.0.13"
PI_USER = "admin"
PI_PASS = "l"

def inspect_raw_spidev_packet():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  PARSING RAW SPI PACKET HEADERS & 16-ZONE DISTANCE DATA")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > parse_packet.c
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "vl53l8cx_api.h"

int main() {
    VL53L8CX_Configuration Dev;
    VL53L8CX_ResultsData Results;
    uint8_t isAlive = 0;

    memset(&Dev, 0, sizeof(Dev));
    Dev.platform.spi_num = 0;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    vl53l8cx_is_alive(&Dev, &isAlive);
    vl53l8cx_init(&Dev);

    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("\nStarting packet analysis...\n");
    for (int frame = 1; frame <= 5; frame++) {
        uint8_t isReady = 0;
        for (int p = 0; p < 100; p++) {
            VL53L8CX_WaitMs(&Dev.platform, 5);
            vl53l8cx_check_data_ready(&Dev, &isReady);
            if (isReady) break;
        }

        if (isReady) {
            printf("\n=================== FRAME %d (read_size: %u) ===================\n",
                   frame, Dev.data_read_size);
            VL53L8CX_RdMulti(&Dev.platform, 0x0, Dev.temp_buffer, Dev.data_read_size);

            // Print first 64 bytes of raw temp_buffer
            printf("Raw Buffer (first 64 bytes):\n");
            for (int b = 0; b < 64; b++) {
                printf("%02X ", Dev.temp_buffer[b]);
                if ((b + 1) % 16 == 0) printf("\n");
            }

            // Test parsing block headers
            uint32_t i = 16;
            while (i < Dev.data_read_size) {
                uint32_t h_raw = *(uint32_t*)&Dev.temp_buffer[i];
                uint32_t h_swap = (Dev.temp_buffer[i]<<24) | (Dev.temp_buffer[i+1]<<16) | (Dev.temp_buffer[i+2]<<8) | Dev.temp_buffer[i+3];

                uint16_t idx_raw = (h_raw >> 16) & 0xFFFF;
                uint16_t size_raw = (h_raw >> 4) & 0x0FFF;
                uint8_t type_raw = h_raw & 0x0F;

                uint16_t idx_swap = (h_swap >> 16) & 0xFFFF;
                uint16_t size_swap = (h_swap >> 4) & 0x0FFF;
                uint8_t type_swap = h_swap & 0x0F;

                printf("Offset 0x%04X: Raw[idx=0x%04X, size=%u, type=%u] | Swapped[idx=0x%04X, size=%u, type=%u]\n",
                       i, idx_raw, size_raw, type_raw, idx_swap, size_swap, type_swap);

                // If block matches distance (0x5440 or 0x0006 or 0x0008)
                if (idx_raw == 0x5440 || idx_swap == 0x5440 || idx_raw == 0x0006 || idx_raw == 0x0008 || idx_swap == 0x0006) {
                    printf("  --> DISTANCE BLOCK FOUND at offset 0x%04X!\n", i);
                }

                uint32_t msize = (type_raw > 1 && type_raw < 0xD) ? (type_raw * size_raw) : size_raw;
                if (msize == 0 || msize > 2000) break;
                i += 4 + msize;
            }
        }
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -g0 -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o parse_packet parse_packet.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./parse_packet
"""

    sftp = ssh.open_sftp()
    with sftp.file('/tmp/run_parse.sh', 'w') as f:
        f.write(script)
    sftp.close()

    stdin, stdout, stderr = ssh.exec_command("chmod +x /tmp/run_parse.sh && /tmp/run_parse.sh", get_pty=True)

    for line in iter(stdout.readline, ""):
        print(line, end="")

    exit_code = stdout.channel.recv_exit_status()
    print(f"\nExit Code: {exit_code}")
    ssh.close()

if __name__ == "__main__":
    inspect_raw_spidev_packet()
