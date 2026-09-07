import paramiko
import sys

sys.stdout.reconfigure(encoding='utf-8')

PI_IP = "10.43.8.42"
PI_USER = "admin"
PI_PASS = "l"

def dump_ranging_buffer_and_parse():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(PI_IP, port=22, username=PI_USER, password=PI_PASS, timeout=10)

    print("======================================================================")
    print("  DUMPING FULL RANGING BUFFER FOR DIAGNOSIS")
    print("======================================================================")

    script = r"""#!/bin/bash
set -e
cd /home/admin/STSW-IMG042/VL53L8CX_Linux_driver_2.1.0/user/test

cat << 'EOF' > dump_ranging.c
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
    Dev.platform.spi_num = 1;
    Dev.platform.spi_cs = 0;

    vl53l8cx_comms_init(&Dev.platform);
    vl53l8cx_is_alive(&Dev, &isAlive);
    vl53l8cx_init(&Dev);
    vl53l8cx_set_resolution(&Dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&Dev, 15);
    vl53l8cx_start_ranging(&Dev);

    printf("data_read_size = %u bytes\n", Dev.data_read_size);

    uint8_t isReady = 0;
    for (int p = 0; p < 100; p++) {
        VL53L8CX_WaitMs(&Dev.platform, 10);
        vl53l8cx_check_data_ready(&Dev, &isReady);
        if (isReady) break;
    }

    printf("isReady = %d\n", isReady);

    // Read full raw packet
    VL53L8CX_RdMulti(&Dev.platform, 0x0, Dev.temp_buffer, Dev.data_read_size);
    printf("Raw first 64 bytes (before swap):\n");
    for(int k=0; k<64; k++) {
        printf("%02X ", Dev.temp_buffer[k]);
        if((k+1)%16==0) printf("\n");
    }

    VL53L8CX_SwapBuffer(Dev.temp_buffer, (uint16_t)Dev.data_read_size);
    printf("\nRaw first 64 bytes (after swap):\n");
    for(int k=0; k<64; k++) {
        printf("%02X ", Dev.temp_buffer[k]);
        if((k+1)%16==0) printf("\n");
    }

    // Now let's trace block headers
    printf("\nParsing block headers:\n");
    for (uint32_t i = 16; i < Dev.data_read_size; i += 4) {
        union Block_header *bh = (union Block_header *)&(Dev.temp_buffer[i]);
        uint32_t msize = (bh->type > 1 && bh->type < 13) ? (bh->type * bh->size) : bh->size;
        printf("  Offset 0x%04X: idx=0x%04X, type=%u, size=%u, msize=%u\n", i, bh->idx, bh->type, bh->size, msize);
        if (bh->idx == VL53L8CX_DISTANCE_IDX) {
            printf("    -> FOUND DISTANCE BLOCK at offset 0x%04X:\n       ", i+4);
            int16_t *dist = (int16_t*)&(Dev.temp_buffer[i+4]);
            for(int z=0; z<16; z++) printf("%d mm ", dist[z]);
            printf("\n");
        }
        if (bh->idx == VL53L8CX_TARGET_STATUS_IDX) {
            printf("    -> FOUND TARGET STATUS BLOCK at offset 0x%04X:\n       ", i+4);
            uint8_t *st = (uint8_t*)&(Dev.temp_buffer[i+4]);
            for(int z=0; z<16; z++) printf("st:%u ", st[z]);
            printf("\n");
        }
        if (msize == 0 || msize > 2000) break;
        i += msize;
    }

    vl53l8cx_stop_ranging(&Dev);
    vl53l8cx_comms_close(&Dev.platform);
    return 0;
}
EOF

gcc -Wall -Wno-missing-braces -Os -DSPI -I../uld-driver/inc -I../platform -I../examples \
    -o dump_ranging dump_ranging.c \
    ../uld-driver/src/vl53l8cx_api.c \
    ../uld-driver/src/vl53l8cx_plugin_detection_thresholds.c \
    ../uld-driver/src/vl53l8cx_plugin_motion_indicator.c \
    ../uld-driver/src/vl53l8cx_plugin_xtalk.c \
    ../platform/platform.c

./dump_ranging
"""

    stdin, stdout, stderr = ssh.exec_command(script, get_pty=True)
    for line in iter(stdout.readline, ""):
        print(line, end="")

    ssh.close()

if __name__ == "__main__":
    dump_ranging_buffer_and_parse()
