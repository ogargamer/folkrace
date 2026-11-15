---
name: esp32-pi-flash
description: >-
  Build ESP32-S3 firmware locally and flash ESP32_Main or ESP32_ToF over USB via Raspberry Pi 4.
---

# ESP32 Flashing via Pi 4

## Device Mapping
- `esp32_main`: `/dev/ttyACM0` (USB serial `5B90128539`, MAC `ac:a7:04:1d:a4:fc`)
- `esp32_tof`: `/dev/ttyACM1` (USB serial `5B8E139240`)
- Verify if ports swap: `ssh robot 'cd /home/admin/folkrace && python3 -c "import mcu_uart_bridge as m; print(m.probe_port_role(\"/dev/ttyACM0\"), m.probe_port_role(\"/dev/ttyACM1\"))"'`

## Build & Flash (`<env>` = `esp32_main` | `esp32_tof`, `<port>` = `/dev/ttyACM0` | `/dev/ttyACM1`)
```powershell
pio run -e <env>
scp .pio/build/<env>/bootloader.bin .pio/build/<env>/partitions.bin .pio/build/<env>/firmware.bin admin@robot:/home/admin/folkrace/esp_flash/
ssh robot 'echo l | sudo -S systemctl stop folkrace && cd /home/admin/folkrace/esp_flash && python3 -m esptool --chip esp32s3 --port <port> --baud 921600 write-flash 0x0 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin && echo l | sudo -S systemctl start folkrace'
```
