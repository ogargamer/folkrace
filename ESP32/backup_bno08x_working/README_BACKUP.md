# GY-BNO08X SPI WORKING FIRMWARE BACKUP
**Timestamp**: 2026-08-03
**Status**: 100% OPERATIONAL - VERIFIED WORKING

> [!IMPORTANT]
> DO NOT EDIT ANY FILES IN THIS BACKUP DIRECTORY! THIS DIRECTORY IS A READ-ONLY REFERENCE BACKUP OF THE WORKING GY-BNO08X SPI TELEMETRY FIRMWARE SYSTEM. ONLY COPY FROM THIS DIRECTORY IF RESTORING A WORKING BACKUP.

## Working Wiring Configuration
- **IMU_SPI_CS**: GPIO 10 (Pin 15)
- **IMU_SPI_INT**: GPIO 9 (Pin 14)
- **IMU_SPI_MOSI**: GPIO 11 (Pin 16)
- **IMU_SPI_SCK**: GPIO 12 (Pin 17)
- **IMU_SPI_MISO**: GPIO 13 (Pin 18)
- **IMU_SPI_RST**: GPIO 14 (Pin 19)
- **PS0**: 3.3V (Soldered onboard resistor pad)
- **PS1**: 3.3V (Soldered onboard resistor pad)
- **Baud Rate**: 921600 baud

## Backup Files Included
1. `imu_bno08x.h` - Working SPI driver header
2. `imu_bno08x.cpp` - Working SparkFun BNO080 SHTP queue handler
3. `main.cpp` - Working 400Hz main control & telemetry loop
