# Raspberry Pi 5 (RP1) Pin Mapping & Pin Solver

This folder contains the complete pinout database and allocation solver for the **Raspberry Pi 5** (utilizing the RP1 I/O controller chip).

## RP1 Silicon SPI Hardware Note
**You are 100% correct!** In the RP1 silicon peripheral specification on Raspberry Pi 5:
- **SPI0, SPI1, SPI2, SPI3, SPI5**: Hardwired as **Master-only** SPI controllers.
- **SPI4 (and SPI7)**: Hardwired statically in RP1 silicon as **Slave-only** SPI controllers.

> [!IMPORTANT]
> Because Raspberry Pi 5 operates as **SPI Master** to control the ESP32 (Slave), **SPI4 cannot be used for Master output**.
> The pin solver configures the **SPI0 Master controller** (`MOSI=GPIO10`, `MISO=GPIO9`, `SCLK=GPIO11`) as the shared SPI Master bus, assigning a dedicated Chip Select (`GPIO18`) and Handshake line (`GPIO20`) for the ESP32 link.

---

## Project Requirements & Pin Mapping
1. **5x VL53L8CX Time-of-Flight (ToF) Distance Sensors**:
   - Allocated to the **ESP32-S3 sensor hub**, not the Pi 5 header.
2. **MS200K LiDAR Sensor**:
   - Hardware Serial UART (**UART4**: TX=GPIO12 / Pin 32, RX=GPIO13 / Pin 33 @ 230,400 baud)
   - Port: `/dev/ttyAMA4` (Zero Wi-Fi interference / isolated from serial console)
3. **ESP32 Duplex Link (Pi 5 Master -> ESP32 Slave)**:
   - Shared **SPI0 Master** lines + Dedicated Chip Select (`GPIO18`) + Data Ready Handshake Interrupt line (`GPIO20`).

---

## File Overview

* `pin_solver.py`: Python script containing the parsed Raspberry Pi 5 RP1 pin database, hardware audit functions, and optimal pin assignment solver logic.
* `pi5_pinout.csv`: Complete raw CSV data for the Raspberry Pi 5 40-Pin Header, onboard buttons, Debug header (J2), RTC connector (J5), and Fan connector (J1).

---

## How to Run

To execute the pin solver audit and check allocations:
```bash
python pin_solver.py
# or on Windows
py -3 pin_solver.py
```
