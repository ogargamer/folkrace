# BACKUP: MT6816 / MT6818 SPI & PWM Encoder Working Test Bench

This directory contains the verified working firmware, diagnostic tools, and graphical user interface for the **MagnTek MT6816 / MT6818 Magnetic Angle Sensor** connected to the **STM32G474 (Relaxed Pinout Mode)**.

---

## 📁 Backup Contents
- [`main.cpp`](file:///c:/Users/nurmi/Documents/Da%20Winna/STM32/backup_mt6818_encoder_working/main.cpp): Complete STM32 firmware supporting 3-Wire Half-Duplex SPI (`PA7` SDAT), 4-Wire Hardware SPI1 (`PA5`, `PA7`, `PB4`), 971 Hz Hardware PWM capture (`PA0`), Even Parity verification, and SimpleFOC Motor 1 Open/Closed-loop driver.
- [`test_mt6818_ui.py`](file:///c:/Users/nurmi/Documents/Da%20Winna/STM32/backup_mt6818_encoder_working/test_mt6818_ui.py): Tkinter graphical user interface with live compass dial, real-time strip-chart oscilloscope, health badges, auto-reconnecting serial engine, and command control buttons.

---

## 🔌 Hardware Wiring & Pin Mapping (Relaxed Mode)

| MT6816 Sensor Pin | Breakout Silkscreen | STM32 MCU Pin | Board Header Pin | Role & Signal |
| :--- | :--- | :---: | :---: | :--- |
| **Pin 1** | **`CSN`** | **`PA3`** | `A3` | Chip Select for Encoder 1 (Active LOW) |
| **Pin 2** | **`HVPP`** | *(Solder Pad `R3`)* | `3.3V` | Pulled HIGH via R3 to enable SPI peripheral |
| **Pin 3** | **`PWM`** | **`PA0`** | `A0` | 971.1 Hz Absolute PWM Angle Signal |
| **Pin 4** | **`VDD`** | **`3.3V`** | `3V3` | Sensor Power Supply (3.3V) |
| **Pin 5** | **`SCK`** | **`PA5`** | `A5` | SPI Clock (Mode 3: Idle HIGH, CPHA=1) |
| **Pin 7** | **`MOSI`** | **`PA7`** | `A7` | **Single Bi-Directional `SDAT` Line (3-Wire Mode)** |
| **Pin 6** | **`MISO`** | **`PB4`** | `B4` | MISO (Used only in 4-Wire SPI mode) |
| **Pin 8** | **`GND`** | **`GND`** | `GND` | Ground Reference |

---

## ⚙️ Protocol & Silicon Characteristics

1. **Native Factory Mode:**
   - The MT6816 breakout board with `R3` bridged defaults to **3-Wire Half-Duplex SPI Mode (`SPI_Mode = 1`)**.
   - Communication occurs over **`PA7` (`SDAT`)**.
2. **Registers:**
   - `0x03`: Angle High Byte (`Angle<13:6>`)
   - `0x04`: Angle Low Byte (`Angle<5:0>`, Bit 1 = `No_Mag_Warn`, Bit 0 = `Parity`)
3. **Parity Formula:**
   - **Even Parity** over all 16 bits: `__builtin_parity(angle_data) == 0`.
4. **PWM Output:**
   - 12-bit linear absolute output on Pin 3 at **971.1 Hz (Period ~1029.7 µs)**.
   - Active continuously in all modes.

---

## 🚀 How to Run

### 1. Build & Upload Firmware:
```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e encoder_test -t upload
```

### 2. Launch Graphical UI:
```powershell
python launch_mt6818_test_gui.py
```

### 3. Interactive Keyboard Commands:
- `[1]`: Toggle Live Stream (Start / Pause)
- `[2]`: Raw Register Inspector
- `[3]`: 10,000 Sample High-Speed Latency Benchmark
- `[4]`: Motor 1 Open-Loop Spin Test (`+` / `-` / `Space`)
- `[5]`: SimpleFOC Electrical Angle Alignment & Closed-Loop
- `[t]`: Toggle 3-Wire vs 4-Wire SPI Mode
- `[z]`: Tare / Zero Angle Offset
