# ESP32-S3 SimpleFOC & MT6816 SPI Encoder Test Suite

This suite provides the complete firmware and visual testing interface for driving a GM3506 brushless gimbal motor using SimpleFOC with an MT6816 magnetic angle encoder on an ESP32-S3.

## Features
- **11-Pole Continuous Zero Vector alignment** for GM3506 (24N22P)
- **256-Point 2-Harmonic Fourier linearization** for sub-degree angle precision
- **Dynamic Speed Lag Lead Compensation**
- **NVS Flash persistence** for calibration offsets
- **Dual SPI support**: 4-Wire Hardware SPI & 3-Wire SPI

## Hardware Connections (ESP32-S3 DevKitC-1)
| Function | GPIO Pin | Notes |
| :--- | :--- | :--- |
| Driver Phase A | GPIO 15 | PWM Output |
| Driver Phase B | GPIO 16 | PWM Output |
| Driver Phase C | GPIO 17 | PWM Output |
| Driver Enable | GPIO 2 | High = Enabled |
| Encoder SCK | GPIO 12 | SPI Clock |
| Encoder MOSI | GPIO 11 | SPI Data Out / 3-Wire SDAT |
| Encoder MISO | GPIO 13 | SPI Data In |
| Encoder CS | GPIO 10 | Chip Select |
| Thermistor | GPIO 4 | ADC1_CH3 |

## Visual UI & Calibration
Run the companion visual interface:
```bash
python test_mt6818_ui.py
```
This renders a visual dial, raw 14-bit parity status, magnetic field strength checks, and allows 1-click Fourier calibration commands.
