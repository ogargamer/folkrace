---
name: stm32-dfu
description: >-
  Put STM32 into System ROM DFU bootloader and flash firmware over USB hands-free.
---

# STM32 DFU Flashing

Firmware jumps to ST System ROM DFU when receiving `DFU\n` over USB CDC (VID `0x0483`, PID `0x5740`, 921600 baud).

## Commands

- **One-step DFU reboot & flash:**
  ```powershell
  python .agents/skills/stm32-dfu/scripts/enter_dfu.py --upload
  ```

- **DFU reboot only:**
  ```powershell
  python .agents/skills/stm32-dfu/scripts/enter_dfu.py
  ```

- **Inline CLI:**
  ```powershell
  python -c "import serial, time; s = serial.Serial('COM11', 921600); s.write(b'DFU\n'); s.flush(); time.sleep(0.15); s.close()"; Start-Sleep -Milliseconds 1500; pio run -e stm32_dual_foc -t upload
  ```

## Notes
- If device is already in DFU mode (`0483:df11`), run `pio run -e stm32_dual_foc -t upload` directly.
- Ignore Windows Bluetooth ports (`COM9`, `COM10`).
