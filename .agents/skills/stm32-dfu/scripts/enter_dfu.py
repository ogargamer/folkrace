#!/usr/bin/env python3
"""
Automatically puts STM32 into System ROM DFU mode without physical buttons.
Identifies STMicroelectronics USB CDC port (VID 0x0483, PID 0x5740),
sends 'DFU\\n' to trigger firmware software jump to bootloader, and optionally
executes PlatformIO upload.
"""

import sys
import time
import argparse
import subprocess
import serial
import serial.tools.list_ports

def find_stm32_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x0483 and p.pid == 0x5740:
            return p.device
        if "STM" in (p.description or "") or "STMicroelectronics" in (p.manufacturer or ""):
            return p.device
    return None

def trigger_dfu(port=None):
    if not port:
        port = find_stm32_port()
    if not port:
        print("[ERR] No active STM32 USB CDC port found (VID: 0x0483, PID: 0x5740).")
        print("      Device may already be in DFU mode or disconnected.")
        return False

    print(f"[INFO] Found STM32 on {port}. Sending DFU command...")
    try:
        ser = serial.Serial(port, baudrate=921600, timeout=1.0)
        ser.write(b"DFU\n")
        ser.flush()
        time.sleep(0.15)
        ser.close()
        print(f"[OK] DFU trigger sent. Device on {port} is rebooting to ST System DFU Bootloader...")
        time.sleep(1.5)
        return True
    except Exception as e:
        print(f"[WARN] Error sending DFU command to {port}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Reboot STM32 into DFU mode and optionally upload firmware.")
    parser.add_argument("--port", "-p", help="Explicit COM port (e.g. COM11). Auto-detects if omitted.")
    parser.add_argument("--upload", "-u", action="store_true", help="Run 'pio run -e stm32_dual_foc -t upload' after rebooting to DFU.")
    parser.add_argument("--env", "-e", default="stm32_dual_foc", help="PlatformIO environment name (default: stm32_dual_foc)")
    args = parser.parse_args()

    success = trigger_dfu(args.port)

    if args.upload:
        cmd = ["pio", "run", "-e", args.env, "-t", "upload"]
        print(f"\n[RUN] Running: {' '.join(cmd)}")
        ret = subprocess.run(cmd)
        sys.exit(ret.returncode)

if __name__ == "__main__":
    main()
