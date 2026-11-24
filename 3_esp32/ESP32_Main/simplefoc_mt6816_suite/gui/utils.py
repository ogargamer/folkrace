"""
Utilities for finding PlatformIO and ESP32 serial COM ports.
"""

import os
import shutil
import serial.tools.list_ports


def find_pio_executable():
    """Locate platformio executable in PATH or user home directory."""
    for name in ["pio", "platformio", "pio.exe", "platformio.exe"]:
        p = shutil.which(name)
        if p:
            return p
    user_home = os.path.expanduser("~")
    candidates = [
        os.path.join(user_home, ".platformio", "penv", "Scripts", "platformio.exe"),
        os.path.join(user_home, ".platformio", "penv", "Scripts", "pio.exe"),
        os.path.join(user_home, ".platformio", "penv", "bin", "platformio"),
        os.path.join(user_home, ".platformio", "penv", "bin", "pio"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return "platformio"


def find_esp32_port():
    """Auto-detect connected ESP32 USB serial COM port."""
    for p in serial.tools.list_ports.comports():
        hw = (p.hwid or "").upper()
        desc = (p.description or "").upper()
        if (
            "303A" in hw
            or "1A86" in hw
            or "10C4" in hw
            or "ESP" in desc
            or "CH34" in desc
            or "CP210" in desc
            or "USB" in desc
            or "SERIAL" in desc
        ):
            return p.device
    return "COM5"
