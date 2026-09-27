#!/usr/bin/env python3
"""
DA WINNA — MT6816 / MT6818 SPI Encoder & Closed-Loop KV Test Suite GUI (GM3506 @ 12V)
Modular Entry-Point Runner.
"""

import argparse
import os
import sys
import time
import tkinter as tk

# Ensure current suite directory is in sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from gui import ESP32SerialReader, MT6818DashboardGUI


def main():
    parser = argparse.ArgumentParser(description="MT6816 SPI Encoder & Motor 1 Test Bench with Closed-Loop KV Analyzer")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate (default: 921600)")
    args = parser.parse_args()

    reader = ESP32SerialReader(port=args.port, baud=args.baud)
    try:
        reader.connect()
    except Exception:
        pass

    root = tk.Tk()
    app = MT6818DashboardGUI(root, reader)

    def on_close():
        try:
            app.emergency_stop()
            time.sleep(0.08)
        except Exception:
            pass
        reader.disconnect()
        try:
            root.destroy()
        except Exception:
            pass
        os._exit(0)

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    on_close()


if __name__ == "__main__":
    main()
