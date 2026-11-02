#!/usr/bin/env python3
"""
FOLKRACE — MT6816 / MT6818 SPI Encoder & Closed-Loop KV Test Suite GUI (GM3506 @ 12V)
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


def run_cli_spi_test(port=None, baud=921600, keep=False):
    print("=" * 80)
    print("⚡ ESP32-S3 MT6816 SPI CLOCK 4-16 MHz LATENCY & INTEGRITY TEST")
    print("=" * 80)
    reader = ESP32SerialReader(port=port, baud=baud)
    try:
        reader.connect()
    except Exception as e:
        print(f"[Error] Failed to connect to ESP32: {e}")
        return 1

    print(f"Connected to ESP32 on {reader.port} @ {baud} baud.")
    time.sleep(0.5)

    cmd = "SPI_TEST keep" if keep else "SPI_TEST"
    print(f"Sending command: '{cmd}'...")
    reader.send_cmd(cmd)

    t_start = time.time()
    bench_done = False

    while time.time() - t_start < 15.0:
        try:
            line = reader.raw_log_queue.get(timeout=0.1)
            print(line)
            if "[SPI_BENCH_END]" in line or "Original SPI speed restored" in line or "Highest stable SPI speed retained" in line:
                bench_done = True
                time.sleep(0.2)
                while not reader.raw_log_queue.empty():
                    print(reader.raw_log_queue.get_nowait())
                break
        except Exception:
            continue

    reader.disconnect()
    if bench_done:
        print("\n✅ SPI Benchmark successfully completed.")
        return 0
    else:
        print("\n⚠️ Test timed out waiting for benchmark completion.")
        return 1


def main():
    parser = argparse.ArgumentParser(description="MT6816 SPI Encoder & Motor 1 Test Bench with Closed-Loop KV Analyzer")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate (default: 921600)")
    parser.add_argument("--spi-test", action="store_true", help="Run automated 4-16 MHz SPI latency and stability test in terminal and exit")
    parser.add_argument("--keep", action="store_true", help="Retain highest stable SPI frequency after test")
    args = parser.parse_args()

    if args.spi_test:
        sys.exit(run_cli_spi_test(port=args.port, baud=args.baud, keep=args.keep))

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
