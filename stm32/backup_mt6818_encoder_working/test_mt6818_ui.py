"""
DA WINNA — MT6818 SPI Encoder & Motor 1 Test Bench GUI & Monitor
================================================================
A dual-mode (Interactive Terminal + Full Tkinter GUI) test interface
for validating MagnTek MT6818 / MT6816 absolute magnetic angle encoders
over SPI with the STM32G474 motor controller.
"""

import sys
import os
import time
import math
import threading
import queue
import subprocess
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox

# ---------------------------------------------------------------------------
# Serial Communicator & Auto-Detector
# ---------------------------------------------------------------------------

def find_stm32_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # Check for STM32 USB VID/PID (0483:5740) or friendly description
        if "0483" in p.hwid or "STM" in p.description or "USB Serial Device" in p.description:
            return p.device
    for p in ports:
        if "Bluetooth" not in p.description:
            return p.device
    return None

class STM32SerialReader:
    def __init__(self, port=None, baud=921600):
        self.port = port or find_stm32_port()
        self.baud = baud
        self.ser = None
        self.running = False
        self.connected = False
        self.thread = None
        self.data_queue = queue.Queue(maxsize=500)
        self.raw_log_queue = queue.Queue(maxsize=1000)

        # Telemetry State
        self.last_angle_deg = 0.0
        self.last_angle_rad = 0.0
        self.last_raw_14bit = 0
        self.last_rpm = 0.0
        self.last_turns = 0
        self.last_mag_ok = True
        self.last_parity_ok = True
        self.history_time = []
        self.history_angle = []
        self.history_rpm = []
        self.start_time = time.time()

    def connect(self):
        if not self.port:
            self.port = find_stm32_port()
        if not self.port:
            raise RuntimeError("No STM32 USB Serial port found! Ensure STM32 is plugged in.")
        try:
            self.disconnect()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.2)
            self.ser.dtr = True
            self.ser.rts = True
            self.running = True
            self.connected = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            self.running = False
            self.connected = False
            raise e

    def disconnect(self):
        self.running = False
        self.connected = False
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open and self.connected:
            try:
                self.ser.write(cmd_str.encode("utf-8"))
            except Exception:
                self.connected = False

    def _read_loop(self):
        buffer = ""
        while self.running:
            if not self.ser or not self.ser.is_open or not self.connected:
                # Auto-reconnect worker
                time.sleep(0.5)
                try:
                    port = find_stm32_port()
                    if port:
                        self.port = port
                        self.ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.2)
                        self.ser.dtr = True
                        self.ser.rts = True
                        self.connected = True
                        buffer = ""
                except Exception:
                    continue

            try:
                data = self.ser.read(128).decode("utf-8", errors="ignore")
                if not data:
                    time.sleep(0.01)
                    continue
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip("\r")
                    self.raw_log_queue.put(line)
                    self._parse_line(line)
            except Exception:
                self.connected = False
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                time.sleep(0.2)

    def _parse_line(self, line):
        # Format: [ENC1] Angle:  182.45 deg ( 3.184 rad) | Raw:  8295/16383 | Spd:  +12.0 RPM | Turns: 0 | Mag: [ OK ] | Parity: [ PASS ]
        if "[ENC1]" in line and "Angle:" in line:
            try:
                # Extract Angle Deg
                idx_angle = line.find("Angle:") + 6
                idx_deg = line.find("deg", idx_angle)
                angle_deg = float(line[idx_angle:idx_deg].strip())

                # Extract Raw count
                idx_raw = line.find("Raw:") + 4
                idx_slash = line.find("/16383", idx_raw)
                raw_count = int(line[idx_raw:idx_slash].strip()) if idx_raw > 3 and idx_slash > idx_raw else int((angle_deg / 360.0) * 16384)

                # Extract RPM
                idx_spd = line.find("Spd:") + 4
                idx_rpm = line.find("RPM", idx_spd)
                rpm = float(line[idx_spd:idx_rpm].strip()) if idx_spd > 3 and idx_rpm > idx_spd else 0.0

                # Extract Turns
                turns = 0
                if "Turns:" in line:
                    idx_turns = line.find("Turns:") + 6
                    idx_pipe = line.find("|", idx_turns)
                    turns = int(line[idx_turns:idx_pipe].strip())

                # Health Flags
                mag_ok = "Mag: [ OK ]" in line or "[MAGNET OK]" in line
                parity_ok = "Parity: [ PASS ]" in line or "[PARITY CHECK PASSED" in line or "Parity: OK" in line

                self.last_angle_deg = angle_deg
                self.last_angle_rad = (angle_deg / 180.0) * math.pi
                self.last_raw_14bit = raw_count
                self.last_rpm = rpm
                self.last_turns = turns
                self.last_mag_ok = mag_ok
                self.last_parity_ok = parity_ok

                # Record history for plotting
                t_now = time.time() - self.start_time
                self.history_time.append(t_now)
                self.history_angle.append(angle_deg)
                self.history_rpm.append(rpm)
                if len(self.history_time) > 150:
                    self.history_time.pop(0)
                    self.history_angle.pop(0)
                    self.history_rpm.pop(0)

                self.data_queue.put({
                    "deg": angle_deg,
                    "rad": self.last_angle_rad,
                    "raw": raw_count,
                    "rpm": rpm,
                    "turns": turns,
                    "mag_ok": mag_ok,
                    "parity_ok": parity_ok,
                    "time": t_now
                })
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Tkinter Graphical Dashboard
# ---------------------------------------------------------------------------

class MT6818DashboardGUI:
    def __init__(self, root, reader):
        self.root = root
        self.reader = reader
        self.root.title("DA WINNA — MT6818 SPI Encoder Test Bench")
        self.root.geometry("1100x750")
        self.root.minsize(950, 650)
        self.root.configure(bg="#12161E")

        self._setup_styles()
        self._build_ui()
        self._schedule_update()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#12161E")
        style.configure("Card.TFrame", background="#1A2230", relief="flat")
        style.configure("Header.TLabel", background="#12161E", foreground="#00E5FF", font=("Segoe UI", 16, "bold"))
        style.configure("SubHeader.TLabel", background="#12161E", foreground="#8A99AD", font=("Segoe UI", 10))
        style.configure("CardTitle.TLabel", background="#1A2230", foreground="#8A99AD", font=("Segoe UI", 9, "bold"))
        style.configure("CardVal.TLabel", background="#1A2230", foreground="#FFFFFF", font=("Segoe UI", 20, "bold"))
        style.configure("Action.TButton", font=("Segoe UI", 10, "bold"), padding=6)

    def _build_ui(self):
        # 1. Header Bar
        header_frame = ttk.Frame(self.root, padding="15 12 15 10")
        header_frame.pack(fill="x")

        ttk.Label(header_frame, text="🏎️ DA WINNA — MT6818 SPI ENCODER & MOTOR 1 BENCH", style="Header.TLabel").pack(side="left")

        self.conn_label = tk.Label(header_frame, text="DISCONNECTED", font=("Segoe UI", 10, "bold"), fg="#FF5252", bg="#12161E")
        self.conn_label.pack(side="right", padx=10)

        self.port_entry = tk.Entry(header_frame, width=10, font=("Segoe UI", 10), bg="#222D3E", fg="#FFFFFF", insertbackground="#FFF")
        self.port_entry.insert(0, self.reader.port or "COM3")
        self.port_entry.pack(side="right", padx=5)

        self.btn_connect = tk.Button(header_frame, text="Connect", font=("Segoe UI", 9, "bold"), bg="#00E5FF", fg="#000",
                                     activebackground="#00B4D8", command=self._toggle_connect, relief="flat", padx=10)
        self.btn_connect.pack(side="right", padx=5)

        # 2. Main Content Split
        main_paned = tk.PanedWindow(self.root, orient="horizontal", bg="#12161E", bd=0, sashwidth=4)
        main_paned.pack(fill="both", expand=True, padx=15, pady=5)

        # Left Column: Gauge Needle Dial & Value Cards
        left_col = ttk.Frame(main_paned)
        main_paned.add(left_col, minsize=480)

        # Value Cards Frame (2x2 Grid)
        cards_frame = ttk.Frame(left_col)
        cards_frame.pack(fill="x", pady=5)

        self.card_angle = self._create_card(cards_frame, "ABSOLUTE ANGLE", "0.00°", "#00E5FF", 0, 0)
        self.card_rpm = self._create_card(cards_frame, "ROTATIONAL SPEED", "+0.0 RPM", "#69F0AE", 0, 1)
        self.card_raw = self._create_card(cards_frame, "14-BIT RAW COUNT", "0 / 16383", "#FFD740", 1, 0)
        self.card_turns = self._create_card(cards_frame, "TOTAL REVOLUTIONS", "0 Turns", "#E040FB", 1, 1)

        # Canvas Gauge (Dial Needle)
        gauge_card = ttk.Frame(left_col, style="Card.TFrame", padding=10)
        gauge_card.pack(fill="both", expand=True, pady=10)

        ttk.Label(gauge_card, text="REALTIME ROTARY COMPASS (0° — 360°)", style="CardTitle.TLabel").pack(anchor="w")
        self.gauge_canvas = tk.Canvas(gauge_card, bg="#1A2230", highlightthickness=0, width=320, height=320)
        self.gauge_canvas.pack(fill="both", expand=True, pady=5)

        # Health Badges Bar
        health_frame = ttk.Frame(left_col)
        health_frame.pack(fill="x", pady=5)

        self.lbl_mag = tk.Label(health_frame, text="MAGNET: OK", font=("Segoe UI", 9, "bold"), fg="#1A2230", bg="#69F0AE", padx=10, pady=4)
        self.lbl_mag.pack(side="left", fill="x", expand=True, padx=2)

        self.lbl_parity = tk.Label(health_frame, text="PARITY: PASS", font=("Segoe UI", 9, "bold"), fg="#1A2230", bg="#69F0AE", padx=10, pady=4)
        self.lbl_parity.pack(side="left", fill="x", expand=True, padx=2)

        # Right Column: Controls, Oscilloscope Chart, & Raw Log
        right_col = ttk.Frame(main_paned)
        main_paned.add(right_col, minsize=480)

        # Control Action Buttons Grid
        btn_frame = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        btn_frame.pack(fill="x", pady=5)

        ttk.Label(btn_frame, text="TEST CONTROLS & COMMANDS", style="CardTitle.TLabel").grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))

        self.btn_stream = self._create_btn(btn_frame, "⏸️ Pause Stream", "1", "#00E676", 1, 0)
        self._create_btn(btn_frame, "🔍 Raw Register Dump", "2", "#7C4DFF", 1, 1)
        self._create_btn(btn_frame, "⚡ 10k Benchmark", "3", "#FF9100", 1, 2)
        self.btn_bus_mode = self._create_btn(btn_frame, "🔌 3-Wire / 4-Wire", "t", "#00B0FF", 1, 3)

        self._create_btn(btn_frame, "🏎️ Motor +10 RPM", "+", "#00E676", 2, 0)
        self._create_btn(btn_frame, "🏎️ Motor -10 RPM", "-", "#FF5252", 2, 1)
        self._create_btn(btn_frame, "🛑 Motor STOP", " ", "#D50000", 2, 2)
        self._create_btn(btn_frame, "🎯 Zero / Tare", "z", "#FFD600", 2, 3)

        self._create_btn(btn_frame, "🔄 FOC Align & Run", "5", "#9C27B0", 3, 0)
        self._create_btn(btn_frame, "⚙️ Cycle Clock", "s", "#455A64", 3, 1)
        self._create_btn(btn_frame, "🔄 Cycle CS Pin", "c", "#455A64", 3, 2)
        self._create_btn(btn_frame, "⚡ Flash Firmware", "FLASH", "#E91E63", 3, 3)

        # Strip-Chart Canvas (Oscilloscope)
        chart_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        chart_card.pack(fill="x", pady=5)

        ttk.Label(chart_card, text="REALTIME ANGLE & VELOCITY STRIP-CHART", style="CardTitle.TLabel").pack(anchor="w")
        self.chart_canvas = tk.Canvas(chart_card, bg="#10151E", height=130, highlightthickness=0)
        self.chart_canvas.pack(fill="x", expand=True, pady=5)

        # Raw Serial Console Log
        log_card = ttk.Frame(right_col, style="Card.TFrame", padding=10)
        log_card.pack(fill="both", expand=True, pady=5)

        ttk.Label(log_card, text="SERIAL MONITOR CONSOLE OUTPUT", style="CardTitle.TLabel").pack(anchor="w")
        self.log_text = tk.Text(log_card, bg="#0E131B", fg="#B0BEC5", font=("Consolas", 9), height=10, relief="flat", wrap="none")
        self.log_text.pack(fill="both", expand=True, pady=5)

    def _create_card(self, parent, title, init_val, color, row, col):
        card = ttk.Frame(parent, style="Card.TFrame", padding="10 8 10 8")
        card.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
        parent.columnconfigure(col, weight=1)

        ttk.Label(card, text=title, style="CardTitle.TLabel").pack(anchor="w")
        val_lbl = tk.Label(card, text=init_val, font=("Segoe UI", 16, "bold"), fg=color, bg="#1A2230")
        val_lbl.pack(anchor="w", pady=(2, 0))
        return val_lbl

    def _create_btn(self, parent, text, cmd, color, row, col):
        b = tk.Button(parent, text=text, font=("Segoe UI", 9, "bold"), bg=color, fg="#FFFFFF" if color != "#00E676" and color != "#FFD600" else "#000000",
                      activebackground="#ECEFF1", relief="flat", padx=6, pady=4,
                      command=lambda: self._handle_btn(cmd))
        b.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)
        parent.columnconfigure(col, weight=1)
        return b

    def _handle_btn(self, cmd):
        if cmd == "FLASH":
            self._flash_firmware()
        elif cmd == "1":
            self.stream_active = not getattr(self, "stream_active", True)
            if self.stream_active:
                self.btn_stream.config(text="⏸️ Pause Stream", bg="#00E676", fg="#000000")
            else:
                self.btn_stream.config(text="▶️ Resume Stream", bg="#2979FF", fg="#FFFFFF")
            self.reader.send_cmd("1")
        elif cmd == "t" or cmd == "T":
            self.bus_mode_3wire = not getattr(self, "bus_mode_3wire", True)
            if self.bus_mode_3wire:
                self.btn_bus_mode.config(text="🔌 Mode: 3-Wire", bg="#00B0FF", fg="#FFFFFF")
            else:
                self.btn_bus_mode.config(text="🔌 Mode: 4-Wire", bg="#7C4DFF", fg="#FFFFFF")
            self.reader.send_cmd("t")
        else:
            self.reader.send_cmd(cmd)

    def _flash_firmware(self):
        ans = messagebox.askyesno("Flash STM32", "Upload MT6818 test firmware to STM32?\n(STM32 should be in DFU / Boot mode)")
        if not ans:
            return
        self.reader.disconnect()
        self.conn_label.config(text="FLASHING...", fg="#FFD600")
        self.root.update()

        def do_flash():
            cmd = [os.path.expandvars(r"%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"), "run", "-e", "encoder_test", "-t", "upload"]
            res = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(os.path.abspath(__file__)) + "/..")
            if res.returncode == 0:
                messagebox.showinfo("Flash Success", "STM32 firmware flashed successfully!")
            else:
                messagebox.showerror("Flash Error", f"Upload failed:\n{res.stdout[-400:]}")
            self._toggle_connect(force_connect=True)

        threading.Thread(target=do_flash, daemon=True).start()

    def _toggle_connect(self, force_connect=False):
        if self.reader.running and not force_connect:
            self.reader.disconnect()
            self.conn_label.config(text="DISCONNECTED", fg="#FF5252")
            self.btn_connect.config(text="Connect", bg="#00E5FF")
        else:
            self.reader.port = self.port_entry.get().strip()
            try:
                self.reader.connect()
                self.conn_label.config(text=f"ONLINE ({self.reader.port})", fg="#69F0AE")
                self.btn_connect.config(text="Disconnect", bg="#FF5252")
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def _schedule_update(self):
        self._update_telemetry()
        self.root.after(30, self._schedule_update)

    def _update_telemetry(self):
        # 1. Update text logs
        while not self.reader.raw_log_queue.empty():
            line = self.reader.raw_log_queue.get_nowait()
            self.log_text.insert("end", line + "\n")
            if float(self.log_text.index("end-1c")) > 200:
                self.log_text.delete("1.0", "50.0")
            self.log_text.see("end")

        # 2. Update value cards
        deg = self.reader.last_angle_deg
        rpm = self.reader.last_rpm
        raw = self.reader.last_raw_14bit
        turns = self.reader.last_turns

        self.card_angle.config(text=f"{deg:.2f}°  ({self.reader.last_angle_rad:.3f} rad)")
        self.card_rpm.config(text=f"{rpm:+.1f} RPM")
        self.card_raw.config(text=f"{raw} / 16383  ({(raw / 16384.0)*100:.1f}%)")
        self.card_turns.config(text=f"{turns} Turns")

        # 3. Update Health Badges
        if self.reader.last_mag_ok:
            self.lbl_mag.config(text="MAGNET: OK (DETECTED)", bg="#69F0AE", fg="#1A2230")
        else:
            self.lbl_mag.config(text="MAGNET: FAULT / WEAK", bg="#FF5252", fg="#FFFFFF")

        if self.reader.last_parity_ok:
            self.lbl_parity.config(text="PARITY: PASS (VALID)", bg="#69F0AE", fg="#1A2230")
        else:
            self.lbl_parity.config(text="PARITY: ERROR / NOISE", bg="#FF5252", fg="#FFFFFF")

        # 4. Draw Compass Dial Gauge
        self._draw_gauge(deg)

        # 5. Draw Strip-Chart
        self._draw_chart()

    def _draw_gauge(self, deg):
        canvas = self.gauge_canvas
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 10 or h < 10:
            return

        cx, cy = w / 2, h / 2
        r = min(cx, cy) - 25

        # Background Circle & Ring
        canvas.create_oval(cx - r, cy - r, cx + r, cy + r, outline="#2A374A", width=4, fill="#121824")
        canvas.create_oval(cx - (r - 12), cy - (r - 12), cx + (r - 12), cy + (r - 12), outline="#1F2A38", width=1)

        # Cardinal / Degree Tick marks
        for a in range(0, 360, 30):
            rad_a = math.radians(a - 90)
            inner_r = r - 16 if a % 90 == 0 else r - 8
            x1 = cx + (r - 2) * math.cos(rad_a)
            y1 = cy + (r - 2) * math.sin(rad_a)
            x2 = cx + inner_r * math.cos(rad_a)
            y2 = cy + inner_r * math.sin(rad_a)
            color = "#00E5FF" if a % 90 == 0 else "#4A5D78"
            canvas.create_line(x1, y1, x2, y2, fill=color, width=2 if a % 90 == 0 else 1)

            # Degree labels
            if a % 90 == 0:
                lx = cx + (r - 28) * math.cos(rad_a)
                ly = cy + (r - 28) * math.sin(rad_a)
                canvas.create_text(lx, ly, text=f"{a}°", fill="#8A99AD", font=("Segoe UI", 8, "bold"))

        # Center Hub
        canvas.create_oval(cx - 10, cy - 10, cx + 10, cy + 10, fill="#00E5FF", outline="#FFFFFF", width=2)

        # Rotating Needle (Red tip, cyan pointer)
        needle_rad = math.radians(deg - 90)
        nx = cx + (r - 18) * math.cos(needle_rad)
        ny = cy + (r - 18) * math.sin(needle_rad)

        # Needle base triangle
        perp_rad = needle_rad + math.pi / 2
        bx1 = cx + 8 * math.cos(perp_rad)
        by1 = cy + 8 * math.sin(perp_rad)
        bx2 = cx - 8 * math.cos(perp_rad)
        by2 = cy - 8 * math.sin(perp_rad)

        canvas.create_polygon(bx1, by1, nx, ny, bx2, by2, fill="#FF5252", outline="#FF1744", width=1)
        canvas.create_line(cx, cy, nx, ny, fill="#FFFFFF", width=2)

        # Center Text Display
        canvas.create_text(cx, cy + (r / 2) + 10, text=f"{deg:.1f}°", fill="#00E5FF", font=("Segoe UI", 16, "bold"))

    def _draw_chart(self):
        canvas = self.chart_canvas
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 20 or h < 20 or len(self.reader.history_angle) < 2:
            return

        # Grid lines (0, 90, 180, 270, 360 deg)
        for y_deg in [0, 90, 180, 270, 360]:
            y = h - ((y_deg / 360.0) * (h - 20) + 10)
            canvas.create_line(35, y, w - 10, y, fill="#1B2433", dash=(2, 4))
            canvas.create_text(20, y, text=f"{y_deg}°", fill="#4A5D78", font=("Consolas", 7))

        # Plot Angle Line (Cyan)
        points = []
        n = len(self.reader.history_angle)
        dx = (w - 50) / max(n - 1, 1)

        for i, val in enumerate(self.reader.history_angle):
            x = 40 + i * dx
            y = h - ((val / 360.0) * (h - 20) + 10)
            points.append((x, y))

        for i in range(len(points) - 1):
            canvas.create_line(points[i][0], points[i][1], points[i+1][0], points[i+1][1], fill="#00E5FF", width=2)


# ---------------------------------------------------------------------------
# Terminal CLI Mode
# ---------------------------------------------------------------------------

def run_terminal_mode(port=None):
    print("=" * 75)
    print("  DA WINNA — MT6818 SPI ENCODER INTERACTIVE TERMINAL MONITOR")
    print("=" * 75)

    reader = STM32SerialReader(port=port)
    try:
        reader.connect()
        print(f"Connected to STM32 on {reader.port} @ 921600 Baud.\n")
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    print("Keyboard shortcuts:")
    print("  [1] Live Stream  [2] Raw Dump  [3] Benchmark  [4] Motor Spin  [ ] Stop")
    print("  [s] Speed Cycle  [p] Mode      [z] Tare Zero  [q] Quit")
    print("-" * 75)

    try:
        while True:
            while not reader.raw_log_queue.empty():
                line = reader.raw_log_queue.get_nowait()
                print(line)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        reader.disconnect()


# ---------------------------------------------------------------------------
# Main Entry Point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MT6818 SPI Encoder Test Bench")
    parser.add_argument("--port", type=str, default=None, help="Serial COM port (e.g. COM3)")
    parser.add_argument("--cli", action="store_true", help="Run in command-line terminal mode instead of GUI")
    args = parser.parse_args()

    if args.cli:
        run_terminal_mode(port=args.port)
    else:
        reader = STM32SerialReader(port=args.port)
        try:
            reader.connect()
        except Exception:
            pass
        root = tk.Tk()
        app = MT6818DashboardGUI(root, reader)

        def on_close():
            reader.disconnect()
            try:
                root.destroy()
            except Exception:
                pass
            os._exit(0)

        root.protocol("WM_DELETE_WINDOW", on_close)
        root.mainloop()
        on_close()
