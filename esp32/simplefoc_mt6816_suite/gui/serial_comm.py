"""
High-Speed Serial Communication Engine for ESP32-S3 SimpleFOC & MT6816/MT6818.
Handles asynchronous RX parsing, thread safety, auto-reconnect, and emergency stop bursts.
"""

import math
import queue
import re
import threading
import time
import serial

from .utils import find_esp32_port


class ESP32SerialReader:
    def __init__(self, port=None, baud=921600):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.connected = False
        self.thread = None
        self.raw_log_queue = queue.Queue(maxsize=3000)
        self.lock = threading.Lock()

        # Telemetry State
        self.last_angle_deg = 0.0
        self.last_angle_rad = 0.0
        self.last_raw_14bit = 0
        self.last_target_rpm = 0.0
        self.last_actual_rpm = 0.0
        self.last_uq = 0.0
        self.last_ud = 0.0
        self.last_pwm_deg = 0.0
        self.last_pwm_duty = 0.0
        self.last_pwm_freq = 0
        self.last_control_mode = "DISABLED"
        self.last_bus_mode = "4-Wire"
        self.last_lut_active = False
        self.last_mag_ok = True
        self.last_parity_ok = True
        self.last_stall_alarm = False
        self.last_stall_guard = False
        self.last_foc_khz = 0.0
        self.last_lat_avg_us = 0.0
        self.last_lat_max_us = 0.0
        self.last_spikes_count = 0
        self.calibrating = False
        self.calib_stage_name = "Ready"
        self.calib_progress_pct = 0.0
        self.calib_cooldown_sec = 0
        self.last_calib_zero_angle = None
        self.last_zero_electric_angle = 0.97555
        self.last_calib_success = False
        self.last_calib_failed = False
        self.last_calib_grade = ""
        self.last_calib_error = ""
        self.last_sensor_direction = -1
        self.msg_count = 0

        # Harmonic Calibration & Benchmark State
        self.harmonic_results = {}
        self.harmonic_bins = []
        self.bench_compare_results = {}
        self.last_correction_mode = 0
        self.spi_bench_results = []
        self.active_spi_mhz = 4.0

        # KV Test Streaming & Results State
        self.kv_test_active = False
        self.kv_points = []
        self.kv_results = {}

        # Dedicated GA Telemetry State & Sample Buffer
        self.ga_stream_active = False
        self.ga_samples = []

        self.history_time = []
        self.history_target_rpm = []
        self.history_actual_rpm = []
        self.history_angle = []
        self.start_time = time.time()

    def connect(self):
        if not self.port:
            self.port = find_esp32_port()
        if not self.port:
            raise RuntimeError("No ESP32-S3 USB Serial port found! Ensure ESP32-S3 is plugged in.")
        try:
            self.disconnect()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.2)
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
    def clear_ga_samples(self):
        with self.lock:
            self.ga_samples.clear()

    def get_ga_samples(self):
        with self.lock:
            return list(self.ga_samples)

    def send_cmd(self, cmd_str):
        if self.ser and self.ser.is_open and self.connected:
            try:
                if not cmd_str.endswith("\n"):
                    cmd_str += "\n"
                self.ser.write(cmd_str.encode("utf-8"))
            except Exception:
                self.connected = False

    def send_emergency_stop(self):
        """High-priority emergency stop burst that guarantees packet delivery even with dirty RX buffer."""
        with self.lock:
            self.kv_test_active = False
            self.calibrating = False
        if self.ser and self.ser.is_open:
            try:
                # Transmit a clean newline break followed by multiple 'x' commands
                burst = b"\n\nx\nx\nx\n"
                self.ser.write(burst)
                try:
                    self.ser.flush()
                except Exception:
                    pass
                self.connected = True
            except Exception:
                self.connected = False
        else:
            # If temporarily marked disconnected, attempt quick connect and send
            try:
                if self.port:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.2)
                    self.ser.write(b"\n\nx\nx\nx\n")
                    self.ser.flush()
                    self.connected = True
            except Exception:
                pass

    def _read_loop(self):
        buffer = ""
        while self.running:
            if not self.ser or not self.ser.is_open or not self.connected:
                time.sleep(0.5)
                try:
                    if self.ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass
                        self.ser = None
                    port = self.port or find_esp32_port()
                    if port:
                        self.port = port
                        self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.2)
                        self.ser.dtr = True
                        self.ser.rts = True
                        self.connected = True
                        buffer = ""
                except Exception:
                    continue

            try:
                data = self.ser.read(256).decode("utf-8", errors="ignore")
                if not data:
                    time.sleep(0.005)
                    continue
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip("\r")
                    if line:
                        if not line.startswith("[GA]:"):
                            try:
                                self.raw_log_queue.put_nowait(line)
                            except queue.Full:
                                try:
                                    self.raw_log_queue.get_nowait()
                                    self.raw_log_queue.put_nowait(line)
                                except Exception:
                                    pass
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
        if line.startswith("[GA]:"):
            try:
                payload = line[5:].strip()
                parts = payload.split(",")
                act = float(parts[0])
                uq = float(parts[1])
                t_now = time.time()
                with self.lock:
                    self.last_actual_rpm = act
                    self.last_uq = uq
                    self.ga_samples.append((t_now, act, uq))
                    if len(self.ga_samples) > 2000:
                        self.ga_samples = self.ga_samples[-1000:]
                    self.history_time.append(t_now - self.start_time)
                    self.history_target_rpm.append(self.last_target_rpm)
                    self.history_actual_rpm.append(act)
                    self.history_angle.append(self.last_angle_deg)
                    if len(self.history_time) > 150:
                        self.history_time.pop(0)
                        self.history_target_rpm.pop(0)
                        self.history_actual_rpm.pop(0)
                        self.history_angle.pop(0)
                    self.msg_count += 1
            except Exception:
                pass
            return

        if "SAFETY SHUTDOWN: Motor Stall" in line or "Motor stall/overload" in line or "Motor Stall / Overload" in line:
            with self.lock:
                self.last_stall_alarm = True
                self.last_control_mode = "DISABLED"
            return

        if "[KV_START]" in line:
            with self.lock:
                self.kv_test_active = True
                self.kv_points = []
                self.kv_results = {}
            return

        if "[KV_POINT]:" in line:
            try:
                m_stg = re.search(r'Stage=(\d+)/(\d+)', line)
                m_v = re.search(r'V=([+-]?\d+\.?\d*)', line)
                m_rpm = re.search(r'RPM=([+-]?\d+\.?\d*)', line)
                m_rip = re.search(r'Ripple=([+-]?\d+\.?\d*)', line)
                m_sd = re.search(r'StdDev=([+-]?\d+\.?\d*)', line)
                if m_v and m_rpm:
                    pt = {
                        "stage": int(m_stg.group(1)) if m_stg else len(self.kv_points) + 1,
                        "total_stages": int(m_stg.group(2)) if m_stg else 18,
                        "voltage": float(m_v.group(1)),
                        "rpm": float(m_rpm.group(1)),
                        "ripple": float(m_rip.group(1)) if m_rip else 0.0,
                        "std_dev": float(m_sd.group(1)) if m_sd else 0.0
                    }
                    with self.lock:
                        self.kv_points.append(pt)
                        self.kv_test_active = True
            except Exception:
                pass
            return

        if "[KV_RESULT]:" in line:
            try:
                res = {}
                for item in line.split("[KV_RESULT]:")[1].split(","):
                    if "=" in item:
                        k, v = item.strip().split("=", 1)
                        res[k.strip()] = float(v.strip())
                with self.lock:
                    self.kv_results = res
                    self.kv_test_active = False
            except Exception:
                pass
            return

        if "KV TEST ABORTED" in line:
            with self.lock:
                self.kv_test_active = False
            return

        if "STARTING CONTINUOUS SPATIAL-BINNED HARMONIC CALIBRATION" in line:
            with self.lock:
                self.calibrating = True
                self.calib_stage_name = "Phase 0: Soft DC Aligning & Settle..."
                self.calib_progress_pct = 5.0
                self.last_calib_failed = False
                self.last_calib_success = False
                self.last_calib_grade = ""
                self.last_calib_error = ""
            return

        if "Phase 1: Forward Run-Up Runway" in line:
            with self.lock:
                self.calib_stage_name = "Phase 1: CW Runway (0 -> 38 RPM)..."
                self.calib_progress_pct = 15.0
            return

        if "Phase 2: Forward Spatial Acquisition" in line:
            with self.lock:
                self.calib_stage_name = "Phase 2: Forward 128-Bin Sampling (CW)..."
                self.calib_progress_pct = 35.0
            return

        if "Phase 3a: Forward Deceleration" in line or "Phase 3b: Phase-Continuous Reversal" in line or "Phase 3c: Reverse Run-Up" in line:
            with self.lock:
                self.calib_stage_name = "Phase 3: Continuous Phase Reversal..."
                self.calib_progress_pct = 55.0
            return

        if "Phase 4: Reverse Spatial Acquisition" in line:
            with self.lock:
                self.calib_stage_name = "Phase 4: Reverse 128-Bin Sampling (CCW)..."
                self.calib_progress_pct = 75.0
            return

        if "Phase 5: Point-by-Point Cancellation" in line:
            with self.lock:
                self.calib_stage_name = "Phase 5: Drag Cancellation & A0 Nulling..."
                self.calib_progress_pct = 88.0
            return

        if "Phase 6: Direct Discrete Fourier Transform" in line:
            with self.lock:
                self.calib_stage_name = "Phase 6: Direct DFT Extraction (Orders 1-4)..."
                self.calib_progress_pct = 94.0
            return

        if "[HARMONIC_RESULTS]:" in line:
            try:
                res = {}
                for item in line.split("[HARMONIC_RESULTS]:")[1].split(","):
                    if "=" in item:
                        k, v = item.strip().split("=", 1)
                        res[k.strip()] = float(v.strip())
                with self.lock:
                    self.harmonic_results = res
                    self.last_correction_mode = 2
                    self.last_lut_active = True
            except Exception:
                pass
            return

        if "[HARMONIC_BINS]:" in line:
            try:
                raw_bins = line.split("[HARMONIC_BINS]:")[1].strip()
                pts = [float(x.strip()) for x in raw_bins.split(",") if x.strip()]
                with self.lock:
                    self.harmonic_bins = pts
            except Exception:
                pass
            return

        if "[SPI_BENCH_START]" in line:
            with self.lock:
                self.spi_bench_results = []
            return

        if "[SPI_BENCH_RESULT]:" in line:
            try:
                res = {}
                for item in line.split("[SPI_BENCH_RESULT]:")[1].split(","):
                    if "=" in item:
                        k, v = item.strip().split("=", 1)
                        try:
                            res[k.strip()] = float(v.strip())
                        except ValueError:
                            res[k.strip()] = v.strip()
                with self.lock:
                    self.spi_bench_results.append(res)
            except Exception:
                pass
            return

        if "Active MT6816 SPI Clock set to" in line or "Active MT6816 SPI Master Clock:" in line:
            m_spi = re.search(r'([\d\.]+)\s*MHz', line)
            if m_spi:
                with self.lock:
                    self.active_spi_mhz = float(m_spi.group(1))
            return

        if "[BENCH_COMPARE_RESULT]:" in line:
            try:
                res = {}
                for item in line.split("[BENCH_COMPARE_RESULT]:")[1].split(","):
                    if "=" in item:
                        k, v = item.strip().split("=", 1)
                        try:
                            res[k.strip()] = float(v.strip())
                        except ValueError:
                            res[k.strip()] = v.strip()
                with self.lock:
                    if isinstance(self.bench_compare_results, list):
                        self.bench_compare_results = {}
                    m_id = int(res.get("Mode", len(self.bench_compare_results)))
                    self.bench_compare_results[m_id] = res
            except Exception:
                pass
            return

        if "POLE-SLIP ABORT" in line:
            with self.lock:
                self.last_calib_failed = True
                self.calibrating = False
                self.last_calib_error = "Pole-Slip Abort (Torque limit exceeded)"
                self.calib_stage_name = "❌ Pole-Slip Stall Abort"
            return

        if "Sensor Correction Mode set to:" in line or "Sensor Correction Mode activated: MODE" in line:
            m_m = re.search(r'MODE\s*(\d)', line, re.IGNORECASE) or re.search(r'set to:\s*(\d)', line, re.IGNORECASE)
            if m_m:
                with self.lock:
                    self.last_correction_mode = int(m_m.group(1))
            return

        if "STARTING 128-POINT DUAL-PASS NON-LINEARITY LUT CALIBRATION" in line:
            with self.lock:
                self.calibrating = True
                self.calib_stage_name = "128-Point LUT: Settling start..."
                self.calib_progress_pct = 5.0
                self.last_calib_failed = False
                self.last_calib_success = False
                self.last_calib_grade = ""
                self.last_calib_error = ""
            return

        if "Forward Sweep:" in line:
            m_fwd = re.search(r'Forward Sweep:\s*(\d+)%', line)
            if m_fwd:
                pct = int(m_fwd.group(1))
                with self.lock:
                    self.calib_stage_name = f"LUT Pass 1: Forward Sweep ({pct}%)..."
                    self.calib_progress_pct = 5.0 + 0.35 * pct
            return

        if "THERMAL COOL-DOWN PAUSE" in line:
            with self.lock:
                self.calib_stage_name = "❄️ Thermal Cool-Down: De-energizing coils..."
            return

        if "Coil cool-down:" in line:
            m_cd = re.search(r'Coil cool-down:\s*(\d+)s', line)
            if m_cd:
                sec = int(m_cd.group(1))
                with self.lock:
                    self.calib_cooldown_sec = sec
                    self.calib_stage_name = f"❄️ Thermal Cool-Down ({sec}s remaining)..."
            return

        if "Reverse Sweep:" in line:
            m_rev = re.search(r'Reverse Sweep:\s*(\d+)%', line)
            if m_rev:
                pct = int(m_rev.group(1))
                with self.lock:
                    self.calib_stage_name = f"LUT Pass 2: Reverse Sweep ({pct}%)..."
                    self.calib_progress_pct = 45.0 + 0.35 * pct
            return

        if "128-Point LUT ACTIVE" in line:
            with self.lock:
                self.last_lut_active = True
                self.calib_stage_name = "✅ 128-Point LUT Calibrated & Active!"
                self.calib_progress_pct = 82.0
            return

        if "LUT REJECTED" in line:
            with self.lock:
                self.last_lut_active = False
                self.last_calib_failed = True
                self.calibrating = False
                self.last_calib_error = "LUT Rejected: Anomalies/Pole-Slip Detected"
                self.calib_stage_name = "❌ LUT Rejected (Anomalies/Pole-Slip)"
            return

        if "STARTING COMPREHENSIVE MULTI-POINT ELECTRICAL ZERO CALIBRATION" in line:
            with self.lock:
                self.calibrating = True
                self.calib_stage_name = "Scanning 11 Electrical Poles..."
                self.calib_progress_pct = max(self.calib_progress_pct, 82.0)
            return

        if "Pole #" in line or ("#" in line and "deg" in line and "rad" in line):
            m_pole = re.search(r'#(\d+)\s+\|\s*([0-9\.]+)\s*deg', line)
            if m_pole:
                p_idx = int(m_pole.group(1))
                with self.lock:
                    self.calib_stage_name = f"11-Pole Sweep: Pole #{p_idx}/11..."
                    self.calib_progress_pct = 82.0 + (p_idx / 11.0) * 16.0

        if "motor.zero_electric_angle =" in line:
            m = re.search(r'motor\.zero_electric_angle\s*=\s*([0-9]+\.[0-9]+)', line)
            if m:
                with self.lock:
                    self.last_calib_zero_angle = float(m.group(1))
                    self.last_zero_electric_angle = float(m.group(1))
                    self.last_calib_success = True
                    self.last_calib_failed = False
                    self.calibrating = False
                    self.calib_progress_pct = 100.0

        if "motor.sensor_direction" in line:
            m_dir = re.search(r'Direction::(CW|CCW)', line)
            if m_dir:
                with self.lock:
                    self.last_sensor_direction = 1 if m_dir.group(1) == "CW" else -1

        if any(f"GRADE {g}" in line for g in ["A", "B", "C", "D", "E"]):
            with self.lock:
                if "GRADE A" in line:
                    self.last_calib_grade = "GRADE A (Exceptional)"
                elif "GRADE B" in line:
                    self.last_calib_grade = "GRADE B (Excellent)"
                elif "GRADE C" in line:
                    self.last_calib_grade = "GRADE C (Good)"
                elif "GRADE D" in line:
                    self.last_calib_grade = "GRADE D (Fair)"
                elif "GRADE E" in line:
                    self.last_calib_grade = "GRADE E (Marginal)"

        if "CALIBRATION REJECTED" in line or "GRADE F" in line:
            with self.lock:
                self.last_calib_failed = True
                self.calibrating = False
                self.last_calib_grade = "GRADE F (FAILED)"
                self.last_calib_error = "11-Pole Alignment Consistency Failed (>25 deg elec)"
                self.calib_stage_name = "❌ Calibration Rejected by Gate"

        if "FAILED! ERROR: Motor failed to move" in line or "Motor failed to move" in line:
            with self.lock:
                self.last_calib_failed = True
                self.calibrating = False
                self.last_calib_error = "Motor failed to move (check power/wiring)"
                self.calib_stage_name = "❌ Motor Failed to Move"

        if "Updated Base Electrical Zero Angle:" in line:
            m = re.search(r'([0-9]+\.[0-9]+)\s*rad', line)
            if m:
                with self.lock:
                    self.last_zero_electric_angle = float(m.group(1))

        if "Driver ENABLED" in line:
            with self.lock:
                self.last_control_mode = "ENABLED"
        elif "Driver DISABLED" in line or "MOTOR STOPPED" in line:
            with self.lock:
                self.last_control_mode = "DISABLED"
        elif "FOC Calibration Complete" in line or "CERTIFIED ELECTRICAL ALIGNMENT RESULTS" in line:
            with self.lock:
                self.last_control_mode = "CLOSED-LOOP FOC"
                self.calibrating = False
        elif "OPEN-LOOP Velocity Mode" in line:
            with self.lock:
                self.last_control_mode = "OPEN-LOOP"
        elif "Stall Watchdog: ENABLED" in line:
            with self.lock:
                self.last_stall_guard = True
        elif "Stall Watchdog: DISABLED" in line:
            with self.lock:
                self.last_stall_guard = False
        elif "3-Wire Half-Duplex" in line:
            with self.lock:
                self.last_bus_mode = "3-Wire"
        elif "4-Wire Full Hardware" in line:
            with self.lock:
                self.last_bus_mode = "4-Wire"

        if "Ctrl:" in line and "Angle:" in line:
            try:
                m_ang = re.search(r'Angle:\s*([+-]?\d+\.?\d*)\s*deg', line)
                angle_deg = float(m_ang.group(1)) if m_ang else self.last_angle_deg

                m_tgt = re.search(r'Target:\s*([+-]?\d+\.?\d*)\s*(RPM|V)', line)
                target_val = float(m_tgt.group(1)) if m_tgt else self.last_target_rpm

                m_act = re.search(r'Actual:\s*([+-]?\d+\.?\d*)\s*RPM', line)
                actual_rpm = float(m_act.group(1)) if m_act else self.last_actual_rpm

                m_raw = re.search(r'Raw:\s*(\d+)/16383', line)
                raw_count = int(m_raw.group(1)) if m_raw else int((angle_deg / 360.0) * 16384)

                m_uq = re.search(r'Uq:\s*([+-]?\d+\.?\d*)V', line)
                uq = float(m_uq.group(1)) if m_uq else self.last_uq

                m_ud = re.search(r'Ud:\s*([+-]?\d+\.?\d*)V', line)
                ud = float(m_ud.group(1)) if m_ud else self.last_ud

                ctrl_mode = "CLOSED-LOOP FOC"
                if "Ctrl: DISBL" in line:
                    ctrl_mode = "DISABLED"
                elif "Ctrl: KV-TEST" in line:
                    ctrl_mode = "KV-TEST (VOLT)"
                elif "Ctrl: OPEN" in line:
                    ctrl_mode = "OPEN-LOOP"
                elif "Ctrl: VOLT" in line:
                    ctrl_mode = "CLOSED-LOOP VOLTAGE"
                elif "Ctrl: CLOSE" in line:
                    ctrl_mode = "CLOSED-LOOP VELOCITY"

                bus_mode = "3-Wire" if "Mode: 3-Wire" in line else "4-Wire"
                lut_active = "(FOURIER:128)" in line or "128-Point LUT" in line or "128-PT LUT" in line or "(HARMONIC)" in line
                if "(HARMONIC)" in line:
                    self.last_correction_mode = 2
                elif "(FOURIER:128)" in line:
                    self.last_correction_mode = 1
                elif "(RAW)" in line:
                    self.last_correction_mode = 0
                mag_ok = "[ OK ]" in line or "[MAGNET OK]" in line
                parity_ok = "[PASS]" in line or "Parity: [ PASS ]" in line

                m_foc = re.search(r'FOC:\s*([+-]?\d+\.?\d*)\s*kHz', line)
                foc_khz = float(m_foc.group(1)) if m_foc else 0.0

                m_lat = re.search(r'Lat:\s*([+-]?\d+\.?\d*)/([+-]?\d+\.?\d*)\s*us', line)
                lat_avg = float(m_lat.group(1)) if m_lat else 0.0
                lat_max = float(m_lat.group(2)) if m_lat else 0.0

                m_spk = re.search(r'Spk:\s*(\d+)', line)
                spk_count = int(m_spk.group(1)) if m_spk else 0

                if "DISBL" in ctrl_mode:
                    pass
                else:
                    self.last_stall_alarm = False

                t_now = time.time() - self.start_time
                with self.lock:
                    self.last_angle_deg = angle_deg
                    self.last_angle_rad = (angle_deg / 180.0) * math.pi
                    self.last_raw_14bit = raw_count
                    self.last_target_rpm = target_val
                    self.last_actual_rpm = actual_rpm
                    self.last_uq = uq
                    self.last_ud = ud
                    self.last_control_mode = ctrl_mode
                    self.last_bus_mode = bus_mode
                    self.last_lut_active = lut_active
                    self.last_mag_ok = mag_ok
                    self.last_parity_ok = parity_ok
                    if foc_khz > 0.0:
                        self.last_foc_khz = foc_khz
                    if lat_avg > 0.0:
                        self.last_lat_avg_us = lat_avg
                        self.last_lat_max_us = lat_max
                    self.last_spikes_count = spk_count
                    self.msg_count += 1

                    self.history_time.append(t_now)
                    self.history_target_rpm.append(target_val)
                    self.history_actual_rpm.append(actual_rpm)
                    self.history_angle.append(angle_deg)
                    if len(self.history_time) > 150:
                        self.history_time.pop(0)
                        self.history_target_rpm.pop(0)
                        self.history_actual_rpm.pop(0)
                        self.history_angle.pop(0)
            except Exception:
                pass
