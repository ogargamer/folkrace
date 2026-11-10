"""
serial_comm.py
==============
Thread-safe serial reader and command engine for STM32G474 20 kHz Dual FOC Controller.
Demuxes 26-byte binary telemetry frames (b"\\xAA\\x55" + XOR checksum) and UTF-8 ASCII log lines,
tracks rolling velocity oscilloscope history, and captures JSON configuration exports.
"""

import struct
import threading
import queue
import time
import collections
import math
import serial
import serial.tools.list_ports

# 26-Byte Binary Telemetry Packet Format:
# < (little-endian), 2s (header), H (uint16), 3h (int16), B (uint8), H, 3h, B, H, 4B
PACKET_FMT = "<2sHhhhBHhhhBHBBBB"
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # Exactly 26 bytes
SYNC_HEADER = b"\xAA\x55"


def wrap_angle_rad(angle):
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def normalize_angle_rad(angle):
    a = math.fmod(angle, 2.0 * math.pi)
    if a < 0.0:
        a += 2.0 * math.pi
    return a


def analyze_harmonic_lut(mid_angles, dir_sign=1):
    n = len(mid_angles)
    if n != 128:
        return f"⚠️ [LUT ANALYSIS] Expected 128 samples, got {n}\n"

    # 1. 3-point circular median filter
    filtered = [0.0] * n
    for i in range(n):
        a_prev = mid_angles[(i - 1) % n]
        a_curr = mid_angles[i]
        a_next = mid_angles[(i + 1) % n]
        d_prev = wrap_angle_rad(a_prev - a_curr)
        d_next = wrap_angle_rad(a_next - a_curr)
        d_med = sorted([d_prev, 0.0, d_next])[1]
        filtered[i] = normalize_angle_rad(a_curr + d_med)

    # 2. Linear expected angles & midpoint errors
    step_mech = (2.0 * math.pi) / float(n)
    start_mid = filtered[0]
    errors = [0.0] * n
    for i in range(n):
        exp = normalize_angle_rad(start_mid + float(dir_sign) * (float(i) * step_mech))
        errors[i] = wrap_angle_rad(filtered[i] - exp)

    # 3. Fourier harmonic decomposition (H1, H2, H4)
    harmonics = {}
    for h in [1, 2, 4]:
        c_sum = 0.0
        s_sum = 0.0
        for i in range(n):
            theta = 2.0 * math.pi * float(h * i) / float(n)
            c_sum += errors[i] * math.cos(theta)
            s_sum += errors[i] * math.sin(theta)
        c_k = (2.0 / float(n)) * c_sum
        s_k = (2.0 / float(n)) * s_sum
        harmonics[h] = math.sqrt(c_k * c_k + s_k * s_k)

    pp_deg = (max(errors) - min(errors)) * 57.2957795
    rms_deg = math.sqrt(sum(e * e for e in errors) / float(n)) * 57.2957795

    lines = [
        "\n--------------------------------------------------------------------------------",
        "🔬 [GUI] 128-POINT HIGH-PRECISION HARMONIC LUT ANALYSIS REPORT",
        "--------------------------------------------------------------------------------",
        f"  - Peak-to-Peak Periodic Error : {pp_deg:.3f} deg",
        f"  - RMS Deviation               : {rms_deg:.3f} deg",
        f"  - Harmonic 1 (Eccentricity)   : {harmonics[1] * 57.2957795:.4f} deg ({harmonics[1]:.5f} rad)",
        f"  - Harmonic 2 (Ellipticity)    : {harmonics[2] * 57.2957795:.4f} deg ({harmonics[2]:.5f} rad)",
        f"  - Harmonic 4 (Pole-Coupling)  : {harmonics[4] * 57.2957795:.4f} deg ({harmonics[4]:.5f} rad)",
        "  - 3-Point Circular Median Filter Applied: 128 Points Fitted & Verified",
        "--------------------------------------------------------------------------------\n"
    ]
    return "\n".join(lines)


def analyze_electrical_poles(motor_id, dir_str, pole_data_str):
    num_poles = 11
    dir_sign = 1 if dir_str == "CW" else -1
    pairs = [p for p in pole_data_str.strip().split(";") if p.strip()]
    if len(pairs) < num_poles:
        return f"⚠️ [ELEC POLES] Expected 11 pole pairs, got {len(pairs)}\n", 0.0, "UNKNOWN", dir_str

    fwd_mechs = []
    rev_mechs = []
    fwd_zeros = []
    rev_zeros = []
    true_zeros = []
    hysteresis_degs = []

    sum_sin = 0.0
    sum_cos = 0.0

    for p in range(num_poles):
        parts = pairs[p].split(",")
        f_m = float(parts[0])
        r_m = float(parts[1])
        fwd_mechs.append(f_m)
        rev_mechs.append(r_m)

        f_z = normalize_angle_rad(float(dir_sign * num_poles) * f_m)
        r_z = normalize_angle_rad(float(dir_sign * num_poles) * r_m)
        fwd_zeros.append(f_z)
        rev_zeros.append(r_z)

        t_z = normalize_angle_rad(math.atan2(math.sin(f_z) + math.sin(r_z),
                                             math.cos(f_z) + math.cos(r_z)))
        true_zeros.append(t_z)
        sum_sin += math.sin(t_z)
        sum_cos += math.cos(t_z)

        h_deg = abs(wrap_angle_rad(f_z - r_z)) * 57.2957795
        hysteresis_degs.append(h_deg)

    mean_zero = normalize_angle_rad(math.atan2(sum_sin / float(num_poles),
                                               sum_cos / float(num_poles)))

    sq_diff = 0.0
    for z in true_zeros:
        diff = wrap_angle_rad(z - mean_zero)
        sq_diff += diff * diff
    std_dev_rad = math.sqrt(sq_diff / float(num_poles))
    std_dev_deg = std_dev_rad * 57.2957795
    max_hyst_deg = max(hysteresis_degs)

    kt_eff = math.cos(std_dev_rad) * 100.0
    d_leak = math.sin(std_dev_rad) * 100.0

    if std_dev_rad < 0.05:
        grade = "GRADE A (EXCELLENT)"
    elif std_dev_rad < 0.10:
        grade = "GRADE B (GOOD)"
    elif std_dev_rad < 0.18:
        grade = "GRADE C (FAIR)"
    elif std_dev_rad < 0.30:
        grade = "GRADE D (NOISY)"
    else:
        grade = "GRADE E (POOR)"

    lines = [
        "\n================================================================================",
        f"🏆 COMPREHENSIVE 11-POLE ELECTRICAL ALIGNMENT REPORT (MOTOR {motor_id})",
        "================================================================================",
        f"  Commutation Direction : {dir_str} ({dir_sign:+d})",
        f"  Mean Electric Zero    : {mean_zero:.5f} rad ({mean_zero * 57.2957795:.2f} deg)",
        f"  Pole Dispersion (Std) : {std_dev_deg:.3f} deg ({std_dev_rad:.4f} rad)",
        f"  Max Hysteresis        : {max_hyst_deg:.2f} el.deg",
        f"  Kt Efficiency Factor  : {kt_eff:.2f}% (d-axis leakage: {d_leak:.2f}%)",
        f"  Certified Quality     : {grade}",
        "--------------------------------------------------------------------------------",
        "  Pole | Fwd Mech (deg) | Rev Mech (deg) | True Zero (rad) | Hysteresis (el.deg)",
        "--------------------------------------------------------------------------------"
    ]
    for p in range(num_poles):
        f_deg = fwd_mechs[p] * 57.2957795
        r_deg = rev_mechs[p] * 57.2957795
        lines.append(f"  {p:4d} | {f_deg:14.2f} | {r_deg:14.2f} | {true_zeros[p]:15.4f} | {hysteresis_degs[p]:17.2f}")
    lines.append("================================================================================\n")

    return "\n".join(lines), mean_zero, grade, dir_str


def format_profiler(prof_str):
    parts = prof_str.replace("[PROF]:", "").strip().split(":")
    data = {}
    for p in parts:
        if "=" in p:
            k, v = p.split("=", 1)
            data[k] = v

    try:
        sysclk = float(data.get("SYSCLK", 170000000))
    except Exception:
        sysclk = 170000000.0
    us_scale = 1.0 / (sysclk / 1000000.0)

    def to_cyc_us(cyc_str):
        try:
            cyc = float(cyc_str)
            return f"{int(cyc):5d} cyc ({cyc * us_scale:5.2f} us)"
        except Exception:
            return "N/A"

    lines = [
        "\n--- [REAL-TIME LATENCY PROFILER BREAKDOWN] ---",
        f"SystemCoreClock       : {int(sysclk)} Hz",
        f"PCLK1 (APB1)          : {data.get('PCLK1', 'N/A')} Hz (SPI2)",
        f"PCLK2 (APB2)          : {data.get('PCLK2', 'N/A')} Hz (SPI1)",
        f"SPI1 CR1/CR2          : {data.get('SPI1', 'N/A')}",
        f"SPI2 CR1/CR2          : {data.get('SPI2', 'N/A')}",
        f"TIM2 CCMR1/CCMR2/CR1  : {data.get('TIM2', 'N/A')}",
        f"TIM3 CCMR1/CCMR2/CR1  : {data.get('TIM3', 'N/A')}",
        f"Parallel SPI     Last : {to_cyc_us(data.get('SPI_LAST', '0'))} | Peak: {to_cyc_us(data.get('SPI_MAX', '0'))}",
        f"Sensor Update    Last : {to_cyc_us(data.get('UPD_LAST', '0'))} | Peak: {to_cyc_us(data.get('UPD_MAX', '0'))}",
        f"Velocity PID     Last : {to_cyc_us(data.get('MOVE_LAST', '0'))} | Peak: {to_cyc_us(data.get('MOVE_MAX', '0'))}",
        f"M1 fastLoopFOC   Last : {to_cyc_us(data.get('M1_LAST', '0'))} | Peak: {to_cyc_us(data.get('M1_MAX', '0'))}",
        f"M2 fastLoopFOC   Last : {to_cyc_us(data.get('M2_LAST', '0'))} | Peak: {to_cyc_us(data.get('M2_MAX', '0'))}",
        f"Total ISR Exec   Last : {to_cyc_us(data.get('TOT_LAST', '0'))} | Peak: {to_cyc_us(data.get('TOT_MAX', '0'))} | All-Time Peak: {to_cyc_us(data.get('ALL_PEAK', '0'))}",
        f"20kHz Timer Spacing   : {to_cyc_us(data.get('PER_LAST', '8500'))} (50.00 us target) | Jitter: {data.get('JIT_MAX', '0.00')} us (Min: {data.get('PER_MIN', '0.00')} us, Max: {data.get('PER_MAX', '0.00')} us)",
        f"Jitter Spikes Count   : {data.get('SPIKES', '0')}",
        "----------------------------------------------\n"
    ]
    return "\n".join(lines)


def format_encoder_diag(enc_str):
    parts = enc_str.replace("[ENC]:", "").strip().split(":")
    m = parts[0] if len(parts) > 0 else "M1"
    data = {}
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            data[k] = v

    bus_name = "Encoder 3 (PA15/SPI1 - M1)" if m == "M1" else "Encoder 4 (PB11/SPI2 - M2)"
    line = f"{bus_name}: Raw14={data.get('RAW', '0')} | Angle={data.get('DEG', '0.00')} deg | Parity={data.get('PARITY', 'OK')} | Mag={data.get('MAG', 'OK')} | TotalReads={data.get('READS', '0')} | Retries={data.get('RETRIES', '0')} | ParityErrs={data.get('ERRS', '0')}\n"
    return line


def format_friendly_error(err_str):
    if "[ERR]:NO_MAG" in err_str:
        mid = "1" if "M1" in err_str else "2"
        return f"\n❌ [ENCODER FAULT] Motor {mid}: MT6816 magnet not detected or magnetic field too weak!\n   💡 Check magnet-to-sensor gap (~1-2mm) and physical alignment.\n"
    elif "[ERR]:PARITY" in err_str:
        parts = err_str.split(":")
        mid = parts[2].replace("M", "") if len(parts) > 2 else "1"
        err_count = parts[3] if len(parts) > 3 else "0"
        return f"\n❌ [SPI BUS FAULT] Motor {mid}: MT6816 SPI parity burst error ({err_count} errors)!\n   💡 Inspect SPI wiring, MOSI/MISO lines, CS pull-up, and shared ground.\n"
    elif "[ERR]:MOTION_CHECK" in err_str:
        parts = err_str.split(":")
        mid = "1" if "M1" in err_str else "2"
        moved = "0.0"
        exp = "0.0"
        for p in parts:
            if p.startswith("moved="): moved = p.split("=")[1]
            elif p.startswith("expected="): exp = p.split("=")[1]
        return f"\n❌ [ROTOR MOTION CHECK FAILED] Motor {mid}: Rotor failed to move during alignment check!\n   - Actual movement: {moved} deg (Expected >= {exp} deg)\n   💡 Increase alignment voltage (e.g. 4.0V - 4.2V) or check for mechanical binding.\n"
    elif "[STALL_FOLDBACK]" in err_str:
        mid = "1" if "M1" in err_str else "2"
        return f"\n⚠️ [STALL GUARD] Motor {mid}: Thermal I²·t threshold exceeded! Voltage clamped to safe foldback.\n"
    elif "[STALL_FAULT]" in err_str:
        mid = "1" if "M1" in err_str else "2"
        return f"\n🚨 [SAFETY SHUTDOWN] Motor {mid}: Latched Stall Fault (Obstruction Unresolved)!\n   💡 Clear the physical blockage and press 'K' or Reset Fault to resume.\n"
    elif "[STALL_CLEAR]" in err_str:
        mid = "1" if "M1" in err_str else "2"
        return f"\n✅ [STALL RESTORED] Motor {mid}: Normal rotor motion restored. Voltage limit reset.\n"
    elif "[CALIB_BOOST]" in err_str:
        mid = "1" if "M1" in err_str else "2"
        return f"\n⚡ [CALIBRATION] Motor {mid}: Stiction detected. Retrying with boost voltage (4.2V)...\n"
    elif "[CALIB_CERTIFIED]" in err_str:
        mid = "1" if "M1" in err_str else "2"
        return f"\n🎉 [CALIBRATION CERTIFIED] Motor {mid}: Calibration completed and verified.\n"
    return err_str + "\n"


class STM32DualSerial:
    def __init__(self, port="COM3", baud=921600, on_telemetry=None, on_log=None, on_config_json=None):
        self.port = port
        self.baud = baud
        self.on_telemetry = on_telemetry
        self.on_log = on_log
        self.on_config_json = on_config_json

        self.ser = None
        self.running = False
        self.thread = None
        self.cmd_queue = queue.Queue()
        self.lock = threading.Lock()

        # Calibration & Diagnostics Buffers
        self._sweep_mid_chunks = {1: {}, 2: {}}
        self._enc_diag_buffer = {}

        # Rolling history for real-time Velocity Tracking Oscilloscope (last 240 points)
        self.max_history = 240
        self.invert_direction = True  # Default True: inverts sign so physical CW is positive/clockwise
        self.history_time = collections.deque(maxlen=self.max_history)
        self.history_target_rpm = collections.deque(maxlen=self.max_history)
        self.history_actual_rpm = collections.deque(maxlen=self.max_history)
        self.history_uq = collections.deque(maxlen=self.max_history)
        self.history_m2_target_rpm = collections.deque(maxlen=self.max_history)
        self.history_m2_actual_rpm = collections.deque(maxlen=self.max_history)
        self.history_m2_uq = collections.deque(maxlen=self.max_history)
        self._last_hist_push = 0.0

        # JSON Config Export Capture State
        self._capturing_json = False
        self._json_lines = []
        self.last_exported_json = None

        # Telemetry & Calibration State Dict
        self.state = {
            "m1_raw_angle": 0,
            "m1_actual_rpm": 0.0,
            "m1_target_rpm": 0.0,
            "m1_uq_v": 0.0,
            "m1_enabled": False,
            "m1_lut_active": False,
            "m1_mag_ok": True,
            "m1_parity_ok": True,
            "m1_stalled": False,
            "m1_fault": False,
            "m1_calib": False,
            "m1_zero_angle": 0.0,
            "m1_direction": "CW",
            "m1_lag_comp_us": 35.0,
            "m1_calib_grade": "NOT CALIBRATED",
            "m1_calib_stage": "Idle",

            "m2_raw_angle": 0,
            "m2_actual_rpm": 0.0,
            "m2_target_rpm": 0.0,
            "m2_uq_v": 0.0,
            "m2_enabled": False,
            "m2_lut_active": False,
            "m2_mag_ok": True,
            "m2_parity_ok": True,
            "m2_stalled": False,
            "m2_fault": False,
            "m2_calib": False,
            "m2_zero_angle": 0.0,
            "m2_direction": "CW",
            "m2_lag_comp_us": 35.0,
            "m2_calib_grade": "NOT CALIBRATED",
            "m2_calib_stage": "Idle",

            "calib_motor": 1,
            "foc_khz": 20.0,
            "isr_lat_us": 0,
            "isr_peak_hold_us": 0,
            "spikes_count": 0,
            "calib_progress": 0,
            "connected": False,
            "last_packet_time": 0.0
        }

    @staticmethod
    def list_ports():
        ports = []
        for p in serial.tools.list_ports.comports():
            desc = p.description or ""
            ports.append((p.device, desc))
        return ports

    def connect(self, port=None, baud=None):
        if port:
            self.port = port
        if baud:
            self.baud = baud

        self.disconnect()
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.05,
                write_timeout=1.0
            )
            try:
                self.ser.dtr = True
                self.ser.rts = True
            except Exception:
                pass
            self.running = True
            self.state["connected"] = True
            self.state["m1_parity_ok"] = True
            self.state["m2_parity_ok"] = True
            self.thread = threading.Thread(target=self._io_worker, daemon=True)
            self.thread.start()
            if self.on_log:
                self.on_log(f"[SYS] Connected to {self.port} @ {self.baud} baud\n")
            return True, "Connected"
        except Exception as ex:
            self.state["connected"] = False
            self.state["m1_parity_ok"] = True
            self.state["m2_parity_ok"] = True
            return False, str(ex)

    def disconnect(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.state["connected"] = False
        self.state["m1_parity_ok"] = True
        self.state["m2_parity_ok"] = True
        if self.on_log:
            self.on_log(f"[SYS] Disconnected from {self.port}\n")

    def set_direction_inverted(self, inverted: bool):
        """Set whether sign is inverted so physical CW rotation yields positive RPM/angle."""
        self.invert_direction = bool(inverted)

    def send_command(self, cmd_str):
        if not cmd_str.endswith("\n"):
            cmd_str += "\n"
        self.cmd_queue.put(cmd_str)

    def reset_peak_hold(self):
        with self.lock:
            self.state["isr_peak_hold_us"] = self.state["isr_lat_us"]
        self.send_command("PROF")

    def send_emergency_stop(self):
        """Immediately flush pending commands and send hardware emergency stop burst."""
        try:
            while not self.cmd_queue.empty():
                self.cmd_queue.get_nowait()
        except Exception:
            pass
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"\x1b\nSTOP\n")
                self.ser.flush()
            except Exception:
                pass
        self.send_command("STOP")

    def _io_worker(self):
        buf = bytearray()
        ascii_accum = ""
        while self.running and self.ser and self.ser.is_open:
            # 1. Handle outgoing commands
            try:
                while not self.cmd_queue.empty():
                    out_cmd = self.cmd_queue.get_nowait()
                    self.ser.write(out_cmd.encode("utf-8"))
            except Exception as ex:
                if self.on_log:
                    self.on_log(f"[TX ERR] {ex}\n")

            # 2. Handle incoming stream
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    time.sleep(0.002)
                    continue
                buf.extend(chunk)

                # Process buffer: scan for binary telemetry or ASCII log lines
                while buf:
                    idx = buf.find(SYNC_HEADER)
                    if idx == -1:
                        # No sync header found; decode all complete lines as ASCII
                        line_end = buf.find(b"\n")
                        if line_end != -1:
                            line_bytes = buf[:line_end + 1]
                            buf = buf[line_end + 1:]
                            text = line_bytes.decode("utf-8", errors="replace")
                            ascii_accum = self._process_ascii_chunk(ascii_accum + text)
                            continue
                        elif len(buf) > 512:
                            text = buf[:256].decode("utf-8", errors="replace")
                            buf = buf[256:]
                            ascii_accum = self._process_ascii_chunk(ascii_accum + text)
                            continue
                        break
                    elif idx > 0:
                        # Log text preceding the sync header
                        prefix = buf[:idx]
                        buf = buf[idx:]
                        text = prefix.decode("utf-8", errors="replace")
                        ascii_accum = self._process_ascii_chunk(ascii_accum + text)
                        continue

                    if len(buf) < PACKET_SIZE:
                        break

                    # Verify XOR checksum over bytes 2 through 24
                    calc_chk = 0
                    for b in buf[2:PACKET_SIZE - 1]:
                        calc_chk ^= b

                    if calc_chk == buf[PACKET_SIZE - 1]:
                        payload = struct.unpack(PACKET_FMT, buf[:PACKET_SIZE])
                        self._handle_telemetry_payload(payload)
                        buf = buf[PACKET_SIZE:]
                    else:
                        # Checksum failure or false sync word: slide by 1 byte
                        buf = buf[1:]

            except Exception as ex:
                if self.running and self.on_log:
                    self.on_log(f"[RX ERR] {ex}\n")
                time.sleep(0.01)

    def _process_ascii_chunk(self, text_buf):
        while "\n" in text_buf:
            line, text_buf = text_buf.split("\n", 1)
            clean_line = line.strip()
            expanded = self._parse_ascii_line(clean_line)
            if self.on_log:
                if expanded is not None:
                    if expanded != "":
                        self.on_log(expanded)
                else:
                    self.on_log(line + "\n")
        return text_buf

    def _parse_ascii_line(self, s):
        if not s:
            return None

        # 1. JSON Config Export Capture
        if s == "[CONFIG_START]":
            self._capturing_json = True
            self._json_lines = []
            return ""
        elif s == "[CONFIG_END]":
            if self._capturing_json:
                self._capturing_json = False
                json_str = "\n".join(self._json_lines)
                self.last_exported_json = json_str
                if self.on_config_json:
                    self.on_config_json(json_str)
            return "\n📋 [CONFIG] Exported JSON Configuration received and updated.\n"
        elif self._capturing_json:
            self._json_lines.append(s)
            return ""

        # 2. Calibration Stage & Result Parsing
        with self.lock:
            # Keep alive during calibration even when binary telemetry is muted
            if self.state.get("m1_calib", False) or self.state.get("m2_calib", False):
                self.state["last_packet_time"] = time.time()

            # Handle 128-point LUT Midpoint Sweep Streaming
            if s.startswith("[SWEEP_START]:M"):
                try:
                    cal_m = int(s.split("[SWEEP_START]:M")[1][0])
                    self._sweep_mid_chunks[cal_m] = {}
                    return f"\n📊 [LUT SWEEP] Receptive buffer open for 128 midpoints (Motor {cal_m})...\n"
                except Exception:
                    return None

            elif s.startswith("[SWEEP_MID]:M"):
                try:
                    parts = s.split(":")
                    cal_m = int(parts[1][1])
                    chunk_idx = int(parts[2])
                    floats = [float(x) for x in parts[3].split(",") if x.strip()]
                    self._sweep_mid_chunks[cal_m][chunk_idx] = floats
                    return ""  # Suppress raw float lines
                except Exception:
                    return ""

            elif s.startswith("[SWEEP_DONE]:M"):
                try:
                    cal_m = int(s.split("[SWEEP_DONE]:M")[1][0])
                    chunks = self._sweep_mid_chunks.get(cal_m, {})
                    if len(chunks) == 8:
                        all_mids = []
                        for ch in range(8):
                            all_mids.extend(chunks[ch])
                        dir_sign = 1 if self.state.get(f"m{cal_m}_direction", "CW") == "CW" else -1
                        rep = analyze_harmonic_lut(all_mids, dir_sign)
                        self.state[f"m{cal_m}_lut_active"] = True
                        return rep
                    else:
                        return f"⚠️ [LUT SWEEP] Incomplete chunks ({len(chunks)}/8 chunks received)\n"
                except Exception as ex:
                    return f"⚠️ [LUT SWEEP ERR] {ex}\n"

            # Handle Comprehensive Electrical Multi-Pole Alignment Streaming
            elif s.startswith("[ELEC_POLES]:M"):
                try:
                    cal_m = int(s.split("[ELEC_POLES]:M")[1][0])
                    dir_str = "CW" if "DIR=CW" in s else "CCW"
                    data_str = s.split("DATA=")[1].strip()
                    rep, mean_z, grade, _ = analyze_electrical_poles(cal_m, dir_str, data_str)
                    self.state[f"m{cal_m}_zero_angle"] = mean_z
                    self.state[f"m{cal_m}_calib_grade"] = grade
                    self.state[f"m{cal_m}_direction"] = dir_str
                    return rep
                except Exception as ex:
                    return f"⚠️ [ELEC POLES ERR] {ex}\n"

            # Handle Real-Time Latency Profiler Breakdown
            elif s.startswith("[PROF]:"):
                return format_profiler(s)

            # Handle Live Encoder Diagnostics
            elif s.startswith("[ENC]:"):
                try:
                    mid = "M1" if "M1" in s else "M2"
                    self._enc_diag_buffer[mid] = s
                    if "M1" in self._enc_diag_buffer and "M2" in self._enc_diag_buffer:
                        out = (
                            "\n--- [MT6816 ENCODER LIVE DIAGNOSTICS] ---\n"
                            + format_encoder_diag(self._enc_diag_buffer["M1"])
                            + format_encoder_diag(self._enc_diag_buffer["M2"])
                            + "------------------------------------------\n"
                        )
                        self._enc_diag_buffer.clear()
                        return out
                    return ""
                except Exception:
                    return s + "\n"

            # Detect active calibrating motor from banner or start tag
            if "[CALIB_START]:M" in s:
                try:
                    cal_m = int(s.split("[CALIB_START]:M")[1][0])
                    self.state["calib_motor"] = cal_m
                    self.state[f"m{cal_m}_calib"] = True
                    self.state[f"m{3-cal_m}_calib"] = False
                    self.state["calib_progress"] = 0
                    self.state["last_packet_time"] = time.time()
                    return f"\n🛡️ [CALIBRATION ISOLATION ACTIVE] Motor {cal_m} isolated. Motor {3-cal_m} driver & SPI bus silenced to prevent EMI.\n"
                except Exception:
                    pass
            elif "[CALIB_END]:M" in s:
                try:
                    cal_m = int(s.split("[CALIB_END]:M")[1][0])
                    self.state["m1_calib"] = False
                    self.state["m2_calib"] = False
                    self.state["last_packet_time"] = time.time()
                    return f"\n🛡️ [CALIBRATION ISOLATION RELEASED] Motor {cal_m} sequence complete. Normal operation restored.\n"
                except Exception:
                    pass

            cal_m = self.state.get("calib_motor", 1)
            stage_key = f"m{cal_m}_calib_stage"
            grade_key = f"m{cal_m}_calib_grade"
            lut_key = f"m{cal_m}_lut_active"
            zero_key = f"m{cal_m}_zero_angle"
            dir_key = f"m{cal_m}_direction"
            lag_key = f"m{cal_m}_lag_comp_us"

            if "[CALIB_DIR]:M" in s:
                try:
                    m = int(s.split("[CALIB_DIR]:M")[1][0])
                    d_str = s.split(":")[-1].strip()
                    self.state[f"m{m}_direction"] = d_str
                    return f">>> Motor {m} Sensor Direction: {d_str}\n"
                except Exception:
                    pass

            if "[CALIB_PROG]:" in s:
                try:
                    p_str = s.split("[CALIB_PROG]:")[1].replace("%", "").strip()
                    val = int(p_str)
                    self.state["calib_progress"] = val
                    self.state[f"m{cal_m}_calib"] = True
                    self.state["last_packet_time"] = time.time()
                except Exception:
                    pass
            elif "[CALIB_DONE]" in s or "[CALIB_FAILED]" in s or "[LUT_READY]" in s:
                self.state["m1_calib"] = False
                self.state["m2_calib"] = False
                self.state["last_packet_time"] = time.time()

            if s.startswith("[CALIB_DONE]:M"):
                try:
                    m = int(s.split("[CALIB_DONE]:M")[1][0])
                    if "ZERO=" in s:
                        z_val = float(s.split("ZERO=")[1].strip())
                        self.state[f"m{m}_zero_angle"] = z_val
                    return f"✅ [CALIB_DONE] Motor {m} certified! Zero Electric Angle = {self.state.get(f'm{m}_zero_angle', 0.0):.5f} rad\n"
                except Exception:
                    pass

            if s.startswith("[LUT_READY]"):
                m_tag = s.split("[LUT_READY]:")[-1].strip()
                return f"[LUT_READY]:{m_tag}\n>>> [LUT] {m_tag} 128-point LUT successfully uploaded & activated!\n"

            if s.startswith("[CALIB_FAILED]"):
                return f"{s}\n" + format_friendly_error(s)

            if s.startswith("[ERR]:") or s.startswith("[STALL_") or s.startswith("[CALIB_BOOST]") or s.startswith("[CALIB_CERTIFIED]"):
                return format_friendly_error(s)

            if s.startswith("[FLASH]:"):
                return f"\n💾 [FLASH] Configuration saved to STM32 Flash ({s})\n"
            if s.startswith("[FLASH_ERR]:"):
                return f"\n❌ [FLASH ERROR] {s}\n"

            if "Forward Sweep:" in s or "Reverse Sweep:" in s or "Coil cool-down:" in s:
                self.state[stage_key] = s
                self.state["last_packet_time"] = time.time()
            elif "128-Point LUT ACTIVE" in s:
                self.state[lut_key] = True
                self.state[stage_key] = s
                self.state["last_packet_time"] = time.time()
            elif "LUT REJECTED" in s:
                self.state[lut_key] = False
                self.state[grade_key] = "LUT REJECTED"
                self.state[stage_key] = s
                self.state["last_packet_time"] = time.time()
            elif "[RESET]" in s or "[STARTUP]" in s or "FOLKRACE" in s or "Cleared Parity Errors" in s:
                self.state["m1_parity_ok"] = True
                self.state["m2_parity_ok"] = True
                self.state["spikes_count"] = 0

        return None

    def _handle_telemetry_payload(self, p):
        # Format: (header, m1_raw, m1_act_rpm, m1_tgt_rpm, m1_uq_mv, m1_status,
        #          m2_raw, m2_act_rpm, m2_tgt_rpm, m2_uq_mv, m2_status,
        #          foc_khz_x10, isr_lat_us, spikes_count, calib_prog, chk)
        dir_mult = -1.0 if self.invert_direction else 1.0
        m1_raw     = ((16384 - p[1]) & 0x3FFF) if self.invert_direction else p[1]
        m1_act_rpm = (p[2] / 10.0) * dir_mult
        m1_tgt_rpm = (p[3] / 10.0) * dir_mult
        m1_uq_v    = (p[4] / 1000.0) * dir_mult
        s1         = p[5]

        m2_raw     = ((16384 - p[6]) & 0x3FFF) if self.invert_direction else p[6]
        m2_act_rpm = (p[7] / 10.0) * dir_mult
        m2_tgt_rpm = (p[8] / 10.0) * dir_mult
        m2_uq_v    = (p[9] / 1000.0) * dir_mult
        s2         = p[10]

        foc_khz    = p[11] / 10.0
        isr_lat_us = p[12]
        spikes     = p[13]
        calib_prog = p[14]

        now = time.time()
        with self.lock:
            self.state["m1_raw_angle"]   = m1_raw
            self.state["m1_actual_rpm"]  = m1_act_rpm
            self.state["m1_target_rpm"]  = m1_tgt_rpm
            self.state["m1_uq_v"]        = m1_uq_v
            self.state["m1_enabled"]     = bool(s1 & (1 << 0))
            self.state["m1_lut_active"]  = bool(s1 & (1 << 1))
            self.state["m1_mag_ok"]      = bool(s1 & (1 << 2))
            self.state["m1_parity_ok"]   = bool(s1 & (1 << 3))
            self.state["m1_stalled"]     = bool(s1 & (1 << 4))
            self.state["m1_fault"]       = bool(s1 & (1 << 6))
            self.state["m1_calib"]       = bool(s1 & (1 << 5))

            self.state["m2_raw_angle"]   = m2_raw
            self.state["m2_actual_rpm"]  = m2_act_rpm
            self.state["m2_target_rpm"]  = m2_tgt_rpm
            self.state["m2_uq_v"]        = m2_uq_v
            self.state["m2_enabled"]     = bool(s2 & (1 << 0))
            self.state["m2_lut_active"]  = bool(s2 & (1 << 1))
            self.state["m2_mag_ok"]      = bool(s2 & (1 << 2))
            self.state["m2_parity_ok"]   = bool(s2 & (1 << 3))
            self.state["m2_stalled"]     = bool(s2 & (1 << 4))
            self.state["m2_fault"]       = bool(s2 & (1 << 6))
            self.state["m2_calib"]       = bool(s2 & (1 << 5))

            self.state["foc_khz"]          = foc_khz
            self.state["isr_lat_us"]       = isr_lat_us
            if spikes < self.state["spikes_count"] and spikes == 0:
                self.state["isr_peak_hold_us"] = isr_lat_us
            elif isr_lat_us > self.state["isr_peak_hold_us"]:
                self.state["isr_peak_hold_us"] = isr_lat_us
            self.state["spikes_count"]     = spikes
            self.state["calib_progress"]   = calib_prog
            self.state["last_packet_time"] = now

            # Push to velocity oscilloscope history at ~25 Hz (every 40 ms)
            if now - self._last_hist_push >= 0.038:
                self._last_hist_push = now
                self.history_time.append(now)
                self.history_target_rpm.append(m1_tgt_rpm)
                self.history_actual_rpm.append(m1_act_rpm)
                self.history_uq.append(m1_uq_v)
                self.history_m2_target_rpm.append(m2_tgt_rpm)
                self.history_m2_actual_rpm.append(m2_act_rpm)
                self.history_m2_uq.append(m2_uq_v)

        if self.on_telemetry:
            self.on_telemetry(self.state)
