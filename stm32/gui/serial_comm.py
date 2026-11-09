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
import serial
import serial.tools.list_ports

# 26-Byte Binary Telemetry Packet Format:
# < (little-endian), 2s (header), H (uint16), 3h (int16), B (uint8), H, 3h, B, H, 4B
PACKET_FMT = "<2sHhhhBHhhhBHBBBB"
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # Exactly 26 bytes
SYNC_HEADER = b"\xAA\x55"


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

    def send_emergency_stop(self):
        """Immediately flush pending commands and send hardware emergency stop burst."""
        try:
            while not self.cmd_queue.empty():
                self.cmd_queue.get_nowait()
        except Exception:
            pass
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"\n\nx\nx\nSTOP\n")
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
        if self.on_log:
            # Extract complete lines for state parsing while forwarding raw text to log
            pass
        while "\n" in text_buf:
            line, text_buf = text_buf.split("\n", 1)
            full_line = line + "\n"
            self._parse_ascii_line(line.strip())
            if self.on_log:
                self.on_log(full_line)
        return text_buf

    def _parse_ascii_line(self, s):
        if not s:
            return

        # 1. JSON Config Export Capture
        if s == "[CONFIG_START]":
            self._capturing_json = True
            self._json_lines = []
            return
        elif s == "[CONFIG_END]":
            if self._capturing_json:
                self._capturing_json = False
                json_str = "\n".join(self._json_lines)
                self.last_exported_json = json_str
                if self.on_config_json:
                    self.on_config_json(json_str)
            return
        elif self._capturing_json:
            self._json_lines.append(s)
            return

        # 2. Calibration Stage & Result Parsing
        with self.lock:
            # Detect active calibrating motor from banner
            if "Target Motor       : GM3506 (11 Pole Pairs on STM32G474 M" in s:
                self.state["calib_motor"] = 2 if "M2)" in s else 1

            cal_m = self.state.get("calib_motor", 1)
            stage_key = f"m{cal_m}_calib_stage"
            grade_key = f"m{cal_m}_calib_grade"
            lut_key = f"m{cal_m}_lut_active"
            zero_key = f"m{cal_m}_zero_angle"
            dir_key = f"m{cal_m}_direction"
            lag_key = f"m{cal_m}_lag_comp_us"

            if "Forward Sweep:" in s or "Reverse Sweep:" in s or "Coil cool-down:" in s:
                self.state[stage_key] = s
            elif "STEP 1: Locking Rotor" in s or "STEP 2: Continuous 360" in s:
                self.state[stage_key] = s
            elif "128-Point LUT ACTIVE" in s:
                self.state[lut_key] = True
                self.state[stage_key] = s
            elif "LUT REJECTED" in s:
                self.state[lut_key] = False
                self.state[grade_key] = "LUT REJECTED"
                self.state[stage_key] = s
            elif "motor.zero_electric_angle =" in s:
                try:
                    val_str = s.split("=")[1].replace(";", "").strip()
                    self.state[zero_key] = float(val_str)
                except Exception:
                    pass
            elif "motor.sensor_direction = Direction::" in s:
                self.state[dir_key] = "CCW" if "CCW" in s else "CW"
            elif "GRADE A" in s:
                self.state[grade_key] = "GRADE A (EXCELLENT)"
            elif "GRADE B" in s:
                self.state[grade_key] = "GRADE B (GOOD)"
            elif "GRADE C" in s:
                self.state[grade_key] = "GRADE C (FAIR)"
            elif "GRADE D" in s:
                self.state[grade_key] = "GRADE D (NOISY)"
            elif "GRADE E" in s:
                self.state[grade_key] = "GRADE E (POOR)"
            elif "Optimal Lag Trim =" in s:
                try:
                    part = s.split("Optimal Lag Trim =")[1].split("us")[0].strip()
                    self.state[lag_key] = float(part)
                except Exception:
                    pass
            elif "Flash Config Loaded:" in s:
                try:
                    if "Zero=" in s:
                        z_part = s.split("Zero=")[1].split("rad")[0].strip()
                        self.state["m1_zero_angle"] = float(z_part)
                    if "LUT=ACTIVE" in s:
                        self.state["m1_lut_active"] = True
                        self.state["m1_calib_grade"] = "FLASH LUT ACTIVE"
                except Exception:
                    pass
            elif "[RESET]" in s or "[STARTUP]" in s or "FOLKRACE" in s or "Cleared Parity Errors" in s:
                self.state["m1_parity_ok"] = True
                self.state["m2_parity_ok"] = True
                self.state["spikes_count"] = 0

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
                self.history_uq.append(m1_uq_v)

        if self.on_telemetry:
            self.on_telemetry(self.state)
