"""
serial_comm.py
==============
Thread-safe serial reader and command engine for STM32G474 20 kHz Dual FOC Controller.
Demuxes 26-byte binary telemetry frames (b"\\xAA\\x55" + XOR checksum) and UTF-8 ASCII log lines.
"""

import struct
import threading
import queue
import time
import serial
import serial.tools.list_ports

# 26-Byte Binary Telemetry Packet Format:
# < (little-endian), 2s (header), H (uint16), 3h (int16), B (uint8), H, 3h, B, H, 4B
PACKET_FMT = "<2sHhhhBHhhhBHBBBB"
PACKET_SIZE = struct.calcsize(PACKET_FMT) # Exactly 26 bytes
SYNC_HEADER = b"\xAA\x55"

class STM32DualSerial:
    def __init__(self, port="COM3", baud=921600, on_telemetry=None, on_log=None):
        self.port = port
        self.baud = baud
        self.on_telemetry = on_telemetry
        self.on_log = on_log

        self.ser = None
        self.running = False
        self.thread = None
        self.cmd_queue = queue.Queue()
        self.lock = threading.Lock()

        # Telemetry State Dict
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
            "m1_calib": False,

            "m2_raw_angle": 0,
            "m2_actual_rpm": 0.0,
            "m2_target_rpm": 0.0,
            "m2_uq_v": 0.0,
            "m2_enabled": False,
            "m2_lut_active": False,
            "m2_mag_ok": True,
            "m2_parity_ok": True,
            "m2_stalled": False,
            "m2_calib": False,

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
            self.thread = threading.Thread(target=self._io_worker, daemon=True)
            self.thread.start()
            if self.on_log:
                self.on_log(f"[SYS] Connected to {self.port} @ {self.baud} baud\n")
            return True, "Connected"
        except Exception as ex:
            self.state["connected"] = False
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
        if self.on_log:
            self.on_log(f"[SYS] Disconnected from {self.port}\n")

    def send_command(self, cmd_str):
        if not cmd_str.endswith("\n"):
            cmd_str += "\n"
        self.cmd_queue.put(cmd_str)

    def _io_worker(self):
        buf = bytearray()
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
                while len(buf) >= PACKET_SIZE:
                    idx = buf.find(SYNC_HEADER)
                    if idx == -1:
                        # No sync header found; decode non-binary bytes as ASCII log lines
                        line_end = buf.find(b"\n")
                        if line_end != -1:
                            line_bytes = buf[:line_end + 1]
                            buf = buf[line_end + 1:]
                            text = line_bytes.decode("utf-8", errors="replace")
                            if self.on_log:
                                self.on_log(text)
                        elif len(buf) > 512:
                            text = buf[:256].decode("utf-8", errors="replace")
                            buf = buf[256:]
                            if self.on_log:
                                self.on_log(text)
                        break
                    elif idx > 0:
                        # Log text preceding the sync header
                        prefix = buf[:idx]
                        buf = buf[idx:]
                        text = prefix.decode("utf-8", errors="replace")
                        if self.on_log:
                            self.on_log(text)

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

    def _handle_telemetry_payload(self, p):
        # Format: (header, m1_raw, m1_act_rpm, m1_tgt_rpm, m1_uq_mv, m1_status,
        #          m2_raw, m2_act_rpm, m2_tgt_rpm, m2_uq_mv, m2_status,
        #          foc_khz_x10, isr_lat_us, spikes_count, calib_prog, chk)
        m1_raw      = p[1]
        m1_act_rpm  = p[2] / 10.0
        m1_tgt_rpm  = p[3] / 10.0
        m1_uq_v     = p[4] / 1000.0
        s1          = p[5]

        m2_raw      = p[6]
        m2_act_rpm  = p[7] / 10.0
        m2_tgt_rpm  = p[8] / 10.0
        m2_uq_v     = p[9] / 1000.0
        s2          = p[10]

        foc_khz     = p[11] / 10.0
        isr_lat_us  = p[12]
        spikes      = p[13]
        calib_prog  = p[14]

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
            self.state["m2_calib"]       = bool(s2 & (1 << 5))

            self.state["foc_khz"]        = foc_khz
            self.state["isr_lat_us"]     = isr_lat_us
            self.state["spikes_count"]   = spikes
            self.state["calib_progress"] = calib_prog
            self.state["last_packet_time"] = time.time()

        if self.on_telemetry:
            self.on_telemetry(self.state)
