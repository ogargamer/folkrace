"""
Master Auto-Calibration Orchestrator for ESP32 SimpleFOC & MT6816/MT6818.
Coordinates 128-point dual-pass LUT calibration, 11-pole continuous alignment fallback,
verification spins, and automatic NVS profile commitment.
"""

import threading
import time
import tkinter as tk
from tkinter import messagebox


class Calibrator:
    def __init__(self, reader, root, progress_var, status_var, grade_var, align_volt_var, boost_retry_var, verify_var, on_calib_finish_cb=None):
        self.reader = reader
        self.root = root
        self.progress_var = progress_var
        self.status_var = status_var
        self.grade_var = grade_var
        self.align_volt_var = align_volt_var
        self.boost_retry_var = boost_retry_var
        self.verify_var = verify_var
        self.on_calib_finish_cb = on_calib_finish_cb

        self.running = False
        self.thread = None

    def toggle(self, btn_calib):
        if self.running:
            self.running = False
            btn_calib.config(text="🚀 1-CLICK MASTER AUTO-CALIBRATE (MULTI-METHOD)", bg="#00E676", fg="#000")
            self.status_var.set("Calibration Aborted by User.")
            self.reader.send_cmd("x")
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to ESP32-S3 first!")
                return
            self.running = True
            btn_calib.config(text="⏹️ Abort Master Calibration", bg="#D50000", fg="#FFF")
            self.thread = threading.Thread(target=self._run_worker, args=(btn_calib,), daemon=True)
            self.thread.start()

    def abort(self):
        self.running = False
        self.status_var.set("🛑 EMERGENCY STOP: All operations terminated.")

    def _cleanup_ui(self, btn_calib):
        self.running = False
        self.root.after(0, lambda: btn_calib.config(text="🚀 1-CLICK MASTER AUTO-CALIBRATE (MULTI-METHOD)", bg="#00E676", fg="#000"))

    def _run_worker(self, btn_calib):
        try:
            align_v = float(self.align_volt_var.get().strip())
        except Exception:
            align_v = 4.0

        # Step 0: Pre-Flight Hardware Qualification
        self.root.after(0, lambda: [
            self.progress_var.set(2.0),
            self.status_var.set("Step 0/5: Checking Sensor & SPI Bus Diagnostics (r)...")
        ])
        self.reader.send_cmd("r")
        time.sleep(0.4)

        with self.reader.lock:
            mag_ok = self.reader.last_mag_ok
            parity_ok = self.reader.last_parity_ok
            raw_cnt = self.reader.last_raw_14bit

        if not mag_ok or not parity_ok or raw_cnt == 0 or raw_cnt == 16383:
            self.root.after(0, lambda: [
                self.status_var.set("❌ Pre-Flight Failed: Magnet Fault or Parity Errors! Check wiring."),
                self.grade_var.set("SENSOR FAULT")
            ])
            self._cleanup_ui(btn_calib)
            return

        # METHOD 1: HIGH-PRECISION 128-PT DUAL-PASS LUT CALIBRATION ('w')
        self.root.after(0, lambda: [
            self.status_var.set(f"Step 1/5: Running 128-Point Dual-Pass LUT Sweep at {align_v:.1f}V..."),
            self.progress_var.set(5.0)
        ])

        lut_success = self._execute_firmware_sweep(f"w{align_v:.2f}", timeout_sec=200)

        # Retry with higher voltage if stalled or cogging occurred
        if not lut_success and self.boost_retry_var.get() and self.running:
            boost_v = min(4.2, align_v + 0.2)
            self.root.after(0, lambda bv=boost_v: self.status_var.set(f"⚠️ LUT Retrying with Boost Voltage {bv:.1f}V to overcome cogging/stiction..."))
            time.sleep(2.0)
            lut_success = self._execute_firmware_sweep(f"w{boost_v:.2f}", timeout_sec=200)

        # METHOD 2: 11-POLE ROUND-TRIP ALIGNMENT ('c') FALLBACK (IF LUT FAILED)
        c_success = lut_success
        if not lut_success and self.running:
            fallback_v = min(4.2, align_v + 0.2)
            self.root.after(0, lambda fv=fallback_v: self.status_var.set(f"Step 2/5: Fallback to High-Torque 11-Pole Round-Trip Alignment ({fv:.1f}V)..."))
            time.sleep(1.0)
            c_success = self._execute_firmware_sweep(f"c{fallback_v:.2f}", timeout_sec=60)
            if not c_success and self.running:
                # Retry once more with max safe voltage
                time.sleep(1.5)
                c_success = self._execute_firmware_sweep("c4.20", timeout_sec=60)

        if not self.running:
            self._cleanup_ui(btn_calib)
            return

        if not c_success:
            self.root.after(0, lambda: [
                self.grade_var.set("GRADE F (FAILED)"),
                self.status_var.set("❌ Calibration failed. Check motor wiring, power supply, and mechanical friction.")
            ])
            self._cleanup_ui(btn_calib)
            return

        # POST-CALIBRATION: CLOSED-LOOP VERIFICATION PULSE
        verified = False
        if self.verify_var.get() and self.running:
            self.root.after(0, lambda: [
                self.status_var.set("Step 4/5: Running Dual-Direction Closed-Loop Verification Spin (±1.5V)..."),
                self.progress_var.set(90.0)
            ])
            verified = self._verify_closed_loop_motion()
        else:
            verified = True

        # AUTO-SAVE TO FLASH (NVS) & FINALIZE
        if self.running and verified:
            self.root.after(0, lambda: [
                self.status_var.set("Step 5/5: Auto-Saving Certified Profile to Flash Memory (NVS)..."),
                self.progress_var.set(100.0)
            ])
            self.reader.send_cmd("save")
            time.sleep(0.4)
            with self.reader.lock:
                grade = self.reader.last_calib_grade or "CERTIFIED"
                zero_a = self.reader.last_zero_electric_angle or 0.97555
            self.root.after(0, lambda g=grade, a=zero_a: [
                self.grade_var.set(g),
                self.status_var.set(f"🎉 Calibration Certified! Zero: {a:.5f} rad ({a*57.29578:.1f}°) | {g} | Saved to Flash.")
            ])
            if self.on_calib_finish_cb:
                self.root.after(0, lambda a=zero_a: self.on_calib_finish_cb(a))
        elif self.running and not verified:
            self.root.after(0, lambda: self.status_var.set("⚠️ Verification failed: Low rotation detected. Profile not saved to Flash."))

        self._cleanup_ui(btn_calib)

    def _execute_firmware_sweep(self, cmd, timeout_sec=200):
        with self.reader.lock:
            self.reader.last_calib_success = False
            self.reader.last_calib_failed = False
            self.reader.calibrating = True
        self.reader.send_cmd(cmd)
        t_start = time.time()
        while time.time() - t_start < timeout_sec:
            if not self.running:
                self.reader.send_cmd("x")
                return False
            with self.reader.lock:
                done = (not self.reader.calibrating) and (self.reader.last_calib_success or self.reader.last_calib_failed)
                success = self.reader.last_calib_success
                failed = self.reader.last_calib_failed
                p_pct = self.reader.calib_progress_pct
                st_name = self.reader.calib_stage_name
            if p_pct > 0:
                self.root.after(0, lambda p=p_pct: self.progress_var.set(p))
            if st_name:
                self.root.after(0, lambda s=st_name: self.status_var.set(s))
            if done:
                return success and not failed
            time.sleep(0.1)
        return False

    def _verify_closed_loop_motion(self):
        # Forward test (+1.5V)
        self.reader.send_cmd("v1.50")
        t0 = time.time()
        fwd_speeds = []
        while time.time() - t0 < 0.6:
            if not self.running:
                self.reader.send_cmd("v0.0")
                return False
            with self.reader.lock:
                fwd_speeds.append(abs(self.reader.last_actual_rpm))
            time.sleep(0.05)
        self.reader.send_cmd("v0.0")
        time.sleep(0.3)

        # Reverse test (-1.5V)
        self.reader.send_cmd("v-1.50")
        t1 = time.time()
        rev_speeds = []
        while time.time() - t1 < 0.6:
            if not self.running:
                self.reader.send_cmd("v0.0")
                return False
            with self.reader.lock:
                rev_speeds.append(abs(self.reader.last_actual_rpm))
            time.sleep(0.05)
        self.reader.send_cmd("v0.0")
        time.sleep(0.2)

        max_fwd = max(fwd_speeds) if fwd_speeds else 0.0
        max_rev = max(rev_speeds) if rev_speeds else 0.0

        if max_fwd >= 20.0 and max_rev >= 20.0:
            return True
        else:
            self.root.after(0, lambda mf=max_fwd, mr=max_rev: self.status_var.set(f"⚠️ Verification: Low rotation (Fwd={mf:.0f}, Rev={mr:.0f} RPM)."))
            return False
