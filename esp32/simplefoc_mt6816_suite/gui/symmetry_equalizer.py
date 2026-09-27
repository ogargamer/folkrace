"""
Rotational symmetry test, multi-resolution brute-force electrical zero optimizer,
and CW/CCW directional equalizer engine for SimpleFOC.
"""

import math
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox


class SymmetryEqualizer:
    def __init__(self, reader, root, progress_var, status_var, on_zero_angle_updated=None):
        self.reader = reader
        self.root = root
        self.progress_var = progress_var
        self.status_var = status_var
        self.on_zero_angle_updated = on_zero_angle_updated

        self.current_zero_angle_rad = 0.11275
        self.running = False
        self.thread = None
        self.fwd_rpm = 0.0
        self.rev_rpm = 0.0

    def toggle(self, btn_auto_sym, sym_tree, entry_sym_v, entry_zero_angle, entry_align_v, entry_kv, chk_recalib_var):
        if self.running:
            self.running = False
            btn_auto_sym.config(text="🚀 1-CLICK AUTO-EQUALIZE SYMMETRY", bg="#00E676", fg="#000")
            self.status_var.set("Auto Equalizer Aborted by User.")
            self.reader.send_cmd("v0.0")
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to ESP32-S3 first!")
                return
            self.running = True
            btn_auto_sym.config(text="⏹️ Stop Auto Equalizer", bg="#D50000", fg="#FFF")
            self.thread = threading.Thread(
                target=self._run_worker,
                args=(btn_auto_sym, sym_tree, entry_sym_v, entry_zero_angle, entry_align_v, entry_kv, chk_recalib_var),
                daemon=True
            )
            self.thread.start()

    def abort(self):
        self.running = False
        self.status_var.set("🛑 EMERGENCY STOP: Stator coils de-energized.")

    def _cleanup_ui(self, btn_auto_sym):
        self.running = False
        self.root.after(0, lambda: btn_auto_sym.config(text="🚀 1-CLICK AUTO-EQUALIZE SYMMETRY", bg="#00E676", fg="#000"))

    def _measure_candidate_angle(self, candidate_angle, stage_name, v_test, base_zero, sym_tree):
        if not self.running:
            return None

        # 1. Update Zero Angle on ESP32-S3
        self.reader.send_cmd(f"j{candidate_angle:.5f}")
        self.current_zero_angle_rad = candidate_angle
        if self.on_zero_angle_updated:
            self.root.after(0, lambda a=candidate_angle: self.on_zero_angle_updated(a))
        time.sleep(0.05)

        # 2. Forward Spin (+V): Smooth 1.8s settle with stall penalty (no E-stop shutdown)
        self.reader.send_cmd(f"v{v_test:.2f}")
        t_spin = time.time()
        stall_time_s = 0.0
        last_t = time.time()
        while time.time() - t_spin < 1.8:
            if not self.running:
                self.reader.send_cmd("v0.0")
                return None

            now = time.time()
            dt = now - last_t
            last_t = now

            with self.reader.lock:
                live_spd = abs(self.reader.last_actual_rpm)

            if (now - t_spin) > 0.2:
                if live_spd < 3.0:
                    stall_time_s += dt
                    if stall_time_s >= 0.50:
                        self.reader.send_cmd("v0.0")
                        time.sleep(0.3)
                        self._record_sym_row(sym_tree, stage_name, candidate_angle, base_zero, 0.0, 0.0, "[STALL - 0 RPM]", v_test=v_test)
                        return {"angle": candidate_angle, "fwd": 0, "rev": 0, "delta": 0, "sym": 0, "kv": 0, "score": 0, "status": "[STALL]", "is_valid": False}
                else:
                    stall_time_s = 0.0
            time.sleep(0.02)

        fwd_samples = []
        for _ in range(30):  # 30 samples over ~0.9s for noise immunity
            if not self.running:
                self.reader.send_cmd("v0.0")
                return None
            with self.reader.lock:
                fwd_samples.append(abs(self.reader.last_actual_rpm))
            time.sleep(0.03)
        fwd_rpm = sum(fwd_samples) / max(1, len(fwd_samples))
        self.fwd_rpm = fwd_rpm

        # Soft stop & cool-down pause
        self.reader.send_cmd("v0.0")
        t_stop = time.time()
        while time.time() - t_stop < 0.50:
            if not self.running:
                return None
            time.sleep(0.02)

        if not self.running:
            return None

        # 3. Reverse Spin (-V): Smooth 1.8s settle
        self.reader.send_cmd(f"v-{v_test:.2f}")
        t_rev = time.time()
        stall_time_s = 0.0
        last_t = time.time()
        while time.time() - t_rev < 1.8:
            if not self.running:
                self.reader.send_cmd("v0.0")
                return None

            now = time.time()
            dt = now - last_t
            last_t = now

            with self.reader.lock:
                live_spd = abs(self.reader.last_actual_rpm)

            if (now - t_rev) > 0.2:
                if live_spd < 3.0:
                    stall_time_s += dt
                    if stall_time_s >= 0.50:
                        self.reader.send_cmd("v0.0")
                        time.sleep(0.3)
                        self._record_sym_row(sym_tree, stage_name, candidate_angle, base_zero, fwd_rpm, 0.0, "[STALL - 0 RPM]", v_test=v_test)
                        return {"angle": candidate_angle, "fwd": fwd_rpm, "rev": 0, "delta": 0, "sym": 0, "kv": 0, "score": 0, "status": "[STALL]", "is_valid": False}
                else:
                    stall_time_s = 0.0
            time.sleep(0.02)

        rev_samples = []
        for _ in range(30):
            if not self.running:
                self.reader.send_cmd("v0.0")
                return None
            with self.reader.lock:
                rev_samples.append(abs(self.reader.last_actual_rpm))
            time.sleep(0.03)
        rev_rpm = sum(rev_samples) / max(1, len(rev_samples))
        self.rev_rpm = rev_rpm

        # Soft stop & inter-candidate cooldown
        self.reader.send_cmd("v0.0")
        t_stop2 = time.time()
        while time.time() - t_stop2 < 0.50:
            if not self.running:
                return None
            time.sleep(0.02)

        # 4. Evaluation & Precision Score
        delta_rpm = fwd_rpm - rev_rpm
        is_stalled = (fwd_rpm < 15.0 or rev_rpm < 15.0)
        mean_rpm = (fwd_rpm + rev_rpm) * 0.5
        stage_kv = mean_rpm / max(0.1, v_test)
        max_rpm = max(fwd_rpm, rev_rpm, 1.0)
        sym_pct = (min(fwd_rpm, rev_rpm) / max_rpm) * 100.0 if not is_stalled else 0.0

        if is_stalled:
            score = 0.0
            status = "[STALL - LOW RPM]"
            is_valid = False
        else:
            score = sym_pct * (1.0 + 0.001 * mean_rpm)
            status = f"OK ({sym_pct:.1f}%)"
            is_valid = True

        self._record_sym_row(sym_tree, stage_name, candidate_angle, base_zero, fwd_rpm, rev_rpm, status, v_test=v_test)

        return {
            "angle": candidate_angle,
            "fwd": fwd_rpm,
            "rev": rev_rpm,
            "delta": delta_rpm,
            "sym": sym_pct,
            "kv": stage_kv,
            "score": score,
            "status": status,
            "is_valid": is_valid
        }

    def _record_sym_row(self, sym_tree, stage_name, candidate_angle, base_zero, fwd_rpm, rev_rpm, status, v_test=4.0):
        nudge_val = (candidate_angle - base_zero + math.pi) % (2.0 * math.pi) - math.pi
        delta_rpm = fwd_rpm - rev_rpm
        max_rpm = max(fwd_rpm, rev_rpm, 1.0)
        sym_pct = (min(fwd_rpm, rev_rpm) / max_rpm) * 100.0 if (fwd_rpm > 5.0 and rev_rpm > 5.0) else 0.0
        stage_kv = ((fwd_rpm + rev_rpm) * 0.5) / max(0.1, v_test)

        row_data = (
            stage_name,
            f"{candidate_angle:.5f}",
            f"{candidate_angle*57.29578:.2f}°",
            f"{nudge_val:+.5f} rad",
            f"{fwd_rpm:+.1f} RPM",
            f"-{rev_rpm:.1f} RPM",
            f"{delta_rpm:+.1f} RPM",
            f"{sym_pct:.1f} %",
            f"{stage_kv:.1f} RPM/V",
            status
        )
        self.root.after(0, lambda r=row_data: [
            sym_tree.insert("", "end", values=r),
            sym_tree.see(sym_tree.get_children()[-1])
        ])

    def _run_worker(self, btn_auto_sym, sym_tree, entry_sym_v, entry_zero_angle, entry_align_v, entry_kv, chk_recalib_var):
        try:
            v_test = abs(float(entry_sym_v.get().strip()))
        except Exception:
            v_test = 4.0

        # Clear report table in UI
        self.root.after(0, lambda: [
            sym_tree.delete(item) for item in sym_tree.get_children()
        ])

        # Step 0: If checked, re-run 11-pole continuous alignment first
        if chk_recalib_var.get() and self.running:
            self.status_var.set("Phase 0/3: Re-running 11-Pole Alignment Sweep first...")
            with self.reader.lock:
                self.reader.last_calib_zero_angle = None
                self.reader.calibrating = True
                self.reader.last_calib_failed = False
            cal_v = entry_align_v.get().strip() or "4.0"
            self.reader.send_cmd(f"c{cal_v}")
            t_align = time.time()
            time.sleep(1.0)
            while time.time() - t_align < 35:
                if not self.running:
                    self.reader.send_cmd("x")
                    return
                with self.reader.lock:
                    if not self.reader.calibrating and (self.reader.last_calib_zero_angle is not None or self.reader.last_calib_failed):
                        break
                time.sleep(0.3)
            time.sleep(0.5)

        # Baseline Zero
        with self.reader.lock:
            latest_zero = self.reader.last_calib_zero_angle or self.reader.last_zero_electric_angle
        try:
            base_zero = latest_zero if (chk_recalib_var.get() and latest_zero is not None) else float(entry_zero_angle.get().strip())
        except ValueError:
            base_zero = latest_zero if latest_zero is not None else self.current_zero_angle_rad

        if self.on_zero_angle_updated:
            self.root.after(0, lambda z=base_zero: self.on_zero_angle_updated(z))

        self.current_zero_angle_rad = base_zero
        all_results = []

        # PASS 1: WIDE COARSE EXPLORATION (±0.040 rad / ±2.3° elec)
        self.status_var.set("Pass 1/3: Coarse Bracket Sweep across Baseline ±0.040 rad...")
        coarse_offsets = [0.0000, +0.0200, -0.0200, +0.0400, -0.0400]
        pass1_results = []

        for i, off in enumerate(coarse_offsets, 1):
            if not self.running:
                break
            self.progress_var.set((i / len(coarse_offsets)) * 35.0)
            ang = (base_zero + off) % (2.0 * math.pi)
            label = f"Coarse #{i} ({off:+.4f})"
            res = self._measure_candidate_angle(ang, label, v_test, base_zero, sym_tree)
            if res:
                pass1_results.append(res)
                all_results.append(res)

        valid_pass1 = [r for r in pass1_results if r["is_valid"]]
        if not valid_pass1:
            self.status_var.set("❌ Motor stalled or no valid movements recorded during Pass 1.")
            self._cleanup_ui(btn_auto_sym)
            return

        best_coarse = max(valid_pass1, key=lambda r: r["score"])
        best_angle = best_coarse["angle"]

        # PASS 2: MEDIUM ZOOM SWEEP (±0.008 rad around Coarse Winner)
        pass2_results = [best_coarse]
        if self.running:
            self.status_var.set(f"Pass 2/3: Medium Zooming around Winner ({best_angle*57.3:.2f}°)...")
            med_offsets = [+0.0080, -0.0080, +0.0040, -0.0040]

            for i, off in enumerate(med_offsets, 1):
                if not self.running:
                    break
                self.progress_var.set(35.0 + (i / len(med_offsets)) * 35.0)
                ang = (best_angle + off) % (2.0 * math.pi)
                label = f"Med #{i} ({off:+.4f})"
                res = self._measure_candidate_angle(ang, label, v_test, base_zero, sym_tree)
                if res:
                    pass2_results.append(res)
                    all_results.append(res)

        valid_pass2 = [r for r in pass2_results if r["is_valid"]]
        best_med = max(valid_pass2, key=lambda r: r["score"]) if valid_pass2 else best_coarse
        best_angle2 = best_med["angle"]

        # PASS 3: FINE MICRO-TRIM (±0.0015 rad for >99% Symmetry)
        pass3_results = [best_med]
        if self.running:
            self.status_var.set("Pass 3/3: Fine Micro-Trimming for >99% Symmetry...")
            fine_offsets = [+0.0015, -0.0015, +0.0007, -0.0007]

            for i, off in enumerate(fine_offsets, 1):
                if not self.running:
                    break
                self.progress_var.set(70.0 + (i / len(fine_offsets)) * 30.0)
                ang = (best_angle2 + off) % (2.0 * math.pi)
                label = f"Fine #{i} ({off:+.4f})"
                res = self._measure_candidate_angle(ang, label, v_test, base_zero, sym_tree)
                if res:
                    pass3_results.append(res)
                    all_results.append(res)

        valid_pass3 = [r for r in pass3_results if r["is_valid"]]
        winner = max(valid_pass3, key=lambda r: r["score"]) if valid_pass3 else best_med

        if not self.running:
            self.status_var.set("Auto Equalizer Aborted by User.")
            self._cleanup_ui(btn_auto_sym)
            self.reader.send_cmd("v0.0")
            return

        final_ang = winner["angle"]
        final_kv = max(100.0, min(180.0, winner.get("kv", 130.0)))
        final_sym = winner.get("sym", 95.0)

        self.current_zero_angle_rad = final_ang
        self.reader.send_cmd(f"j{final_ang:.5f}")
        self.reader.send_cmd("v0.0")

        self.progress_var.set(100.0)
        self.status_var.set(
            f"🎉 Optimization Complete! Neutral Zero = {final_ang:.5f} rad ({final_ang*57.29578:.2f}°) | Sym = {final_sym:.1f}% | Δ = {winner['delta']:+.1f} RPM"
        )
        self._cleanup_ui(btn_auto_sym)

        # Update UI Entries
        self.root.after(0, lambda k=final_kv, a=final_ang: [
            entry_kv.delete(0, "end"),
            entry_kv.insert(0, f"{k:.1f}"),
        ])
        if self.on_zero_angle_updated:
            self.root.after(0, lambda a=final_ang: self.on_zero_angle_updated(a))
