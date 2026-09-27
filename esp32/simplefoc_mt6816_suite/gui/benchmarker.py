"""
Automated 2-minute PID sweep and speed variation benchmark for GM3506.
Tests 12 distinct PID and LPF configurations, computes jitter/ripple/MAE, and generates leaderboard ranking.
"""

import math
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox


class BenchmarkRunner:
    def __init__(self, reader, root, progress_var, status_var, set_param_fn, on_benchmark_finish_cb=None):
        self.reader = reader
        self.root = root
        self.progress_var = progress_var
        self.status_var = status_var
        self.set_param_fn = set_param_fn
        self.on_benchmark_finish_cb = on_benchmark_finish_cb

        self.running = False
        self.thread = None
        self.results = []

    def toggle(self, btn_start, btn_results):
        if self.running:
            self.running = False
            btn_start.config(text="▶️ Start 2-Min PID Benchmark", bg="#00E676", fg="#000")
            self.status_var.set("Benchmark Aborted by User.")
            self.reader.send_cmd("x")
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to ESP32-S3 first!")
                return
            self.running = True
            btn_start.config(text="⏹️ Stop Benchmark", bg="#D50000", fg="#FFF")
            btn_results.config(state="disabled")
            self.results = []
            self.thread = threading.Thread(target=self._run_worker, args=(btn_start, btn_results), daemon=True)
            self.thread.start()

    def abort(self):
        self.running = False
        self.status_var.set("🛑 EMERGENCY STOP: Benchmark aborted.")

    def _run_worker(self, btn_start, btn_results):
        presets = [
            {"name": "GM3506 Baseline Smooth", "p": 0.08, "i": 0.80, "d": 0.000, "tf": 0.035, "target": 80.0},
            {"name": "GM3506 Optimal Balanced", "p": 0.12, "i": 1.20, "d": 0.001, "tf": 0.015, "target": 80.0},
            {"name": "Crisp Direct Tracking", "p": 0.15, "i": 1.50, "d": 0.001, "tf": 0.020, "target": 80.0},
            {"name": "High Integral Authority", "p": 0.10, "i": 2.00, "d": 0.001, "tf": 0.025, "target": 80.0},
            {"name": "Ultra-Fast LPF Response", "p": 0.12, "i": 1.20, "d": 0.000, "tf": 0.010, "target": 80.0},
            {"name": "Heavy Damped Smooth", "p": 0.10, "i": 1.00, "d": 0.002, "tf": 0.030, "target": 80.0},
            {"name": "High Speed 150 RPM", "p": 0.12, "i": 1.20, "d": 0.001, "tf": 0.015, "target": 150.0},
            {"name": "High Speed 250 RPM", "p": 0.14, "i": 1.50, "d": 0.001, "tf": 0.015, "target": 250.0},
            {"name": "Aggressive High-P", "p": 0.18, "i": 1.80, "d": 0.001, "tf": 0.015, "target": 80.0},
            {"name": "Gentle Low-Current", "p": 0.06, "i": 0.60, "d": 0.000, "tf": 0.025, "target": 80.0},
            {"name": "Precise D-Damped", "p": 0.12, "i": 1.00, "d": 0.003, "tf": 0.020, "target": 80.0},
            {"name": "Maximum Velocity 400", "p": 0.15, "i": 1.50, "d": 0.001, "tf": 0.015, "target": 400.0},
        ]

        total_stages = len(presets)
        stage_duration = 10.0
        settle_time = 2.5

        # Step 0: Ensure SimpleFOC has calibrated electrical zero
        with self.reader.lock:
            already_calibrated = self.reader.last_calib_success and self.reader.last_calib_zero_angle is not None
        if not already_calibrated:
            self.root.after(0, lambda: self.status_var.set("Phase 0: Executing 11-Pole Alignment before benchmark..."))
            with self.reader.lock:
                self.reader.last_calib_zero_angle = None
                self.reader.calibrating = True
                self.reader.last_calib_failed = False
            self.reader.send_cmd("c4.0")
            t_cal = time.time()
            time.sleep(1.0)
            while time.time() - t_cal < 35:
                if not self.running:
                    self.reader.send_cmd("x")
                    return
                with self.reader.lock:
                    if not self.reader.calibrating and (self.reader.last_calib_zero_angle is not None or self.reader.last_calib_failed):
                        break
                time.sleep(0.3)
            time.sleep(0.5)

        start_bench_time = time.time()
        was_aborted = False

        for idx, p in enumerate(presets):
            if not self.running:
                was_aborted = True
                break

            self.reader.send_cmd(f"p{p['p']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"i{p['i']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"d{p['d']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"f{p['tf']}")
            time.sleep(0.05)
            self.reader.send_cmd(f"t{p['target']}")

            stage_start = time.time()
            samples = []

            while time.time() - stage_start < stage_duration:
                if not self.running:
                    was_aborted = True
                    break

                elapsed_in_stage = time.time() - stage_start
                total_elapsed = time.time() - start_bench_time
                total_progress = min(100.0, (total_elapsed / (total_stages * stage_duration)) * 100.0)

                remaining_sec = max(0, int((total_stages * stage_duration) - total_elapsed))
                def _update_status(i=idx, pr=p, prog=total_progress, rem=remaining_sec):
                    self.progress_var.set(prog)
                    self.status_var.set(
                        f"Testing [{i+1}/{total_stages}] {pr['name']} (P={pr['p']}, I={pr['i']}, Tf={pr['tf']}) | "
                        f"Target: {pr['target']} RPM | {rem}s remaining"
                    )
                self.root.after(0, _update_status)

                if elapsed_in_stage >= settle_time:
                    with self.reader.lock:
                        act = self.reader.last_actual_rpm
                        uq = self.reader.last_uq
                    samples.append((act, uq))

                time.sleep(0.05)

            if samples:
                speeds = [s[0] for s in samples]
                uqs = [s[1] for s in samples]
                target = p['target']

                mean_speed = sum(speeds) / len(speeds)
                errors = [abs(s - target) for s in speeds]
                mae = sum(errors) / len(errors)

                variance = sum((s - mean_speed) ** 2 for s in speeds) / len(speeds)
                std_dev = math.sqrt(variance)

                ripple_pk_pk = max(speeds) - min(speeds)
                mean_uq = sum(uqs) / len(uqs)

                score = std_dev + (0.5 * mae)

                result_entry = {
                    "rank": 0,
                    "name": p['name'],
                    "p": p['p'],
                    "i": p['i'],
                    "d": p['d'],
                    "tf": p['tf'],
                    "target": target,
                    "mean_speed": mean_speed,
                    "mae": mae,
                    "std_dev": std_dev,
                    "ripple_pk_pk": ripple_pk_pk,
                    "mean_uq": mean_uq,
                    "score": score
                }
                self.results.append(result_entry)

        self.running = False
        if was_aborted:
            self.reader.send_cmd("x")
        else:
            self.reader.send_cmd("t0")

        def _finish(aborted=was_aborted):
            btn_start.config(text="▶️ Start 2-Min PID Benchmark", bg="#00E676", fg="#000")
            if aborted:
                self.status_var.set("Benchmark Aborted.")
                if self.results:
                    btn_results.config(state="normal")
            elif self.results:
                self.results.sort(key=lambda x: x['score'])
                for rank_idx, r in enumerate(self.results):
                    r['rank'] = rank_idx + 1

                best = self.results[0]
                self.progress_var.set(100.0)
                self.status_var.set(
                    f"✅ Benchmark Complete! Best Preset: #{best['rank']} '{best['name']}' "
                    f"(P={best['p']}, I={best['i']}, Tf={best['tf']} | Jitter: ±{best['std_dev']:.2f} RPM)"
                )
                btn_results.config(state="normal")
                if self.on_benchmark_finish_cb:
                    self.root.after(200, self.on_benchmark_finish_cb)
        self.root.after(0, _finish)

    def show_results_modal(self, apply_callback=None):
        if not self.results:
            messagebox.showinfo("No Results", "Please run the 2-minute benchmark first!")
            return

        modal = tk.Toplevel(self.root)
        modal.title("🏆 PID Benchmark & Speed Variation Leaderboard")
        modal.geometry("920x530")
        modal.configure(bg="#12161E")
        modal.transient(self.root)

        header = tk.Frame(modal, bg="#1A2230", padx=15, pady=12)
        header.pack(fill="x")

        best = self.results[0]
        tk.Label(header, text="🏆 PID TUNING LEADERBOARD & VARIATION ANALYSIS", font=("Segoe UI", 14, "bold"), fg="#00E5FF", bg="#1A2230").pack(anchor="w")
        tk.Label(
            header,
            text=f"Top Performer: #{best['rank']} {best['name']} | P={best['p']}, I={best['i']}, D={best['d']}, Tf={best['tf']} | Jitter: ±{best['std_dev']:.2f} RPM | Ripple: {best['ripple_pk_pk']:.1f} RPM",
            font=("Segoe UI", 10, "bold"),
            fg="#69F0AE",
            bg="#1A2230"
        ).pack(anchor="w", pady=(4, 0))

        table_frame = tk.Frame(modal, bg="#12161E", padx=10, pady=10)
        table_frame.pack(fill="both", expand=True)

        cols = ("Rank", "Preset Name", "P Gain", "I Gain", "D Gain", "LPF Tf", "Target", "Actual", "MAE Err", "StdDev (Jitter)", "Ripple Pk-Pk")
        tree = ttk.Treeview(table_frame, columns=cols, show="headings", height=12)

        col_widths = [50, 160, 60, 60, 60, 60, 65, 65, 75, 100, 90]
        for c, w in zip(cols, col_widths):
            tree.heading(c, text=c)
            tree.column(c, width=w, anchor="center")

        for r in self.results:
            values = (
                f"#{r['rank']}",
                r['name'],
                f"{r['p']:.2f}",
                f"{r['i']:.2f}",
                f"{r['d']:.3f}",
                f"{r['tf']:.3f}",
                f"{r['target']:.1f}",
                f"{r['mean_speed']:.1f}",
                f"±{r['mae']:.2f}",
                f"±{r['std_dev']:.2f} RPM",
                f"{r['ripple_pk_pk']:.1f} RPM"
            )
            tree.insert("", "end", values=values)

        tree.pack(fill="both", expand=True)

        btn_bar = tk.Frame(modal, bg="#12161E", padx=15, pady=10)
        btn_bar.pack(fill="x")

        def apply_selected():
            sel = tree.selection()
            if not sel:
                target_res = self.results[0]
            else:
                idx = tree.index(sel[0])
                target_res = self.results[idx]

            if apply_callback:
                apply_callback(target_res)

            messagebox.showinfo("Preset Applied", f"Applied Preset #{target_res['rank']} '{target_res['name']}' to active motor controller!")
            modal.destroy()

        tk.Button(
            btn_bar, text="🚀 Apply Selected / #1 Best Preset", font=("Segoe UI", 10, "bold"),
            bg="#00E676", fg="#000", padx=12, pady=6, relief="flat", command=apply_selected
        ).pack(side="left", padx=5)

        tk.Button(
            btn_bar, text="Close", font=("Segoe UI", 9), bg="#455A64", fg="#FFF", padx=10, pady=6, relief="flat",
            command=modal.destroy
        ).pack(side="right", padx=5)
