"""
Brand New PID Benchmark & 5-Minute Genetic Algorithm (GA) Auto-Tuner for GM3506.
Features:
  1. 5-Minute Adaptive GA Optimizer:
     - 10 Generations x 10 Individuals = 100 candidate evaluations (300s total budget).
     - D is strictly 0.000.
     - LPF Tf strictly constrained between 0.8 ms (0.0008s) and 3.0 ms (0.0030s).
     - P gain in [0.04, 0.30], I gain in [0.40, 4.00].
     - Elitism, tournament selection, BLX-alpha crossover, and bounded Gaussian mutation.
     - Safety watchdog with early abort on instability / severe oscillation.
  2. 18-Stage Systematic Variations Benchmark:
     - Deterministic sweep across 6 LPF steps (0.8ms to 3.0ms), D=0.000, and multiple speeds.
  3. Leaderboard Modal:
     - Ranked evaluation results, CSV export, clipboard copy, and 1-click apply to active motor.
"""

import csv
import math
import random
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog


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
        self.active_mode = None  # "ga" or "sweep"

    def toggle_ga(self, btn_ga, btn_sweep, btn_results):
        """Starts or stops the 5-Minute Genetic Algorithm Auto-Tuner."""
        if self.running:
            self.abort()
            self._reset_buttons(btn_ga, btn_sweep, btn_results)
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to motor controller serial first!")
                return
            self.running = True
            self.active_mode = "ga"
            btn_ga.config(text="⏹️ Stop GA Tuner", bg="#D50000", fg="#FFF")
            btn_sweep.config(state="disabled")
            btn_results.config(state="disabled")
            self.results = []
            self.thread = threading.Thread(target=self._run_ga_worker, args=(btn_ga, btn_sweep, btn_results), daemon=True)
            self.thread.start()

    def toggle_sweep(self, btn_sweep, btn_ga, btn_results):
        """Starts or stops the 18-Stage Systematic Variations Benchmark."""
        if self.running:
            self.abort()
            self._reset_buttons(btn_ga, btn_sweep, btn_results)
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to motor controller serial first!")
                return
            self.running = True
            self.active_mode = "sweep"
            btn_sweep.config(text="⏹️ Stop Sweep", bg="#D50000", fg="#FFF")
            btn_ga.config(state="disabled")
            btn_results.config(state="disabled")
            self.results = []
            self.thread = threading.Thread(target=self._run_sweep_worker, args=(btn_sweep, btn_ga, btn_results), daemon=True)
            self.thread.start()

    def toggle(self, btn_start, btn_results):
        """Legacy compatibility wrapper defaults to 5-Min GA."""
        if self.running:
            self.abort()
            btn_start.config(text="🧬 5-Min GA Auto-Tuner", bg="#00E676", fg="#000")
            if self.results:
                btn_results.config(state="normal")
        else:
            if not self.reader.connected:
                messagebox.showerror("Connection Error", "Please connect to motor controller serial first!")
                return
            self.running = True
            self.active_mode = "ga"
            btn_start.config(text="⏹️ Stop Benchmark", bg="#D50000", fg="#FFF")
            btn_results.config(state="disabled")
            self.results = []
            # Mock sibling button for backward compatibility
            dummy_btn = tk.Button(self.root)
            self.thread = threading.Thread(target=self._run_ga_worker, args=(btn_start, dummy_btn, btn_results), daemon=True)
            self.thread.start()

    def abort(self):
        self.running = False
        self.status_var.set("🛑 EMERGENCY STOP: Benchmark / GA aborted.")
        try:
            self.reader.send_cmd("GA_STREAM 0")
            self.reader.send_cmd("x")
        except Exception:
            pass

    def _reset_buttons(self, btn_primary, btn_secondary, btn_results):
        btn_primary.config(text="🧬 5-Min GA Auto-Tuner", bg="#00E676", fg="#000", state="normal")
        btn_secondary.config(text="🧪 18-Stage Sweep", bg="#00E5FF", fg="#000", state="normal")
        if self.results:
            btn_results.config(state="normal")

    def _ensure_foc_calibration(self):
        """Checks electrical zero alignment before tuning."""
        with self.reader.lock:
            already_calibrated = getattr(self.reader, "last_calib_success", False) and getattr(self.reader, "last_calib_zero_angle", None) is not None
        if not already_calibrated:
            self.root.after(0, lambda: self.status_var.set("Phase 0: Verifying 11-Pole Zero Alignment before tuning..."))
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
                    return False
                with self.reader.lock:
                    if not getattr(self.reader, "calibrating", False) and (
                        getattr(self.reader, "last_calib_zero_angle", None) is not None or getattr(self.reader, "last_calib_failed", False)
                    ):
                        break
                time.sleep(0.3)
            time.sleep(0.5)
        return True

    def _evaluate_candidate(self, p, i, d, tf, target_rpm, stage_duration=3.0, settle_time=1.0, origin_label=""):
        """
        Commands gains and measures steady-state tracking, jitter, ripple, and voltage noise.
        Uses dedicated high-rate (50 Hz) GA stream on ESP32 without requiring full telemetry streaming.
        Includes safety watchdog that detects runaway oscillation early.
        """
        # Clear dedicated sample buffer before stage
        self.reader.clear_ga_samples()

        # Command gains and target (use atomic PID_EVAL for zero latency + individual fallback)
        self.reader.send_cmd(f"PID_EVAL {p:.4f} {i:.4f} {d:.4f} {tf:.5f} {target_rpm:.1f}")
        self.reader.send_cmd(f"p{p:.4f}\ni{i:.4f}\nd{d:.4f}\nf{tf:.5f}\nt{target_rpm:.1f}")

        stage_start = time.time()
        polled_samples = []

        while time.time() - stage_start < stage_duration:
            if not self.running:
                return None

            elapsed = time.time() - stage_start

            # Read latest telemetry for fast watchdog
            with self.reader.lock:
                act = getattr(self.reader, "last_actual_rpm", 0.0)
                uq = getattr(self.reader, "last_uq", 0.0)

            # Dynamic Safety Watchdog: aborts runaway oscillation early across any target speed
            watchdog_limit = max(35.0, target_rpm * 0.28)
            if elapsed > 0.5 and abs(act - target_rpm) > watchdog_limit:
                # Early abort to prevent motor heating / driver tripping
                self.reader.send_cmd("t0")
                time.sleep(0.05)
                return {
                    "origin": origin_label,
                    "p": p, "i": i, "d": d, "tf": tf, "tf_ms": tf * 1000.0,
                    "target": target_rpm,
                    "mean_speed": act,
                    "mae": 150.0,
                    "std_dev": 99.0,
                    "ripple_pk_pk": 190.0,
                    "mean_uq": uq,
                    "score": 999.0,
                    "status": f"⚠️ UNSTABLE @ {target_rpm:.0f} RPM"
                }

            if elapsed >= settle_time:
                polled_samples.append((act, uq))

            time.sleep(0.02) # Responsive 50 Hz watchdog check loop

        # Retrieve high-rate streaming samples captured by reader
        captured = self.reader.get_ga_samples()
        valid_captured = [(pt[1], pt[2]) for pt in captured if pt[0] >= (stage_start + settle_time)]

        # Use high-rate captured samples if available (>= 10 samples), else fall back to polled samples
        samples = valid_captured if len(valid_captured) >= 10 else polled_samples

        if not samples:
            return None

        speeds = [s[0] for s in samples]
        uqs = [s[1] for s in samples]

        mean_speed = sum(speeds) / len(speeds)
        errors = [abs(s - target_rpm) for s in speeds]
        mae = sum(errors) / len(errors)

        variance = sum((s - mean_speed) ** 2 for s in speeds) / len(speeds)
        std_dev = math.sqrt(variance)

        ripple_pk_pk = max(speeds) - min(speeds)
        mean_uq = sum(uqs) / len(uqs)
        uq_var = sum((u - mean_uq) ** 2 for u in uqs) / len(uqs)
        uq_jitter = math.sqrt(uq_var)

        # Composite Fitness Metric: Lower is better
        # Balances speed tracking accuracy (MAE), high-frequency jitter (std_dev), and commutation chatter (uq_jitter)
        score = mae + (1.2 * std_dev) + (0.3 * ripple_pk_pk) + (0.5 * uq_jitter)

        return {
            "origin": origin_label,
            "p": round(p, 4),
            "i": round(i, 4),
            "d": round(d, 4),
            "tf": round(tf, 5),
            "tf_ms": round(tf * 1000.0, 2),
            "target": target_rpm,
            "mean_speed": round(mean_speed, 2),
            "mae": round(mae, 2),
            "std_dev": round(std_dev, 2),
            "ripple_pk_pk": round(ripple_pk_pk, 2),
            "mean_uq": round(mean_uq, 2),
            "score": round(score, 3),
            "status": "✅ Stable"
        }

    def _evaluate_multi_speed(self, p, i, d, tf, speeds=(50.0, 400.0, 1000.0), stage_duration=2.0, settle_time=0.6, origin_label="", on_speed_step_cb=None):
        """
        Evaluates candidate stability and tracking across 3 distinct operating regimes:
        50 RPM (low-speed cornering), 400 RPM (cruise/base), and 1000 RPM (top-end sprint).
        Returns combined multi-speed performance. If any speed is unstable, candidate is penalized.
        """
        sub_results = []
        for spd in speeds:
            if not self.running:
                return None
            if on_speed_step_cb:
                on_speed_step_cb(spd)
            res = self._evaluate_candidate(
                p, i, d, tf, spd,
                stage_duration=stage_duration, settle_time=settle_time,
                origin_label=f"{origin_label} @ {spd:.0f} RPM"
            )
            if res is None or not self.running:
                return None
            if "UNSTABLE" in res.get("status", "") or res.get("score", 999.0) >= 500.0:
                res["origin"] = origin_label
                res["target"] = "50/400/1000"
                res["status"] = f"⚠️ Unstable @ {spd:.0f} RPM"
                return res
            sub_results.append(res)

        mean_mae = sum(r["mae"] for r in sub_results) / len(sub_results)
        mean_std = sum(r["std_dev"] for r in sub_results) / len(sub_results)
        mean_rip = sum(r["ripple_pk_pk"] for r in sub_results) / len(sub_results)
        mean_uq = sum(r["mean_uq"] for r in sub_results) / len(sub_results)
        composite_score = sum(r["score"] for r in sub_results) / len(sub_results)

        return {
            "origin": origin_label,
            "p": round(p, 4),
            "i": round(i, 4),
            "d": round(d, 4),
            "tf": round(tf, 5),
            "tf_ms": round(tf * 1000.0, 2),
            "target": "50/400/1000",
            "mean_speed": round(sum(r["mean_speed"] for r in sub_results) / len(sub_results), 1),
            "mae": round(mean_mae, 2),
            "std_dev": round(mean_std, 2),
            "ripple_pk_pk": round(mean_rip, 2),
            "mean_uq": round(mean_uq, 2),
            "score": round(composite_score, 3),
            "scores_by_speed": {spd: r["score"] for spd, r in zip(speeds, sub_results)},
            "status": "✅ 3-Speed Certified (50/400/1000)"
        }

    def _run_ga_worker(self, btn_ga, btn_sweep, btn_results):
        """
        Multi-Regime Genetic Algorithm Auto-Tuner:
        Phase 1: Base Exploration @ 400 RPM (6 Generations x 10 Individuals = 60 evaluations).
        Phase 2: Multi-Speed Gauntlet (50, 400, and 1000 RPM) across 2 Refinement Generations
                 seeded with the Top 10 unique elites from Phase 1.
        Bounds:
          P: [0.04, 0.30]
          I: [0.40, 4.00]
          D: 0.000 (fixed)
          Tf: [0.0008, 0.0030] (0.8ms to 3.0ms)
        """
        if not self._ensure_foc_calibration():
            self._reset_buttons(btn_ga, btn_sweep, btn_results)
            return

        # Ensure dedicated 50 Hz GA telemetry streaming is active on ESP32 (independent of show_telemetry)
        self.reader.send_cmd("GA_STREAM 50")
        time.sleep(0.05)

        POP_SIZE = 10
        NUM_BASE_GENS = 6
        NUM_REFINE_GENS = 2
        BASE_RPM = 400.0
        MULTI_SPEEDS = (50.0, 400.0, 1000.0)

        CANDIDATE_DURATION = 2.4
        SETTLE_TIME = 0.7

        P_MIN, P_MAX = 0.04, 0.30
        I_MIN, I_MAX = 0.40, 4.00
        TF_MIN, TF_MAX = 0.0008, 0.0030  # 0.8 ms to 3.0 ms
        D_VAL = 0.000

        # Generation 0 Seeds (Proven diverse operating points in the hypercube)
        seeds = [
            {"p": 0.12, "i": 1.20, "tf": 0.0015},  # Crisp Balanced (1.5ms)
            {"p": 0.10, "i": 1.00, "tf": 0.0030},  # Smooth Baseline (3.0ms)
            {"p": 0.14, "i": 1.60, "tf": 0.0010},  # Fast Low-Lag (1.0ms)
            {"p": 0.08, "i": 0.80, "tf": 0.0020},  # Gentle Damped (2.0ms)
            {"p": 0.16, "i": 2.20, "tf": 0.0008},  # Punchy Ultra-Fast (0.8ms)
            {"p": 0.18, "i": 2.80, "tf": 0.0025},  # Stiff High-Authority (2.5ms)
            {"p": 0.06, "i": 0.60, "tf": 0.0012},  # Quiet Low-Current (1.2ms)
        ]

        population = list(seeds)
        while len(population) < POP_SIZE:
            population.append({
                "p": round(random.uniform(P_MIN, P_MAX), 3),
                "i": round(random.uniform(I_MIN, I_MAX), 2),
                "tf": round(random.uniform(TF_MIN, TF_MAX), 5)
            })

        total_base_evals = NUM_BASE_GENS * POP_SIZE
        total_refine_evals = NUM_REFINE_GENS * POP_SIZE
        total_evaluations = total_base_evals + total_refine_evals
        eval_count = 0
        start_time = time.time()
        best_overall = None
        was_aborted = False

        # --- PHASE 1: Base GA Exploration @ 400 RPM ---
        for gen_idx in range(NUM_BASE_GENS):
            if not self.running:
                was_aborted = True
                break

            gen_evaluated = []
            for ind_idx, indiv in enumerate(population):
                if not self.running:
                    was_aborted = True
                    break

                eval_count += 1
                total_progress = min(99.0, (eval_count / total_evaluations) * 100.0)
                remaining_sec = max(0, int((total_base_evals - eval_count) * (CANDIDATE_DURATION + 0.1) + total_refine_evals * 5.5))

                p_val, i_val, tf_val = indiv["p"], indiv["i"], indiv["tf"]
                origin_name = f"🧬 GA Base Gen {gen_idx+1} [#{ind_idx+1}]"

                def _update_status(g=gen_idx+1, ind=ind_idx+1, p=p_val, i=i_val, tf_ms=tf_val*1000.0,
                                   prog=total_progress, rem=remaining_sec, b=best_overall):
                    self.progress_var.set(prog)
                    b_str = f" | Best: {b['score']:.2f} (P={b['p']}, I={b['i']}, Tf={b['tf_ms']}ms)" if b else ""
                    self.status_var.set(
                        f"🧬 Phase 1 Base GA [{g}/{NUM_BASE_GENS}] Indiv [{ind}/{POP_SIZE}] @ 400 RPM: "
                        f"P={p:.3f}, I={i:.2f}, Tf={tf_ms:.2f}ms{b_str} | ~{rem}s left"
                    )
                self.root.after(0, _update_status)

                res = self._evaluate_candidate(
                    p_val, i_val, D_VAL, tf_val, BASE_RPM,
                    stage_duration=CANDIDATE_DURATION, settle_time=SETTLE_TIME, origin_label=origin_name
                )

                if res is not None:
                    gen_evaluated.append(res)
                    self.results.append(res)
                    if best_overall is None or res["score"] < best_overall["score"]:
                        best_overall = res

            if not self.running or len(gen_evaluated) < 2:
                break

            gen_evaluated.sort(key=lambda x: x["score"])

            # Reproduction for next Phase 1 generation
            if gen_idx < NUM_BASE_GENS - 1:
                next_pop = []
                next_pop.append({"p": gen_evaluated[0]["p"], "i": gen_evaluated[0]["i"], "tf": gen_evaluated[0]["tf"]})
                next_pop.append({"p": gen_evaluated[1]["p"], "i": gen_evaluated[1]["i"], "tf": gen_evaluated[1]["tf"]})

                def select_parent():
                    candidates = random.sample(gen_evaluated[:min(6, len(gen_evaluated))], 2)
                    candidates.sort(key=lambda x: x["score"])
                    return candidates[0]

                while len(next_pop) < POP_SIZE:
                    p1 = select_parent()
                    p2 = select_parent()

                    def cross(v1, v2, vmin, vmax):
                        low = min(v1, v2)
                        high = max(v1, v2)
                        d = high - low
                        val = random.uniform(low - 0.3 * d, high + 0.3 * d)
                        return max(vmin, min(vmax, val))

                    child_p = cross(p1["p"], p2["p"], P_MIN, P_MAX)
                    child_i = cross(p1["i"], p2["i"], I_MIN, I_MAX)
                    child_tf = cross(p1["tf"], p2["tf"], TF_MIN, TF_MAX)

                    if random.random() < 0.25:
                        child_p = max(P_MIN, min(P_MAX, child_p + random.gauss(0, 0.025)))
                    if random.random() < 0.25:
                        child_i = max(I_MIN, min(I_MAX, child_i + random.gauss(0, 0.35)))
                    if random.random() < 0.25:
                        child_tf = max(TF_MIN, min(TF_MAX, child_tf + random.gauss(0, 0.0003)))

                    next_pop.append({
                        "p": round(child_p, 4),
                        "i": round(child_i, 3),
                        "tf": round(child_tf, 5)
                    })
                population = next_pop

        # --- PHASE 2: Multi-Speed Gauntlet (50, 400, 1000 RPM) for 2 Generations ---
        if self.running and not was_aborted:
            # Extract Top 10 unique elites from Phase 1
            seen = set()
            top_elites = []
            stable_p1 = [r for r in self.results if "Stable" in r.get("status", "")]
            stable_p1.sort(key=lambda x: x["score"])
            for r in stable_p1:
                key = (round(r["p"], 3), round(r["i"], 2), round(r["tf"], 5))
                if key not in seen:
                    seen.add(key)
                    top_elites.append({"p": r["p"], "i": r["i"], "tf": r["tf"]})
                if len(top_elites) >= POP_SIZE:
                    break

            while len(top_elites) < POP_SIZE:
                base = top_elites[0] if top_elites else {"p": 0.12, "i": 1.2, "tf": 0.0015}
                top_elites.append({
                    "p": round(max(P_MIN, min(P_MAX, base["p"] + random.gauss(0, 0.02))), 3),
                    "i": round(max(I_MIN, min(I_MAX, base["i"] + random.gauss(0, 0.3))), 2),
                    "tf": round(max(TF_MIN, min(TF_MAX, base["tf"] + random.gauss(0, 0.0003))), 5)
                })

            multi_pop = list(top_elites)

            for refine_gen in range(NUM_REFINE_GENS):
                if not self.running:
                    was_aborted = True
                    break

                refine_evaluated = []
                for ind_idx, indiv in enumerate(multi_pop):
                    if not self.running:
                        was_aborted = True
                        break

                    eval_count += 1
                    total_progress = min(99.0, (eval_count / total_evaluations) * 100.0)
                    rem_refine = max(0, int((total_evaluations - eval_count) * 5.5))

                    p_val, i_val, tf_val = indiv["p"], indiv["i"], indiv["tf"]
                    origin_name = f"🧬 GA 3-Speed Gen {refine_gen+1} [#{ind_idx+1}]"

                    def _update_status_multi(rg=refine_gen+1, ind=ind_idx+1, p=p_val, i=i_val, tf_ms=tf_val*1000.0,
                                             prog=total_progress, rem=rem_refine, b=best_overall):
                        self.progress_var.set(prog)
                        b_str = f" | Champion: {b['score']:.2f} ({b['status']})" if b and "Certified" in b.get("status", "") else ""
                        self.status_var.set(
                            f"🏎️ Phase 2 [Gen {rg}/{NUM_REFINE_GENS}] Elite [{ind}/{POP_SIZE}]: Testing 50/400/1000 RPM "
                            f"(P={p:.3f}, I={i:.2f}, Tf={tf_ms:.2f}ms){b_str} | ~{rem}s left"
                        )
                    self.root.after(0, _update_status_multi)

                    def _on_spd(cur_spd, rg=refine_gen+1, ind=ind_idx+1, p=p_val, i=i_val, tf_ms=tf_val*1000.0, prog=total_progress):
                        self.progress_var.set(prog)
                        self.status_var.set(
                            f"🏎️ Phase 2 [Gen {rg}/{NUM_REFINE_GENS}] Elite [{ind}/{POP_SIZE}] -> Testing {cur_spd:.0f} RPM "
                            f"(P={p:.3f}, I={i:.2f}, Tf={tf_ms:.2f}ms)"
                        )

                    multi_res = self._evaluate_multi_speed(
                        p_val, i_val, D_VAL, tf_val, speeds=MULTI_SPEEDS,
                        stage_duration=2.0, settle_time=0.6, origin_label=origin_name,
                        on_speed_step_cb=lambda s: self.root.after(0, lambda sp=s: _on_spd(sp))
                    )

                    if multi_res is not None:
                        refine_evaluated.append(multi_res)
                        self.results.append(multi_res)
                        if "Certified" in multi_res.get("status", ""):
                            if best_overall is None or "Certified" not in best_overall.get("status", "") or multi_res["score"] < best_overall["score"]:
                                best_overall = multi_res

                if not self.running or len(refine_evaluated) < 2:
                    break

                refine_evaluated.sort(key=lambda x: (0 if "Certified" in x.get("status", "") else 1, x["score"]))

                # Reproduction for Refine Generation 2
                if refine_gen < NUM_REFINE_GENS - 1:
                    next_multi_pop = []
                    next_multi_pop.append({"p": refine_evaluated[0]["p"], "i": refine_evaluated[0]["i"], "tf": refine_evaluated[0]["tf"]})
                    next_multi_pop.append({"p": refine_evaluated[1]["p"], "i": refine_evaluated[1]["i"], "tf": refine_evaluated[1]["tf"]})

                    def select_multi_parent():
                        cands = random.sample(refine_evaluated[:min(5, len(refine_evaluated))], 2)
                        cands.sort(key=lambda x: (0 if "Certified" in x.get("status", "") else 1, x["score"]))
                        return cands[0]

                    while len(next_multi_pop) < POP_SIZE:
                        p1 = select_multi_parent()
                        p2 = select_multi_parent()

                        def cross(v1, v2, vmin, vmax):
                            low = min(v1, v2)
                            high = max(v1, v2)
                            d = high - low
                            val = random.uniform(low - 0.25 * d, high + 0.25 * d)
                            return max(vmin, min(vmax, val))

                        child_p = cross(p1["p"], p2["p"], P_MIN, P_MAX)
                        child_i = cross(p1["i"], p2["i"], I_MIN, I_MAX)
                        child_tf = cross(p1["tf"], p2["tf"], TF_MIN, TF_MAX)

                        if random.random() < 0.30:
                            child_p = max(P_MIN, min(P_MAX, child_p + random.gauss(0, 0.015)))
                        if random.random() < 0.30:
                            child_i = max(I_MIN, min(I_MAX, child_i + random.gauss(0, 0.20)))
                        if random.random() < 0.30:
                            child_tf = max(TF_MIN, min(TF_MAX, child_tf + random.gauss(0, 0.0002)))

                        next_multi_pop.append({
                            "p": round(child_p, 4),
                            "i": round(child_i, 3),
                            "tf": round(child_tf, 5)
                        })
                    multi_pop = next_multi_pop

        self.running = False
        try:
            self.reader.send_cmd("GA_STREAM 0")
        except Exception:
            pass
        if was_aborted:
            self.reader.send_cmd("x")
        else:
            self.reader.send_cmd("t0")

        def _finish(aborted=was_aborted):
            btn_ga.config(text="🧬 5-Min GA Auto-Tuner", bg="#00E676", fg="#000", state="normal")
            btn_sweep.config(text="🧪 18-Stage Sweep", bg="#00E5FF", fg="#000", state="normal")
            if aborted:
                self.status_var.set("GA Optimization Aborted.")
                if self.results:
                    btn_results.config(state="normal")
            elif self.results:
                # Prioritize 3-Speed Certified candidates at top of leaderboard
                self.results.sort(key=lambda x: (0 if "Certified" in x.get("status", "") else 1, x['score']))
                for rank_idx, r in enumerate(self.results):
                    r['rank'] = rank_idx + 1

                best = self.results[0]
                self.progress_var.set(100.0)
                cert_str = " (3-Speed Certified @ 50, 400 & 1000 RPM)" if "Certified" in best.get("status", "") else ""
                self.status_var.set(
                    f"✅ Multi-Speed GA Complete! Winner: #{best['rank']} (P={best['p']}, I={best['i']}, Tf={best['tf_ms']}ms, D=0){cert_str} "
                    f"| Jitter: ±{best['std_dev']:.2f} RPM | MAE: ±{best['mae']:.2f} RPM"
                )
                btn_results.config(state="normal")
                if self.on_benchmark_finish_cb:
                    self.root.after(200, self.on_benchmark_finish_cb)

        self.root.after(0, _finish)

    def _run_sweep_worker(self, btn_sweep, btn_ga, btn_results):
        """
        18-Stage Systematic Variations Benchmark:
        Strictly tests D=0.000 across 6 LPF steps (0.8ms to 3.0ms) and curated P & I bounds.
        """
        if not self._ensure_foc_calibration():
            self._reset_buttons(btn_ga, btn_sweep, btn_results)
            return

        presets = [
            # Group 1: 0.8 ms LPF (Ultra-Fast Razor Response)
            {"name": "0.8ms Ultra-Fast Baseline",   "p": 0.10, "i": 1.00, "d": 0.000, "tf": 0.0008, "target": 80.0},
            {"name": "0.8ms Crisp Punchy",         "p": 0.14, "i": 1.50, "d": 0.000, "tf": 0.0008, "target": 120.0},
            {"name": "0.8ms Stiff High-Speed",     "p": 0.18, "i": 2.20, "d": 0.000, "tf": 0.0008, "target": 200.0},

            # Group 2: 1.2 ms LPF (Direct Cornering Fast Filter)
            {"name": "1.2ms Low-Current Smooth",   "p": 0.08, "i": 0.80, "d": 0.000, "tf": 0.0012, "target": 80.0},
            {"name": "1.2ms Balanced Track",       "p": 0.12, "i": 1.30, "d": 0.000, "tf": 0.0012, "target": 120.0},
            {"name": "1.2ms Aggressive Sprint",    "p": 0.16, "i": 2.00, "d": 0.000, "tf": 0.0012, "target": 250.0},

            # Group 3: 1.5 ms LPF (Crisp Balanced)
            {"name": "1.5ms Smooth Cruiser",       "p": 0.10, "i": 1.00, "d": 0.000, "tf": 0.0015, "target": 80.0},
            {"name": "1.5ms Folkrace Direct",      "p": 0.14, "i": 1.60, "d": 0.000, "tf": 0.0015, "target": 150.0},
            {"name": "1.5ms High-I Authority",     "p": 0.12, "i": 2.50, "d": 0.000, "tf": 0.0015, "target": 150.0},

            # Group 4: 2.0 ms LPF (Low-Lag Damped)
            {"name": "2.0ms Gentle Low-Vibration", "p": 0.08, "i": 0.90, "d": 0.000, "tf": 0.0020, "target": 80.0},
            {"name": "2.0ms Balanced Optimal",     "p": 0.12, "i": 1.40, "d": 0.000, "tf": 0.0020, "target": 120.0},
            {"name": "2.0ms High Torque Rigidity", "p": 0.18, "i": 2.40, "d": 0.000, "tf": 0.0020, "target": 250.0},

            # Group 5: 2.5 ms LPF (Quiet Damped Tracking)
            {"name": "2.5ms Quiet Baseline",       "p": 0.10, "i": 1.10, "d": 0.000, "tf": 0.0025, "target": 80.0},
            {"name": "2.5ms Stiff Load Recovery",  "p": 0.15, "i": 2.20, "d": 0.000, "tf": 0.0025, "target": 150.0},
            {"name": "2.5ms Top-End 350 RPM",      "p": 0.16, "i": 1.80, "d": 0.000, "tf": 0.0025, "target": 350.0},

            # Group 6: 3.0 ms LPF (Firmware Default Smooth)
            {"name": "3.0ms Default Smooth",       "p": 0.12, "i": 1.20, "d": 0.000, "tf": 0.0030, "target": 80.0},
            {"name": "3.0ms Heavy Damped Sprint",  "p": 0.15, "i": 1.80, "d": 0.000, "tf": 0.0030, "target": 180.0},
            {"name": "3.0ms Maximum Velocity 400", "p": 0.16, "i": 1.60, "d": 0.000, "tf": 0.0030, "target": 400.0},
        ]

        # Ensure dedicated 50 Hz GA telemetry streaming is active on ESP32
        self.reader.send_cmd("GA_STREAM 50")
        time.sleep(0.05)

        total_stages = len(presets)
        stage_duration = 6.0
        settle_time = 1.5
        start_time = time.time()
        was_aborted = False

        for idx, p in enumerate(presets):
            if not self.running:
                was_aborted = True
                break

            total_elapsed = time.time() - start_time
            total_progress = min(99.0, (idx / total_stages) * 100.0)
            remaining_sec = max(0, int((total_stages - idx) * stage_duration))

            def _update_status(i=idx, pr=p, prog=total_progress, rem=remaining_sec):
                self.progress_var.set(prog)
                self.status_var.set(
                    f"🧪 Sweep [{i+1}/{total_stages}] '{pr['name']}' (P={pr['p']}, I={pr['i']}, Tf={pr['tf']*1000:.1f}ms, D=0) | "
                    f"Target: {pr['target']:.0f} RPM | {rem}s remaining"
                )
            self.root.after(0, _update_status)

            res = self._evaluate_candidate(
                p["p"], p["i"], p["d"], p["tf"], p["target"],
                stage_duration=stage_duration, settle_time=settle_time, origin_label=f"🧪 {p['name']}"
            )

            if res is not None:
                self.results.append(res)

        self.running = False
        try:
            self.reader.send_cmd("GA_STREAM 0")
        except Exception:
            pass
        if was_aborted:
            self.reader.send_cmd("x")
        else:
            self.reader.send_cmd("t0")

        def _finish(aborted=was_aborted):
            btn_sweep.config(text="🧪 18-Stage Sweep", bg="#00E5FF", fg="#000", state="normal")
            btn_ga.config(text="🧬 5-Min GA Auto-Tuner", bg="#00E676", fg="#000", state="normal")
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
                    f"✅ Sweep Complete! Best Preset: #{best['rank']} '{best['origin']}' "
                    f"(P={best['p']}, I={best['i']}, Tf={best['tf_ms']}ms | Jitter: ±{best['std_dev']:.2f} RPM)"
                )
                btn_results.config(state="normal")
                if self.on_benchmark_finish_cb:
                    self.root.after(200, self.on_benchmark_finish_cb)

        self.root.after(0, _finish)

    def show_results_modal(self, apply_callback=None):
        if not self.results:
            messagebox.showinfo("No Results", "Please run either the 5-minute GA tuner or variations sweep first!")
            return

        modal = tk.Toplevel(self.root)
        modal.title("🏆 PID Benchmark & 5-Min GA Leaderboard (D=0, LPF 0.8-3.0ms)")
        modal.geometry("980x560")
        modal.configure(bg="#12161E")
        modal.transient(self.root)

        header = tk.Frame(modal, bg="#1A2230", padx=15, pady=12)
        header.pack(fill="x")

        best = self.results[0]
        tk.Label(header, text="🏆 PID TUNING & SPEED VARIATION LEADERBOARD", font=("Segoe UI", 14, "bold"), fg="#00E5FF", bg="#1A2230").pack(anchor="w")
        tk.Label(
            header,
            text=f"Top Performer: #{best.get('rank', 1)} {best['origin']} | P={best['p']}, I={best['i']}, D={best['d']} (D=0), "
                 f"Tf={best['tf_ms']:.2f} ms | Jitter: ±{best['std_dev']:.2f} RPM | MAE: ±{best['mae']:.2f} RPM | Ripple: {best['ripple_pk_pk']:.1f} RPM",
            font=("Segoe UI", 10, "bold"),
            fg="#69F0AE",
            bg="#1A2230"
        ).pack(anchor="w", pady=(4, 0))

        table_frame = tk.Frame(modal, bg="#12161E", padx=10, pady=10)
        table_frame.pack(fill="both", expand=True)

        cols = ("Rank", "Source / Preset", "P Gain", "I Gain", "D Gain", "LPF (ms)", "Target", "Actual", "MAE Err", "StdDev (Jitter)", "Ripple Pk-Pk", "Score")
        tree = ttk.Treeview(table_frame, columns=cols, show="headings", height=12)

        col_widths = [45, 190, 55, 55, 50, 65, 90, 65, 70, 95, 85, 60]
        for c, w in zip(cols, col_widths):
            tree.heading(c, text=c)
            tree.column(c, width=w, anchor="center")

        for r in self.results:
            tgt_disp = f"{r['target']:.1f}" if isinstance(r.get('target'), (int, float)) else str(r.get('target', ''))
            values = (
                f"#{r.get('rank', '-')}",
                r['origin'],
                f"{r['p']:.3f}",
                f"{r['i']:.2f}",
                f"{r['d']:.3f}",
                f"{r['tf_ms']:.2f} ms",
                tgt_disp,
                f"{r['mean_speed']:.1f}",
                f"±{r['mae']:.2f}",
                f"±{r['std_dev']:.2f} RPM",
                f"{r['ripple_pk_pk']:.1f} RPM",
                f"{r['score']:.2f}"
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

            messagebox.showinfo("Preset Applied", f"Applied Preset #{target_res.get('rank', 1)} '{target_res['origin']}' to active motor controller!")
            modal.destroy()

        def copy_report():
            lines = [
                "=== PID BENCHMARK & MULTI-SPEED GA AUTO-TUNER LEADERBOARD ===",
                f"Top Ranked Winner: #{best.get('rank', 1)} {best['origin']} (Score: {best['score']:.2f})",
                f"Gains: P={best['p']}, I={best['i']}, D={best['d']}, Tf={best['tf_ms']:.2f}ms",
                f"Metrics: Jitter=±{best['std_dev']:.2f} RPM, MAE=±{best['mae']:.2f} RPM, Ripple={best['ripple_pk_pk']:.1f} RPM",
                "----------------------------------------------------------------------------------------------------------------",
                f"{'Rank':<5} | {'Source / Preset':<24} | {'P':<6} | {'I':<6} | {'D':<6} | {'LPF (ms)':<8} | {'Target':<12} | {'Actual':<6} | {'StdDev':<10} | {'Score':<6}",
                "------+--------------------------+--------+--------+--------+----------+--------------+--------+------------+-------"
            ]
            for r in self.results:
                tgt_str = f"{r['target']:<12.1f}" if isinstance(r.get('target'), (int, float)) else f"{str(r.get('target', '')):<12}"
                lines.append(
                    f"#{r.get('rank', '-'):<4} | {r['origin']:<24} | {r['p']:<6.3f} | {r['i']:<6.2f} | {r['d']:<6.3f} | "
                    f"{r['tf_ms']:<8.2f} | {tgt_str} | {r['mean_speed']:<6.1f} | ±{r['std_dev']:<8.2f} | {r['score']:<6.2f}"
                )
            text_rep = "\n".join(lines)
            self.root.clipboard_clear()
            self.root.clipboard_append(text_rep)
            messagebox.showinfo("Report Copied", "Leaderboard text successfully copied to clipboard!")

        def export_csv():
            filename = filedialog.asksaveasfilename(
                title="Save Benchmark Leaderboard CSV",
                defaultextension=".csv",
                filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
                initialfile="pid_benchmark_results.csv"
            )
            if not filename:
                return
            try:
                with open(filename, "w", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow(["Rank", "Source_Preset", "P", "I", "D", "LPF_Tf_s", "LPF_Tf_ms", "Target_RPM", "Actual_RPM", "MAE_RPM", "StdDev_RPM", "Ripple_RPM", "Score"])
                    for r in self.results:
                        writer.writerow([
                            r.get('rank', '-'), r['origin'], r['p'], r['i'], r['d'], r['tf'], r['tf_ms'],
                            r['target'], r['mean_speed'], r['mae'], r['std_dev'], r['ripple_pk_pk'], r['score']
                        ])
                messagebox.showinfo("CSV Exported", f"Successfully saved {len(self.results)} benchmark records to {filename}!")
            except Exception as e:
                messagebox.showerror("Export Error", f"Failed to export CSV: {e}")

        tk.Button(
            btn_bar, text="🚀 Apply Selected / #1 Best Preset", font=("Segoe UI", 10, "bold"),
            bg="#00E676", fg="#000", padx=12, pady=6, relief="flat", command=apply_selected
        ).pack(side="left", padx=5)

        tk.Button(
            btn_bar, text="📋 Copy Report", font=("Segoe UI", 9, "bold"),
            bg="#00E5FF", fg="#000", padx=10, pady=6, relief="flat", command=copy_report
        ).pack(side="left", padx=5)

        tk.Button(
            btn_bar, text="💾 Export CSV", font=("Segoe UI", 9, "bold"),
            bg="#FFD600", fg="#000", padx=10, pady=6, relief="flat", command=export_csv
        ).pack(side="left", padx=5)

        tk.Button(
            btn_bar, text="Close", font=("Segoe UI", 9), bg="#455A64", fg="#FFF", padx=10, pady=6, relief="flat",
            command=modal.destroy
        ).pack(side="right", padx=5)
