import os
import sys
import time

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.dirname(CURR_DIR)

for p in [CURR_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from ga_evaluator import HeadlessWebotsWorker
from ga_planner_config import Option3SepticGenome

from ga_evaluator import HeadlessWebotsWorker
from ga_planner_config import Option3SepticGenome

if __name__ == "__main__":
    print("Testing conservative candidate startup and evaluation...", flush=True)
    worker = HeadlessWebotsWorker(worker_id=0, base_port=10200)
    
    conservative_candidate = {
        "v_entry_max": 5.50,
        "v_cruise_min": 1.50,
        "mu_friction": 0.85,
        "r_inflation_m": 0.090,
        "w_clearance": 2.50,
        "w_chord_tangent": 1.50,
        "kappa_smooth_gain": 0.20,
        "pp_lookahead_gain": 0.35,
        "slew_deg_max": 12.03,
        "steer_kp": 0.75,
        "blend_w0": 0.08,
        "blend_w1": 0.20,
        "blend_w2": 0.40,
        "target_lookahead_m": 1.35,
        "probe_samples": 36,
        "num_lat_offsets": 5,
        "lat_offset_max": 0.16,
        "num_head_offsets": 1,
        "head_offset_max": 0.00,
        "num_blend_weights": 3,
        "max_extra_targets": 2,
        "w_time": 1.00,
        "w_steer_effort": 0.80,
        "aggr_dist_far_m": 1.70,
        "aggr_dist_close_m": 1.35,
        "aggr_min_scale": 0.90,
        "steer_deadband_rad": 0.002,
        "straight_lock_dist": 1.70,
    }

    print("Sending evaluation request for 10.0 seconds...", flush=True)
    t0 = time.time()
    fitness, metrics = worker.evaluate("OPTION_3_SEPTIC", conservative_candidate, duration_s=10.0)
    elapsed = time.time() - t0
    print(f"Eval finished in {elapsed:.2f}s | Fitness: {fitness:.1f} | Metrics: {metrics}", flush=True)
    worker.stop_process()
    print("Test finished successfully!", flush=True)
