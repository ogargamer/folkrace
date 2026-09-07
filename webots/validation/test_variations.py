import os
import sys
import numpy as np
from tools.ga_simulation_worker import evaluate_genome
from webots/simulation/my_project.controllers.folkrace_drive.mppi_folkrace_controller import MPPIGenome

def run_test(name, **kwargs):
    g = MPPIGenome(**kwargs)
    fit, met = evaluate_genome(g, worker_id=0, sim_duration_s=20.0, verbose=False)
    dist = met.get("distance_m", 0.0)
    speed = met.get("mean_speed_mps", 0.0)
    coll = met.get("collision_count", 0)
    status = met.get("eval_status", "UNKNOWN")
    print(f"[{name}] Dist: {dist:.2f}m | Speed: {speed:.2f}m/s | Collisions: {coll} | Status: {status} | Fitness: {fit:.1f}", flush=True)
    return dist, coll, status, g

if __name__ == "__main__":
    variations = [
        ("Base Low MU 0.68", dict(H=28, K=512, dt=0.025, lambda_temp=0.25, ar1_alpha=0.88, gamma_discount=0.96, mu_friction=0.68, w_cte=12.0, w_heading=6.0, w_speed=1.5, w_hazard=300000.0, w_prob_costmap=95.0, w_unknown=15.0, w_steer_rate=1.8, w_accel_rate=0.6, w_terminal=10.0, sigma_steer=0.08, sigma_speed=0.20, lognorm_shape=0.15, ratio_center=0.50, ratio_left=0.25, pure_pursuit_lookahead=0.52, r_inflation_m=0.125)),
        ("Low MU 0.62 Moderate", dict(H=26, K=512, dt=0.025, lambda_temp=0.22, ar1_alpha=0.90, gamma_discount=0.95, mu_friction=0.62, w_cte=14.0, w_heading=7.0, w_speed=1.8, w_hazard=350000.0, w_prob_costmap=100.0, w_unknown=15.0, w_steer_rate=2.0, w_accel_rate=0.8, w_terminal=12.0, sigma_steer=0.07, sigma_speed=0.18, lognorm_shape=0.15, ratio_center=0.45, ratio_left=0.30, pure_pursuit_lookahead=0.55, r_inflation_m=0.125)),
        ("Low MU 0.65 Wide Obstacle", dict(H=30, K=512, dt=0.028, lambda_temp=0.28, ar1_alpha=0.86, gamma_discount=0.97, mu_friction=0.65, w_cte=10.0, w_heading=5.0, w_speed=2.0, w_hazard=400000.0, w_prob_costmap=110.0, w_unknown=20.0, w_steer_rate=1.5, w_accel_rate=0.5, w_terminal=15.0, sigma_steer=0.08, sigma_speed=0.22, lognorm_shape=0.18, ratio_center=0.40, ratio_left=0.35, pure_pursuit_lookahead=0.50, r_inflation_m=0.130)),
        ("Low MU 0.70 Agile", dict(H=28, K=512, dt=0.025, lambda_temp=0.30, ar1_alpha=0.85, gamma_discount=0.95, mu_friction=0.70, w_cte=8.0, w_heading=4.5, w_speed=2.2, w_hazard=300000.0, w_prob_costmap=85.0, w_unknown=12.0, w_steer_rate=1.4, w_accel_rate=0.4, w_terminal=8.0, sigma_steer=0.09, sigma_speed=0.25, lognorm_shape=0.20, ratio_center=0.40, ratio_left=0.30, pure_pursuit_lookahead=0.48, r_inflation_m=0.120)),
    ]
    for name, kwargs in variations:
        run_test(name, **kwargs)
