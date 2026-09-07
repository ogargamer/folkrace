from tools.ga_simulation_worker import evaluate_genome
from my_project.controllers.folkrace_drive.mppi_folkrace_controller import MPPIGenome

def run():
    # Test realistic inflation for 60cm track (r_inflation_m = 0.055m)
    g = MPPIGenome(
        H=28, K=512, dt=0.025,
        lambda_temp=0.25, ar1_alpha=0.88, gamma_discount=0.96,
        mu_friction=0.72,
        w_cte=4.0, w_heading=3.0, w_speed=1.5,
        w_hazard=300000.0, w_prob_costmap=60.0, w_unknown=10.0,
        w_steer_rate=1.2, w_accel_rate=0.4, w_terminal=8.0,
        sigma_steer=0.12, sigma_speed=0.20, lognorm_shape=0.15,
        ratio_center=0.20, ratio_left=0.40,  # 40% left offset, 40% right offset
        pure_pursuit_lookahead=0.45,
        r_inflation_m=0.055,  # 5.5cm inflation fits in 60cm track with obstacle!
    )
    fit, met = evaluate_genome(g, worker_id=0, sim_duration_s=20.0, verbose=True)
    dist = met.get("distance_m", 0.0)
    speed = met.get("mean_speed_mps", 0.0)
    coll = met.get("collision_count", 0)
    status = met.get("eval_status", "UNKNOWN")
    print(f"RESULT -> Dist: {dist:.2f}m | Speed: {speed:.2f}m/s | Collisions: {coll} | Status: {status} | Fitness: {fit:.1f}", flush=True)

if __name__ == "__main__":
    run()
