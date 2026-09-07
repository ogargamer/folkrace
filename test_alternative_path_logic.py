"""
================================================================================
  CONCRETE TEST SUITE: MPPI ALTERNATIVE PATH & MULTI-CORRIDOR LOGIC VERIFICATION
================================================================================
"""

import os
import sys
import math
import numpy as np

WORKSPACE_ROOT = os.path.dirname(os.path.abspath(__file__))
FOLKRACE_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "folkrace_drive")
MPPI_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "mppi_controller")
SCANNER_DIR = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "tof_16ch_grid_scanner")

for p in [WORKSPACE_ROOT, FOLKRACE_DIR, MPPI_DIR, SCANNER_DIR]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from tof_pathfinders import FrenetTransformer
from mppi_folkrace_controller import MPPIController, MPPIGenome, COSTMAP_X_MIN, COSTMAP_Y_MIN, COSTMAP_RES


def run_concrete_tests():
    print("================================================================================", flush=True)
    print("          CONCRETE TEST SUITE: MPPI ALTERNATIVE PATH & CORRIDOR LOGIC          ", flush=True)
    print("================================================================================\n", flush=True)

    # 1. Load Reference Trajectories (Optimal Racing Line and Smooth Midpoint Corridors)
    c_csv = os.path.join(MPPI_DIR, "cyan_champion_trajectory.csv")
    l_csv = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "folkrace_drive", "midpoint_offset_left.csv")
    r_csv = os.path.join(WORKSPACE_ROOT, "my_project", "controllers", "folkrace_drive", "midpoint_offset_right.csv")

    c_data = np.genfromtxt(c_csv, delimiter=',', skip_header=1)
    l_data = np.genfromtxt(l_csv, delimiter=',', skip_header=1)
    r_data = np.genfromtxt(r_csv, delimiter=',', skip_header=1)

    fc = FrenetTransformer(c_data[:, 5], c_data[:, 0], c_data[:, 1], c_data[:, 3], c_data[:, 4])
    fl = FrenetTransformer(l_data[:, 5], l_data[:, 0], l_data[:, 1], l_data[:, 3], l_data[:, 4])
    fr = FrenetTransformer(r_data[:, 5], r_data[:, 0], r_data[:, 1], r_data[:, 3], r_data[:, 4])

    genome = MPPIGenome()
    ctrl = MPPIController(genome)

    # --------------------------------------------------------------------------
    # TEST 1: Kinematic Line Follower Convergence on Left & Right Corridor Lines
    # --------------------------------------------------------------------------
    print("[TEST 1] Testing Kinematic Seed Trajectories on Left and Right Corridors...")
    test_poses = [
        ("Start Straight", 0.60, 0.00, 0.00, 2.0),
        ("Turn 1 Apex", 3.088, 1.631, 0.909, 2.2),
        ("North Hairpin", 2.304, 3.850, -3.088, 1.8),
        ("West Corridor", -0.965, 2.791, -2.234, 2.2),
    ]

    for label, rx, ry, ryaw, v_mps in test_poses:
        seeds = ctrl._generate_seed_controls(
            rx, ry, ryaw, v_mps,
            frenet_helper=fc, curr_idx=0, v_target_ref=2.80,
            frenet_left=fl, frenet_right=fr
        )
        u_center, u_straight, u_left, u_right = seeds[0], seeds[1], seeds[2], seeds[3]

        # Verify shapes
        assert u_center.shape == (ctrl.H, 2), f"u_center shape mismatch: {u_center.shape}"
        assert u_left.shape == (ctrl.H, 2), f"u_left shape mismatch: {u_left.shape}"
        assert u_right.shape == (ctrl.H, 2), f"u_right shape mismatch: {u_right.shape}"

        # Verify steering bounds
        assert np.all(np.abs(u_left[:, 0]) <= ctrl.max_steer + 1e-4), "u_left exceeds max_steer"
        assert np.all(np.abs(u_right[:, 0]) <= ctrl.max_steer + 1e-4), "u_right exceeds max_steer"

        # Simulate rollout forward from u_left
        sim_x, sim_y, sim_yaw = rx, ry, ryaw
        for t in range(ctrl.H):
            delta = float(u_left[t, 0])
            spd = float(u_left[t, 1])
            sim_yaw += (spd / ctrl.L) * math.tan(delta) * ctrl.dt
            sim_x += spd * math.cos(sim_yaw) * ctrl.dt
            sim_y += spd * math.sin(sim_yaw) * ctrl.dt

        # Check distance of final point on rollout to Left line
        d_sqs_l = (fl.traj_x - sim_x) ** 2 + (fl.traj_y - sim_y) ** 2
        min_d_l = math.sqrt(float(np.min(d_sqs_l)))

        # Simulate rollout forward from u_right
        sim_x_r, sim_y_r, sim_yaw_r = rx, ry, ryaw
        for t in range(ctrl.H):
            delta = float(u_right[t, 0])
            spd = float(u_right[t, 1])
            sim_yaw_r += (spd / ctrl.L) * math.tan(delta) * ctrl.dt
            sim_x_r += spd * math.cos(sim_yaw_r) * ctrl.dt
            sim_y_r += spd * math.sin(sim_yaw_r) * ctrl.dt

        d_sqs_r = (fr.traj_x - sim_x_r) ** 2 + (fr.traj_y - sim_y_r) ** 2
        min_d_r = math.sqrt(float(np.min(d_sqs_r)))

        print(f"  - Pose: {label:<22} | Final Left dist: {min_d_l*100:4.1f} cm | Final Right dist: {min_d_r*100:4.1f} cm [PASSED]")
        assert min_d_l < 0.30, f"Left seed failed to track left corridor: {min_d_l}m"
        assert min_d_r < 0.30, f"Right seed failed to track right corridor: {min_d_r}m"

    # --------------------------------------------------------------------------
    # TEST 2: Multi-Corridor CTE Fairness (Zero Artificial Penalty on Left/Right)
    # --------------------------------------------------------------------------
    print("\n[TEST 2] Testing Multi-Corridor CTE Scoring Fairness...")
    rx, ry, ryaw, v_mps = 0.60, 0.00, 0.00, 2.0
    prob_map = np.full((101, 101), 0.10, dtype=np.float32)  # Open clear track
    hazard_mask = np.zeros((101, 101), dtype=np.uint8)

    steer_cmd, speed_cmd, diag = ctrl.compute_control(
        rx=rx, ry=ry, ryaw=ryaw, v_mps=v_mps,
        active_steer_rad=0.0,
        prob_map=prob_map, hazard_mask=hazard_mask,
        frenet_helper=fc, curr_idx=0, v_target_ref=2.80,
        frenet_left=fl, frenet_right=fr
    )

    print(f"  - Open Track: steer = {math.degrees(steer_cmd):+5.2f} deg, speed = {speed_cmd:.2f} m/s, best_cost = {diag['best_cost']:.2f}")
    assert abs(diag['best_cost']) < 1000.0, f"Unexpected high cost on open track: {diag['best_cost']}"
    assert diag['collision_rate'] < 0.05, f"Unexpected collisions on open track: {diag['collision_rate']}"

    # --------------------------------------------------------------------------
    # TEST 3: Center Obstacle Blocking -> Dynamic Alternative Corridor Avoidance
    # --------------------------------------------------------------------------
    print("\n[TEST 3] Testing Center Obstacle Avoidance (Lethal barrier on center line)...")
    ctrl_obs = MPPIController(genome)
    # Place obstacle directly in front center: X [-10cm, +10cm], Y [40cm, 90cm] in local frame
    col_c0 = int(round((-0.10 - COSTMAP_X_MIN) / COSTMAP_RES))
    col_c1 = int(round((0.10 - COSTMAP_X_MIN) / COSTMAP_RES))
    row_c0 = int(round((0.40 - COSTMAP_Y_MIN) / COSTMAP_RES))
    row_c1 = int(round((0.90 - COSTMAP_Y_MIN) / COSTMAP_RES))

    hazard_blocked = np.zeros((101, 101), dtype=np.uint8)
    hazard_blocked[row_c0:row_c1, col_c0:col_c1] = 1

    steer_avoid, speed_avoid, diag_avoid = ctrl_obs.compute_control(
        rx=rx, ry=ry, ryaw=ryaw, v_mps=v_mps,
        active_steer_rad=0.0,
        prob_map=prob_map, hazard_mask=hazard_blocked,
        frenet_helper=fc, curr_idx=0, v_target_ref=2.80,
        frenet_left=fl, frenet_right=fr
    )

    w_c = diag_avoid["seed_weights"].get("center", 0.0)
    w_l = diag_avoid["seed_weights"].get("left", 0.0)
    w_r = diag_avoid["seed_weights"].get("right", 0.0)
    w_s = diag_avoid["seed_weights"].get("straight", 0.0)

    print(f"  - Center Blocked Weights: Center={w_c:.2f}, Left={w_l:.2f}, Right={w_r:.2f}, Straight={w_s:.2f}")
    print(f"  - Avoidance Steering: {math.degrees(steer_avoid):+5.2f} deg, Commanded Speed: {speed_avoid:.2f} m/s, Pruned: {diag_avoid['collision_rate']*100:.1f}%")

    # Center seed must NOT win when blocked! Left or Right alternative corridor must dominate
    assert w_c < 0.25, f"Center path was blocked but received high weight: {w_c}"
    assert (w_l + w_r) > 0.60, f"Lateral bypass channels did not receive majority weight: Left={w_l}, Right={w_r}"
    assert speed_avoid > 0.50, f"Car stalled instead of driving around obstacle: {speed_avoid} m/s"

    print("\n================================================================================", flush=True)
    print("               ALL CONCRETE ALTERNATIVE PATH TESTS PASSED (100%)                ", flush=True)
    print("================================================================================\n", flush=True)


if __name__ == "__main__":
    run_concrete_tests()
