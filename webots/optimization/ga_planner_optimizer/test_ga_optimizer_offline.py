"""
Offline unit and validation tests for GA planner config, chromosome codecs, and reverse obstacle engine.
"""

import os
import sys
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
GA_DIR = CURR_DIR
WEBOTS_DIR = os.path.dirname(os.path.dirname(GA_DIR))
MY_PROJECT_DIR = os.path.join(WEBOTS_DIR, "simulation", "my_project")
PLANNER_DIR = os.path.join(MY_PROJECT_DIR, "controllers", "tof_path_planner")
WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)

for p in [GA_DIR, PLANNER_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from ga_planner_config import (
    Option2SLMPCGenome,
    get_genome_class,
    compute_scalar_fitness,
    OPTION_2_SL_MPC_GENE_SPECS,
)
from obstacle_engine import ReverseObstacleEngine


def test_genome_codecs():
    print("Testing Genome Codecs...")
    g2 = Option2SLMPCGenome()
    arr2 = g2.to_array()
    assert len(arr2) == len(OPTION_2_SL_MPC_GENE_SPECS)
    g2.from_array(arr2)
    g2_dict = g2.to_dict()
    assert "horizon_N" in g2_dict
    assert isinstance(g2_dict["horizon_N"], int)
    print(f"  -> Option 2 C++ SL-MPC: {len(arr2)} genes verified.")


def test_reverse_obstacle_engine():
    print("Testing Reverse Obstacle Engine...")
    csv_path = os.path.join(GA_DIR, "data", "cyan_champion_trajectory.csv")
    engine = ReverseObstacleEngine(csv_path, speed_mps=0.50)
    assert engine.total_length > 10.0
    print(f"  -> Track total length: {engine.total_length:.3f} m")

    p0 = engine.get_position_at_s(0.0)
    p5 = engine.get_position_at_s(5.0)
    p10 = engine.get_position_at_s(10.0)
    assert isinstance(p0, tuple) and len(p0) == 2
    assert isinstance(p5, tuple) and len(p5) == 2
    assert isinstance(p10, tuple) and len(p10) == 2
    print(f"  -> Pose at s=0m:  ({p0[0]:.3f}, {p0[1]:.3f})")
    print(f"  -> Pose at s=5m:  ({p5[0]:.3f}, {p5[1]:.3f})")
    print(f"  -> Pose at s=10m: ({p10[0]:.3f}, {p10[1]:.3f})")

    dt = 0.016
    n_obs = 3
    for step in range(5):
        sim_t = step * dt
        for k in range(n_obs):
            obs_s = (k * (engine.total_length / n_obs) - 0.50 * sim_t) % engine.total_length
            x, y = engine.get_position_at_s(obs_s)
            assert not np.isnan(x) and not np.isnan(y)

    # Verify start/finish line (s=0) continuity without teleportation jump
    p_pos = engine.get_position_at_s(0.001)
    p_neg = engine.get_position_at_s(-0.001)
    gap_across_start = np.hypot(p_pos[0] - p_neg[0], p_pos[1] - p_neg[1])
    assert gap_across_start < 0.005, f"Discontinuity at start line: {gap_across_start:.4f} m"
    print(f"  -> Start line s=0 continuity verified (gap for 2mm ds = {gap_across_start*1000:.3f} mm).")
    print("  -> Reverse progression test passed.")


def test_fitness_computation():
    print("Testing Fitness Function...")
    clean_metrics = {"distance_m": 22.5, "mean_speed_mps": 2.2, "laps_completed": 1, "crashed": False, "stalled": False}
    fit_clean = compute_scalar_fitness(clean_metrics)
    assert fit_clean > 2000.0
    print(f"  -> Clean run fitness (22.5m, 1 lap): {fit_clean:.1f}")

    crash_metrics = {"distance_m": 5.0, "mean_speed_mps": 1.5, "laps_completed": 0, "crashed": True, "stalled": False}
    fit_crash = compute_scalar_fitness(crash_metrics)
    print(f"  -> Crashed run fitness (5.0m): {fit_crash:.1f}")
    assert fit_crash < fit_clean and fit_crash > 0.0


if __name__ == "__main__":
    test_genome_codecs()
    test_reverse_obstacle_engine()
    test_fitness_computation()
    print("\nALL GA OPTIMIZER OFFLINE TESTS PASSED!")
