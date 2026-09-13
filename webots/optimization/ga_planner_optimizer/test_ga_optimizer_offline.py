"""
Offline unit and validation tests for GA planner config, chromosome codecs, and reverse obstacle engine.
"""

import os
import sys
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
GA_DIR = CURR_DIR
WORKSPACE_ROOT = os.path.dirname(GA_DIR)

for p in [GA_DIR, os.path.join(GA_DIR, "planners"), os.path.join(GA_DIR, "controllers", "ga_supervisor_worker"), WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from ga_planner_config import (
    Option1BVPGenome,
    Option2SLMPCGenome,
    Option3SepticGenome,
    get_genome_class,
    compute_scalar_fitness,
    OPTION_1_BVP_GENE_SPECS,
    OPTION_2_SL_MPC_GENE_SPECS,
    OPTION_3_SEPTIC_GENE_SPECS,
)
from ga_supervisor_worker import ReverseObstacleEngine


def test_genome_codecs():
    print("Testing Genome Codecs...")
    # 1. Option 1 BVP
    g1 = Option1BVPGenome()
    arr1 = g1.to_array()
    assert len(arr1) == len(OPTION_1_BVP_GENE_SPECS)
    g1.from_array(arr1 + 0.1)
    g1_dict = g1.to_dict()
    assert "v_cruise_max" in g1_dict
    print(f"  -> Option 1 BVP: {len(arr1)} genes verified.")

    # 2. Option 2 SL-MPC
    g2 = Option2SLMPCGenome()
    arr2 = g2.to_array()
    assert len(arr2) == len(OPTION_2_SL_MPC_GENE_SPECS)
    g2.from_array(arr2)
    g2_dict = g2.to_dict()
    assert "horizon_N" in g2_dict
    assert isinstance(g2_dict["horizon_N"], int)
    print(f"  -> Option 2 SL-MPC: {len(arr2)} genes verified.")

    # 3. Option 3 Septic
    g3 = Option3SepticGenome()
    arr3 = g3.to_array()
    assert len(arr3) == len(OPTION_3_SEPTIC_GENE_SPECS)
    g3.from_array(arr3)
    g3_dict = g3.to_dict()
    assert "v_entry_max" in g3_dict
    print(f"  -> Option 3 Septic: {len(arr3)} genes verified.")


def test_reverse_obstacle_engine():
    print("Testing Reverse Obstacle Engine...")
    csv_path = os.path.join(GA_DIR, "data", "cyan_champion_trajectory.csv")
    engine = ReverseObstacleEngine(csv_path, speed_mps=0.50)
    assert engine.total_length > 10.0
    print(f"  -> Track total length: {engine.total_length:.3f} m")

    # Verify positions at s = 0, 5, 10
    p0 = engine.get_position_at_s(0.0)
    p5 = engine.get_position_at_s(5.0)
    p10 = engine.get_position_at_s(10.0)
    assert isinstance(p0, tuple) and len(p0) == 2
    assert isinstance(p5, tuple) and len(p5) == 2
    assert isinstance(p10, tuple) and len(p10) == 2
    print(f"  -> Pose at s=0m:  ({p0[0]:.3f}, {p0[1]:.3f})")
    print(f"  -> Pose at s=5m:  ({p5[0]:.3f}, {p5[1]:.3f})")
    print(f"  -> Pose at s=10m: ({p10[0]:.3f}, {p10[1]:.3f})")

    # Test reverse progression
    dt = 0.016
    n_obs = 3
    for step in range(5):
        sim_t = step * dt
        for k in range(n_obs):
            obs_s = (k * (engine.total_length / n_obs) - 0.50 * sim_t) % engine.total_length
            x, y = engine.get_position_at_s(obs_s)
            assert not np.isnan(x) and not np.isnan(y)
    print("  -> Reverse progression test passed.")


def test_fitness_computation():
    print("Testing Fitness Function...")
    # Clean run
    clean_metrics = {"distance_m": 22.5, "mean_speed_mps": 2.2, "laps_completed": 1, "crashed": False, "stalled": False}
    fit_clean = compute_scalar_fitness(clean_metrics)
    assert fit_clean > 2000.0
    print(f"  -> Clean run fitness (22.5m, 1 lap): {fit_clean:.1f}")

    # Crashed run: main penalty is lost distance, with a flat -50 deduction
    crash_metrics = {"distance_m": 5.0, "mean_speed_mps": 1.5, "laps_completed": 0, "crashed": True, "stalled": False}
    fit_crash = compute_scalar_fitness(crash_metrics)
    print(f"  -> Crashed run fitness (5.0m): {fit_crash:.1f}")
    assert fit_crash < fit_clean and fit_crash > 0.0


if __name__ == "__main__":
    test_genome_codecs()
    test_reverse_obstacle_engine()
    test_fitness_computation()
    print("\nALL GA OPTIMIZER OFFLINE TESTS PASSED!")
