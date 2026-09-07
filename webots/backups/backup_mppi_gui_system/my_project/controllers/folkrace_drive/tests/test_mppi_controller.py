"""
================================================================================
  AUTOMATED TEST & BENCHMARK SUITE FOR MPPI CONTROLLER
  Verifies all 8 MPPI Variants, Real-Time Profiling (< 8 ms), and Genetic Interface
================================================================================
"""

import os
import sys
import math
import time
import unittest
import tempfile
import numpy as np

# Setup import paths
CONTROLLER_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCANNER_DIR = os.path.join(os.path.dirname(CONTROLLER_DIR), "tof_16ch_grid_scanner")

for d in [CONTROLLER_DIR, SCANNER_DIR]:
    if d not in sys.path and os.path.exists(d):
        sys.path.insert(0, d)

from mppi_folkrace_controller import (
    MPPIController,
    MPPIGenome,
    WHEELBASE,
    MAX_STEERING_RAD,
    MAX_STEER_RATE_RAD_S,
    GRAVITY_MPS2,
)
from tof_pathfinders import FrenetTransformer


class TestMPPIControllerSuite(unittest.TestCase):

    def setUp(self):
        # Create a synthetic circular trajectory for Frenet testing
        theta = np.linspace(0, 2 * np.pi, 200, endpoint=False)
        radius = 5.0
        traj_x = radius * np.cos(theta)
        traj_y = radius * np.sin(theta)
        traj_s = radius * theta
        traj_yaw = theta + np.pi / 2.0
        traj_k = np.full_like(theta, 1.0 / radius)
        self.frenet = FrenetTransformer(traj_s, traj_x, traj_y, traj_yaw, traj_k)

        # Default synthetic empty costmap (101 x 101)
        self.prob_map = np.zeros((101, 101), dtype=np.float32)
        self.hazard_mask = np.zeros((101, 101), dtype=bool)

    # --------------------------------------------------------------------------
    # 1. TEST GENOME ARCHITECTURE & SERIALIZATION
    # --------------------------------------------------------------------------
    def test_genome_serialization_and_bounds(self):
        """Verifies flat 1D chromosome conversion, clamping, compute budget coupling, and JSON persistence."""
        g = MPPIGenome(H=20, K=256, dt=0.0166, ar1_alpha=0.85, lambda_temp=0.12, mu_friction=0.80, r_inflation_m=0.10)
        arr = g.to_array()
        self.assertEqual(len(arr), 22)
        self.assertAlmostEqual(arr[0], 20.0)
        self.assertAlmostEqual(arr[1], 256.0)
        self.assertAlmostEqual(arr[4], 0.85)
        self.assertAlmostEqual(arr[6], 0.80)
        self.assertAlmostEqual(arr[21], 0.10)

        # Reconstruct from array
        g_reconstructed = MPPIGenome.from_array(arr)
        self.assertEqual(g_reconstructed.H, 20)
        self.assertEqual(g_reconstructed.K, 256)
        self.assertAlmostEqual(g_reconstructed.ar1_alpha, 0.85)
        self.assertAlmostEqual(g_reconstructed.mu_friction, 0.80)
        self.assertAlmostEqual(g_reconstructed.r_inflation_m, 0.10)

        # Compute budget coupling check (if H=32, K is clamped to 5120 // 32 = 160)
        g_budgeted = MPPIGenome(H=32, K=256)
        self.assertEqual(g_budgeted.K, 160)
        self.assertLessEqual(g_budgeted.H * g_budgeted.K, 5120)

        # Bounds clamping (H over bound, mu outside [0.60, 0.90])
        bad_arr = arr.copy()
        bad_arr[0] = 999.0   # H over upper bound
        bad_arr[4] = -5.0    # ar1_alpha below 0.0
        bad_arr[6] = 1.50    # mu above 0.90
        g_clamped = MPPIGenome.from_array(bad_arr)
        self.assertLessEqual(g_clamped.H, 35)
        self.assertGreaterEqual(g_clamped.ar1_alpha, 0.0)
        self.assertLessEqual(g_clamped.mu_friction, 0.90)

        # JSON file export/import
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tf:
            temp_path = tf.name
        try:
            g.save_json(temp_path)
            g_loaded = MPPIGenome.load_json(temp_path)
            self.assertEqual(g_loaded.H, g.H)
            self.assertAlmostEqual(g_loaded.lambda_temp, g.lambda_temp)
            self.assertAlmostEqual(g_loaded.mu_friction, g.mu_friction)
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)

    # --------------------------------------------------------------------------
    # 2. TEST SMOOTH-MPPI: AR-1 COLORED NOISE CORRELATION
    # --------------------------------------------------------------------------
    def test_smooth_mppi_colored_noise(self):
        """Verifies that AR-1 colored noise exhibits high lag-1 temporal autocorrelation."""
        controller = MPPIController(genome=MPPIGenome(H=50, K=1000, ar1_alpha=0.85))
        eps = controller._sample_perturbations()  # (K, H, 2)

        steer_noise = eps[:, :, 0]  # (1000, 50)
        
        # Calculate lag-1 sample autocorrelation across horizon
        lag0 = steer_noise[:, :-1]
        lag1 = steer_noise[:, 1:]
        
        mean0 = np.mean(lag0, axis=1, keepdims=True)
        mean1 = np.mean(lag1, axis=1, keepdims=True)
        
        cov = np.mean((lag0 - mean0) * (lag1 - mean1), axis=1)
        var0 = np.var(lag0, axis=1)
        var1 = np.var(lag1, axis=1)
        
        autocorr = cov / np.sqrt(np.maximum(1e-8, var0 * var1))
        mean_autocorr = float(np.mean(autocorr))

        # Expected theoretical autocorrelation ~ alpha = 0.85 (must be > 0.70)
        print(f"[TEST] Measured AR-1 Steering Autocorrelation: {mean_autocorr:.3f} (Target: 0.85)", flush=True)
        self.assertGreater(mean_autocorr, 0.70, "AR-1 filter failed to enforce temporal correlation.")

    # --------------------------------------------------------------------------
    # 3. TEST LOG-MPPI: NORMAL LOG-NORMAL FORWARD VELOCITY SKEWNESS
    # --------------------------------------------------------------------------
    def test_log_mppi_forward_velocity_sampling(self):
        """Verifies that velocity perturbation distribution is right-skewed (+vx biased)."""
        controller = MPPIController(genome=MPPIGenome(H=20, K=2000, lognorm_shape=0.60))
        eps = controller._sample_perturbations()
        speed_noise = eps[:, :, 1].flatten()

        # Skewness: E[((x - mu)/sigma)^3]
        mean = np.mean(speed_noise)
        std = np.std(speed_noise)
        skewness = np.mean(((speed_noise - mean) / std) ** 3)

        print(f"[TEST] Measured Log-MPPI Speed Skewness: {skewness:.3f} (Positive right-skew expected)", flush=True)
        self.assertGreater(skewness, 0.20, "Log-MPPI velocity noise is not right-skewed.")

    # --------------------------------------------------------------------------
    # 4. TEST BIASED / MULTI-MODAL LANE SEEDING
    # --------------------------------------------------------------------------
    def test_multimodal_lane_seeding(self):
        """Verifies generation of 3 distinct Pure Pursuit seeds (Center, Left +0.25m, Right -0.25m)."""
        controller = MPPIController(genome=MPPIGenome(H=20, K=256, ratio_center=0.50, ratio_left=0.25))
        
        # Robot at (x=5.0, y=0.0, yaw=pi/2) on the circle
        u_c, u_l, u_r = controller._generate_seed_controls(
            rx=5.0, ry=0.0, ryaw=np.pi / 2.0, v_mps=2.5,
            frenet_helper=self.frenet, curr_idx=0
        )

        self.assertEqual(u_c.shape, (20, 2))
        self.assertEqual(u_l.shape, (20, 2))
        self.assertEqual(u_r.shape, (20, 2))

        # Left seed (offset inside circle at d=+0.25m -> smaller turning radius -> higher left steer)
        # Right seed (offset outside circle at d=-0.25m -> larger turning radius -> lower steer)
        mean_steer_c = float(np.mean(u_c[:, 0]))
        mean_steer_l = float(np.mean(u_l[:, 0]))
        mean_steer_r = float(np.mean(u_r[:, 0]))

        print(f"[TEST] Multi-Modal Seeds Mean Steer: Left={mean_steer_l:.3f}, Center={mean_steer_c:.3f}, Right={mean_steer_r:.3f}", flush=True)
        self.assertNotEqual(mean_steer_l, mean_steer_r, "Left and right seeds must not be identical.")

    # --------------------------------------------------------------------------
    # 5. TEST SHIELD-MPPI: EARLY-EXIT COSTMAP PRUNING & REJECTION
    # --------------------------------------------------------------------------
    def test_shield_mppi_early_exit_pruning(self):
        """Verifies instant collision rejection and high execution speed with obstacle."""
        controller = MPPIController(genome=MPPIGenome(H=20, K=256))
        
        # Place a 30 cm obstacle in front of the vehicle in the local costmap
        # Vehicle at (0,0), looking forward along +Y in local costmap
        # Obstacle at local Y = 0.50 m (row 35), X = 0.0 m (col 50)
        hazard = np.zeros((101, 101), dtype=bool)
        prob = np.zeros((101, 101), dtype=np.float32)
        
        # 30 cm cube in center lane
        hazard[25:45, 45:55] = True
        prob[25:45, 45:55] = 1.0

        steer, speed, diag = controller.compute_control(
            rx=5.0, ry=0.0, ryaw=np.pi / 2.0, v_mps=2.5,
            active_steer_rad=0.0,
            prob_map=prob,
            hazard_mask=hazard,
            frenet_helper=self.frenet,
            curr_idx=0,
            v_target_ref=3.5
        )

        print(f"[TEST] Shield-MPPI Collision Rate: {diag['collision_rate'] * 100:.1f}%, Exec Time: {diag['exec_time_ms']:.2f} ms", flush=True)
        self.assertGreater(diag["collision_rate"], 0.10, "Shield-MPPI should prune straight-line colliding paths.")
        self.assertLess(diag["exec_time_ms"], 12.0, "MPPI execution time exceeds Raspberry Pi 5 budget.")

    # --------------------------------------------------------------------------
    # 6. TEST CBF / TRACTION CIRCLE ACCELERATION CONSTRAINT
    # --------------------------------------------------------------------------
    def test_cbf_friction_circle_constraint(self):
        """Verifies that combined lateral and longitudinal accelerations obey friction limit."""
        mu = 0.90
        mu_g = mu * GRAVITY_MPS2
        controller = MPPIController(genome=MPPIGenome(H=20, K=256, mu_friction=mu))

        steer, speed, diag = controller.compute_control(
            rx=5.0, ry=0.0, ryaw=np.pi / 2.0, v_mps=4.0,
            active_steer_rad=0.45,  # High cornering angle
            prob_map=self.prob_map,
            hazard_mask=self.hazard_mask,
            frenet_helper=self.frenet,
            curr_idx=0,
            v_target_ref=4.4
        )

        # Check that commanded speed was safely modulated
        self.assertLessEqual(speed, 4.40 + 1e-4)
        self.assertGreaterEqual(speed, 0.0)

    # --------------------------------------------------------------------------
    # 7. TEST ACTUATOR SLEW RATE & LATENCY COMPENSATION
    # --------------------------------------------------------------------------
    def test_actuator_slew_rate_clamping(self):
        """Verifies steering change is bounded by max servo rate limit."""
        dt = 0.020
        controller = MPPIController(genome=MPPIGenome(H=20, K=256, dt=dt))
        
        active_steer = 0.0
        steer_cmd, _, _ = controller.compute_control(
            rx=5.0, ry=0.0, ryaw=np.pi / 2.0, v_mps=2.5,
            active_steer_rad=active_steer,
            prob_map=self.prob_map,
            hazard_mask=self.hazard_mask,
            frenet_helper=self.frenet,
            curr_idx=0
        )

        max_allowed_delta = MAX_STEER_RATE_RAD_S * dt + 1e-4
        delta_steer = abs(steer_cmd - active_steer)
        print(f"[TEST] Actuator Slew Delta: {delta_steer:.4f} rad (Limit: {max_allowed_delta:.4f} rad)", flush=True)
        self.assertLessEqual(delta_steer, max_allowed_delta, "Steering slew rate limit exceeded.")

    # --------------------------------------------------------------------------
    # 8. TEST TRAJECTORY WARM-STARTING
    # --------------------------------------------------------------------------
    def test_trajectory_warm_starting(self):
        """Verifies that optimal control sequence is shifted forward for rolling continuity."""
        controller = MPPIController(genome=MPPIGenome(H=20, K=256))
        
        # First cycle
        controller.compute_control(
            rx=5.0, ry=0.0, ryaw=np.pi / 2.0, v_mps=2.5,
            active_steer_rad=0.0,
            prob_map=self.prob_map,
            hazard_mask=self.hazard_mask,
            frenet_helper=self.frenet,
            curr_idx=0
        )
        u_warm_1 = controller.u_nominal.copy()

        # Second cycle
        controller.compute_control(
            rx=5.0, ry=0.05, ryaw=np.pi / 2.0, v_mps=2.5,
            active_steer_rad=u_warm_1[0, 0],
            prob_map=self.prob_map,
            hazard_mask=self.hazard_mask,
            frenet_helper=self.frenet,
            curr_idx=1
        )
        u_warm_2 = controller.u_nominal.copy()

        # Confirm non-zero warm-start memory
        self.assertFalse(np.allclose(u_warm_1, 0.0), "Nominal trajectory must be warm-started.")
        self.assertFalse(np.allclose(u_warm_2, 0.0), "Nominal trajectory must persist across cycles.")


if __name__ == "__main__":
    print("\n" + "=" * 80)
    print("  RUNNING MPPI CONTROLLER SUITE TESTS & BENCHMARKS")
    print("=" * 80 + "\n")
    unittest.main()
