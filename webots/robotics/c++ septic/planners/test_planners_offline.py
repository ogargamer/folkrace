"""
================================================================================
  OFFLINE VALIDATION & BENCHMARK: C++ SEPTIC SPLINE TRACTION PROFILER
================================================================================
  - Self-contained test suite for C++ Septic Spline Planner in webots/robotics/c++ septic
  - Validates turn responses, counter-steer recovery, and latency against CppSmartHybridCostmap
================================================================================
"""

import os
import sys
import time
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from cpp_planner_option3_septic import CppSepticSplineTractionProfiler

cm = CppSmartHybridCostmap()
opt3 = CppSepticSplineTractionProfiler(costmap_engine=cm)

print("=" * 80)
print("TEST 1: TARGET ON LEFT (xg = -0.60m, yg = 1.20m), steer_current = 0.0")
print("=" * 80)
r3 = opt3.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
s3 = np.degrees(r3['cmd_steer_rad'])
p1_3 = r3['best_line'][1]
print(f"C++ Septic: steer = {s3:+6.1f} deg | pt[1] = ({p1_3[0]:+.3f}, {p1_3[1]:.3f})")
assert r3['cmd_steer_rad'] > 0, "Option 3 should turn LEFT (positive steer)"

print("\n" + "=" * 80)
print("TEST 2: TARGET ON RIGHT (xg = +0.60m, yg = 1.20m), steer_current = 0.0")
print("=" * 80)
r3 = opt3.plan(xg=0.6, yg=1.2, theta_g=0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
s3 = np.degrees(r3['cmd_steer_rad'])
p1_3 = r3['best_line'][1]
print(f"C++ Septic: steer = {s3:+6.1f} deg | pt[1] = ({p1_3[0]:+.3f}, {p1_3[1]:.3f})")
assert r3['cmd_steer_rad'] < 0, "Option 3 should turn RIGHT (negative steer)"

print("\n" + "=" * 80)
print("TEST 3: COUNTER-STEER RECOVERY (Target Left, steer_current = -0.25 rad / -14.3 deg right)")
print("=" * 80)
r3 = opt3.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=-0.25, yaw_rate_current=0.0)
s3 = np.degrees(r3['cmd_steer_rad'])
print(f"C++ Septic: steer = {s3:+6.1f} deg (slew from -14.3 deg toward positive/left)")
assert r3['cmd_steer_rad'] > -0.25, "Option 3 should steer towards left"

# Latency Benchmark
latencies = []
for _ in range(100):
    t0 = time.perf_counter()
    opt3.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
    latencies.append((time.perf_counter() - t0) * 1000.0)

print("\n" + "=" * 80)
print(f"C++ SEPTIC BENCHMARK (100 runs): Mean={np.mean(latencies):.3f} ms, Min={np.min(latencies):.3f} ms, Max={np.max(latencies):.3f} ms")
print("=" * 80)
print("ALL C++ SEPTIC PLANNER TESTS PASSED SUCCESSFULLY!")
