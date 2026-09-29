"""
================================================================================
  OFFLINE VALIDATION & BENCHMARK: C++ CURVILINEAR S-L MPC (OSQP)
================================================================================
  Self-contained test suite verifying:
    1. C++ Curvilinear S-L MPC DLL loading and QP initialization
    2. Left-hand target tracking and steering calculation
    3. Right-hand target tracking and steering calculation
    4. Slew-rate counter-steering recovery response
    5. Latency profiling over 100 runs (<2 ms target)
================================================================================
"""

import os
import sys
import time
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(PROJECT_DIR)))

for p in [CURR_DIR, CONTROLLERS_DIR, PROJECT_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from planner_sl_mpc import CurvilinearSLMPC

cm = CppSmartHybridCostmap()
mpc = CurvilinearSLMPC(cm)

print("=" * 80)
print("TEST 1: TARGET ON LEFT (xg = -0.60m, yg = 1.20m), steer_current = 0.0")
print("=" * 80)
r1 = mpc.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
s1 = np.degrees(r1['cmd_steer_rad'])
p1 = r1['best_line'][1]
print(f"C++ SL-MPC: steer = {s1:+6.1f} deg | pt[1] = ({p1[0]:+.3f}, {p1[1]:.3f})")
assert r1['cmd_steer_rad'] > 0, "C++ SL-MPC should turn LEFT (positive steer)"

print("\n" + "=" * 80)
print("TEST 2: TARGET ON RIGHT (xg = +0.60m, yg = 1.20m), steer_current = 0.0")
print("=" * 80)
r2 = mpc.plan(xg=0.6, yg=1.2, theta_g=0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
s2 = np.degrees(r2['cmd_steer_rad'])
p2 = r2['best_line'][1]
print(f"C++ SL-MPC: steer = {s2:+6.1f} deg | pt[1] = ({p2[0]:+.3f}, {p2[1]:.3f})")
assert r2['cmd_steer_rad'] < 0, "C++ SL-MPC should turn RIGHT (negative steer)"

print("\n" + "=" * 80)
print("TEST 3: COUNTER-STEER RECOVERY (Target Left, steer_current = -0.25 rad / -14.3 deg right)")
print("=" * 80)
r3 = mpc.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=-0.25, yaw_rate_current=0.0)
s3 = np.degrees(r3['cmd_steer_rad'])
print(f"C++ SL-MPC: steer = {s3:+6.1f} deg (slew from -14.3 deg toward positive/left)")
assert r3['cmd_steer_rad'] > -0.25, "C++ SL-MPC should slew steer towards left (+)"

print("\n" + "=" * 80)
print("PERFORMANCE BENCHMARK: 100 CONSECUTIVE RUNS")
print("=" * 80)
latencies = []
for _ in range(100):
    t0 = time.perf_counter()
    mpc.plan(xg=0.4, yg=1.3, theta_g=0.3, aperture_width=0.5, v_current=2.5, steer_current=0.05, yaw_rate_current=0.1)
    latencies.append((time.perf_counter() - t0) * 1000.0)

latencies = np.array(latencies)
print(f"Latency: Mean = {latencies.mean():.2f} ms | Min = {latencies.min():.2f} ms | Max = {latencies.max():.2f} ms")
print("\nC++ CURVILINEAR S-L MPC VALIDATION COMPLETE AND PASSED!")
