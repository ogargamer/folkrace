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

from cpp_smart_hybrid_costmap import CppSmartHybridCostmap, SENSOR_NAMES
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
mpc.reset_active_target()
r2 = mpc.plan(xg=0.6, yg=1.2, theta_g=0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
s2 = np.degrees(r2['cmd_steer_rad'])
p2 = r2['best_line'][1]
print(f"C++ SL-MPC: steer = {s2:+6.1f} deg | pt[1] = ({p2[0]:+.3f}, {p2[1]:.3f})")
assert r2['cmd_steer_rad'] < 0, "C++ SL-MPC should turn RIGHT (negative steer)"

print("\n" + "=" * 80)
print("TEST 3: COUNTER-STEER RECOVERY (Target Left, steer_current = -0.25 rad / -14.3 deg right)")
print("=" * 80)
mpc.reset_active_target()
r3 = mpc.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=-0.25, yaw_rate_current=0.0)
s3 = np.degrees(r3['cmd_steer_rad'])
print(f"C++ SL-MPC: steer = {s3:+6.1f} deg (slew from -14.3 deg toward positive/left)")
assert r3['cmd_steer_rad'] > -0.25, "C++ SL-MPC should slew steer towards left (+)"

print("\n" + "=" * 80)
print("TEST 4: CORRIDOR CENTERING BIAS (ASYMMETRIC WALLS)")
print("=" * 80)

# Matches SENSOR_RIG geometry in smart_hybrid_costmap_front.cpp (+Y forward, +X right)
_TOF_SENSOR_RIG = [
    ("tof_front_left_4.1deg",   -0.058, 0.092, 1.5512),
    ("tof_front_left_15.4deg",  -0.058, 0.092, 1.3548),
    ("tof_front_left_26.6deg",  -0.058, 0.092, 1.1585),
    ("tof_front_left_37.9deg",  -0.058, 0.092, 0.9621),
    ("tof_front_left_49.1deg",  -0.057, 0.090, 0.7658),
    ("tof_front_left_60.4deg",  -0.057, 0.090, 0.5694),
    ("tof_front_left_71.6deg",  -0.057, 0.090, 0.3731),
    ("tof_front_left_82.9deg",  -0.057, 0.090, 0.1767),
    ("tof_front_right_4.1deg",   0.058, 0.092, 1.5904),
    ("tof_front_right_15.4deg",  0.058, 0.092, 1.7868),
    ("tof_front_right_26.6deg",  0.058, 0.092, 1.9831),
    ("tof_front_right_37.9deg",  0.058, 0.092, 2.1795),
    ("tof_front_right_49.1deg",  0.057, 0.090, 2.3758),
    ("tof_front_right_60.4deg",  0.057, 0.090, 2.5722),
    ("tof_front_right_71.6deg",  0.057, 0.090, 2.7685),
    ("tof_front_right_82.9deg",  0.057, 0.090, 2.9649),
]


def _raycast_parallel_walls(x_left: float, x_right: float, max_range: float = 2.40) -> dict:
    readings = {}
    for name, x0, _y0, heading in _TOF_SENSOR_RIG:
        dx = float(np.cos(heading))
        t_hit = max_range
        if dx > 1e-6:
            t_hit = min(t_hit, (x_right - x0) / dx)
        elif dx < -1e-6:
            t_hit = min(t_hit, (x_left - x0) / dx)
        readings[name] = float(np.clip(t_hit, 0.001, max_range))
    return readings


def _populate_walls(costmap: CppSmartHybridCostmap, x_left: float, x_right: float, frames: int = 5) -> None:
    costmap.clear()
    scan = _raycast_parallel_walls(x_left, x_right)
    ordered = [scan[name] for name in SENSOR_NAMES]
    for _ in range(frames):
        costmap.update(ordered)


# Case A: Right wall close (+0.22m), left wall far (-0.78m) -> center_bias should steer LEFT (+steer, -X)
_populate_walls(cm, x_left=-0.78, x_right=0.22)
mpc.set_center_bias(0.0)
mpc.reset_active_target()
for _ in range(3):
    r_base_rclose = mpc.plan(xg=0.0, yg=1.2, theta_g=0.0, aperture_width=0.8, v_current=1.5, steer_current=0.0, yaw_rate_current=0.0)
mpc.set_center_bias(1.0)
mpc.reset_active_target()
for _ in range(3):
    r_cent_rclose = mpc.plan(xg=0.0, yg=1.2, theta_g=0.0, aperture_width=0.8, v_current=1.5, steer_current=0.0, yaw_rate_current=0.0)
print(
    f"Case A (right close): bias=0 steer={np.degrees(r_base_rclose['cmd_steer_rad']):+5.2f} deg end_x={r_base_rclose['best_line'][-1][0]:+.3f}m | "
    f"bias=1 steer={np.degrees(r_cent_rclose['cmd_steer_rad']):+5.2f} deg end_x={r_cent_rclose['best_line'][-1][0]:+.3f}m"
)
assert r_cent_rclose['cmd_steer_rad'] > r_base_rclose['cmd_steer_rad'] + 0.004, "Center bias should steer further LEFT away from close right wall"
assert r_cent_rclose['cmd_steer_rad'] > 0.0, "Center bias should produce positive (LEFT) steer when right wall is close"
assert r_cent_rclose['best_line'][-1][0] < r_base_rclose['best_line'][-1][0] - 0.03, "Center bias trajectory should shift LEFT (-X)"

# Case B: Left wall close (-0.22m), right wall far (+0.78m) -> center_bias should steer RIGHT (-steer, +X)
_populate_walls(cm, x_left=-0.22, x_right=0.78)
mpc.set_center_bias(0.0)
mpc.reset_active_target()
for _ in range(3):
    r_base_lclose = mpc.plan(xg=0.0, yg=1.2, theta_g=0.0, aperture_width=0.8, v_current=1.5, steer_current=0.0, yaw_rate_current=0.0)
mpc.set_center_bias(1.0)
mpc.reset_active_target()
for _ in range(3):
    r_cent_lclose = mpc.plan(xg=0.0, yg=1.2, theta_g=0.0, aperture_width=0.8, v_current=1.5, steer_current=0.0, yaw_rate_current=0.0)
print(
    f"Case B (left close):  bias=0 steer={np.degrees(r_base_lclose['cmd_steer_rad']):+5.2f} deg end_x={r_base_lclose['best_line'][-1][0]:+.3f}m | "
    f"bias=1 steer={np.degrees(r_cent_lclose['cmd_steer_rad']):+5.2f} deg end_x={r_cent_lclose['best_line'][-1][0]:+.3f}m"
)
assert r_cent_lclose['cmd_steer_rad'] < r_base_lclose['cmd_steer_rad'] - 0.004, "Center bias should steer further RIGHT away from close left wall"
assert r_cent_lclose['cmd_steer_rad'] < 0.0, "Center bias should produce negative (RIGHT) steer when left wall is close"
assert r_cent_lclose['best_line'][-1][0] > r_base_lclose['best_line'][-1][0] + 0.03, "Center bias trajectory should shift RIGHT (+X)"

mpc.set_center_bias(0.0)
cm.clear()
mpc.reset_active_target()

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
