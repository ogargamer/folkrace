import os
import sys
import time
import numpy as np

PLANNERS_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(PLANNERS_DIR)
PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(PROJECT_DIR)))
FRONT_COSTMAP_DIR = os.path.join(CONTROLLERS_DIR, "tof_costmap_front_only")
SCANNER_DIR = os.path.join(CONTROLLERS_DIR, "tof_16ch_grid_scanner")

for p in [SCANNER_DIR, FRONT_COSTMAP_DIR]:
    while p in sys.path:
        sys.path.remove(p)

for p in [WORKSPACE_ROOT, PROJECT_DIR, CONTROLLERS_DIR, PLANNERS_DIR]:
    if p not in sys.path:
        sys.path.append(p)

if os.path.exists(FRONT_COSTMAP_DIR):
    sys.path.insert(0, FRONT_COSTMAP_DIR)

from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from tof_gap_scout import ToFGapScout
from planner_option1_bvp import BVPDynamicStateLattice
from planner_option2_sl_mpc import CurvilinearSLMPC
from planner_option3_septic import SepticSplineTractionProfiler

cm = CppSmartHybridCostmap()
opt1 = BVPDynamicStateLattice(cm)
opt2 = CurvilinearSLMPC(cm)
opt3 = SepticSplineTractionProfiler(cm)

print("=" * 80)
print("TEST 1: TARGET ON LEFT (xg = -0.60m, yg = 1.20m), steer_current = 0.0")
print("=" * 80)
r1 = opt1.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
r2 = opt2.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
r3 = opt3.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)

s1, s2, s3 = np.degrees(r1['cmd_steer_rad']), np.degrees(r2['cmd_steer_rad']), np.degrees(r3['cmd_steer_rad'])
p1_1 = r1['best_line'][1]
p1_2 = r2['best_line'][1]
p1_3 = r3['best_line'][1]

print(f"Option 1 (BVP):    steer = {s1:+6.1f} deg | pt[1] = ({p1_1[0]:+.3f}, {p1_1[1]:.3f})")
print(f"Option 2 (SL-MPC): steer = {s2:+6.1f} deg | pt[1] = ({p1_2[0]:+.3f}, {p1_2[1]:.3f})")
print(f"Option 3 (Septic): steer = {s3:+6.1f} deg | pt[1] = ({p1_3[0]:+.3f}, {p1_3[1]:.3f})")

assert r1['cmd_steer_rad'] > 0, "Option 1 should turn LEFT (positive steer)"
assert r2['cmd_steer_rad'] > 0, "Option 2 should turn LEFT (positive steer)"
assert r3['cmd_steer_rad'] > 0, "Option 3 should turn LEFT (positive steer)"

print("\n" + "=" * 80)
print("TEST 2: TARGET ON RIGHT (xg = +0.60m, yg = 1.20m), steer_current = 0.0")
print("=" * 80)
r1 = opt1.plan(xg=0.6, yg=1.2, theta_g=0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
r2 = opt2.plan(xg=0.6, yg=1.2, theta_g=0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)
r3 = opt3.plan(xg=0.6, yg=1.2, theta_g=0.46, aperture_width=0.6, v_current=1.2, steer_current=0.0, yaw_rate_current=0.0)

s1, s2, s3 = np.degrees(r1['cmd_steer_rad']), np.degrees(r2['cmd_steer_rad']), np.degrees(r3['cmd_steer_rad'])
p1_1 = r1['best_line'][1]
p1_2 = r2['best_line'][1]
p1_3 = r3['best_line'][1]

print(f"Option 1 (BVP):    steer = {s1:+6.1f} deg | pt[1] = ({p1_1[0]:+.3f}, {p1_1[1]:.3f})")
print(f"Option 2 (SL-MPC): steer = {s2:+6.1f} deg | pt[1] = ({p1_2[0]:+.3f}, {p1_2[1]:.3f})")
print(f"Option 3 (Septic): steer = {s3:+6.1f} deg | pt[1] = ({p1_3[0]:+.3f}, {p1_3[1]:.3f})")

assert r1['cmd_steer_rad'] < 0, "Option 1 should turn RIGHT (negative steer)"
assert r2['cmd_steer_rad'] < 0, "Option 2 should turn RIGHT (negative steer)"
assert r3['cmd_steer_rad'] < 0, "Option 3 should turn RIGHT (negative steer)"

print("\n" + "=" * 80)
print("TEST 3: COUNTER-STEER RECOVERY (Target Left, steer_current = -0.25 rad / -14.3 deg right)")
print("=" * 80)
r1 = opt1.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=-0.25, yaw_rate_current=0.0)
r2 = opt2.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=-0.25, yaw_rate_current=0.0)
r3 = opt3.plan(xg=-0.6, yg=1.2, theta_g=-0.46, aperture_width=0.6, v_current=1.2, steer_current=-0.25, yaw_rate_current=0.0)

s1, s2, s3 = np.degrees(r1['cmd_steer_rad']), np.degrees(r2['cmd_steer_rad']), np.degrees(r3['cmd_steer_rad'])

print(f"Option 1 (BVP):    steer = {s1:+6.1f} deg (slew from -14.3 deg toward positive/left)")
print(f"Option 2 (SL-MPC): steer = {s2:+6.1f} deg (slew from -14.3 deg toward positive/left)")
print(f"Option 3 (Septic): steer = {s3:+6.1f} deg (slew from -14.3 deg toward positive/left)")

assert r1['cmd_steer_rad'] > -0.25, "Option 1 should steer towards left"
assert r2['cmd_steer_rad'] > -0.25, "Option 2 should steer towards left"
assert r3['cmd_steer_rad'] > -0.25, "Option 3 should steer towards left"

print("\nALL 3 PLANNERS FULLY VALIDATED AND ALL TESTS PASSED SUCCESSFULLY!")
