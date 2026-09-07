"""
================================================================================
  UNIT TESTS & ALGORITHM COMPARISON FOR 12-CHANNEL ToF PATHFINDER SUITE
  Tests:
    1. ToFFollowTheGap (FGM) vs ToFVectorFieldHistogram (VFH+) vs ToFPotentialField (APF)
    2. Obstacle evasion in middle of corridor
    3. Racing line suggestion bias blending
    4. Wrong-Way Lap Prevention logic
    5. Non-Blocking Reverse Recovery State Machine
================================================================================
"""

import math
import numpy as np
import sys
import os

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from tof_pathfinders import (
    NUM_TOF_ZONES,
    TOF_ZONE_ANGLES,
    TOF_MAX_RANGE_M,
    ToFFollowTheGap,
    ToFVectorFieldHistogram,
    ToFPotentialField,
    WrongWayDetector,
    StuckRecoveryManager,
    StuckRecoveryState
)

def run_tests():
    print("==============================================================================")
    print("  RUNNING 12-CHANNEL ToF PATHFINDER & RECOVERY UNIT TESTS")
    print("==============================================================================\n")

    fgm = ToFFollowTheGap()
    vfh = ToFVectorFieldHistogram()
    apf = ToFPotentialField()
    wrong_way = WrongWayDetector()
    recovery = StuckRecoveryManager()

    # --------------------------------------------------------------------------
    # TEST 1: Open Track (All ToF readings at max range 1.50m)
    # --------------------------------------------------------------------------
    print("--- TEST 1: Open Clear Track (All ToFs = 1.50m) ---")
    open_tofs = np.full(NUM_TOF_ZONES, 1.50)
    suggested_steer = math.radians(10.0) # +10 deg line suggestion

    steer_fgm, spd_fgm = fgm.compute(open_tofs, suggested_steer, line_weight=0.5)
    steer_vfh, spd_vfh = vfh.compute(open_tofs, suggested_steer, line_weight=0.5)
    steer_apf, spd_apf = apf.compute(open_tofs, suggested_steer, line_weight=0.5)

    print(f"FGM:  Steer={math.degrees(steer_fgm):+5.1f}°, SpeedFactor={spd_fgm:.2f}")
    print(f"VFH+: Steer={math.degrees(steer_vfh):+5.1f}°, SpeedFactor={spd_vfh:.2f}")
    print(f"APF:  Steer={math.degrees(steer_apf):+5.1f}°, SpeedFactor={spd_apf:.2f}")
    assert spd_fgm >= 0.90 and spd_vfh >= 0.90 and spd_apf >= 0.90, "Speed factors must be high in open track"
    print(">> TEST 1 PASSED!\n")

    # --------------------------------------------------------------------------
    # TEST 2: Center Obstacle (Obstacle at 0.35m directly ahead, Open left & right)
    # --------------------------------------------------------------------------
    print("--- TEST 2: Center Obstacle (0.35m directly ahead, Open on sides) ---")
    # Zones 5 and 6 are center
    center_obs_tofs = np.array([1.50, 1.40, 1.20, 0.90, 0.50, 0.35, 0.35, 0.50, 0.90, 1.20, 1.40, 1.50])
    suggested_steer_center = 0.0 # line wants to go straight through obstacle

    steer_fgm, spd_fgm = fgm.compute(center_obs_tofs, suggested_steer_center, line_weight=0.3)
    steer_vfh, spd_vfh = vfh.compute(center_obs_tofs, suggested_steer_center, line_weight=0.3)
    steer_apf, spd_apf = apf.compute(center_obs_tofs, suggested_steer_center, line_weight=0.3)

    print(f"FGM:  Steer={math.degrees(steer_fgm):+5.1f}°, SpeedFactor={spd_fgm:.2f}")
    print(f"VFH+: Steer={math.degrees(steer_vfh):+5.1f}°, SpeedFactor={spd_vfh:.2f}")
    print(f"APF:  Steer={math.degrees(steer_apf):+5.1f}°, SpeedFactor={spd_apf:.2f}")
    assert abs(steer_fgm) > math.radians(10.0), "FGM must steer away from center obstacle"
    assert abs(steer_vfh) > math.radians(10.0), "VFH+ must steer away from center obstacle"
    print(">> TEST 2 PASSED!\n")

    # --------------------------------------------------------------------------
    # TEST 3: Right Wall Approaching (Wall on Right at 0.15m, Open Corridor on Left)
    # --------------------------------------------------------------------------
    print("--- TEST 3: Right Wall Approaching (Right=0.15m, Left=1.20m) ---")
    # Right zones 8 to 11 are close
    right_wall_tofs = np.array([1.30, 1.25, 1.20, 1.10, 0.85, 0.60, 0.35, 0.20, 0.15, 0.15, 0.15, 0.15])
    suggested_steer_right = math.radians(-15.0) # line wants to cut right into wall

    steer_fgm, spd_fgm = fgm.compute(right_wall_tofs, suggested_steer_right, line_weight=0.5)
    steer_vfh, spd_vfh = vfh.compute(right_wall_tofs, suggested_steer_right, line_weight=0.5)
    steer_apf, spd_apf = apf.compute(right_wall_tofs, suggested_steer_right, line_weight=0.5)

    print(f"FGM:  Steer={math.degrees(steer_fgm):+5.1f}° (Pushed Left into open gap)")
    print(f"VFH+: Steer={math.degrees(steer_vfh):+5.1f}° (Pushed Left into open gap)")
    print(f"APF:  Steer={math.degrees(steer_apf):+5.1f}° (Pushed Left into open gap)")
    assert steer_fgm > 0 and steer_vfh > 0 and steer_apf > 0, "All algorithms must steer LEFT away from right wall"
    print(">> TEST 3 PASSED!\n")

    # --------------------------------------------------------------------------
    # TEST 4: Wrong-Way Lap Prevention
    # --------------------------------------------------------------------------
    print("--- TEST 4: Wrong-Way Lap Prevention ---")
    track_flow_yaw = 0.0 # Valid race flow is along +X (yaw = 0)
    
    # Facing forward (+10 deg)
    is_ww, diff = wrong_way.evaluate(current_yaw=math.radians(10.0), track_flow_yaw=track_flow_yaw)
    assert not is_ww, "Must not flag wrong-way when facing forward"

    # Spun around facing backward (180 deg)
    for _ in range(5):
        is_ww, diff = wrong_way.evaluate(current_yaw=math.radians(175.0), track_flow_yaw=track_flow_yaw)
    assert is_ww, "Must flag wrong-way when facing 175 deg opposite to track flow"
    print(f"Wrong-way triggered correctly! Heading error: {math.degrees(diff):.1f}°")
    print(">> TEST 4 PASSED!\n")

    # --------------------------------------------------------------------------
    # TEST 5: Non-Blocking Stuck & Reverse Recovery State Machine
    # --------------------------------------------------------------------------
    print("--- TEST 5: Stuck Recovery State Machine ---")
    stuck_tofs = np.array([0.20, 0.15, 0.10, 0.08, 0.05, 0.04, 0.04, 0.05, 0.12, 0.18, 0.25, 0.35])
    
    # Step 1: Normal drive into barrier
    is_rec, rec_v, rec_steer = recovery.update(current_v=0.02, commanded_v=2.0, tof_distances=stuck_tofs, dt=0.05)
    print(f"Step 1 (Detecting stall): State={recovery.state}, Recovering={is_rec}")
    
    # Step 2: Confirmation timer triggers reverse
    for _ in range(7):
        is_rec, rec_v, rec_steer = recovery.update(current_v=0.01, commanded_v=2.0, tof_distances=stuck_tofs, dt=0.05)
    print(f"Step 2 (Active Reverse): State={recovery.state}, TargetSpeed={rec_v:.2f} m/s, TargetSteer={math.degrees(rec_steer):+.1f}°")
    assert is_rec and rec_v < 0, "Must actively reverse when stuck"

    # Step 3: Clearance restored while backing up
    cleared_tofs = np.full(NUM_TOF_ZONES, 0.65)
    for _ in range(12):
        is_rec, rec_v, rec_steer = recovery.update(current_v=-1.0, commanded_v=2.0, tof_distances=cleared_tofs, dt=0.05)
    print(f"Step 3 (Realign & Resume): State={recovery.state}")
    
    # Finish realign
    for _ in range(10):
        is_rec, rec_v, rec_steer = recovery.update(current_v=0.8, commanded_v=2.0, tof_distances=cleared_tofs, dt=0.05)
    print(f"Step 4 (Back to Normal): State={recovery.state}, Recovering={is_rec}")
    assert recovery.state == StuckRecoveryState.NORMAL_DRIVE and not is_rec
    print(">> TEST 5 PASSED!\n")

    print("==============================================================================")
    print("  ALL 5 UNIT & COMPARISON TESTS PASSED WITH 100% SUCCESS!")
    print("==============================================================================")

if __name__ == "__main__":
    run_tests()
