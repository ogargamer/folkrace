"""
================================================================================
  OFFLINE TEST SUITE: ANTI-WRONG-WAY & U-TURN DISCRIMINATION WATCHDOG
================================================================================
  Validates all 4 options and hybrid mode against:
    1. Legitimate Track Hairpins (U-turn into adjacent corridor) -> Must NOT trigger
    2. In-Corridor Wrong-Way Reversals (spinning 180 inside same lane) -> MUST trigger
    3. Performance & compute latency (< 0.25 ms per cycle)
================================================================================
"""

import math
import sys
import time
import os

# Ensure local planners directory is in python path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
if CURR_DIR not in sys.path:
    sys.path.insert(0, CURR_DIR)

import numpy as np

from anti_wrong_way_watchdog import (
    AntiWrongWayWatchdog,
    BreadcrumbWatchdog,
    KinematicArcFilter,
    DirectionalFlowCostmap,
    LateralSeparationWatchdog,
    WrongWayOptionMode,
    wrap_to_pi,
)
from stuck_recovery_manager import StuckRecoveryManager, StuckRecoveryState


def test_option1_breadcrumb():
    print("\n--- Testing Option 1: LOS Breadcrumb Raycasting & Anti-Contamination ---")
    dog = BreadcrumbWatchdog(lookback_m=8.0, spacing_m=0.05, corridor_half_width_m=0.25)

    # 1. Drive forward along +Y axis from (0, 0) to (0, 3.0) with yaw = 0.0
    t = 0.0
    for i in range(60):
        x = 0.0
        y = i * 0.05
        yaw = 0.0
        v = 1.5
        dog.add_point(x, y, yaw, v, t, state="DRIVE", yaw_rate=0.0)
        t += 0.033

    # Anti-Contamination Verification: Points during spin / reverse must be dropped
    n_before = len(dog.breadcrumbs)
    dog.add_point(0.0, 3.05, 0.0, -0.5, t, state="PIVOT_REVERSE", yaw_rate=0.0)
    dog.add_point(0.0, 3.05, 0.0, 1.5, t, state="CORRECTIVE_180", yaw_rate=4.0)
    assert len(dog.breadcrumbs) == n_before, "Anti-contamination failed: points added during reverse/spin!"
    print(f"  [Anti-Contamination] Passed: buffer clean ({len(dog.breadcrumbs)} valid forward points)")

    # Test 2: Unobstructed 180 reversal in open track -> Raycast clear -> Wrong-way confirmed
    # Vehicle spins 180 in place at (0.0, 3.0) facing yaw = pi (looking back down +Y towards y=0)
    hazard_mask_clear = np.zeros((190, 250), dtype=np.uint8)
    is_ww, conf, target_yaw, rays = dog.check_wrong_way(0.0, 3.0, math.pi, hazard_mask=hazard_mask_clear)
    print(f"  [Test 2 - Open Track Reversal] Wrong Way: {is_ww} (Confidence: {conf:.2f}, Target Yaw: {target_yaw:.2f} rad, Rays: {len(rays)})")
    assert is_ww is True, "Option 1 failed to catch unobstructed in-lane wrong-way reversal!"
    assert conf >= 0.70, f"Expected confidence >= 0.70, got {conf}"
    assert abs(wrap_to_pi(target_yaw - 0.0)) < 0.1, f"Expected recovery target yaw ~ 0.0, got {target_yaw}"

    # Test 1: Hairpin track with wall in between -> Raycast blocked -> Zero false positives
    # Vehicle traversed 180 hairpin into adjacent corridor at x = 0.60 m, y = 2.0 m, facing yaw = pi
    # Dividing wall is at x_world = 0.30 m. In vehicle frame (at x=0.60, yaw=pi):
    # xv_wall = 0.30 m -> col_w = int(round((0.30 - (-1.25)) / 0.010)) = 155
    hazard_mask_wall = np.zeros((190, 250), dtype=np.uint8)
    hazard_mask_wall[:, 150:160] = 1  # 10cm thick dividing wall between lanes

    is_ww_hairpin, conf_hp, _, rays_hp = dog.check_wrong_way(0.60, 2.0, math.pi, hazard_mask=hazard_mask_wall)
    print(f"  [Test 1 - Hairpin Dividing Wall] Wrong Way: {is_ww_hairpin} (Confidence: {conf_hp:.2f}, Blocked Rays: {sum(1 for r in rays_hp if not r['clear'])})")
    assert is_ww_hairpin is False, "Option 1 falsely triggered on hairpin with dividing wall!"
    assert conf_hp == 0.0, f"Expected confidence 0.0 on blocked sightlines, got {conf_hp}"

    print("  -> Option 1: PASSED (100% detection, 0% false positives)")


def test_smart_reverse_recovery():
    print("\n--- Testing Smart Reverse Recovery (Kinematics & Arbitration) ---")
    rec = StuckRecoveryManager(steer_angle_rad=0.60, reverse_speed_m_s=1.20)

    # Case 3A: Tightly Pinned Wall Escape (front ToF <= 0.03m / 3cm)
    rec.state = StuckRecoveryState.PIVOT_REVERSE
    rec.stroke_timer = 0.1

    # Left front is tightly pinned (2.5cm)
    tofs_left_pinned = {
        "tof_front_left_4.1deg": 0.025,
        "tof_front_right_4.1deg": 0.50,
    }
    is_rec, v_cmd, steer_cmd = rec.update(
        dt=0.016, current_v=-1.0, commanded_v=-1.0, cached_tofs=tofs_left_pinned, current_yaw=0.0
    )
    print(f"  [Case 3A - Left Pinned <= 3cm] Steer: {steer_cmd:+.3f} rad (Target: > 0 steer LEFT to swing nose RIGHT)")
    assert is_rec is True
    assert steer_cmd > 0.0, f"Expected positive steer (LEFT) when left-pinned, got {steer_cmd}"

    # Right front is tightly pinned (2.0cm)
    tofs_right_pinned = {
        "tof_front_left_4.1deg": 0.50,
        "tof_front_right_4.1deg": 0.020,
    }
    is_rec, v_cmd, steer_cmd = rec.update(
        dt=0.016, current_v=-1.0, commanded_v=-1.0, cached_tofs=tofs_right_pinned, current_yaw=0.0
    )
    print(f"  [Case 3A - Right Pinned <= 3cm] Steer: {steer_cmd:+.3f} rad (Target: < 0 steer RIGHT to swing nose LEFT)")
    assert is_rec is True
    assert steer_cmd < 0.0, f"Expected negative steer (RIGHT) when right-pinned, got {steer_cmd}"

    # Case 3B: Free Reverse Breadcrumb Tracking (> 3cm) with Inverted Steering Law
    tofs_open = {
        "tof_front_left_4.1deg": 0.20,
        "tof_front_right_4.1deg": 0.20,
    }
    # Corridor target is to the Left (target_breadcrumb_yaw = +0.5 rad, current_yaw = 0.0 -> e_psi > 0)
    # When v < 0: steer_sign = -sign(e_psi) = -1.0 (steer RIGHT) -> nose swings LEFT
    is_rec, v_cmd, steer_cmd = rec.update(
        dt=0.016,
        current_v=-1.0,
        commanded_v=-1.0,
        cached_tofs=tofs_open,
        current_yaw=0.0,
        target_breadcrumb_yaw=+0.5,
    )
    print(f"  [Case 3B - Free Reverse Left Target] Steer: {steer_cmd:+.3f} rad (Target: < 0 inverted sign)")
    assert is_rec is True
    assert steer_cmd < 0.0, f"Expected inverted negative steer for positive heading error in reverse, got {steer_cmd}"

    # Corridor target is to the Right (target_breadcrumb_yaw = -0.5 rad, current_yaw = 0.0 -> e_psi < 0)
    # When v < 0: steer_sign = -sign(e_psi) = +1.0 (steer LEFT) -> nose swings RIGHT
    is_rec, v_cmd, steer_cmd = rec.update(
        dt=0.016,
        current_v=-1.0,
        commanded_v=-1.0,
        cached_tofs=tofs_open,
        current_yaw=0.0,
        target_breadcrumb_yaw=-0.5,
    )
    print(f"  [Case 3B - Free Reverse Right Target] Steer: {steer_cmd:+.3f} rad (Target: > 0 inverted sign)")
    assert is_rec is True
    assert steer_cmd > 0.0, f"Expected inverted positive steer for negative heading error in reverse, got {steer_cmd}"

    # Case 3C: Reverse Priority Guard (Reverse ALWAYS takes priority over CORRECTIVE_180)
    rec.state = StuckRecoveryState.NORMAL_DRIVE
    tofs_wide_open = {
        "tof_front_left_4.1deg": 0.60,
        "tof_front_right_4.1deg": 0.60,
    }
    rec.trigger_emergency_recovery(
        heading_err=0.0,
        current_v=0.0,
        front_clearance_m=0.60,
        target_breadcrumb_yaw=0.0,
        current_yaw=math.pi,
    )
    print(f"  [Case 3C - Reverse Priority] Trigger State: {rec.state} (Expected: PIVOT_REVERSE, v_cmd < 0)")
    assert rec.state == StuckRecoveryState.PIVOT_REVERSE, f"Expected PIVOT_REVERSE, got {rec.state}"
    assert rec.current_cmd_v < 0.0, f"Expected negative reverse velocity, got {rec.current_cmd_v}"

    # Verify that while in PIVOT_REVERSE, calling trigger_corrective_180 does NOT override reverse
    rec.trigger_corrective_180(target_yaw=0.0, steer_sign=1.0, current_yaw=math.pi)
    assert rec.state == StuckRecoveryState.PIVOT_REVERSE, f"Reverse overridden! State: {rec.state}"
    print("  [Case 3C - Reverse Immunity] trigger_corrective_180 was safely blocked during PIVOT_REVERSE")

    # Verify that while in CORRECTIVE_180, hitting a wall engages reverse support phase
    rec.state = StuckRecoveryState.NORMAL_DRIVE
    rec.trigger_corrective_180(target_yaw=0.0, steer_sign=1.0, current_yaw=math.pi, front_clearance_m=0.50)
    tofs_wall_blocked = {
        "tof_front_left_4.1deg": 0.10,
        "tof_front_right_4.1deg": 0.10,
    }
    is_rec, v_cmd, steer_cmd = rec.update(
        dt=0.016,
        current_v=0.05,
        commanded_v=1.20,
        cached_tofs=tofs_wall_blocked,
        current_yaw=math.pi,
        target_breadcrumb_yaw=0.0,
    )
    print(f"  [Case 3C - Wall Collision in 180] State: {rec.state}, Substate: {rec.corrective_substate}, target_v: {v_cmd:.2f}")
    assert rec.state == StuckRecoveryState.CORRECTIVE_180, f"Expected state CORRECTIVE_180, got {rec.state}"
    assert rec.corrective_substate == "REVERSE", f"Expected substate REVERSE, got {rec.corrective_substate}"
    assert v_cmd < 0.0, f"Expected negative reverse velocity, got {v_cmd}"

    # Case 3D: Open Side Resolution for CORRECTIVE_180
    # Test 1: Left arc more open
    dir_l = StuckRecoveryManager.resolve_open_side(arc_clearance=(0.50, 0.15, 0.20))
    assert dir_l == 1.0, f"Expected +1.0 (Left), got {dir_l}"
    # Test 2: Right arc more open
    dir_r = StuckRecoveryManager.resolve_open_side(arc_clearance=(0.15, 0.50, 0.20))
    assert dir_r == -1.0, f"Expected -1.0 (Right), got {dir_r}"
    # Test 3: Arcs symmetric, left side pinned by wall (<= 10cm) -> choose Right
    tofs_pinned_l = {"tof_side_left_90deg": 0.08, "tof_side_right_90deg": 0.40}
    dir_tofs_r = StuckRecoveryManager.resolve_open_side(arc_clearance=(0.50, 0.50, 0.50), cached_tofs=tofs_pinned_l)
    assert dir_tofs_r == -1.0, f"Expected -1.0 (Right), got {dir_tofs_r}"
    # Test 4: Arcs symmetric, right side pinned by wall (<= 10cm) -> choose Left
    tofs_pinned_r = {"tof_side_left_90deg": 0.40, "tof_side_right_90deg": 0.08}
    dir_tofs_l = StuckRecoveryManager.resolve_open_side(arc_clearance=(0.50, 0.50, 0.50), cached_tofs=tofs_pinned_r)
    assert dir_tofs_l == 1.0, f"Expected +1.0 (Left), got {dir_tofs_l}"
    print("  [Case 3D - Open Side Selection] PASSED: Successfully resolves more open side via arcs and ToFs")

    # Case 3E: CORRECTIVE_180 Reverse-to-Forward Recovery Transition
    # Vehicle reverses for min reverse time, clearance opens up (>= 0.35m)
    tofs_open = {
        "tof_front_left_4.1deg": 0.45,
        "tof_front_right_4.1deg": 0.45,
    }
    rec.corrective_reverse_timer = 0.30  # simulate elapsed reverse time >= RECOVERY_EXIT_MIN_TIME_S
    is_rec, v_cmd_fwd, steer_cmd_fwd = rec.update(
        dt=0.016,
        current_v=-0.80,
        commanded_v=-1.20,
        cached_tofs=tofs_open,
        current_yaw=math.pi,
        target_breadcrumb_yaw=0.0,
    )
    assert rec.corrective_substate == "FORWARD", f"Expected return to FORWARD substate, got {rec.corrective_substate}"
    assert v_cmd_fwd > 0.0, f"Expected positive forward velocity after clearance opened, got {v_cmd_fwd}"
    print("  [Case 3E - Reverse Support Resumes Forward 180] PASSED: Resumes forward 180 after clearance achieved")

    print("  -> Smart Reverse Recovery: PASSED (Kinematics, Open Side & Reverse Support 100% verified)")


def test_option2_kinematic_arc():
    print("\n--- Testing Option 2: Kinematic Arc Radius Filter ---")
    filter_arc = KinematicArcFilter(min_hairpin_radius_m=0.32)

    # Case A: High-speed legitimate hairpin curve (v = 1.8 m/s, yaw_rate = 3.6 rad/s -> R = 0.50m)
    filter_arc.reset()
    yaw = 0.0
    is_pivot = False
    for step in range(30):
        dt = 0.03
        yaw += 3.6 * dt
        is_pivot, r_m = filter_arc.evaluate_motion(v_est=1.8, yaw_rate=3.6, current_yaw=yaw)

    print(f"  [Case A - Racing Hairpin] Radius: {r_m:.3f} m, Invalid Pivot Flag: {is_pivot}")
    assert is_pivot is False, "Option 2 falsely blocked racing hairpin turn!"
    assert r_m >= 0.32, f"Expected R >= 0.32m, got {r_m}"

    # Case B: In-lane spin / pivot stall (v = 0.15 m/s, yaw_rate = 4.0 rad/s -> R = 0.037m)
    filter_arc.reset()
    yaw = 0.0
    caught_pivot = False
    for step in range(30):
        dt = 0.03
        yaw += 4.0 * dt
        is_pivot, r_m = filter_arc.evaluate_motion(v_est=0.15, yaw_rate=4.0, current_yaw=yaw)
        if is_pivot:
            caught_pivot = True

    print(f"  [Case B - In-Lane Spin] Radius: {r_m:.3f} m, Invalid Pivot Caught: {caught_pivot}")
    assert caught_pivot is True, "Option 2 failed to catch in-lane spin pivot!"

    print("  -> Option 2: PASSED (100% detection, 0% false positives)")


def test_option3_flow_costmap():
    print("\n--- Testing Option 3: Directional Flow Costmap Stamping ---")
    flow_map = DirectionalFlowCostmap(grid_res_m=0.05, decay_s=12.0)

    # Vehicle drives along X from X=0 to X=2.0 (stamping forward flow [cos(0), sin(0)] = [1, 0])
    t = 1.0
    for i in range(40):
        x = i * 0.05
        y = 0.0
        flow_map.stamp_flow(x, y, yaw=0.0, now_s=t)
        t += 0.03

    # Case A: Looking backwards along X in the same lane (target at X=1.5, Y=0.0 with yaw = pi)
    is_opposing, dot = flow_map.check_flow_alignment(1.5, 0.0, target_yaw=math.pi, now_s=t)
    print(f"  [Case A - In-Lane Reverse] Opposing Flow: {is_opposing} (Dot Product: {dot:.2f})")
    assert is_opposing is True, "Option 3 failed to detect opposing corridor flow!"
    assert dot <= -0.65, f"Expected dot <= -0.65, got {dot}"

    # Case B: Looking backwards in adjacent lane (target at X=1.5, Y=0.80 with yaw = pi)
    is_opposing_hp, dot_hp = flow_map.check_flow_alignment(1.5, 0.80, target_yaw=math.pi, now_s=t)
    print(f"  [Case B - Adjacent Lane Hairpin] Opposing Flow: {is_opposing_hp} (Dot Product: {dot_hp:.2f})")
    assert is_opposing_hp is False, "Option 3 falsely blocked adjacent lane in hairpin!"

    print("  -> Option 3: PASSED (100% detection, 0% false positives)")


def test_option4_lateral_separation():
    print("\n--- Testing Option 4: Lateral Corridor Separation Watchdog ---")
    lat_dog = LateralSeparationWatchdog(min_separation_m=0.40)

    # Case A: In-lane 180 spin (robot enters at (0, 0) yaw=0, spins around inside same lane to yaw=pi, ending at (0.05, 0.05))
    lat_dog.reset()
    lat_dog.update(0.0, 0.0, 0.0, yaw_rate=0.0)
    # Start turning
    caught_in_lane = False
    for step in range(1, 31):
        yaw = step * (math.pi / 30.0)  # 0 to pi
        x = 0.05 * (step / 30.0)
        y = 0.05 * math.sin(yaw)       # stays near Y=0.05
        is_u, lat_m = lat_dog.update(x, y, yaw, yaw_rate=3.5)
        if is_u:
            caught_in_lane = True

    print(f"  [Case A - In-Lane Spin] In-Lane U-Turn Caught: {caught_in_lane} (Final Lateral Offset: {lat_m:.3f} m)")
    assert caught_in_lane is True, "Option 4 failed to detect in-lane U-turn!"

    # Case B: Legitimate track hairpin (robot curves around into adjacent corridor at Y=0.85 m)
    lat_dog.reset()
    lat_dog.update(0.0, 0.0, 0.0, yaw_rate=0.0)
    falsely_flagged = False
    max_lat_m = 0.0
    for step in range(1, 31):
        frac = step / 30.0
        yaw = frac * math.pi
        x = 0.40 * math.sin(yaw)
        y = 0.85 * frac  # shifts laterally by 0.85m
        is_u, lat_m = lat_dog.update(x, y, yaw, yaw_rate=3.0)
        if lat_m > max_lat_m:
            max_lat_m = lat_m
        if is_u:
            falsely_flagged = True

    print(f"  [Case B - Track Hairpin] False Alarm: {falsely_flagged} (Final Lateral Offset: {max_lat_m:.3f} m)")
    assert falsely_flagged is False, "Option 4 falsely triggered on legitimate hairpin!"
    assert max_lat_m >= 0.40, f"Expected lateral offset >= 0.40m, got {max_lat_m}"

    print("  -> Option 4: PASSED (100% detection, 0% false positives)")



def test_unified_coordinator_and_benchmark():
    print("\n--- Testing Unified Coordinator & Latency Benchmark ---")
    watchdog = AntiWrongWayWatchdog(mode=WrongWayOptionMode.OPTION_COMBINED_HYBRID)

    # 1. Simulate 500 steps of normal straight driving
    start_time = time.perf_counter()
    for i in range(500):
        t = i * 0.0083  # 120 Hz
        x = 0.0
        y = i * 0.02
        yaw = 0.0
        status = watchdog.update(x, y, yaw, v_est=2.4, yaw_rate=0.01, now_s=t)
        assert status.is_wrong_way is False, "Hybrid watchdog false positive during straight driving!"

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    avg_latency_us = (elapsed_ms / 500.0) * 1000.0
    print(f"  500 Cycles Execution Time: {elapsed_ms:.2f} ms ({avg_latency_us:.1f} microseconds per 120 Hz cycle)")
    assert avg_latency_us < 250.0, f"Latency {avg_latency_us:.1f} us exceeds 250 us budget!"

    # 2. Test in-lane reversal event (must satisfy confirmation threshold >= 5 consecutive frames)
    t_now = 500 * 0.0083
    # Vehicle spins to 180 degrees at current x=0.0, y=10.0
    for step in range(6):
        t_now += 0.0083
        rev_status = watchdog.update(0.0, 10.0, math.pi, v_est=0.1, yaw_rate=4.0, now_s=t_now)
    print(f"  Hybrid Response on Reversal: Detected = {rev_status.is_wrong_way}, By = {rev_status.detected_by_option}, Action = {rev_status.corrective_action}")
    assert rev_status.is_wrong_way is True, "Hybrid watchdog failed to trigger on wrong-way reversal!"
    assert rev_status.corrective_action in ("CORRECTIVE_180", "RESTRICT_STEERING"), "Expected corrective action!"

    print("  -> Unified Coordinator & Benchmark: PASSED (< 0.25 ms compute budget confirmed)")


if __name__ == "__main__":
    print("================================================================================")
    print("  RUNNING ANTI-WRONG-WAY & U-TURN DISCRIMINATION TEST SUITE")
    print("================================================================================")
    test_option1_breadcrumb()
    test_smart_reverse_recovery()
    test_option2_kinematic_arc()
    test_option3_flow_costmap()
    test_option4_lateral_separation()
    test_unified_coordinator_and_benchmark()
    print("\n================================================================================")
    print("  ALL TESTS PASSED SUCCESSFULLY! 100% PASS RATE ACROSS ALL MODES.")
    print("================================================================================")
