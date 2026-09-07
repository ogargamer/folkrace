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

from anti_wrong_way_watchdog import (
    AntiWrongWayWatchdog,
    BreadcrumbWatchdog,
    KinematicArcFilter,
    DirectionalFlowCostmap,
    LateralSeparationWatchdog,
    WrongWayOptionMode,
    wrap_to_pi,
)


def test_option1_breadcrumb():
    print("\n--- Testing Option 1: Breadcrumb Reverse-Trail Overlap ---")
    dog = BreadcrumbWatchdog(lookback_m=4.0, spacing_m=0.05, corridor_half_width_m=0.25)

    # 1. Drive straight along X axis from X=0 to X=3.0 (corridor centered at Y=0)
    t = 0.0
    for i in range(60):
        x = i * 0.05
        y = 0.0
        yaw = 0.0
        v = 1.5
        dog.add_point(x, y, yaw, v, t)
        t += 0.033

    # Case A: Vehicle spins 180 in place at (3.0, 0.0) facing yaw = pi (looking down same lane)
    is_ww, score, target_yaw = dog.check_wrong_way(3.0, 0.0, math.pi, lookahead_m=1.5)
    print(f"  [Case A - In-Lane Spin] Wrong Way: {is_ww} (Overlap Score: {score:.2f}, Target Yaw: {target_yaw:.2f} rad)")
    assert is_ww is True, "Option 1 failed to catch in-lane wrong-way reversal!"
    assert score >= 0.55, f"Expected overlap score >= 0.55, got {score}"

    # Case B: Legitimate track hairpin: vehicle traversed arc into adjacent corridor at Y = 0.90 m
    # Now at (3.0, 0.90) facing yaw = pi
    is_ww_hairpin, score_hp, _ = dog.check_wrong_way(3.0, 0.90, math.pi, lookahead_m=1.5)
    print(f"  [Case B - Hairpin Turn] Wrong Way: {is_ww_hairpin} (Overlap Score: {score_hp:.2f})")
    assert is_ww_hairpin is False, "Option 1 falsely triggered on legitimate track hairpin!"

    print("  -> Option 1: PASSED (100% detection, 0% false positives)")


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
        x = i * 0.02
        y = 0.0
        yaw = 0.0
        status = watchdog.update(x, y, yaw, v_est=2.4, yaw_rate=0.01, now_s=t)
        assert status.is_wrong_way is False, "Hybrid watchdog false positive during straight driving!"

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    avg_latency_us = (elapsed_ms / 500.0) * 1000.0
    print(f"  500 Cycles Execution Time: {elapsed_ms:.2f} ms ({avg_latency_us:.1f} microseconds per 120 Hz cycle)")
    assert avg_latency_us < 250.0, f"Latency {avg_latency_us:.1f} us exceeds 250 us budget!"

    # 2. Test in-lane reversal event
    t_now = 500 * 0.0083
    # Vehicle spins to 180 degrees at current x=10.0, y=0.0
    rev_status = watchdog.update(10.0, 0.0, math.pi, v_est=0.1, yaw_rate=4.0, now_s=t_now)
    print(f"  Hybrid Response on Reversal: Detected = {rev_status.is_wrong_way}, By = {rev_status.detected_by_option}, Action = {rev_status.corrective_action}")
    assert rev_status.is_wrong_way is True, "Hybrid watchdog failed to trigger on wrong-way reversal!"
    assert rev_status.corrective_action in ("CORRECTIVE_180", "RESTRICT_STEERING"), "Expected corrective action!"

    print("  -> Unified Coordinator & Benchmark: PASSED (< 0.25 ms compute budget confirmed)")


if __name__ == "__main__":
    print("================================================================================")
    print("  RUNNING ANTI-WRONG-WAY & U-TURN DISCRIMINATION TEST SUITE")
    print("================================================================================")
    test_option1_breadcrumb()
    test_option2_kinematic_arc()
    test_option3_flow_costmap()
    test_option4_lateral_separation()
    test_unified_coordinator_and_benchmark()
    print("\n================================================================================")
    print("  ALL TESTS PASSED SUCCESSFULLY! 100% PASS RATE ACROSS ALL MODES.")
    print("================================================================================")
