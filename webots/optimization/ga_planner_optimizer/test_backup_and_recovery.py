"""
Unit and regression tests for BackupObstacleController and MaplessRecoveryController.
Tests all 4 tiers of the backup steering hierarchy and blind reversing.
"""

import sys
import os
import math

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
PLANNERS_DIR = os.path.join(CURR_DIR, "planners")
for p in [CURR_DIR, PLANNERS_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

from backup_obstacle_controller import BackupObstacleController
from mapless_recovery import MaplessRecoveryController, RecoveryState, wrap_angle


def test_backup_controller():
    print("Testing BackupObstacleController...")
    controller = BackupObstacleController()

    # Case 1: Normal clear readings (> 0.25m)
    tofs_clear = {"tof_fl_3": 1.20, "tof_fr_3": 1.25}
    active, steer, speed = controller.update(tofs_clear, dt=0.02)
    assert not active, "Should not be active when sensors are clear"

    # Case 2: Tier 1 - Front Left blocked <= 0.09m, FR3 clear
    # Obstacle on Left -> Open on Right -> steer Left (+steer) so front swings Right
    controller.reset()
    tofs_tier1_left = {"tof_fl_3": 0.08, "tof_fr_3": 0.35}
    active, steer, speed = controller.update(tofs_tier1_left, dt=0.02)
    assert active
    assert speed < 0.0, "Must reverse"
    assert steer > 0.0, f"Steer must be positive to swing front right, got {steer}"
    assert "TIER1" in controller.decision_reason

    # Case 3: Tier 1 - Front Right blocked <= 0.09m, FL3 clear
    controller.reset()
    tofs_tier1_right = {"tof_fl_3": 0.35, "tof_fr_3": 0.08}
    active, steer, speed = controller.update(tofs_tier1_right, dt=0.02)
    assert active
    assert speed < 0.0
    assert steer < 0.0, f"Steer must be negative to swing front left, got {steer}"
    assert "TIER1" in controller.decision_reason

    # Case 4: Tier 2 - FL3 and FR3 are similar (both 0.08m) -> fallback to BL0 vs BR0
    controller.reset()
    tofs_tier2_right_open = {
        "tof_fl_3": 0.08, "tof_fr_3": 0.08,  # similar
        "tof_bl_0": 0.20, "tof_br_0": 0.60,  # obstacle closer on left BL0
    }
    active, steer, speed = controller.update(tofs_tier2_right_open, dt=0.02)
    assert active
    assert steer > 0.0, "Must steer Left so front swings Right into open BR0 side"
    assert "TIER2_BL0_BR0_RIGHT" in controller.decision_reason

    controller.reset()
    tofs_tier2_left_open = {
        "tof_fl_3": 0.08, "tof_fr_3": 0.08,  # similar
        "tof_bl_0": 0.60, "tof_br_0": 0.20,  # obstacle closer on right BR0
    }
    active, steer, speed = controller.update(tofs_tier2_left_open, dt=0.02)
    assert active
    assert steer < 0.0, "Must steer Right so front swings Left into open BL0 side"
    assert "TIER2_BL0_BR0_LEFT" in controller.decision_reason

    # Case 5: Tier 3 - FL3/FR3 similar AND BL0/BR0 similar -> fallback to BL1 vs BR1
    controller.reset()
    tofs_tier3_right_open = {
        "tof_fl_3": 0.08, "tof_fr_3": 0.08,  # similar
        "tof_bl_0": 0.40, "tof_br_0": 0.40,  # similar
        "tof_bl_1": 0.18, "tof_br_1": 0.65,  # BL1 tighter
    }
    active, steer, speed = controller.update(tofs_tier3_right_open, dt=0.02)
    assert active
    assert steer > 0.0, "Must steer Left so front swings Right into open BR1 side"
    assert "TIER3_BL1_BR1_RIGHT" in controller.decision_reason

    controller.reset()
    tofs_tier3_left_open = {
        "tof_fl_3": 0.08, "tof_fr_3": 0.08,  # similar
        "tof_bl_0": 0.40, "tof_br_0": 0.40,  # similar
        "tof_bl_1": 0.65, "tof_br_1": 0.18,  # BR1 tighter
    }
    active, steer, speed = controller.update(tofs_tier3_left_open, dt=0.02)
    assert active
    assert steer < 0.0, "Must steer Right so front swings Left into open BL1 side"
    assert "TIER3_BL1_BR1_LEFT" in controller.decision_reason

    # Case 6: Tier 4 - All ToF pairs similar -> use IMU history vector!
    controller.reset()
    tofs_all_similar = {
        "tof_fl_3": 0.08, "tof_fr_3": 0.08,
        "tof_bl_0": 0.40, "tof_br_0": 0.40,
        "tof_bl_1": 0.50, "tof_br_1": 0.50,
    }
    # Sub-case A: IMU history target is to the Left (heading_err > 0)
    # We want front to swing Left -> in reverse, steer Right (-steer)
    active, steer, speed = controller.update(
        tofs_all_similar,
        current_yaw=0.0,
        locked_target_yaw=0.60,
        dt=0.02,
    )
    assert active
    assert steer < 0.0, f"Must steer Right in reverse to swing front Left toward IMU target, got {steer}"
    assert "TIER4_IMU_TARGET_LEFT" in controller.decision_reason

    # Sub-case B: IMU history target is to the Right (heading_err < 0)
    # We want front to swing Right -> in reverse, steer Left (+steer)
    controller.reset()
    active, steer, speed = controller.update(
        tofs_all_similar,
        current_yaw=0.0,
        locked_target_yaw=-0.60,
        dt=0.02,
    )
    assert active
    assert steer > 0.0, f"Must steer Left in reverse to swing front Right toward IMU target, got {steer}"
    assert "TIER4_IMU_TARGET_RIGHT" in controller.decision_reason

    # Case 7: 4cm similarity threshold verification
    # FL3 = 0.08m, FR3 = 0.11m (difference is 3cm < 4cm -> must be treated as similar and fall to Tier 2)
    controller.reset()
    tofs_3cm_diff = {
        "tof_fl_3": 0.08, "tof_fr_3": 0.11,   # 3 cm diff < 4 cm -> similar!
        "tof_bl_0": 0.15, "tof_br_0": 0.55,   # Tier 2: BL0 is 40cm tighter -> steer Left (+steer)
    }
    active, steer, speed = controller.update(tofs_3cm_diff, dt=0.02)
    assert active
    assert "TIER2_BL0_BR0_RIGHT" in controller.decision_reason, f"Expected Tier 2 fallback for 3cm diff, got {controller.decision_reason}"

    # Case 8: Blind fast reverse test (speed is fast near traction limit: <= -1.00 m/s)
    assert speed <= -1.00, f"Expected fast reverse near traction limit (<= -1.00 m/s), got {speed}"

    # Case 9: Exit hysteresis -> stays active until BOTH >= 0.25m
    tofs_recovering = {"tof_fl_3": 0.18, "tof_fr_3": 0.28}
    active, steer, speed = controller.update(tofs_recovering, dt=0.02)
    assert active, "Must remain active while tof_fl_3 < 0.25m"

    tofs_cleared = {"tof_fl_3": 0.26, "tof_fr_3": 0.28}
    active, steer, speed = controller.update(tofs_cleared, dt=0.02)
    assert not active, "Must release once both front sensors >= 0.25m"

    print("BackupObstacleController tests PASSED!")


def test_mapless_recovery():
    print("Testing MaplessRecoveryController...")
    recovery = MaplessRecoveryController()

    init_yaw = -1.802
    recovery.reset(init_yaw)
    assert recovery.locked_target_yaw == init_yaw

    curr_yaw = init_yaw
    for step in range(50):
        curr_yaw += 0.02
        active, steer, speed = recovery.update(
            current_yaw=curr_yaw,
            current_yaw_rate=2.5,
            accel_x_g=0.6,
            accel_y_g=0.5,
            current_speed=1.2,
            dt=0.02,
        )
        assert not active, "Recovery must NOT trigger during smooth U-turn"
        assert abs(recovery.locked_target_yaw - curr_yaw) < 0.01

    anchor_before_crash = recovery.locked_target_yaw

    active, steer, speed = recovery.update(
        current_yaw=curr_yaw,
        current_yaw_rate=1.0,
        accel_x_g=3.8,  # Impact
        accel_y_g=1.2,
        current_speed=0.2,
        dt=0.02,
    )
    assert active, "Recovery must trigger on impact shock"
    assert recovery.state == RecoveryState.SPIN_OR_IMPACT_BRAKE
    assert recovery.locked_target_yaw == anchor_before_crash

    # Inverted 180 degrees
    inverted_yaw = wrap_angle(anchor_before_crash + math.pi)
    tofs_close = {"tof_fl_3": 0.12, "tof_fr_3": 0.14}
    active, steer, speed = recovery.update(
        current_yaw=inverted_yaw,
        current_yaw_rate=0.5,
        accel_x_g=0.1,
        accel_y_g=0.1,
        current_speed=0.0,
        tof_readings=tofs_close,
        dt=0.02,
    )
    assert active
    assert recovery.state == RecoveryState.REALIGN_REVERSE_ARC
    assert speed < 0.0

    print("MaplessRecoveryController tests PASSED!")


if __name__ == "__main__":
    test_backup_controller()
    test_mapless_recovery()
    print("ALL TESTS PASSED SUCCESSFULLY!")
