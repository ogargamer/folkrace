#!/usr/bin/env python3
"""
================================================================================
  FAST CLOSED-LOOP 2D WRONG-WAY & RECOVERY SIMULATION BENCH
================================================================================
  Exercises the real AutonomousBrain (C++ Costmap + ToF Gap Scout + SL-MPC +
  AntiWrongWayWatchdog + StuckRecoveryManager) against 2D wall segments and
  Ackermann bicycle physics with friction-circle saturation at 60 Hz.
================================================================================
"""

import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLANNER_DIR = os.path.join(REPO_ROOT, "webots", "simulation", "my_project", "controllers", "tof_path_planner")
for p in (REPO_ROOT, PLANNER_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

from brain.autonomous_brain import AutonomousBrain
from brain.pose_tracker import wrap_to_pi
from brain.stuck_recovery_manager import StuckRecoveryState
from brain.tof_gap_scout import FRONT_TOF_SENSOR_RIG
from brain.planner_config import (
    GRAVITY_G,
    MAX_STEERING_RAD,
    SERVO_MAX_SPEED_RAD_S,
    TIRE_MU,
    TOF_MAX_RANGE_M,
    WHEELBASE,
)
from progress_tracker import load_track_walls


def ray_segment_intersect(
    ox: float, oy: float, dx: float, dy: float,
    ax: float, ay: float, bx: float, by: float,
) -> Optional[float]:
    """2D ray (o + t*d, t>=0) vs segment (a, b) intersection distance t."""
    sx = bx - ax
    sy = by - ay
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None
    qpx = ax - ox
    qpy = ay - oy
    t = (qpx * sy - qpy * sx) / denom
    u = (qpx * dy - qpy * dx) / denom
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return None


def segments_from_polyline(pts: np.ndarray, closed: bool = True) -> List[Tuple[float, float, float, float]]:
    segs: List[Tuple[float, float, float, float]] = []
    n = len(pts)
    if n < 2:
        return segs
    for i in range(n - 1):
        segs.append((float(pts[i, 0]), float(pts[i, 1]), float(pts[i + 1, 0]), float(pts[i + 1, 1])))
    if closed:
        segs.append((float(pts[-1, 0]), float(pts[-1, 1]), float(pts[0, 0]), float(pts[0, 1])))
    return segs


def box_segments(cx: float, cy: float, half_w: float, half_l: float) -> List[Tuple[float, float, float, float]]:
    pts = np.array([
        [cx - half_w, cy - half_l],
        [cx + half_w, cy - half_l],
        [cx + half_w, cy + half_l],
        [cx - half_w, cy + half_l],
    ])
    return segments_from_polyline(pts, closed=True)


def point_to_segments_dist(px: float, py: float, segs: List[Tuple[float, float, float, float]]) -> float:
    best_sq = 1e9
    for ax, ay, bx, by in segs:
        vx, vy = bx - ax, by - ay
        l2 = vx * vx + vy * vy
        if l2 < 1e-9:
            d2 = (px - ax) ** 2 + (py - ay) ** 2
        else:
            t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / l2))
            qx, qy = ax + t * vx, ay + t * vy
            d2 = (px - qx) ** 2 + (py - qy) ** 2
        if d2 < best_sq:
            best_sq = d2
    return math.sqrt(best_sq)


def raycast_16_tofs(
    x: float,
    y: float,
    yaw: float,
    segs: List[Tuple[float, float, float, float]],
) -> Dict[str, float]:
    """Simulates all 16 VL53L8CX beams in world frame (+Y forward, +X right, +yaw CCW)."""
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    readings: Dict[str, float] = {}
    for s_name, sx_loc, sy_loc, h_loc, max_r in FRONT_TOF_SENSOR_RIG:
        # Local sensor origin -> world
        ox = x + cos_y * sx_loc - sin_y * sy_loc
        oy = y + sin_y * sx_loc + cos_y * sy_loc
        # Local sensor ray dir: (cos(h_loc), sin(h_loc)) in (+X right, +Y fwd)
        dlx = math.cos(h_loc)
        dly = math.sin(h_loc)
        dx = cos_y * dlx - sin_y * dly
        dy = sin_y * dlx + cos_y * dly

        best_t = max_r
        for ax, ay, bx, by in segs:
            t = ray_segment_intersect(ox, oy, dx, dy, ax, ay, bx, by)
            if t is not None and t < best_t:
                best_t = t
        readings[s_name] = max(0.030, min(TOF_MAX_RANGE_M, best_t))
    return readings


class SimCar2D:
    """Simple 60 Hz Ackermann bicycle simulator with tire grip limit & wall contact."""

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = 0.0
        self.steer = 0.0
        self.yaw_rate = 0.0
        self.wall_hits = 0

    def step(
        self,
        v_cmd: float,
        steer_cmd: float,
        dt: float,
        segs: List[Tuple[float, float, float, float]],
        force_stall: bool = False,
    ) -> None:
        max_ds = SERVO_MAX_SPEED_RAD_S * dt
        self.steer += max(-max_ds, min(max_ds, steer_cmd - self.steer))
        self.steer = max(-MAX_STEERING_RAD, min(MAX_STEERING_RAD, self.steer))

        max_dv = TIRE_MU * GRAVITY_G * dt
        self.v += max(-max_dv, min(max_dv, v_cmd - self.v))

        if force_stall:
            self.v = 0.0
            self.yaw_rate = 0.0
            return

        # Kinematic yaw rate with friction-circle understeer cap
        raw_omega = (self.v / WHEELBASE) * math.tan(self.steer)
        max_omega = (TIRE_MU * GRAVITY_G) / max(0.5, abs(self.v))
        self.yaw_rate = max(-max_omega, min(max_omega, raw_omega))

        yaw_mid = self.yaw + 0.5 * self.yaw_rate * dt
        nx = self.x - self.v * math.sin(yaw_mid) * dt
        ny = self.y + self.v * math.cos(yaw_mid) * dt
        nyaw = wrap_to_pi(self.yaw + self.yaw_rate * dt)

        # Wall contact clamp (chassis half-width ~ 0.075 m)
        d_wall = point_to_segments_dist(nx, ny, segs)
        if d_wall < 0.070:
            self.wall_hits += 1
            self.v *= 0.15
            self.yaw_rate *= 0.3
        else:
            self.x = nx
            self.y = ny
            self.yaw = nyaw


def make_corridor_walls(width: float = 1.0, length: float = 10.0) -> List[Tuple[float, float, float, float]]:
    hw = width * 0.5
    return [
        (-hw, -2.0, -hw, length),
        (hw, -2.0, hw, length),
        (-hw, -2.0, hw, -2.0),
        (-hw, length, hw, length),
    ]


def make_hairpin_walls() -> List[Tuple[float, float, float, float]]:
    """Closed oval hairpin: 1.0m left lane [-0.5, +0.5], dividing wall at x=0.50..0.60, 1.0m right lane [0.60, 1.60]."""
    return [
        (-0.50, -1.5, -0.50, 5.8),
        (-0.50, 5.8, 1.60, 5.8),
        (1.60, 5.8, 1.60, -1.5),
        (1.60, -1.5, -0.50, -1.5),
        (0.50, -0.3, 0.50, 4.6),
        (0.60, -0.3, 0.60, 4.6),
        (0.50, 4.6, 0.60, 4.6),
        (0.50, -0.3, 0.60, -0.3),
    ]


def run_step(brain: AutonomousBrain, car: SimCar2D, segs: List[Tuple[float, float, float, float]], sim_t: float, dt: float, force_stall: bool = False):
    tofs = raycast_16_tofs(car.x, car.y, car.yaw, segs)
    v_cmd, steer_cmd, telem = brain.step(
        tof_readings=tofs,
        v_est=car.v,
        steer_actual_rad=car.steer,
        gyro_z_rad_s=car.yaw_rate,
        dt=dt,
        sim_time_s=sim_t,
        ground_truth_pose=None,
        full_telemetry=True,
        imu_yaw_rad=car.yaw,
    )
    car.step(v_cmd, steer_cmd, dt, segs, force_stall=force_stall)
    return telem


def scenario_s1_straight_180_spin() -> Tuple[bool, str]:
    brain = AutonomousBrain()
    brain.reset((0.0, 0.0, -math.pi / 2.0))
    car = SimCar2D(0.0, 0.0, -math.pi / 2.0)
    segs = make_corridor_walls(width=1.0, length=40.0)
    dt = 1.0 / 60.0
    sim_t = 0.0

    # Drive forward 2.5 s
    for _ in range(150):
        sim_t += dt
        run_step(brain, car, segs, sim_t, dt)

    # Inject 180 deg spinout
    car.yaw = math.pi
    car.v = 0.1
    brain.pose_tracker.yaw = math.pi
    brain.pose_tracker._imu_yaw_offset = 0.0

    t_spin = sim_t
    detect_lat = None
    rec_done = None
    aligned_at_exit = False
    retriggers = 0
    was_in_180 = False

    for _ in range(300):  # 5.0 s max
        sim_t += dt
        telem = run_step(brain, car, segs, sim_t, dt)
        is_ww = bool(telem.get("is_wrong_way"))
        rec_state = telem.get("recovery_state", "")
        in_180 = (rec_state == StuckRecoveryState.CORRECTIVE_180)

        if is_ww and detect_lat is None:
            detect_lat = sim_t - t_spin
        if rec_done is None and was_in_180 and not in_180:
            rec_done = sim_t - t_spin
            aligned_at_exit = abs(wrap_to_pi(car.yaw - 0.0)) <= math.radians(35.0)
        elif rec_done is not None and is_ww:
            retriggers += 1
        was_in_180 = in_180

    aligned = abs(wrap_to_pi(car.yaw - 0.0)) <= math.radians(35.0)
    ok = (
        detect_lat is not None and detect_lat <= 0.35
        and rec_done is not None and rec_done <= 2.5
        and aligned_at_exit and aligned and retriggers == 0
    )
    return ok, f"detect={detect_lat:.3f}s rec={rec_done:.3f}s aligned={aligned} retriggers={retriggers} hits={car.wall_hits}"


def scenario_s2_hairpin_no_fp() -> Tuple[bool, str]:
    brain = AutonomousBrain()
    brain.reset((0.0, 0.0, -math.pi / 2.0))
    car = SimCar2D(0.0, 0.0, -math.pi / 2.0)
    segs = make_hairpin_walls()
    dt = 1.0 / 60.0
    sim_t = 0.0
    ww_triggers = 0
    entered_return_lane = False
    for _ in range(240):  # 4.0 s: drive up left lane, round hairpin, drive down right lane
        sim_t += dt
        telem = run_step(brain, car, segs, sim_t, dt)
        if car.x > 0.60 and car.y > 0.0:
            entered_return_lane = True
        if telem.get("is_wrong_way"):
            ww_triggers += 1
    ok = ww_triggers == 0 and entered_return_lane
    return ok, f"ww_triggers={ww_triggers} entered_return_lane={entered_return_lane} final=({car.x:.2f},{car.y:.2f}) hits={car.wall_hits}"


def scenario_s3_wall_hugging_3pt() -> Tuple[bool, str]:
    brain = AutonomousBrain()
    brain.reset((0.0, 0.0, -math.pi / 2.0))
    car = SimCar2D(0.0, 0.0, -math.pi / 2.0)
    segs = make_corridor_walls(width=1.0, length=40.0)
    dt = 1.0 / 60.0
    sim_t = 0.0
    for _ in range(150):
        sim_t += dt
        run_step(brain, car, segs, sim_t, dt)

    # Spin 170 deg near right wall (x = +0.32, facing toward wall/south)
    car.x = 0.32
    car.yaw = math.radians(170.0)
    car.v = 0.05
    brain.pose_tracker.x = car.x
    brain.pose_tracker.yaw = car.yaw
    brain.pose_tracker._imu_yaw_offset = 0.0

    t_spin = sim_t
    used_rev = False
    rec_done = None
    was_in_180 = False
    for _ in range(300):
        sim_t += dt
        telem = run_step(brain, car, segs, sim_t, dt)
        if "REVERSE" in str(telem.get("mode", "")):
            used_rev = True
        in_180 = (telem.get("recovery_state") == StuckRecoveryState.CORRECTIVE_180)
        if rec_done is None and was_in_180 and not in_180:
            rec_done = sim_t - t_spin
        was_in_180 = in_180
    aligned = abs(wrap_to_pi(car.yaw - 0.0)) <= math.radians(35.0)
    ok = aligned and (rec_done is not None)
    return ok, f"used_rev={used_rev} rec_done={rec_done} final_yaw_deg={math.degrees(car.yaw):.1f}"


def scenario_s4_shoved_forward_spin() -> Tuple[bool, str]:
    brain = AutonomousBrain()
    brain.reset((0.0, 0.0, -math.pi / 2.0))
    car = SimCar2D(0.0, 0.0, -math.pi / 2.0)
    segs = make_corridor_walls(width=1.0, length=40.0)
    dt = 1.0 / 60.0
    sim_t = 0.0
    for _ in range(150):
        sim_t += dt
        run_step(brain, car, segs, sim_t, dt)

    # Shove +0.95m forward and spin 180 deg
    car.y += 0.95
    car.yaw = math.pi
    car.v = 0.0
    brain.pose_tracker.y = car.y
    brain.pose_tracker.yaw = car.yaw
    brain.pose_tracker._imu_yaw_offset = 0.0

    detect_lat = None
    t_spin = sim_t
    for _ in range(240):
        sim_t += dt
        telem = run_step(brain, car, segs, sim_t, dt)
        if telem.get("is_wrong_way") and detect_lat is None:
            detect_lat = sim_t - t_spin
    aligned = abs(wrap_to_pi(car.yaw - 0.0)) <= math.radians(35.0)
    ok = (detect_lat is not None and detect_lat <= 0.35) and aligned
    return ok, f"detect={detect_lat} aligned={aligned} final_yaw_deg={math.degrees(car.yaw):.1f}"


def scenario_s5_opponent_occlusion_spin() -> Tuple[bool, str]:
    brain = AutonomousBrain()
    brain.reset((0.0, 0.0, -math.pi / 2.0))
    car = SimCar2D(0.0, 0.0, -math.pi / 2.0)
    base_segs = make_corridor_walls(width=1.0, length=40.0)
    dt = 1.0 / 60.0
    sim_t = 0.0
    for _ in range(150):
        sim_t += dt
        run_step(brain, car, base_segs, sim_t, dt)

    # Spin 180 deg and place opponent box 0.80m in front (south of car) for 1.5s
    car.yaw = math.pi
    car.v = 0.0
    brain.pose_tracker.yaw = car.yaw
    brain.pose_tracker._imu_yaw_offset = 0.0
    occ_segs = base_segs + box_segments(car.x, car.y - 0.80, half_w=0.22, half_l=0.10)

    detected_during = False
    for _ in range(90):  # 1.5 s occluded
        sim_t += dt
        telem = run_step(brain, car, occ_segs, sim_t, dt)
        if telem.get("is_wrong_way"):
            detected_during = True

    # Opponent moves away -> trail must still be intact and trigger recovery if not already aligned
    detected_after = False
    for _ in range(210):
        sim_t += dt
        telem = run_step(brain, car, base_segs, sim_t, dt)
        if telem.get("is_wrong_way"):
            detected_after = True

    aligned = abs(wrap_to_pi(car.yaw - 0.0)) <= math.radians(35.0)
    ok = (detected_during or detected_after) and aligned
    return ok, f"det_during={detected_during} det_after={detected_after} aligned={aligned} final_yaw_deg={math.degrees(car.yaw):.1f}"


def scenario_s6_corner_catch_stall() -> Tuple[bool, str]:
    brain = AutonomousBrain()
    brain.reset((0.0, 0.0, -math.pi / 2.0))
    car = SimCar2D(0.0, 0.0, -math.pi / 2.0)
    segs = make_corridor_walls(width=1.0, length=40.0)
    dt = 1.0 / 60.0
    sim_t = 0.0
    for _ in range(150):
        sim_t += dt
        run_step(brain, car, segs, sim_t, dt)

    car.yaw = math.pi
    car.v = 0.0
    brain.pose_tracker.yaw = car.yaw
    brain.pose_tracker._imu_yaw_offset = 0.0

    # Wait for CORRECTIVE_180 to engage, then force stall for 0.6 s
    engaged_rev = False
    stalled_frames = 0
    for _ in range(300):
        sim_t += dt
        in_180_fwd = (
            brain.recovery_manager.state == StuckRecoveryState.CORRECTIVE_180
            and brain.recovery_manager.corrective_substate == "FORWARD"
        )
        force = False
        if in_180_fwd and stalled_frames < 36:
            force = True
            stalled_frames += 1
        telem = run_step(brain, car, segs, sim_t, dt, force_stall=force)
        if brain.recovery_manager.state == StuckRecoveryState.CORRECTIVE_180 and brain.recovery_manager.corrective_substate == "REVERSE":
            engaged_rev = True

    aligned = abs(wrap_to_pi(car.yaw - 0.0)) <= math.radians(35.0)
    ok = engaged_rev and aligned
    return ok, f"engaged_rev={engaged_rev} aligned={aligned} final_yaw_deg={math.degrees(car.yaw):.1f}"


def scenario_s7_midturn_early_exit() -> Tuple[bool, str]:
    brain = AutonomousBrain()
    brain.reset((0.0, 0.0, 0.0))
    for i in range(80):
        brain.watchdog.update(0.0, i * 0.05, 0.0, v_est=2.0, yaw_rate=0.0, now_s=i * 0.05, dt=0.05)

    # Mid-turn at yaw = +pi/2 (facing -X):
    # - gap (-1.5, 0.01) points south (-Y) along old trail -> MUST reject
    # - gap (+1.5, 0.01) points north (+Y) forward -> MUST accept
    ok_bad, info_bad = brain.watchdog.is_gap_early_exit(0.0, 4.0, math.pi / 2.0, -1.5, 0.01)
    ok_good, info_good = brain.watchdog.is_gap_early_exit(0.0, 4.0, math.pi / 2.0, 1.5, 0.01)
    ok = (not ok_bad) and ok_good
    return ok, f"wrong_way_gap_exit={ok_bad} ({info_bad['reason']}), forward_gap_exit={ok_good} ({info_good['reason']})"


def scenario_s8_track_racing_2d_clean_lap() -> Tuple[bool, str]:
    left_pts, right_pts = load_track_walls()
    if len(left_pts) == 0 or len(right_pts) == 0:
        return False, "Failed to load track_racing.wbt walls"
    segs = segments_from_polyline(left_pts, closed=True) + segments_from_polyline(right_pts, closed=True)

    brain = AutonomousBrain()
    brain.reset((0.0, 0.0, -math.pi / 2.0))
    car = SimCar2D(0.0, 0.0, -math.pi / 2.0)
    dt = 1.0 / 60.0
    sim_t = 0.0
    ww_triggers = 0
    dist_traveled = 0.0
    px, py = car.x, car.y

    for _ in range(900):  # 15.0 s
        sim_t += dt
        telem = run_step(brain, car, segs, sim_t, dt)
        dist_traveled += math.hypot(car.x - px, car.y - py)
        px, py = car.x, car.y
        if telem.get("is_wrong_way"):
            ww_triggers += 1

    ok = ww_triggers == 0 and dist_traveled >= 15.0
    return ok, f"ww_triggers={ww_triggers} dist={dist_traveled:.1f}m hits={car.wall_hits}"


def main() -> int:
    scenarios = [
        ("S1_straight_180_spin", scenario_s1_straight_180_spin),
        ("S2_hairpin_no_fp", scenario_s2_hairpin_no_fp),
        ("S3_wall_hugging_3pt", scenario_s3_wall_hugging_3pt),
        ("S4_shoved_forward_spin", scenario_s4_shoved_forward_spin),
        ("S5_opponent_occlusion_spin", scenario_s5_opponent_occlusion_spin),
        ("S6_corner_catch_stall", scenario_s6_corner_catch_stall),
        ("S7_midturn_early_exit", scenario_s7_midturn_early_exit),
        ("S8_track_racing_2d_clean_lap", scenario_s8_track_racing_2d_clean_lap),
    ]

    print("=" * 80)
    print("  2D CLOSED-LOOP WRONG-WAY & RECOVERY SIMULATION BENCH")
    print("=" * 80)
    t0 = time.perf_counter()
    all_ok = True
    for name, fn in scenarios:
        passed, detail = fn()
        tag = "PASS" if passed else "FAIL"
        print(f"  [{tag}] {name:30s} | {detail}")
        if not passed:
            all_ok = False
    print(f"Finished in {time.perf_counter() - t0:.2f}s")
    print("=" * 80)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
