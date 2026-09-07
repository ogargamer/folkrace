"""
================================================================================
  STANDALONE MPPI WEBOTS CONTROLLER & REAL-TIME DRIVER
  Model Predictive Path Integral (MPPI) Controller for 4WD Folkrace Robot
================================================================================
  Executes high-frequency (60-100 Hz) closed-loop path tracking and obstacle
  avoidance with 16-channel ToF sensor costmap integration and 4WD electronic differential.
================================================================================
"""

import os
import sys
import math
import time
import json
from typing import Optional, Dict, Any, Tuple

import numpy as np

# Add local controller and scanner paths to sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
SCANNER_DIR = os.path.join(CONTROLLERS_DIR, "tof_16ch_grid_scanner")

for d in [CURR_DIR, SCANNER_DIR]:
    if d not in sys.path and os.path.exists(d):
        sys.path.insert(0, d)

try:
    from controller import Robot, Supervisor, Keyboard
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False

from electronic_diff_4wd import (
    ElectronicDifferential4WD,
    WHEELBASE,
    WHEEL_RADIUS,
    TRACK_REAR,
    TRACK_FRONT,
    TRACK_KINGPIN,
    MAX_STEERING_RAD,
    MOTOR_DIR,
    V_MAX_WHEEL,
)
from tof_pathfinders import FrenetTransformer
from mppi_folkrace_controller import MPPIController, MPPIGenome

try:
    from fast_tof_carto_costmap import FastCostmapPipeline, SENSOR_RIG
    COSTMAP_AVAILABLE = True
except ImportError:
    COSTMAP_AVAILABLE = False

TIME_STEP = 10  # ms (100 Hz Webots basic step)


def load_trajectory_from_candidates(filenames: list) -> Optional[np.ndarray]:
    """Loads CSV trajectory data from candidate file paths."""
    workspace_root = r"c:\Users\nurmi\Documents\Da Winna"
    offset_dir = os.path.join(workspace_root, "racing_results", "tum_qp_obstacle_aware_25cm_offsets")
    local_traj_dir = os.path.join(CURR_DIR, "trajectories")

    candidate_paths = []
    for fn in filenames:
        candidate_paths.append(os.path.join(offset_dir, fn))
        candidate_paths.append(os.path.join(local_traj_dir, fn))
        candidate_paths.append(os.path.join(CURR_DIR, fn))

    for p in candidate_paths:
        if os.path.exists(p):
            rows = []
            with open(p, "r", encoding="utf-8") as f:
                lines = f.readlines()
                for line in lines[1:]:
                    parts = line.strip().split(",")
                    if len(parts) >= 6:
                        rows.append([float(x) for x in parts])
            if rows:
                print(f"[MPPI Controller] Loaded trajectory ({len(rows)} pts) from {p}", flush=True)
                return np.array(rows)
    return None


class FolkraceMPPIDriver:
    """
    Main Autonomous Driver using the MPPI Controller Suite with 4WD Electronic Differential.
    """

    def __init__(self, genome: Optional[MPPIGenome] = None):
        self.genome = genome if genome is not None else MPPIGenome()
        self.mppi = MPPIController(genome=self.genome)
        
        self.ediff = ElectronicDifferential4WD(
            wheelbase=WHEELBASE,
            track_rear=TRACK_REAR,
            track_front=TRACK_FRONT,
            track_kingpin=TRACK_KINGPIN,
            wheel_radius=WHEEL_RADIUS,
            max_steer=MAX_STEERING_RAD,
            motor_dir=MOTOR_DIR,
            v_max=V_MAX_WHEEL,
        )

        if COSTMAP_AVAILABLE:
            self.costmap_pipe = FastCostmapPipeline(resolution=0.02, enable_latency_comp=True)
        else:
            self.costmap_pipe = None

        # 1. Load Center Optimal Racing Line
        center_raw = load_trajectory_from_candidates([
            "master_cyan_champion_optimal.csv",
            "cyan_champion_trajectory.csv"
        ])
        if center_raw is not None:
            self.frenet = FrenetTransformer(
                center_raw[:, 5], center_raw[:, 0], center_raw[:, 1], center_raw[:, 3], center_raw[:, 4]
            )
            self.traj_v = center_raw[:, 2]
        else:
            self.frenet = None
            self.traj_v = None

        # 2. Load Dedicated Left Offset Line (+25cm)
        left_raw = load_trajectory_from_candidates(["wall_offset_left_25cm.csv"])
        if left_raw is not None:
            self.frenet_left = FrenetTransformer(
                left_raw[:, 5], left_raw[:, 0], left_raw[:, 1], left_raw[:, 3], left_raw[:, 4]
            )
        else:
            self.frenet_left = None

        # 3. Load Dedicated Right Offset Line (-25cm)
        right_raw = load_trajectory_from_candidates(["wall_offset_right_25cm.csv"])
        if right_raw is not None:
            self.frenet_right = FrenetTransformer(
                right_raw[:, 5], right_raw[:, 0], right_raw[:, 1], right_raw[:, 3], right_raw[:, 4]
            )
        else:
            self.frenet_right = None

        self.curr_station_idx = 0
        self.active_steer_rad = 0.0

    def step_control(
        self,
        rx: float,
        ry: float,
        ryaw: float,
        v_mps: float,
        gyro_z: float,
        tof_readings: Optional[Dict[str, float]] = None,
        timestamp_s: float = 0.0,
    ) -> Tuple[Dict[str, float], Dict[str, Any]]:
        """
        Executes one control cycle and produces 4-wheel speeds and Ackermann angles.
        Pure Cartographer localization: uses (rx, ry, ryaw) without encoder integration error.
        """
        # 1. Update pose buffer and build costmap
        if self.costmap_pipe is not None:
            self.costmap_pipe.update_pose(timestamp_s, rx, ry, ryaw, v_mps)
            if tof_readings is not None:
                prob_map, hazard_mask, _ = self.costmap_pipe.process_tof_frame(tof_readings, timestamp_s)
            else:
                prob_map = np.zeros((101, 101), dtype=np.float32)
                hazard_mask = np.zeros((101, 101), dtype=bool)
        else:
            prob_map = np.zeros((101, 101), dtype=np.float32)
            hazard_mask = np.zeros((101, 101), dtype=bool)

        # 2. Update closest trajectory waypoint
        if self.frenet is not None:
            _, _, self.curr_station_idx, _, _ = self.frenet.cartesian_to_frenet(
                rx, ry, guess_idx=self.curr_station_idx
            )
            v_ref = float(self.traj_v[self.curr_station_idx % len(self.traj_v)])
        else:
            v_ref = 3.5

        # 3. Compute optimal MPPI action with 3 dedicated geometric seeds
        steer_cmd, speed_cmd, diag = self.mppi.compute_control(
            rx=rx,
            ry=ry,
            ryaw=ryaw,
            v_mps=v_mps,
            active_steer_rad=self.active_steer_rad,
            prob_map=prob_map,
            hazard_mask=hazard_mask,
            frenet_helper=self.frenet,
            curr_idx=self.curr_station_idx,
            v_target_ref=v_ref,
            frenet_left=self.frenet_left,
            frenet_right=self.frenet_right,
        )

        self.active_steer_rad = steer_cmd

        # 4. Map virtual bicycle command to 4WD Ackermann speeds and servo angles
        actuators = self.ediff.update(
            target_v=speed_cmd,
            target_steer_rad=steer_cmd,
            gyro_z_rad_s=gyro_z
        )

        return actuators, diag


def main():
    if not WEBOTS_AVAILABLE:
        print("[MPPI Controller] Webots library not available. Exiting standalone runner.", flush=True)
        return

    print("[MPPI Controller] Initializing Webots Folkrace MPPI Driver...", flush=True)
    try:
        robot = Supervisor()
    except Exception:
        robot = Robot()

    driver = FolkraceMPPIDriver()

    # Actuators
    motors = {
        "FL": robot.getDevice("motor FL"),
        "FR": robot.getDevice("motor FR"),
        "RL": robot.getDevice("motor RL"),
        "RR": robot.getDevice("motor RR"),
    }
    for m in motors.values():
        if m:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)

    servos = {
        "left": robot.getDevice("servo left"),
        "right": robot.getDevice("servo right"),
    }
    for s in servos.values():
        if s:
            s.setPosition(0.0)

    # Sensors
    gyro = robot.getDevice("gyro")
    if gyro:
        gyro.enable(TIME_STEP)

    tof_devices = {}
    if COSTMAP_AVAILABLE:
        for s_rig in SENSOR_RIG:
            dev = robot.getDevice(s_rig.name)
            if dev:
                dev.enable(TIME_STEP)
                tof_devices[s_rig.name] = dev

    # Supervisor pose access
    robot_node = robot.getSelf()

    print("[MPPI Controller] Driver ready. Starting 60-100 Hz control loop...", flush=True)
    t_start = time.time()

    prev_x, prev_y = 0.0, 0.0
    prev_time = 0.0

    while robot.step(TIME_STEP) != -1:
        sim_time = robot.getTime()
        
        # Read Ground Truth Pose
        if robot_node is not None:
            pos = robot_node.getPosition()
            rot = robot_node.getOrientation()
            rx = float(pos[0])
            ry = float(pos[1])
            ryaw = math.atan2(rot[3], rot[0])
        else:
            rx, ry, ryaw = 0.0, 0.0, 0.0

        # Estimate velocity
        dt_est = max(1e-4, sim_time - prev_time)
        v_est = math.hypot(rx - prev_x, ry - prev_y) / dt_est
        prev_x, prev_y = rx, ry
        prev_time = sim_time

        # Gyro reading
        gyro_z = 0.0
        if gyro:
            g_vals = gyro.getValues()
            if len(g_vals) >= 3:
                gyro_z = float(g_vals[2])

        # ToF readings
        tof_readings = {}
        for name, dev in tof_devices.items():
            if hasattr(dev, "getRangeImage"):
                r_img = dev.getRangeImage()
                valid_r = [v for v in r_img if not math.isinf(v) and not math.isnan(v) and v <= 1.80] if r_img else []
                val = min(valid_r) if valid_r else 1.80
            elif hasattr(dev, "getValue"):
                val = dev.getValue()
                if val > 10.0:
                    val /= 1000.0
            else:
                val = 1.80
            tof_readings[name] = max(0.02, min(1.80, float(val)))

        # MPPI Step
        actuators, diag = driver.step_control(
            rx=rx,
            ry=ry,
            ryaw=ryaw,
            v_mps=v_est,
            gyro_z=gyro_z,
            tof_readings=tof_readings if tof_devices else None,
            timestamp_s=sim_time,
        )

        # Apply Motor Velocities (rad/s)
        if motors["FL"]: motors["FL"].setVelocity(actuators["omega_FL"])
        if motors["FR"]: motors["FR"].setVelocity(actuators["omega_FR"])
        if motors["RL"]: motors["RL"].setVelocity(actuators["omega_RL"])
        if motors["RR"]: motors["RR"].setVelocity(actuators["omega_RR"])

        # Apply Steering Angles (rad)
        if servos["left"]: servos["left"].setPosition(actuators["steer_left"])
        if servos["right"]: servos["right"].setPosition(actuators["steer_right"])

        if int(sim_time * 100) % 50 == 0:
            print(
                f"[MPPI 60Hz] Time: {sim_time:5.2f}s | Speed: {diag['cmd_speed_mps']:4.2f} m/s | "
                f"Steer: {math.degrees(diag['cmd_steer_rad']):+5.1f} deg | Cost: {diag['best_cost']:6.1f} | "
                f"Calc: {diag['exec_time_ms']:4.2f} ms | Seed W: {diag['seed_weights']['center']:.2f}/{diag['seed_weights']['left']:.2f}/{diag['seed_weights']['right']:.2f}",
                flush=True
            )


if __name__ == "__main__":
    main()
