"""
================================================================================
  GENETIC ALGORITHM HEADLESS EVALUATOR CONTROLLER FOR FOLKRACE MPPI
================================================================================
  - High-Throughput Persistent Worker Architecture (TCP Socket / Supervisor Reset)
  - Zero UDP/GUI Overhead, Maximum Simulation Speed (--mode=fast --no-rendering)
  - 60 Hz Sensor Sampling & 16ms Physics Alignment
  - Conservative Early Disqualification for Stalls and Immediate Crash Abort
================================================================================
"""

import os
import sys
import time
import math
import json
import socket
import select
from typing import Dict, List, Optional, Tuple, Any
import numpy as np

# Ensure parent and controller directories are on sys.path
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
FOLKRACE_DIR = os.path.join(CONTROLLERS_DIR, "folkrace_drive")
SCANNER_DIR = os.path.join(CONTROLLERS_DIR, "tof_16ch_grid_scanner")
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)
WORKSPACE_ROOT = os.path.dirname(MY_PROJECT_DIR)

for p in [CURR_DIR, FOLKRACE_DIR, SCANNER_DIR, MY_PROJECT_DIR, WORKSPACE_ROOT]:
    if p not in sys.path and os.path.exists(p):
        sys.path.insert(0, p)

try:
    from controller import Robot, Supervisor
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
from fast_tof_carto_costmap import FastCostmapPipeline, SENSOR_RIG, StampedPose, load_cartographer_map

TIME_STEP_MS = 16  # 60 Hz (16 ms) Webots physics & sensor sampling step
TOF_MAX_RANGE_M = 1.80
TOF_MIN_RANGE_M = 0.001

TOF_SENSOR_NAMES = [
    "tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3",
    "tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3",
    "tof_bl_0", "tof_bl_1", "tof_bl_2", "tof_bl_3",
    "tof_br_0", "tof_br_1", "tof_br_2", "tof_br_3",
]


def load_trajectory(filename: str) -> Optional[np.ndarray]:
    candidate_paths = [
        os.path.join(WORKSPACE_ROOT, "racing_results", "tum_qp_obstacle_aware_25cm_offsets", filename),
        os.path.join(FOLKRACE_DIR, "trajectories", filename),
        os.path.join(FOLKRACE_DIR, filename),
        os.path.join(CURR_DIR, filename),
    ]
    for path in candidate_paths:
        if os.path.exists(path):
            try:
                arr = np.loadtxt(path, delimiter=",", skiprows=1)
                if arr.ndim == 2 and arr.shape[1] >= 6:
                    return arr
            except Exception:
                pass
    return None


class GAWorkerDriver:
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

        grid_bounds = (-1.5, 1.5, -0.5, 2.4)
        self.costmap_pipe = FastCostmapPipeline(
            grid_bounds=grid_bounds,
            resolution=0.02,
            r_inflation_m=float(self.genome.r_inflation_m)
        )

        self.carto_grid = None
        self.carto_origin = (0.0, 0.0)
        self.carto_res = 0.02
        carto_yaml = os.path.join(WORKSPACE_ROOT, "maps", "cyan_champion_clean_static", "cartographer_2d_slam_static_map.yaml")
        if not os.path.exists(carto_yaml):
            carto_yaml = os.path.join(WORKSPACE_ROOT, "ros2_cartographer", "maps", "folkrace_track_map.yaml")
        if os.path.exists(carto_yaml):
            map_data = load_cartographer_map(carto_yaml)
            if map_data is not None:
                self.carto_grid, self.carto_origin, self.carto_res = map_data

        center_raw = load_trajectory("cyan_champion_trajectory.csv")
        if center_raw is None:
            center_raw = load_trajectory("master_cyan_champion_optimal.csv")

        if center_raw is not None:
            self.frenet_center = FrenetTransformer(
                center_raw[:, 5], center_raw[:, 0], center_raw[:, 1], center_raw[:, 3], center_raw[:, 4]
            )
            self.traj_v = center_raw[:, 2]
        else:
            self.frenet_center = None
            self.traj_v = np.full(100, 2.0)

        left_raw = load_trajectory("midpoint_offset_left.csv")
        if left_raw is None:
            left_raw = load_trajectory("wall_offset_left_25cm.csv")
        if left_raw is not None:
            self.frenet_left = FrenetTransformer(
                left_raw[:, 5], left_raw[:, 0], left_raw[:, 1], left_raw[:, 3], left_raw[:, 4]
            )
        else:
            self.frenet_left = None

        right_raw = load_trajectory("midpoint_offset_right.csv")
        if right_raw is None:
            right_raw = load_trajectory("wall_offset_right_25cm.csv")
        if right_raw is not None:
            self.frenet_right = FrenetTransformer(
                right_raw[:, 5], right_raw[:, 0], right_raw[:, 1], right_raw[:, 3], right_raw[:, 4]
            )
        else:
            self.frenet_right = None

        self.curr_station_idx = 0
        self.active_steer_rad = 0.0
        self.step_counter = 0

    def reset(self, genome: MPPIGenome):
        """Instantaneously reconfigures the driver with a new candidate genome."""
        self.genome = genome
        self.mppi = MPPIController(genome=self.genome)
        self.active_steer_rad = 0.0
        self.curr_station_idx = 0
        self.step_counter = 0
        self.costmap_pipe.set_inflation_radius(float(self.genome.r_inflation_m))
        self.costmap_pipe.pose_buffer.buffer.clear()

    def step(
        self,
        rx: float,
        ry: float,
        ryaw: float,
        v_mps: float,
        gyro_z: float,
        tof_readings: Dict[str, float],
        timestamp_s: float,
    ) -> Tuple[Dict[str, float], Dict[str, Any], float]:
        self.step_counter += 1

        if self.carto_grid is not None and (self.step_counter == 1 or self.step_counter % 4 == 0):
            pose_stamped = StampedPose(timestamp_s, rx, ry, ryaw, v_mps)
            self.costmap_pipe.update_slam_map(self.carto_grid, self.carto_origin, self.carto_res, pose_stamped)

        self.costmap_pipe.update_pose(timestamp_s, rx, ry, ryaw, v_mps)
        prob_map, hazard_mask, costmap_time_ms = self.costmap_pipe.process_tof_frame(
            tof_readings, timestamp_s
        )

        cte = 0.0
        if self.frenet_center is not None:
            _, cte, self.curr_station_idx, _, _ = self.frenet_center.cartesian_to_frenet(
                rx, ry, guess_idx=self.curr_station_idx
            )
            raw_v_ref = float(self.traj_v[self.curr_station_idx % len(self.traj_v)])
            v_ref = min(3.80, raw_v_ref)
        else:
            v_ref = 2.80

        steer_cmd, speed_cmd, diag = self.mppi.compute_control(
            rx=rx,
            ry=ry,
            ryaw=ryaw,
            v_mps=v_mps,
            active_steer_rad=self.active_steer_rad,
            prob_map=prob_map,
            hazard_mask=hazard_mask,
            frenet_helper=self.frenet_center,
            curr_idx=self.curr_station_idx,
            v_target_ref=v_ref,
            frenet_left=self.frenet_left,
            frenet_right=self.frenet_right,
        )

        self.active_steer_rad = steer_cmd
        diag["cte_m"] = cte
        diag["v_ref"] = v_ref

        actuators = self.ediff.update(
            target_v=speed_cmd,
            target_steer_rad=steer_cmd,
            gyro_z_rad_s=gyro_z,
        )

        return actuators, diag, costmap_time_ms


def read_tof_fast(dev: Any) -> float:
    val = TOF_MAX_RANGE_M
    if hasattr(dev, "getRangeImage"):
        r_img = dev.getRangeImage()
        if r_img:
            valid_r = [v for v in r_img if not math.isinf(v) and not math.isnan(v) and v <= TOF_MAX_RANGE_M]
            if valid_r:
                val = float(min(valid_r))
    elif hasattr(dev, "getValue"):
        raw = float(dev.getValue())
        if raw > 10.0:
            raw /= 1000.0
        val = raw
    return max(TOF_MIN_RANGE_M, min(TOF_MAX_RANGE_M, val))


def run_single_simulation(
    robot: Any,
    robot_node: Any,
    driver: GAWorkerDriver,
    motors: Dict[str, Any],
    servos: Dict[str, Any],
    gyro: Any,
    tof_devices: Dict[str, Any],
    obs_trans_fields: List[Any],
    active_genome: MPPIGenome,
    max_sim_seconds: float = 20.0,
    enable_moving_obstacle: bool = True,
    moving_obs_speed: float = 0.50,
    obs_start_s: float = 2.50,
) -> Dict[str, Any]:
    """Executes a single simulation evaluation and returns structured metrics."""
    # 1. Reset Driver Internal Buffers
    driver.reset(active_genome)

    # 2. Reset Webots Physics & Pose to Start Line
    if robot_node is not None:
        robot_node.getField("translation").setSFVec3f([0.60, 0.0, 0.028])
        robot_node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, -1.57079632679])
        robot_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        robot_node.resetPhysics()

    for m in motors.values():
        if m:
            m.setVelocity(0.0)
    for s in servos.values():
        if s:
            s.setPosition(0.0)

    # Reset moving obstacle positions
    if obs_trans_fields and driver.frenet_center is not None:
        frenet = driver.frenet_center
        tot_len = frenet.total_length
        n_obs = len(obs_trans_fields)
        for k, field in enumerate(obs_trans_fields):
            obs_s = (obs_start_s + (k * tot_len / n_obs)) % tot_len
            obs_idx = int(np.interp(obs_s, frenet.traj_s, np.arange(frenet.n_pts))) % frenet.n_pts
            field.setSFVec3f([float(frenet.traj_x[obs_idx]), float(frenet.traj_y[obs_idx]), 0.075])

    # Settle physics for 2 steps
    for _ in range(2):
        robot.step(TIME_STEP_MS)

    trial_start_time = robot.getTime()

    rx = 0.60
    ry = 0.00
    prev_pos = (rx, ry)
    v_est = 0.0
    cached_tof_readings = {name: TOF_MAX_RANGE_M for name in TOF_SENSOR_NAMES}

    distance_traveled_m = 0.0
    speeds = []
    steering_angles = []
    wall_proximity_costs = []
    collision_count = 0
    laps_completed = 0
    lap_times = []
    last_lap_time = 0.0
    prev_station_idx = 0
    stuck_start_time = None
    stuck_abort = False
    consecutive_near_wall_ticks = 0
    abort_reason = "COMPLETED"
    abort_sensor = "NONE"
    abort_details = ""
    trace = []
    last_trace_t = 0.0

    while True:
        if robot.step(TIME_STEP_MS) == -1:
            break

        current_sim_time = robot.getTime()
        now = current_sim_time - trial_start_time

        if max_sim_seconds > 0 and now >= max_sim_seconds:
            abort_reason = "MAX_TIME_REACHED"
            break

        # Move obstacles along trajectory
        if enable_moving_obstacle and obs_trans_fields and driver.frenet_center is not None:
            frenet = driver.frenet_center
            tot_len = frenet.total_length
            n_obs = len(obs_trans_fields)
            for k, field in enumerate(obs_trans_fields):
                obs_s = (obs_start_s + (k * tot_len / n_obs) + moving_obs_speed * now) % tot_len
                obs_idx = int(np.interp(obs_s, frenet.traj_s, np.arange(frenet.n_pts))) % frenet.n_pts
                field.setSFVec3f([float(frenet.traj_x[obs_idx]), float(frenet.traj_y[obs_idx]), 0.075])

        carto_wz = 0.0
        if robot_node is not None:
            pos = robot_node.getPosition()
            rot = robot_node.getOrientation()
            vel = robot_node.getVelocity()
            rx = float(pos[0])
            ry = float(pos[1])
            ryaw = math.atan2(float(rot[4]), float(rot[1]))
            v_est = math.hypot(float(vel[0]), float(vel[1]))
            carto_wz = float(vel[5])
        else:
            ryaw = 0.0

        step_dist = math.hypot(rx - prev_pos[0], ry - prev_pos[1])
        if now > 1.2:
            distance_traveled_m += step_dist
            speeds.append(v_est)

        prev_pos = (rx, ry)

        min_all_m = TOF_MAX_RANGE_M
        min_sensor_name = "none"
        for dev_name in TOF_SENSOR_NAMES:
            dev = tof_devices.get(dev_name)
            val = read_tof_fast(dev) if dev else TOF_MAX_RANGE_M
            cached_tof_readings[dev_name] = val
            if val < min_all_m:
                min_all_m = val
                min_sensor_name = dev_name

        tof_readings = cached_tof_readings.copy()
        min_all_cm = min_all_m * 100.0

        if now - last_trace_t >= 0.10:
            last_trace_t = now
            trace.append({
                "t": round(now, 2), "x": round(rx, 2), "y": round(ry, 2),
                "yaw_deg": round(math.degrees(ryaw), 1), "v": round(v_est, 2),
                "steer": round(driver.active_steer_rad, 2), "min_tof_cm": round(min_all_cm, 1),
                "min_sensor": min_sensor_name
            })

        # Collision Disqualification (< 3.0cm for 2 ticks)
        if now > 1.2:
            if min_all_cm < 3.0:
                consecutive_near_wall_ticks += 1
                if consecutive_near_wall_ticks >= 2:
                    collision_count = 1
                    stuck_abort = True
                    abort_reason = "WALL_CONTACT"
                    abort_sensor = min_sensor_name
                    abort_details = f"Sensor {min_sensor_name} read {min_all_cm:.1f}cm (< 3.0cm)"
                    break
            else:
                consecutive_near_wall_ticks = 0

            # Conservative Disqualification: Severe Stalls (failed to travel at least 2.0m after 10.0s)
            if now >= 10.0 and distance_traveled_m < 2.0:
                stuck_abort = True
                abort_reason = "STALL_10S_PROGRESS"
                abort_details = f"Traveled only {distance_traveled_m:.2f}m in 10s"
                break

        actuators, diag, map_time_ms = driver.step(
            rx=rx,
            ry=ry,
            ryaw=ryaw,
            v_mps=v_est,
            gyro_z=carto_wz,
            tof_readings=tof_readings,
            timestamp_s=now,
        )

        if now > 1.2:
            steering_angles.append(diag.get("cmd_steer_rad", 0.0))
            wall_proximity_costs.append(diag.get("best_cost", 0.0))

            curr_st_idx = driver.curr_station_idx
            n_tot_pts = driver.frenet_center.n_pts if driver.frenet_center else 1000
            if prev_station_idx > (n_tot_pts * 0.80) and curr_st_idx < (n_tot_pts * 0.20):
                laps_completed += 1
                lap_t = now - last_lap_time if last_lap_time > 0.0 else now
                lap_times.append(round(lap_t, 3))
                last_lap_time = now
            prev_station_idx = curr_st_idx

            # Continuous Stall Guard (allow up to 4.0s for reversing and slow sharp turns)
            if abs(v_est) < 0.15:
                if stuck_start_time is None:
                    stuck_start_time = now
                elif (now - stuck_start_time) > 4.0:
                    stuck_abort = True
                    abort_reason = "CONTINUOUS_STALL_4S"
                    abort_details = f"Vehicle stopped for > 4.0s at ({rx:.2f}, {ry:.2f})"
                    break
            else:
                stuck_start_time = None

        WARMUP_HOLD_SECONDS = 1.2
        if now < WARMUP_HOLD_SECONDS:
            actuators = {
                "omega_FL": 0.0, "omega_FR": 0.0,
                "omega_RL": 0.0, "omega_RR": 0.0,
                "steer_left": 0.0, "steer_right": 0.0,
            }

        if motors["FL"]: motors["FL"].setVelocity(actuators["omega_FL"])
        if motors["FR"]: motors["FR"].setVelocity(actuators["omega_FR"])
        if motors["RL"]: motors["RL"].setVelocity(actuators["omega_RL"])
        if motors["RR"]: motors["RR"].setVelocity(actuators["omega_RR"])

        if servos["left"]: servos["left"].setPosition(actuators["steer_left"])
        if servos["right"]: servos["right"].setPosition(actuators["steer_right"])

    # Stop motors at conclusion of trial
    for m in motors.values():
        if m: m.setVelocity(0.0)

    mean_speed = float(np.mean(speeds)) if speeds else 0.0
    max_speed = float(np.max(speeds)) if speeds else 0.0
    steering_var = float(np.var(steering_angles)) if steering_angles else 0.0
    mean_wall_cost = float(np.mean(wall_proximity_costs)) if wall_proximity_costs else 0.0

    eval_result = {
        "status": abort_reason if stuck_abort else "COMPLETED",
        "distance_m": round(distance_traveled_m, 3),
        "mean_speed_mps": round(mean_speed, 3),
        "max_speed_mps": round(max_speed, 3),
        "laps_completed": laps_completed,
        "lap_times": lap_times,
        "best_lap_time_s": round(min(lap_times), 3) if lap_times else 999.0,
        "collision_count": collision_count,
        "steering_variance": round(steering_var, 5),
        "mean_wall_proximity_cost": round(mean_wall_cost, 2),
        "sim_duration_s": round(now, 2),
        "stuck_abort": stuck_abort,
        "abort_reason": abort_reason,
        "abort_sensor": abort_sensor,
        "abort_details": abort_details,
        "trace": trace[-15:] if len(trace) > 15 else trace,
    }

    return eval_result


def main():
    if not WEBOTS_AVAILABLE:
        print("[GA Worker] Webots environment not found. Exiting.", flush=True)
        return

    try:
        robot = Supervisor()
        is_supervisor = True
        robot_node = robot.getSelf()
    except Exception:
        robot = Robot()
        is_supervisor = False
        robot_node = None

    driver = GAWorkerDriver()

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

    gyro = robot.getDevice("gyro") or robot.getDevice("bno08x_gyro")
    if gyro:
        gyro.enable(TIME_STEP_MS)

    tof_devices = {}
    for dev_name in TOF_SENSOR_NAMES:
        dev = robot.getDevice(dev_name)
        if dev:
            dev.enable(TIME_STEP_MS)
            tof_devices[dev_name] = dev

    enable_moving_obstacle = os.environ.get("ENABLE_MOVING_OBSTACLE", "1") == "1"
    moving_obs_speed = float(os.environ.get("MOVING_OBS_SPEED", "0.50"))
    obs_start_s = float(os.environ.get("MOVING_OBS_START_S", "2.50"))
    obs_trans_fields = []
    if is_supervisor and enable_moving_obstacle:
        for def_name in ("MOVING_OBSTACLE", "MOVING_OBSTACLE_2", "MOVING_OBSTACLE_3"):
            node = robot.getFromDef(def_name)
            if node is not None:
                field = node.getField("translation")
                if field is not None:
                    obs_trans_fields.append(field)

    worker_port_str = os.environ.get("MPPI_WORKER_PORT", "")

    # ==========================================================================
    # 1. PERSISTENT WORKER SERVER MODE (High-Throughput Pool Architecture)
    # ==========================================================================
    if worker_port_str:
        port = int(worker_port_str)
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(("127.0.0.1", port))
        server_sock.listen(1)
        server_sock.setblocking(False)

        while True:
            conn = None
            try:
                conn, _ = server_sock.accept()
            except (BlockingIOError, socket.error):
                if robot.step(TIME_STEP_MS) == -1:
                    break
                time.sleep(0.005)
                continue

            if conn is None:
                continue

            conn.setblocking(True)
            raw_req = ""
            while "\n" not in raw_req:
                chunk = conn.recv(16384).decode("utf-8")
                if not chunk:
                    break
                raw_req += chunk

            if not raw_req:
                conn.close()
                continue

            try:
                req = json.loads(raw_req.strip())
            except Exception as e:
                print(f"[GA Worker {port}] Failed to decode JSON request: {e}", flush=True)
                conn.close()
                continue

            req_type = req.get("type", "EVAL")
            if req_type == "SHUTDOWN":
                conn.sendall(b'{"status":"SHUTDOWN_ACK"}\n')
                conn.close()
                server_sock.close()
                break

            # Parse genome
            genome_data = req.get("genome", [])
            if isinstance(genome_data, list):
                active_genome = MPPIGenome.from_array(np.array(genome_data, dtype=np.float64))
            elif isinstance(genome_data, dict):
                active_genome = MPPIGenome(**genome_data)
            else:
                active_genome = MPPIGenome()

            sim_duration = float(req.get("duration_s", 20.0))

            # Execute simulation trial
            eval_res = run_single_simulation(
                robot=robot,
                robot_node=robot_node,
                driver=driver,
                motors=motors,
                servos=servos,
                gyro=gyro,
                tof_devices=tof_devices,
                obs_trans_fields=obs_trans_fields,
                active_genome=active_genome,
                max_sim_seconds=sim_duration,
                enable_moving_obstacle=enable_moving_obstacle,
                moving_obs_speed=moving_obs_speed,
                obs_start_s=obs_start_s,
            )

            # Send back evaluation metrics
            res_bytes = (json.dumps(eval_res) + "\n").encode("utf-8")
            conn.sendall(res_bytes)
            conn.close()

        if is_supervisor:
            robot.simulationQuit(0)
        sys.exit(0)

    # ==========================================================================
    # 2. LEGACY SINGLE-SHOT EVALUATION MODE (Smoke Test / Standalone Worker)
    # ==========================================================================
    genome_json_str = os.environ.get("MPPI_GENOME_JSON", "")
    active_genome = MPPIGenome()
    if genome_json_str:
        try:
            parsed_data = json.loads(genome_json_str)
            if isinstance(parsed_data, list):
                active_genome = MPPIGenome.from_array(np.array(parsed_data, dtype=np.float64))
            elif isinstance(parsed_data, dict):
                active_genome = MPPIGenome(**parsed_data)
        except Exception as e:
            print(f"[GA Worker] Failed to parse MPPI_GENOME_JSON: {e}", flush=True)

    max_sim_seconds = float(os.environ.get("WEBOTS_RUN_SECONDS", "20.0"))

    eval_result = run_single_simulation(
        robot=robot,
        robot_node=robot_node,
        driver=driver,
        motors=motors,
        servos=servos,
        gyro=gyro,
        tof_devices=tof_devices,
        obs_trans_fields=obs_trans_fields,
        active_genome=active_genome,
        max_sim_seconds=max_sim_seconds,
        enable_moving_obstacle=enable_moving_obstacle,
        moving_obs_speed=moving_obs_speed,
        obs_start_s=obs_start_s,
    )

    print("\n" + "=" * 80, flush=True)
    print("  GA EVALUATION RESULT SUMMARY", flush=True)
    print("=" * 80, flush=True)
    print(f"[EVAL_METRICS_JSON] {json.dumps(eval_result)}", flush=True)
    print(f"[EVAL_RESULT_JSON] {json.dumps(eval_result)}", flush=True)
    print("=" * 80 + "\n", flush=True)

    eval_file_path = os.environ.get("MPPI_EVAL_FILE", "")
    if eval_file_path:
        try:
            with open(eval_file_path, "w", encoding="utf-8") as ef:
                json.dump(eval_result, ef, indent=2)
        except Exception as e:
            print(f"[GA Worker] Error writing {eval_file_path}: {e}", flush=True)

    if is_supervisor:
        robot.simulationQuit(0)
    sys.exit(0)


if __name__ == "__main__":
    main()
