"""
================================================================================
  DYNAMIC REVERSE OBSTACLE TRAJECTORY ENGINE
================================================================================
  Interpolates moving obstacle positions along the track centerline in reverse
  (counter-flow traffic pace) and updates Webots Solid translation/rotation.
================================================================================
"""

import os
from typing import Dict, List, Optional, Tuple
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)


class ReverseObstacleEngine:
    """
    Interpolates obstacle positions along the track racing line in reverse
    (traveling against robot heading at specified speed).
    """

    def __init__(self, csv_path: str, speed_mps: float = 0.50):
        self.speed_mps = speed_mps
        self.traj = np.zeros((0, 2))
        self.dists = np.zeros(0)
        self.total_length = 14.109

        if os.path.exists(csv_path):
            try:
                arr = np.loadtxt(csv_path, delimiter=",", skiprows=1)
                self.traj = arr[:, 0:2]
                if arr.shape[1] >= 6:
                    self.dists = arr[:, 5]
                else:
                    diffs = np.diff(self.traj, axis=0)
                    seg_dists = np.hypot(diffs[:, 0], diffs[:, 1])
                    self.dists = np.concatenate([[0.0], np.cumsum(seg_dists)])

                # Ensure closed-loop trajectory continuity across the start/finish line (s=0)
                if len(self.traj) > 1:
                    closing_gap = float(np.hypot(self.traj[0, 0] - self.traj[-1, 0], self.traj[0, 1] - self.traj[-1, 1]))
                    if closing_gap > 1e-4:
                        self.traj = np.vstack([self.traj, self.traj[0:1]])
                        self.dists = np.append(self.dists, self.dists[-1] + closing_gap)

                self.total_length = float(self.dists[-1])
            except Exception as e:
                print(f"[ReverseObstacleEngine] Failed to parse trajectory CSV: {e}", flush=True)

    def get_position_at_s(self, s: float) -> Tuple[float, float]:
        if len(self.dists) < 2:
            return 0.0, 0.0
        s_norm = s % self.total_length
        idx = int(np.searchsorted(self.dists, s_norm))
        if idx <= 0:
            return float(self.traj[0, 0]), float(self.traj[0, 1])
        if idx >= len(self.dists):
            return float(self.traj[-1, 0]), float(self.traj[-1, 1])

        s0 = self.dists[idx - 1]
        s1 = self.dists[idx]
        ratio = (s_norm - s0) / max(1e-6, s1 - s0)
        p0 = self.traj[idx - 1]
        p1 = self.traj[idx]
        x = p0[0] + ratio * (p1[0] - p0[0])
        y = p0[1] + ratio * (p1[1] - p0[1])
        return float(x), float(y)


class MovingObstacleManager:
    """Manages Webots solid nodes representing dynamic moving obstacles."""

    def __init__(
        self,
        robot: any,
        engine: ReverseObstacleEngine,
        obstacle_names: Optional[List[str]] = None,
        elevation_z: float = 0.075,
        speed_mps: float = 1.00,
        direction: int = 1,
    ):
        self.robot = robot
        self.engine = engine
        self.elevation_z = elevation_z
        self._speed_mps = abs(float(speed_mps))
        if speed_mps < 0:
            self.direction = -1
        else:
            self.direction = -1 if direction < 0 else 1

        if obstacle_names is None:
            obstacle_names = ["MOVING_OBSTACLE", "MOVING_OBSTACLE_2", "MOVING_OBSTACLE_3"]
        self.obstacle_names = obstacle_names
        self.obs_nodes = []
        self.current_s: List[float] = []
        self.last_sim_time_s = 0.0

        if hasattr(robot, "getFromDef"):
            for name in self.obstacle_names:
                node = robot.getFromDef(name)
                if node:
                    field = node.getField("translation")
                    if field:
                        self.obs_nodes.append(field)

        self._init_positions()

    @property
    def speed_mps(self) -> float:
        return self._speed_mps

    @speed_mps.setter
    def speed_mps(self, val: float):
        v = float(val)
        if v < 0:
            self.direction = -1
            self._speed_mps = abs(v)
        else:
            self._speed_mps = v

    def _init_positions(self) -> None:
        n_obs = len(self.obs_nodes)
        if n_obs > 0 and self.engine.total_length > 0:
            self.current_s = [
                (float(k + 1) * (self.engine.total_length / float(n_obs + 1))) % self.engine.total_length
                for k in range(n_obs)
            ]
        else:
            self.current_s = []
        self.last_sim_time_s = 0.0

    def set_speed(self, speed: float) -> None:
        self.speed_mps = float(speed)

    def set_direction(self, direction: int) -> None:
        self.direction = -1 if int(direction) < 0 else 1

    def toggle_direction(self) -> int:
        self.direction = -self.direction
        return self.direction

    def reset(self) -> None:
        self._init_positions()
        if len(self.obs_nodes) > 0 and self.engine.total_length > 0:
            for k, field in enumerate(self.obs_nodes):
                ox, oy = self.engine.get_position_at_s(self.current_s[k])
                field.setSFVec3f([float(ox), float(oy), self.elevation_z])

    def update(self, sim_time_s: float) -> List[Tuple[float, float]]:
        positions = []
        n_obs = len(self.obs_nodes)
        if n_obs > 0 and self.engine.total_length > 0:
            if not self.current_s or len(self.current_s) != n_obs:
                self._init_positions()

            # Handle reset / rewind
            if sim_time_s < self.last_sim_time_s or (sim_time_s == 0.0 and self.last_sim_time_s > 0.0):
                self._init_positions()
                dt = 0.0
            else:
                dt = max(0.0, sim_time_s - self.last_sim_time_s)
                if dt > 2.0:
                    dt = 0.0

            self.last_sim_time_s = sim_time_s
            step_ds = self.direction * self._speed_mps * dt

            for k, field in enumerate(self.obs_nodes):
                self.current_s[k] = (self.current_s[k] + step_ds) % self.engine.total_length
                ox, oy = self.engine.get_position_at_s(self.current_s[k])
                field.setSFVec3f([float(ox), float(oy), self.elevation_z])
                positions.append((float(ox), float(oy)))
        return positions
