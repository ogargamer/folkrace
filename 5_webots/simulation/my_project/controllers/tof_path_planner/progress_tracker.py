"""
================================================================================
  WEBOTS TRACK PROGRESS TRACKER & WALL GEOMETRY LOADER
================================================================================
  Unified module for computing true ground-truth distance along the track
  centerline without shortcuts, wall-bounces, or reverse progress.
================================================================================
"""

import os
import re
import math
from typing import Dict, List, Optional, Tuple
import numpy as np

# Resolution of directory paths relative to this file
CURR_DIR = os.path.dirname(os.path.abspath(__file__))
CONTROLLERS_DIR = os.path.dirname(CURR_DIR)
MY_PROJECT_DIR = os.path.dirname(CONTROLLERS_DIR)


def load_track_walls(world_path: Optional[str] = None) -> Tuple[np.ndarray, np.ndarray]:
    """Loads exact left and right wall point coordinates from track_racing.wbt."""
    if world_path is None:
        world_path = os.path.join(MY_PROJECT_DIR, "worlds", "track_racing.wbt")

    if not os.path.exists(world_path):
        return np.zeros((0, 2)), np.zeros((0, 2))

    try:
        with open(world_path, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()

        left_pos = text.find("track_left_wall")
        right_pos = text.find("track_right_wall")

        def extract_pts(start_idx: int) -> np.ndarray:
            if start_idx == -1:
                return np.zeros((0, 2))
            m = re.search(r"Coordinate\s*\{\s*point\s*\[\s*([-\d\.\s,]+)\]", text[start_idx:])
            if m:
                return np.array([float(x) for x in m.group(1).replace(",", " ").split() if x]).reshape(-1, 3)[:, :2]
            return np.zeros((0, 2))

        left_pts = extract_pts(left_pos)
        right_pts = extract_pts(right_pos)
        return left_pts, right_pts
    except Exception as e:
        print(f"[ProgressTracker] Warning: Could not load track walls from {world_path}: {e}", flush=True)

    return np.zeros((0, 2)), np.zeros((0, 2))


class WebotsTrackProgressTracker:
    """
    Monotonically tracks true ground-truth progress along the racing line.
    Prevents cheating from:
      - Driving back and forth (distance only advances when pushing new frontiers)
      - Wiggling side to side (orthogonal projection keeps arc-length invariant)
      - Wall bouncing / shuddering
      - Reverse driving
    """

    def __init__(self, traj_csv_path: str):
        self.traj_pts = np.zeros((0, 2))
        self.dists = np.zeros(0)
        self.total_length = 14.109
        self.n_pts = 0

        if os.path.exists(traj_csv_path):
            try:
                arr = np.loadtxt(traj_csv_path, delimiter=",", skiprows=1)
                self.traj_pts = arr[:, 0:2]
                if arr.shape[1] >= 6:
                    self.dists = arr[:, 5]
                else:
                    diffs = np.diff(self.traj_pts, axis=0)
                    seg_dists = np.hypot(diffs[:, 0], diffs[:, 1])
                    self.dists = np.concatenate([[0.0], np.cumsum(seg_dists)])

                if len(self.traj_pts) > 1:
                    closing_gap = float(np.hypot(self.traj_pts[0, 0] - self.traj_pts[-1, 0], self.traj_pts[0, 1] - self.traj_pts[-1, 1]))
                    self.total_length = float(self.dists[-1] + closing_gap)
                else:
                    self.total_length = float(self.dists[-1])
                self.n_pts = len(self.dists)
            except Exception as e:
                print(f"[ProgressTracker] Warning: Failed to load trajectory {traj_csv_path}: {e}", flush=True)

        self.last_idx = 0
        self.prev_s = 0.0
        self.current_lap = 0
        self.current_track_dist = 0.0
        self.max_track_progress = 0.0
        self.prev_pos = np.array([0.0, 0.0])
        self.off_track = False

    def _project_point(self, p: np.ndarray, i0: int, i1: int) -> Tuple[float, float, float]:
        A = self.traj_pts[i0]
        B = self.traj_pts[i1]
        v = B - A
        v_sq = float(np.dot(v, v))
        if v_sq < 1e-9:
            return 0.0, float(np.linalg.norm(p - A)), 0.0
        t = float(np.clip(np.dot(p - A, v) / v_sq, 0.0, 1.0))
        proj = A + t * v
        d_perp = float(np.linalg.norm(p - proj))
        seg_len = math.sqrt(v_sq)
        return t, d_perp, seg_len

    def reset(self, start_pos: List[float]) -> None:
        if self.n_pts < 2:
            return
        cx, cy = float(start_pos[0]), float(start_pos[1])
        dists_sq = (self.traj_pts[:, 0] - cx) ** 2 + (self.traj_pts[:, 1] - cy) ** 2
        self.last_idx = int(np.argmin(dists_sq))

        p = np.array([cx, cy])
        i_next = (self.last_idx + 1) % self.n_pts
        t, _, seg_len = self._project_point(p, self.last_idx, i_next)
        self.prev_s = float(self.dists[self.last_idx] + t * seg_len)
        self.current_lap = 0
        self.current_track_dist = 0.0
        self.max_track_progress = 0.0
        self.prev_pos = np.array([cx, cy], dtype=float)
        self.off_track = False

    def update(self, current_pos: List[float]) -> float:
        if self.n_pts < 2:
            return 0.0
        cx, cy = float(current_pos[0]), float(current_pos[1])
        p = np.array([cx, cy])

        # Local search window: -15 to +35 waypoints around last tracked index
        search_window = [(self.last_idx + offset) % self.n_pts for offset in range(-15, 35)]
        cand_pts = self.traj_pts[search_window]
        dists_sq = (cand_pts[:, 0] - cx) ** 2 + (cand_pts[:, 1] - cy) ** 2
        best_local_idx = int(np.argmin(dists_sq))
        closest_idx = search_window[best_local_idx]

        # Test segments before and after closest waypoint
        i_prev = (closest_idx - 1) % self.n_pts
        i_next = (closest_idx + 1) % self.n_pts

        t_bwd, d_bwd, seg_bwd = self._project_point(p, i_prev, closest_idx)
        t_fwd, d_fwd, seg_fwd = self._project_point(p, closest_idx, i_next)

        if d_fwd <= d_bwd:
            seg_start = closest_idx
            t_chosen = t_fwd
            seg_len = seg_fwd
            min_d_perp = d_fwd
        else:
            seg_start = i_prev
            t_chosen = t_bwd
            seg_len = seg_bwd
            min_d_perp = d_bwd

        # Off track check (track width is 1.0m; racing line hugs inner apex so outer wall is up to 0.86m away)
        if min_d_perp > 0.95:
            self.off_track = True
            return self.max_track_progress

        s_curr = float(self.dists[seg_start] + t_chosen * seg_len)
        ds = s_curr - self.prev_s

        if ds < -0.5 * self.total_length:
            ds += self.total_length
        elif ds > 0.5 * self.total_length:
            ds -= self.total_length

        # Physical ceiling guard: clamp step jump at 0.25m
        step_euclid = float(np.hypot(cx - self.prev_pos[0], cy - self.prev_pos[1]))
        if ds > 0.25:
            ds = min(step_euclid, 0.25)

        self.current_track_dist += ds
        self.current_track_dist = max(0.0, self.current_track_dist)

        # Monotonic track progression: only advances when new forward ground is broken
        if self.current_track_dist > self.max_track_progress:
            self.max_track_progress = self.current_track_dist

        self.last_idx = closest_idx
        self.prev_s = s_curr
        self.prev_pos = np.array([cx, cy])
        self.current_lap = int(self.max_track_progress // self.total_length)
        return self.max_track_progress

    @property
    def total_progress_m(self) -> float:
        return self.max_track_progress
