"""
================================================================================
  DYNAMIC MOVING OBSTACLE TRACKER & CONSERVATIVE VECTOR ENLARGER
================================================================================
  Pipeline:
    1. Binary Costmap Extraction (separating occupied from unknown & free space)
    2. Blob Extraction & Geometric Filtering (cv2.connectedComponentsWithStats)
       - Wall Rejection (touching boundaries or bounding box > 25 cells)
       - Noise Floor (area < 4 cells)
    3. World-Frame Coordinate Projection
    4. Rolling History Buffer with Distance-Sorted Association
       - Dual-Stage Gating: 3 consecutive association frames + 2 dynamic confirmation frames
    5. Velocity Calculation & Strictly 1D Range-Rate Gating (threshold = 0.40 m/s)
       - Pure 1D longitudinal consensus: APPROACHING, RECEDING, or STATIC
    6. Conservative Vector Enlargement towards motion vector & guarded trailing tail reduction
================================================================================
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any

import cv2
import numpy as np


# 16-channel sensor rig coordinates and headings (matching only_front_tofs.proto)
SENSOR_CONFIG = [
    # FL 0..7 (X = -0.070, Y = +0.095, cross-pointing Right)
    (-0.070, 0.095, 1.4987),
    (-0.070, 0.095, 1.3024),
    (-0.070, 0.095, 1.1060),
    (-0.070, 0.095, 0.9097),
    (-0.070, 0.095, 0.7134),
    (-0.070, 0.095, 0.5170),
    (-0.070, 0.095, 0.3207),
    (-0.070, 0.095, 0.1243),
    # FR 8..15 (X = +0.070, Y = +0.095, cross-pointing Left)
    ( 0.070, 0.095, 1.6428),
    ( 0.070, 0.095, 1.8391),
    ( 0.070, 0.095, 2.0356),
    ( 0.070, 0.095, 2.2318),
    ( 0.070, 0.095, 2.4282),
    ( 0.070, 0.095, 2.6245),
    ( 0.070, 0.095, 2.8208),
    ( 0.070, 0.095, 3.0172),
]

SENSOR_NAMES = [
    "tof_front_left_4.1deg",
    "tof_front_left_15.4deg",
    "tof_front_left_26.6deg",
    "tof_front_left_37.9deg",
    "tof_front_left_49.1deg",
    "tof_front_left_60.4deg",
    "tof_front_left_71.6deg",
    "tof_front_left_82.9deg",
    "tof_front_right_4.1deg",
    "tof_front_right_15.4deg",
    "tof_front_right_26.6deg",
    "tof_front_right_37.9deg",
    "tof_front_right_49.1deg",
    "tof_front_right_60.4deg",
    "tof_front_right_71.6deg",
    "tof_front_right_82.9deg",
]


@dataclass
class ObstacleBlob:
    """Surviving candidate obstacle contact from geometric or ray filtering."""
    blob_id: int
    u: float               # Centroid column index in grid
    v: float               # Centroid row index in grid
    x_r: float             # Centroid X in robot frame (meters, +X right)
    y_r: float             # Centroid Y in robot frame (meters, +Y forward)
    area: int = 1          # Cell area or ray cluster weight
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)  # (left, top, width, height)


@dataclass
class ObstacleTrack:
    """Tracked obstacle over rolling 5-frame history window with 1D range-rate estimation."""
    track_id: int
    history: deque = field(default_factory=lambda: deque(maxlen=5))       # [(X_w, Y_w, t_s), ...]
    body_history: deque = field(default_factory=lambda: deque(maxlen=5))  # [(x_r, y_r, t_s), ...]
    speed_history: deque = field(default_factory=lambda: deque(maxlen=6)) # [(v_x, v_y, is_braking), ...]
    median_y_history: deque = field(default_factory=lambda: deque(maxlen=3)) # 3-frame median pre-filter on y_r
    consecutive_associations: int = 1
    last_frame_updated: int = 0
    is_dynamic: bool = False
    is_braking: bool = False
    state: str = "STATIC"  # "APPROACHING", "RECEDING", "STATIC" (strictly 1D longitudinal)
    consensus_count: int = 0
    consensus_dir: str = "STATIC"
    v_x: float = 0.0       # Global X velocity (m/s)
    v_y: float = 0.0       # Global Y velocity (m/s)
    v_mag: float = 0.0     # Speed magnitude (m/s)
    x_r: float = 0.0       # Current robot-relative X (m)
    y_r: float = 0.0       # Current robot-relative Y (m)
    v_rx: float = 0.0      # Current robot-relative X velocity (m/s)
    v_ry: float = 0.0      # Current robot-relative Y velocity (m/s)
    v_comp_x: float = 0.0  # Kinematically compensated X velocity (m/s)
    v_comp_y: float = 0.0  # Kinematically compensated Y velocity (m/s)
    v_longitudinal: float = 0.0 # Range-rate along robot heading (+ = receding, - = approaching)
    x_filt: float = 0.0    # Alpha-Beta filtered world X (m)
    y_filt: float = 0.0    # Alpha-Beta filtered world Y (m)
    filter_initialized: bool = False


class DynamicObstacleTracker:
    """
    High-performance, conservative dynamic moving obstacle detector and tracker
    designed for 60 Hz execution on 10mm/15mm front-ToF occupancy costmaps.
    Features:
      - 3-frame persistence (50 ms detection latency) with 1-frame dynamic confirmation
      - Conical forward search ROI (front bumper out to forward horizon)
      - Sub-pixel centroid stabilization and wall clutter rejection
      - Asymmetric deceleration quenching and kinematic ego-motion compensation
    """

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.25, 1.25, 0.0, 1.90),
        resolution: float = 0.010,
        prob_threshold: float = 0.65,
        max_wall_dim_cells: int = 38,
        min_area_cells: int = 6,
        gating_radius_m: float = 0.20,
        persistence_frames: int = 3,
        dynamic_threshold_mps: float = 0.40,
        lead_time_s: float = 0.20,
        max_lead_m: float = 0.25,
        tail_clear_m: float = 0.05,
        max_lateral_m: Optional[float] = 0.52,
        max_missed_frames: int = 3,
        dt_frame: float = 1.0 / 60.0,
        alpha_ab: float = 0.25,
        beta_ab: float = 0.024,
        vel_decay: float = 0.985,
        dyn_confirm_frames: int = 2,
        roi_conical: bool = True,
        roi_y_min: float = 0.05,
        roi_y_max: float = 1.80,
        roi_half_w_near: float = 0.35,
        roi_half_w_far: float = 0.52,
    ):
        self.x_min, self.x_max, self.y_min, self.y_max = grid_bounds
        self.res = float(resolution)
        self.prob_threshold = float(prob_threshold)

        # Standard grid geometry
        self.width = int(round((self.x_max - self.x_min) / self.res)) + 1
        self.height = int(round((self.y_max - self.y_min) / self.res)) + 1
        self.shape = (self.height, self.width)

        # Origin offsets (u0, v0) where xr = (u - u0)*res, yr = (v - v0)*res
        self.u0 = -self.x_min / self.res
        self.v0 = -self.y_min / self.res

        # Geometric filter parameters
        self.max_wall_dim_cells = int(max_wall_dim_cells)  # ~38 cells at 10mm = 38 cm
        self.min_area_cells = int(min_area_cells)          # 6 cells at 10mm
        self.gating_radius_m = float(gating_radius_m)      # 0.20 m association radius
        self.persistence_frames = int(persistence_frames)  # 3 consecutive frames (50 ms)
        self.dynamic_threshold_mps = float(dynamic_threshold_mps)  # 0.25 m/s
        self.max_lateral_m = float(max_lateral_m) if max_lateral_m is not None else None
        self.max_missed_frames = int(max_missed_frames)    # 2 frames coasting tolerance

        # Conical Forward ROI bounds
        self.roi_conical = bool(roi_conical)
        self.roi_y_min = float(roi_y_min)
        self.roi_y_max = float(roi_y_max)
        self.roi_half_w_near = float(roi_half_w_near)
        self.roi_half_w_far = float(roi_half_w_far)

        # Alpha-Beta filter tuning & conservative velocity decay
        self.alpha_ab = float(alpha_ab)
        self.beta_ab = float(beta_ab)
        self.vel_decay = float(vel_decay)
        self.dyn_confirm_frames = int(dyn_confirm_frames)
        self.dyn_confirm_counts: Dict[int, int] = {}

        # Conservative enlargement parameters
        self.lead_time_s = float(lead_time_s)              # 0.20 s forward horizon
        self.max_lead_m = float(max_lead_m)                # Capped to 0.25 m
        self.tail_clear_m = float(tail_clear_m)            # 0.05 m trailing reduction
        self.dt_frame = float(dt_frame)                    # ~0.01667 s at 60 Hz

        # Active tracks dictionary {track_id: ObstacleTrack}
        self.tracks: Dict[int, ObstacleTrack] = {}
        self.next_track_id = 1
        self.frame_counter = 0

        # Cached outputs for live telemetry
        self.last_surviving_blobs: List[ObstacleBlob] = []
        self.last_dynamic_tracks: List[ObstacleTrack] = []
        self.last_confirmed_tracks: List[ObstacleTrack] = []

    def reset(self) -> None:
        """Resets all active tracks, counters, and confirmation buffers."""
        self.tracks.clear()
        self.dyn_confirm_counts.clear()
        self.next_track_id = 1
        self.frame_counter = 0
        self.last_surviving_blobs.clear()
        self.last_dynamic_tracks.clear()
        self.last_confirmed_tracks.clear()

    def configure(
        self,
        prob_threshold: Optional[float] = None,
        max_wall_dim_cells: Optional[int] = None,
        min_area_cells: Optional[int] = None,
        gating_radius_m: Optional[float] = None,
        persistence_frames: Optional[int] = None,
        dynamic_threshold_mps: Optional[float] = None,
        lead_time_s: Optional[float] = None,
        max_lead_m: Optional[float] = None,
        tail_clear_m: Optional[float] = None,
        max_missed_frames: Optional[int] = None,
        alpha_ab: Optional[float] = None,
        beta_ab: Optional[float] = None,
        vel_decay: Optional[float] = None,
        dyn_confirm_frames: Optional[int] = None,
        roi_conical: Optional[bool] = None,
        roi_y_min: Optional[float] = None,
        roi_y_max: Optional[float] = None,
        roi_half_w_near: Optional[float] = None,
        roi_half_w_far: Optional[float] = None,
        **kwargs: Any,
    ) -> None:
        """Dynamically reconfigures tracking, detection, and enlargement parameters."""
        if prob_threshold is not None:
            self.prob_threshold = float(prob_threshold)
        if max_wall_dim_cells is not None:
            self.max_wall_dim_cells = int(max_wall_dim_cells)
        if min_area_cells is not None:
            self.min_area_cells = int(min_area_cells)
        if gating_radius_m is not None:
            self.gating_radius_m = float(gating_radius_m)
        if persistence_frames is not None:
            self.persistence_frames = int(persistence_frames)
        if dynamic_threshold_mps is not None:
            self.dynamic_threshold_mps = float(dynamic_threshold_mps)
        if lead_time_s is not None:
            self.lead_time_s = float(lead_time_s)
        if max_lead_m is not None:
            self.max_lead_m = float(max_lead_m)
        if tail_clear_m is not None:
            self.tail_clear_m = float(tail_clear_m)
        if max_missed_frames is not None:
            self.max_missed_frames = int(max_missed_frames)
        if alpha_ab is not None:
            self.alpha_ab = float(alpha_ab)
        if beta_ab is not None:
            self.beta_ab = float(beta_ab)
        if vel_decay is not None:
            self.vel_decay = float(vel_decay)
        if dyn_confirm_frames is not None:
            self.dyn_confirm_frames = int(dyn_confirm_frames)
        if roi_conical is not None:
            self.roi_conical = bool(roi_conical)
        if roi_y_min is not None:
            self.roi_y_min = float(roi_y_min)
        if roi_y_max is not None:
            self.roi_y_max = float(roi_y_max)
        if roi_half_w_near is not None:
            self.roi_half_w_near = float(roi_half_w_near)
        if roi_half_w_far is not None:
            self.roi_half_w_far = float(roi_half_w_far)

    # --------------------------------------------------------------------------
    # PIPELINE STEP 1: Occupied Binary Mask
    # --------------------------------------------------------------------------
    def prepare_binary_costmap(
        self, prob_map: np.ndarray, current_bounds: Optional[Tuple[float, float, float, float]] = None
    ) -> np.ndarray:
        """
        Extracts binary occupied map (prob >= threshold -> 1, free & unknown -> 0).
        Adapts dynamically if source costmap has different bounds or resolution.
        """
        # Distinguish confirmed occupied hits from unknown (0.50) and free (< 0.38)
        raw_occupied = (prob_map >= self.prob_threshold).astype(np.uint8)

        # If incoming map matches target shape exactly, return directly
        if raw_occupied.shape == self.shape:
            return raw_occupied

        # If map has different bounds or resolution, crop/interpolate
        target_binary = np.zeros(self.shape, dtype=np.uint8)
        if current_bounds:
            cb_xmin, cb_xmax, cb_ymin, cb_ymax = current_bounds
            c_res_x = (cb_xmax - cb_xmin) / max(1, prob_map.shape[1] - 1)
            c_res_y = (cb_ymax - cb_ymin) / max(1, prob_map.shape[0] - 1)

            # Map target (X, Y) coordinates into source grid indices
            col_start = int(round((self.x_min - cb_xmin) / c_res_x))
            col_end = col_start + self.width
            row_start = int(round((self.y_min - cb_ymin) / c_res_y))
            row_end = row_start + self.height

            c0_src = max(0, col_start)
            c1_src = min(prob_map.shape[1], col_end)
            r0_src = max(0, row_start)
            r1_src = min(prob_map.shape[0], row_end)

            c0_dst = max(0, -col_start)
            r0_dst = max(0, -row_start)
            w_copy = min(c1_src - c0_src, self.width - c0_dst)
            h_copy = min(r1_src - r0_src, self.height - r0_dst)

            if w_copy > 0 and h_copy > 0:
                target_binary[r0_dst:r0_dst + h_copy, c0_dst:c0_dst + w_copy] = raw_occupied[r0_src:r0_src + h_copy, c0_src:c0_src + w_copy]
        else:
            # Fallback resize with nearest-neighbor
            target_binary = cv2.resize(raw_occupied, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

        return target_binary

    # --------------------------------------------------------------------------
    # PIPELINE STEP 2: Blob Extraction & Geometric Wall/Noise Filtering
    # --------------------------------------------------------------------------
    def extract_obstacle_blobs(
        self, binary_mask: np.ndarray, yaw_rate: float = 0.0
    ) -> List[ObstacleBlob]:
        """
        Executes cv2.connectedComponentsWithStats directly on raw obstacle occupancy.
        Extracts genuine obstacle blobs and rejects walls using raw geometric dimensions
        and conical forward ROI without C-space dilation interference.
        """
        h, w = binary_mask.shape

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary_mask, connectivity=8
        )

        surviving: List[ObstacleBlob] = []

        for lbl in range(1, num_labels):
            left = int(stats[lbl, cv2.CC_STAT_LEFT])
            top = int(stats[lbl, cv2.CC_STAT_TOP])
            width = int(stats[lbl, cv2.CC_STAT_WIDTH])
            height = int(stats[lbl, cv2.CC_STAT_HEIGHT])
            area = int(stats[lbl, cv2.CC_STAT_AREA])
            u_c = float(centroids[lbl, 0])
            v_c = float(centroids[lbl, 1])

            # 1. Wall Rejection: Discard wide blobs touching lateral or far boundaries (track walls)
            touches_lateral = (left <= 1 or (left + width >= w - 1))
            touches_far = (top + height >= h - 1)
            if (touches_lateral and (width > 15 or height > 15)) or (touches_far and width > 22):
                continue

            # 2. Wall Rejection: Discard any blob with bounding box dimension > max_wall_dim_cells
            if max(width, height) > self.max_wall_dim_cells:
                continue

            # 3. Noise Floor: Discard tiny blobs with area < min_area_cells
            if area < self.min_area_cells:
                continue

            # Aspect-invariant centroid stabilization:
            # Blend center of mass (u_c, v_c) with geometric bounding box center.
            # This suppresses centroid-walking when viewing rectangular or cylindrical targets at oblique angles.
            u_geom = float(left + width * 0.5)
            v_geom = float(top + height * 0.5)
            u_stable = 0.65 * u_c + 0.35 * u_geom
            v_stable = 0.65 * v_c + 0.35 * v_geom

            # Convert surviving centroid from grid coordinates to robot-relative meters
            x_r = float((u_stable - self.u0) * self.res)
            y_r = float((v_stable - self.v0) * self.res)

            # 4. Conical Forward ROI & Corridor Boundary Filtering:
            if self.roi_conical:
                # Longitudinal check (ahead of front bumper out to horizon)
                if not (self.roi_y_min <= y_r <= self.roi_y_max):
                    continue
                # Conical lateral half-width: expands smoothly from bumper to horizon
                frac = float(np.clip((y_r - self.roi_y_min) / max(0.01, self.roi_y_max - self.roi_y_min), 0.0, 1.0))
                # Widen lateral aperture under high yaw rate so obstacles at turn apex are not clipped
                half_w = (self.roi_half_w_near + (self.roi_half_w_far - self.roi_half_w_near) * frac) + 0.08 * abs(yaw_rate)
                if abs(x_r) > half_w:
                    continue
            elif self.max_lateral_m is not None and abs(x_r) > self.max_lateral_m:
                continue

            surviving.append(
                ObstacleBlob(
                    blob_id=lbl,
                    u=u_stable,
                    v=v_stable,
                    x_r=x_r,
                    y_r=y_r,
                    area=area,
                    bbox=(left, top, width, height),
                )
            )

        self.last_surviving_blobs = surviving
        return surviving

    def extract_corridor_contacts(
        self,
        tof_readings: Optional[Any] = None,
        binary_mask: Optional[np.ndarray] = None,
        yaw_rate: float = 0.0,
    ) -> List[ObstacleBlob]:
        """
        Extracts obstacle contact points in the robot driving corridor.
        Fast-paths directly from 16-channel ToF rays if provided,
        falling back to costmap binary connected-components when rays are absent.
        """
        if tof_readings is not None and len(tof_readings) > 0:
            # Wall Rejection Mask: identify large connected structures (walls, track boundaries)
            # Walls are big objects that shouldn't even be checked as obstacle candidates
            wall_mask = None
            if binary_mask is not None:
                h_m, w_m = binary_mask.shape
                num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
                    binary_mask, connectivity=8
                )
                if num_labels > 1:
                    w_lbls = stats[1:, cv2.CC_STAT_WIDTH]
                    h_lbls = stats[1:, cv2.CC_STAT_HEIGHT]
                    area_lbls = stats[1:, cv2.CC_STAT_AREA]
                    left_lbls = stats[1:, cv2.CC_STAT_LEFT]
                    top_lbls = stats[1:, cv2.CC_STAT_TOP]

                    touches_lateral = (left_lbls <= 1) | (left_lbls + w_lbls >= w_m - 1)
                    touches_far = (top_lbls + h_lbls >= h_m - 1)
                    is_wall = (
                        (np.maximum(w_lbls, h_lbls) > self.max_wall_dim_cells)
                        | (touches_lateral & ((w_lbls > 15) | (h_lbls > 15)))
                        | (touches_far & (w_lbls > 22))
                        | (area_lbls > 200)
                    )
                    wall_label_ids = np.where(is_wall)[0] + 1
                    if len(wall_label_ids) > 0:
                        wall_mask = np.isin(labels, wall_label_ids)

            contacts = []
            for idx, (name, (x0, y0, heading)) in enumerate(zip(SENSOR_NAMES, SENSOR_CONFIG)):
                val = None
                if isinstance(tof_readings, dict):
                    val = tof_readings.get(name, tof_readings.get(f"tof_{idx}", None))
                elif isinstance(tof_readings, (list, tuple, np.ndarray)) and idx < len(tof_readings):
                    val = tof_readings[idx]

                if val is None or val < 0.05 or val >= 1.75:
                    continue

                xr = float(x0 + val * math.cos(heading))
                yr = float(y0 + val * math.sin(heading))

                # Longitudinal & Lateral Corridor Checks
                if not (self.roi_y_min <= yr <= self.roi_y_max):
                    continue

                frac = float(np.clip((yr - self.roi_y_min) / max(0.01, self.roi_y_max - self.roi_y_min), 0.0, 1.0))
                half_w = (self.roi_half_w_near + (self.roi_half_w_far - self.roi_half_w_near) * frac) + 0.08 * abs(yaw_rate)
                if abs(xr) > half_w:
                    continue

                u = float(self.u0 + xr / self.res)
                v = float(self.v0 + yr / self.res)
                iu = int(round(u))
                iv = int(round(v))

                # Wall Rejection: Discard contacts that belong to a large wall or track boundary
                if wall_mask is not None and 0 <= iu < w_m and 0 <= iv < h_m:
                    u_min = max(0, iu - 3)
                    u_max = min(w_m, iu + 4)
                    v_min = max(0, iv - 3)
                    v_max = min(h_m, iv + 4)
                    if wall_mask[v_min:v_max, u_min:u_max].any():
                        continue  # Wall is a big object that shouldn't even be checked

                contacts.append((xr, yr, u, v))

            if contacts:
                # Spatial cluster nearby ray hits within 0.18 m
                clusters: List[List[Tuple[float, float, float, float]]] = []
                for pt in contacts:
                    matched = False
                    for c in clusters:
                        cx = sum(p[0] for p in c) / len(c)
                        cy = sum(p[1] for p in c) / len(c)
                        if math.hypot(pt[0] - cx, pt[1] - cy) < 0.18:
                            c.append(pt)
                            matched = True
                            break
                    if not matched:
                        clusters.append([pt])

                blobs = []
                for cid, c in enumerate(clusters, start=1):
                    mx = float(sum(p[0] for p in c) / len(c))
                    my = float(sum(p[1] for p in c) / len(c))
                    mu = float(sum(p[2] for p in c) / len(c))
                    mv = float(sum(p[3] for p in c) / len(c))
                    blobs.append(
                        ObstacleBlob(
                            blob_id=cid,
                            u=mu,
                            v=mv,
                            x_r=mx,
                            y_r=my,
                            area=len(c),
                            bbox=(int(mu), int(mv), 1, 1),
                        )
                    )
                self.last_surviving_blobs = blobs
                return blobs

        # Fallback to morphological blob extractor
        if binary_mask is not None:
            return self.extract_obstacle_blobs(binary_mask, yaw_rate=yaw_rate)
        return []

    # --------------------------------------------------------------------------
    # PIPELINE STEP 3: World-Frame Projection
    # --------------------------------------------------------------------------
    def project_to_world_frame(
        self, blobs: List[ObstacleBlob], robot_pose: Tuple[float, float, float]
    ) -> List[Tuple[float, float, ObstacleBlob]]:
        """
        Transforms robot-relative coordinates (x_r, y_r) into global odometry
        coordinates (X_w, Y_w) using current robot pose (X, Y, theta):
          [X_w] = [X] + [cos(theta) -sin(theta)] [x_r]
          [Y_w]   [Y]   [sin(theta)  cos(theta)] [y_r]
        """
        X, Y, theta = robot_pose
        cos_th = math.cos(theta)
        sin_th = math.sin(theta)

        world_pts = []
        for b in blobs:
            X_w = float(X + cos_th * b.x_r - sin_th * b.y_r)
            Y_w = float(Y + sin_th * b.x_r + cos_th * b.y_r)
            world_pts.append((X_w, Y_w, b))

        return world_pts

    # --------------------------------------------------------------------------
    # PIPELINE STEP 4 & 5: 5-Frame Rolling Buffer Tracking & Velocity Gate
    # --------------------------------------------------------------------------
    def update_tracks(
        self,
        world_pts: List[Tuple[float, float, ObstacleBlob]],
        robot_pose: Tuple[float, float, float],
        now_s: float,
        v_ego: float = 0.0,
        yaw_rate: float = 0.0,
    ) -> List[ObstacleTrack]:
        """
        Maintains rolling buffer of world-frame centroids across consecutive frames.
        Performs distance-sorted nearest-neighbor association.
        Dual-Stage Gating:
          - Stage 1: Persistence requirement (default 3 consecutive association frames)
          - Stage 2: Dynamic confirmation hysteresis (default 2 confirmed frames)
        Velocity Calculation & Pure 1D Range-Rate Gate:
          Tracks classified strictly into APPROACHING, RECEDING, or STATIC.
          Threshold = dynamic_threshold_mps (0.40 m/s default).
        """
        self.frame_counter += 1
        curr_frame = self.frame_counter
        X, Y, theta = robot_pose
        cos_th = math.cos(theta)
        sin_th = math.sin(theta)

        # Track association matrix
        associated_tracks = set()
        associated_pts = set()

        # Nearest-neighbor matching with speed-adaptive gating radius for 2-5 m/s dynamics
        dynamic_gate_base = max(self.gating_radius_m, 0.28 + 0.04 * abs(v_ego) + 0.08 * abs(yaw_rate))

        # Distance-sorted nearest-neighbor association:
        # Prevent first-come-first-served bias by sorting all potential (track, blob) pairings by distance.
        candidate_pairs = []
        for track_id, track in list(self.tracks.items()):
            if not track.history:
                continue

            dt_pred = self.dt_frame
            if track.filter_initialized:
                pred_X = track.x_filt + track.v_x * dt_pred
                pred_Y = track.y_filt + track.v_y * dt_pred
            else:
                pred_X, pred_Y, _ = track.history[-1]

            for idx, (pw_x, pw_y, blob) in enumerate(world_pts):
                d = math.hypot(pw_x - pred_X, pw_y - pred_Y)
                if d <= dynamic_gate_base:
                    candidate_pairs.append((d, track_id, idx))

        candidate_pairs.sort(key=lambda x: x[0])

        for d, track_id, idx in candidate_pairs:
            if track_id in associated_tracks or idx in associated_pts:
                continue

            track = self.tracks[track_id]
            pw_x, pw_y, blob = world_pts[idx]
            dt_step = (now_s - track.history[-1][2]) if track.history else self.dt_frame
            if dt_step <= 1e-4:
                dt_step = self.dt_frame

            # 3-frame median pre-filter on longitudinal measurement (blob.y_r)
            # Avoid in-place mutation of blob.y_r to prevent coordinate desynchronization
            track.median_y_history.append(blob.y_r)
            med_yr = float(np.median(track.median_y_history))
            pw_x_filt = float(X + cos_th * blob.x_r - sin_th * med_yr)
            pw_y_filt = float(Y + sin_th * blob.x_r + cos_th * med_yr)

            track.history.append((pw_x_filt, pw_y_filt, now_s))
            track.body_history.append((blob.x_r, med_yr, now_s))
            track.consecutive_associations += 1
            track.last_frame_updated = curr_frame
            track.x_r = blob.x_r
            track.y_r = med_yr

            # Alpha-Beta filter state estimation
            if not track.filter_initialized:
                track.x_filt = pw_x_filt
                track.y_filt = pw_y_filt
                track.v_x = 0.0
                track.v_y = 0.0
                track.filter_initialized = True
            elif len(track.history) == 2:
                # Initialize velocity from the first 2-point difference to eliminate initial ramp-up lag
                dt_init = max(1e-4, track.history[1][2] - track.history[0][2])
                v_raw_x = (track.history[1][0] - track.history[0][0]) / dt_init
                v_raw_y = (track.history[1][1] - track.history[0][1]) / dt_init
                v_mag_raw = math.hypot(v_raw_x, v_raw_y)
                if v_mag_raw > 0.15:
                    scale = min(1.0, 3.0 / max(1e-4, v_mag_raw))
                    track.v_x = v_raw_x * scale
                    track.v_y = v_raw_y * scale
                track.x_filt = pw_x_filt
                track.y_filt = pw_y_filt
            else:
                curr_spd = math.hypot(track.v_x, track.v_y)
                if curr_spd < 0.30:
                    alpha_eff = self.alpha_ab
                    beta_eff = self.beta_ab
                    decay_cruise = self.vel_decay
                else:
                    alpha_eff = max(self.alpha_ab, 0.38)
                    beta_eff = max(self.beta_ab, 0.050)
                    decay_cruise = 0.998

                px = track.x_filt + track.v_x * dt_step
                py = track.y_filt + track.v_y * dt_step
                rx = pw_x_filt - px
                ry = pw_y_filt - py
                track.x_filt = px + alpha_eff * rx
                track.y_filt = py + alpha_eff * ry

                # Asymmetric Deceleration Quenching:
                # Detect if position innovation opposes velocity vector (obstacle is genuinely braking, not just noise jitter)
                inno_dot_v = track.v_x * rx + track.v_y * ry
                is_braking = (inno_dot_v < -0.03 * max(0.20, curr_spd))
                track.is_braking = is_braking

                if is_braking:
                    # Braking detected: accelerate decay towards zero to prevent deceleration lag & minimize overshoot
                    decay_eff = 0.940
                    b_apply = beta_eff * 1.20
                else:
                    # Cruising / accelerating: smooth responsive gain
                    decay_eff = decay_cruise
                    b_apply = beta_eff

                track.v_x = (track.v_x * decay_eff) + (b_apply / dt_step) * rx
                track.v_y = (track.v_y * decay_eff) + (b_apply / dt_step) * ry

                # Standstill soft-snap: quench residual velocity when braking near zero speed
                if math.hypot(track.v_x, track.v_y) < 0.25 and is_braking:
                    track.v_x *= 0.80
                    track.v_y *= 0.80

                # Dual-Window AMA buffer: record instantaneous post-update velocity sample
                track.speed_history.append((track.v_x, track.v_y, is_braking))

            associated_tracks.add(track_id)
            associated_pts.add(idx)

        # Handle unassociated candidate blobs -> spawn new tentative tracks
        for idx, (pw_x, pw_y, blob) in enumerate(world_pts):
            if idx not in associated_pts:
                new_track = ObstacleTrack(
                    track_id=self.next_track_id,
                    last_frame_updated=curr_frame,
                    consecutive_associations=1,
                    x_r=blob.x_r,
                    y_r=blob.y_r,
                    x_filt=pw_x,
                    y_filt=pw_y,
                    filter_initialized=True,
                )
                self.next_track_id += 1
                new_track.history.append((pw_x, pw_y, now_s))
                new_track.body_history.append((blob.x_r, blob.y_r, now_s))
                new_track.speed_history.append((0.0, 0.0, False))
                new_track.median_y_history.append(blob.y_r)
                self.tracks[new_track.track_id] = new_track

        # Prune dead tracks not observed for more than max_missed_frames (coasting tolerance)
        dead_tracks = []
        for track_id, track in self.tracks.items():
            if (curr_frame - track.last_frame_updated) > self.max_missed_frames:
                dead_tracks.append(track_id)

        for track_id in dead_tracks:
            del self.tracks[track_id]
            self.dyn_confirm_counts.pop(track_id, None)

        # Velocity Calculation & Dynamic/Static Gate
        dynamic_tracks: List[ObstacleTrack] = []
        confirmed_tracks: List[ObstacleTrack] = []

        for track in self.tracks.values():
            # Persistence Check: exactly 5 consecutive frames required
            if len(track.history) >= self.persistence_frames and track.consecutive_associations >= self.persistence_frames:
                confirmed_tracks.append(track)
                # Apply V3 Dual-Window AMA filtering on speed history:
                if len(track.speed_history) >= 2:
                    if track.is_braking:
                        # Braking: Truncate to latest 2 frames (70% newest, 30% previous)
                        v_ama_x = 0.70 * track.speed_history[-1][0] + 0.30 * track.speed_history[-2][0]
                        v_ama_y = 0.70 * track.speed_history[-1][1] + 0.30 * track.speed_history[-2][1]
                    else:
                        # Cruising/accelerating: Recency-weighted average over full window (up to 6 frames)
                        n = len(track.speed_history)
                        weights = list(range(1, n + 1))
                        w_sum = float(sum(weights))
                        v_ama_x = sum(w * pt[0] for w, pt in zip(weights, track.speed_history)) / w_sum
                        v_ama_y = sum(w * pt[1] for w, pt in zip(weights, track.speed_history)) / w_sum
                else:
                    v_ama_x = track.v_x
                    v_ama_y = track.v_y

                v_mag = math.hypot(v_ama_x, v_ama_y)
                track.v_mag = float(v_mag)

                # Transform global velocity vector into robot-relative coordinate frame
                # vrx =  cos(theta)*vx + sin(theta)*vy
                # vry = -sin(theta)*vx + cos(theta)*vy
                track.v_rx = float(cos_th * v_ama_x + sin_th * v_ama_y)
                track.v_ry = float(-sin_th * v_ama_x + cos_th * v_ama_y)

                # Kinematic body-frame ego-motion cross-check
                if len(track.body_history) >= 2:
                    oldest_xr, oldest_yr, oldest_tb = track.body_history[0]
                    latest_xr, latest_yr, latest_tb = track.body_history[-1]
                    dt_b = latest_tb - oldest_tb
                    if dt_b <= 1e-4:
                        dt_b = (len(track.body_history) - 1) * self.dt_frame
                    v_app_x = (latest_xr - oldest_xr) / dt_b
                    v_app_y = (latest_yr - oldest_yr) / dt_b
                    # Ego-motion compensation in body frame:
                    # v_apparent_static = (yaw_rate * y_r, -v_ego - yaw_rate * x_r)
                    # v_comp = v_apparent - v_apparent_static
                    track.v_comp_x = float(v_app_x - yaw_rate * track.y_r)
                    track.v_comp_y = float(v_app_y + v_ego + yaw_rate * track.x_r)
                else:
                    track.v_comp_x = 0.0
                    track.v_comp_y = 0.0

                v_comp_mag = math.hypot(track.v_comp_x, track.v_comp_y)

                # 1D Longitudinal Range-Rate & Consensus Classification (Strictly 1D: APPROACHING, RECEDING, STATIC)
                v_long = float(track.v_comp_y) if abs(track.v_comp_y) > 1e-4 else float(track.v_ry)
                track.v_longitudinal = v_long

                if v_long < -0.25:
                    current_dir = "APPROACHING"
                elif v_long > 0.25:
                    current_dir = "RECEDING"
                else:
                    current_dir = "STATIC"

                if current_dir != "STATIC":
                    if track.consensus_dir == current_dir:
                        track.consensus_count += 1
                    else:
                        track.consensus_dir = current_dir
                        track.consensus_count = 1
                else:
                    track.consensus_count = max(0, track.consensus_count - 1)
                    if track.consensus_count == 0:
                        track.consensus_dir = "STATIC"

                # Dynamic Confirmation Hysteresis: prevent single-frame noise spikes from triggering dynamic
                # Requires minimum persistence frames before incrementing confirmation counter
                tid = track.track_id
                c = self.dyn_confirm_counts.get(tid, 0)
                has_persistence = (len(track.history) >= self.persistence_frames and track.consecutive_associations >= self.persistence_frames)
                is_speed_dynamic = (v_mag >= self.dynamic_threshold_mps)

                if has_persistence and is_speed_dynamic and (len(track.body_history) < 2 or v_comp_mag >= (self.dynamic_threshold_mps * 0.70)):
                    c += 1
                else:
                    c = max(0, c - 2)
                self.dyn_confirm_counts[tid] = c

                # Static/Dynamic Gate requiring confirmation frames & consensus
                # Strictly 1D longitudinal: only APPROACHING or RECEDING tracks become dynamic
                if has_persistence and c >= self.dyn_confirm_frames and track.consensus_count >= 2 and track.consensus_dir in ("APPROACHING", "RECEDING"):
                    track.is_dynamic = True
                    track.state = track.consensus_dir
                    # Pure 1D Longitudinal Range-Rate constraint:
                    # Velocity vectors strictly point towards (-Y) or away (+Y) from the robot
                    if track.state == "APPROACHING":
                        v_1d = -abs(track.v_longitudinal)
                    else:  # RECEDING
                        v_1d = abs(track.v_longitudinal)

                    track.v_rx = 0.0
                    track.v_ry = float(v_1d)
                    track.v_mag = float(abs(v_1d))
                    track.v_comp_x = 0.0
                    track.v_comp_y = float(v_1d)
                    track.v_x = float(-sin_th * v_1d)
                    track.v_y = float(cos_th * v_1d)
                    dynamic_tracks.append(track)
                else:
                    track.is_dynamic = False
                    track.state = "STATIC"
            else:
                track.is_dynamic = False
                track.state = "STATIC"
                track.consensus_count = 0
                track.consensus_dir = "STATIC"
                track.v_x = 0.0
                track.v_y = 0.0
                track.v_mag = 0.0
                track.v_rx = 0.0
                track.v_ry = 0.0
                track.v_comp_x = 0.0
                track.v_comp_y = 0.0
                track.v_longitudinal = 0.0

        self.last_dynamic_tracks = dynamic_tracks
        self.last_confirmed_tracks = confirmed_tracks
        return dynamic_tracks

    # --------------------------------------------------------------------------
    # PIPELINE STEP 6: Conservative Moving Vector Enlargement
    # --------------------------------------------------------------------------
    def apply_conservative_enlargement(
        self,
        base_hazard_mask: np.ndarray,
        dynamic_tracks: Optional[List[ObstacleTrack]] = None,
        stamp_radius_m: float = 0.075,
        mask_bounds: Optional[Tuple[float, float, float, float]] = None,
    ) -> np.ndarray:
        """
        Enlarges confirmed dynamic obstacles towards their moving vector
        and slightly reduces the trailing wake where they came from.
        Conservative design:
          - Lookahead lead length = min(0.25m, v_mag * 0.20s).
          - Stamped capsule along velocity unit vector.
          - Trailing wake reduction: clears a small conservative 0.05m region behind obstacle.
        """
        enlarged_mask = base_hazard_mask.copy()
        tracks_to_enlarge = dynamic_tracks if dynamic_tracks is not None else self.last_dynamic_tracks

        if not tracks_to_enlarge:
            return enlarged_mask

        try:
            h, w = enlarged_mask.shape
            stamp_r_cells = max(1, int(round(stamp_radius_m / self.res)))
            tail_clear_cells = max(1, int(round(self.tail_clear_m / self.res)))

            if mask_bounds is not None:
                mb_xmin, mb_xmax, mb_ymin, mb_ymax = mask_bounds
                u0 = -mb_xmin / self.res
                v0 = -mb_ymin / self.res
            else:
                u0 = self.u0
                v0 = self.v0

            for track in tracks_to_enlarge:
                if not track.is_dynamic or track.v_mag < 1e-4:
                    continue
                if math.isnan(track.x_r) or math.isnan(track.y_r) or math.isnan(track.v_mag) or math.isnan(track.v_rx) or math.isnan(track.v_ry):
                    continue

                # Centroid in grid coordinates matching the mask coordinate system
                u_c = int(round(u0 + track.x_r / self.res))
                v_c = int(round(v0 + track.y_r / self.res))

                # Unit velocity vector in robot frame
                unit_vx = track.v_rx / track.v_mag
                unit_vy = track.v_ry / track.v_mag

                # Conservative lead length based on 1D classification state
                if track.state == "APPROACHING":
                    v_lead = abs(track.v_longitudinal) if abs(track.v_longitudinal) > 0.05 else track.v_mag
                    lead_m = min(self.max_lead_m, max(0.05, v_lead * self.lead_time_s))
                elif track.state == "RECEDING":
                    lead_m = min(self.max_lead_m, track.v_mag * (self.lead_time_s * 0.5))
                else:
                    lead_m = min(self.max_lead_m, track.v_mag * self.lead_time_s)

                # Braking-aware dynamic capsule collapse:
                if track.is_braking:
                    a_brake_assumed = 4.0  # m/s^2 assumed deceleration
                    stop_dist = (track.v_mag ** 2) / (2.0 * a_brake_assumed)
                    lead_m = min(lead_m, max(0.02, stop_dist))

                lead_cells = int(round(lead_m / self.res))

                # 1. Forward Enlargement Capsule (towards moving vector)
                # Replaces slow Python circle loops with a single cv2.line + circular end-caps
                lead_u = int(round(u_c + lead_cells * unit_vx))
                lead_v = int(round(v_c + lead_cells * unit_vy))
                cv2.line(enlarged_mask, (u_c, v_c), (lead_u, lead_v), color=1, thickness=stamp_r_cells * 2)
                cv2.circle(enlarged_mask, (u_c, v_c), stamp_r_cells, 1, -1)
                cv2.circle(enlarged_mask, (lead_u, lead_v), stamp_r_cells, 1, -1)

                # 2. Conservative Trailing Wake Reduction (where it came from)
                # Expand clearance when RECEDING so following vehicle can draft/overtake without phantom tail obstruction.
                # CRITICAL: Never clear cells that are occupied in base_hazard_mask (static walls/boundaries).
                extra_tail = tail_clear_cells if track.state == "RECEDING" else 0
                tail_dist_cells = stamp_r_cells + tail_clear_cells + extra_tail
                for step in range(stamp_r_cells + 1, tail_dist_cells + 1):
                    tu = int(round(u_c - step * unit_vx))
                    tv = int(round(v_c - step * unit_vy))
                    if 1 <= tu < w - 1 and 1 <= tv < h - 1:
                        if base_hazard_mask[tv, tu] == 0:
                            enlarged_mask[tv, tu] = 0

            return enlarged_mask
        except Exception:
            return base_hazard_mask

    # --------------------------------------------------------------------------
    # FULL PROCESS CYCLE (Drop-in for 60 Hz vehicle control loop)
    # --------------------------------------------------------------------------
    def process_frame(
        self,
        prob_map: np.ndarray,
        robot_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        now_s: float = 0.0,
        base_hazard_mask: Optional[np.ndarray] = None,
        source_grid_bounds: Optional[Tuple[float, float, float, float]] = None,
        v_ego: float = 0.0,
        yaw_rate: float = 0.0,
        tof_readings: Optional[Any] = None,
    ) -> Tuple[np.ndarray, List[ObstacleTrack], Dict]:
        """
        Executes complete dynamic obstacle detection and tracking pipeline.

        Returns:
            Tuple[enlarged_hazard_mask, dynamic_tracks, telemetry_info]
        """
        # Step 1: Binary occupied mask (Raw obstacle hits)
        binary_mask = self.prepare_binary_costmap(prob_map, current_bounds=source_grid_bounds)

        # Step 2: Extract candidate contacts in corridor (from ToF rays if available, else blob morphology)
        blobs = self.extract_corridor_contacts(
            tof_readings=tof_readings, binary_mask=binary_mask, yaw_rate=yaw_rate
        )

        # Step 3: World-frame projection
        world_pts = self.project_to_world_frame(blobs, robot_pose)

        # Step 4 & 5: Rolling buffer tracking & velocity gate (with ego-compensation & 1D range-rate)
        dynamic_tracks = self.update_tracks(
            world_pts=world_pts,
            robot_pose=robot_pose,
            now_s=now_s,
            v_ego=v_ego,
            yaw_rate=yaw_rate,
        )

        # Step 6: Conservative vector enlargement
        if base_hazard_mask is not None:
            active_hazard = base_hazard_mask
            mask_bounds = source_grid_bounds
        else:
            active_hazard = binary_mask
            mask_bounds = None

        enlarged_hazard = self.apply_conservative_enlargement(
            active_hazard, dynamic_tracks, mask_bounds=mask_bounds
        )

        telemetry = {
            "surviving_blobs": len(blobs),
            "active_tracks": len(self.tracks),
            "dynamic_obstacles": len(dynamic_tracks),
            "tracks_detail": [
                {
                    "id": t.track_id,
                    "x_r": round(t.x_r, 3),
                    "y_r": round(t.y_r, 3),
                    "v_mag": round(t.v_mag, 3),
                    "v_rx": round(t.v_rx, 3),
                    "v_ry": round(t.v_ry, 3),
                    "v_comp_x": round(t.v_comp_x, 3),
                    "v_comp_y": round(t.v_comp_y, 3),
                    "v_longitudinal": round(t.v_longitudinal, 3),
                    "state": t.state,
                    "consecutive": t.consecutive_associations,
                    "is_dynamic": t.is_dynamic,
                }
                for t in self.tracks.values()
            ],
        }

        return enlarged_hazard, dynamic_tracks, telemetry
