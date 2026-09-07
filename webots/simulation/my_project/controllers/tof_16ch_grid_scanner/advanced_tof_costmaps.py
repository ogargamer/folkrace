"""
================================================================================
  ADVANCED TOF COSTMAP ENGINES: CROSSOVER & UNCERTAINTY RESOLUTION
  
  Three Architectures:
  1. Global Two-Pass Arc Carving (ArcCarvingToFCostmap)
     - Fast deterministic boolean set subtraction: Candidate Hits & ~Free Space.
     - Post-carving C-space dilation.
     
  2. Bayesian Inverse Sensor Model with Angular Decay (BayesianAngularDecayToFCostmap)
     - Continuous angular Gaussian falloff: Peak +5.0 at beam axis, +1.5 at cone boundary.
     - -2.0 free-space log-odds cleanly subtracts edge hits while preserving cores.
     - Precomputed flat LUT weights for sub-millisecond execution.
     
  3. Cross-Beam Arc Triangulation (TriangulationToFCostmap)
     - Closed-form circle-circle intersection for cross-pointing sensor pairs.
     - Collapses 11.25 deg angular arcs into pinpoint geometric obstacle coordinates.
     - Combined with two-pass free-space carving and fallback for single hits.
================================================================================
"""

import math
import time
from typing import Dict, List, Optional, Tuple
import numpy as np

# Standard 16-Channel Sensor Rig
SENSOR_RIG = [
    # Front-Left Array (X=-0.070, Y=+0.095, cross-pointing Right)
    ("tof_fl_0", -0.070,  0.095, 0.9097),
    ("tof_fl_1", -0.070,  0.095, 1.1060),
    ("tof_fl_2", -0.070,  0.095, 1.3024),
    ("tof_fl_3", -0.070,  0.095, 1.4987),
    # Front-Right Array (X=+0.070, Y=+0.095, cross-pointing Left)
    ("tof_fr_0",  0.070,  0.095, 2.2318),
    ("tof_fr_1",  0.070,  0.095, 2.0356),
    ("tof_fr_2",  0.070,  0.095, 1.8391),
    ("tof_fr_3",  0.070,  0.095, 1.6428),
    # Back-Left Array (X=-0.070, Y=-0.095, fanning Left)
    ("tof_bl_0", -0.070, -0.095, 1.6957),
    ("tof_bl_1", -0.070, -0.095, 1.8920),
    ("tof_bl_2", -0.070, -0.095, 2.0883),
    ("tof_bl_3", -0.070, -0.095, 2.2847),
    # Back-Right Array (X=+0.070, Y=-0.095, fanning Right)
    ("tof_br_0",  0.070, -0.095, 1.4462),
    ("tof_br_1",  0.070, -0.095, 1.2499),
    ("tof_br_2",  0.070, -0.095, 1.0536),
    ("tof_br_3",  0.070, -0.095, 0.8573),
]


# ==============================================================================
# 1. ARCHITECTURE 1: GLOBAL TWO-PASS ARC CARVING
# ==============================================================================
class ArcCarvingToFCostmap:
    """
    Global Two-Pass Arc Carving:
    Pass 1: Accumulate candidate hit arcs (11.25 deg arcs) into candidate_hits mask.
    Pass 2: Accumulate free-space cones into free_space mask.
    Pass 3: Carve candidate hits: surviving = candidate_hits & ~free_space.
    Pass 4: Dilate only surviving obstacle hits by C-space inflation radius.
    """

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.0, 1.0, 0.0, 2.0),
        resolution: float = 0.02,
        range_step: float = 0.01,
        r_inflation_m: float = 0.05,
        fov_rad: float = 0.19634954,
        max_range: float = 1.80,
    ):
        self.x_min, self.x_max, self.y_min, self.y_max = grid_bounds
        self.res = resolution
        self.range_step = range_step
        self.r_inflation_m = r_inflation_m
        self.fov_rad = fov_rad
        self.max_range = max_range

        self.xs = np.arange(self.x_min, self.x_max + self.res / 2.0, self.res, dtype=np.float32)
        self.ys = np.arange(self.y_min, self.y_max + self.res / 2.0, self.res, dtype=np.float32)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)
        self.shape = self.X.shape
        self.num_cells = self.shape[0] * self.shape[1]

        self.num_buckets = int(self.max_range / self.range_step) + 1

        self.lut_free_flat: Dict[str, List[np.ndarray]] = {}
        self.lut_hit_flat: Dict[str, List[np.ndarray]] = {}
        self._build_lut()

        self._build_inflation_kernel()

        # Reusable fast 1D buffers
        self.candidate_hits = np.zeros(self.num_cells, dtype=bool)
        self.free_space = np.zeros(self.num_cells, dtype=bool)

    def _build_inflation_kernel(self):
        rad_cells = int(round(self.r_inflation_m / self.res))
        if rad_cells > 0:
            y, x = np.ogrid[-rad_cells:rad_cells + 1, -rad_cells:rad_cells + 1]
            self.inflation_kernel = (x * x + y * y <= rad_cells * rad_cells)
        else:
            self.inflation_kernel = None

    def _build_lut(self):
        for name, x0, y0, heading in SENSOR_RIG:
            dx = self.X - x0
            dy = self.Y - y0
            r = np.hypot(dx, dy)
            phi = np.arctan2(dy, dx)
            delta_theta = np.abs((phi - heading + np.pi) % (2 * np.pi) - np.pi)

            cone = (delta_theta <= (self.fov_rad / 2.0)) & (r >= 0.02) & (r <= self.max_range)

            self.lut_free_flat[name] = []
            self.lut_hit_flat[name] = []

            for b in range(self.num_buckets):
                z = b * self.range_step
                sigma_r = max(0.04 * z, self.res)

                if z >= (self.max_range - 1e-3):
                    free_idx = np.flatnonzero(cone).astype(np.int32)
                    hit_idx = np.array([], dtype=np.int32)
                else:
                    free_mask = cone & (r < (z - 0.5 * sigma_r))
                    free_idx = np.flatnonzero(free_mask).astype(np.int32)

                    hit_mask = cone & (r >= (z - 0.5 * sigma_r)) & (r <= (z + 1.2 * sigma_r))
                    hit_idx = np.flatnonzero(hit_mask).astype(np.int32)

                self.lut_free_flat[name].append(free_idx)
                self.lut_hit_flat[name].append(hit_idx)

    def build_costmap(self, readings: Dict[str, float], noise_pct: float = 0.0) -> np.ndarray:
        prob_map, _, _ = self.process_tof_frame(readings, noise_pct=noise_pct)
        return prob_map

    def process_tof_frame(
        self, readings: Dict[str, float], noise_pct: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        t0 = time.perf_counter()

        self.candidate_hits.fill(False)
        self.free_space.fill(False)

        inv_step = 1.0 / self.range_step
        max_b = self.num_buckets - 1

        # PASS 1: Accumulate all candidate hit arcs
        for name, _, _, _ in SENSOR_RIG:
            raw_z = readings.get(name, self.max_range)
            if noise_pct > 0.0 and raw_z < (self.max_range - 1e-3):
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.02, self.max_range))
            else:
                z = raw_z

            if z < (self.max_range - 1e-3):
                b = min(max_b, max(0, int(round(z * inv_step))))
                h_idx = self.lut_hit_flat[name][b]
                if len(h_idx) > 0:
                    self.candidate_hits[h_idx] = True

        # PASS 2: Accumulate all free-space cones
        for name, _, _, _ in SENSOR_RIG:
            raw_z = readings.get(name, self.max_range)
            if noise_pct > 0.0 and raw_z < (self.max_range - 1e-3):
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.02, self.max_range))
            else:
                z = raw_z

            b = min(max_b, max(0, int(round(z * inv_step))))
            f_idx = self.lut_free_flat[name][b]
            if len(f_idx) > 0:
                self.free_space[f_idx] = True

        # PASS 3: Arc Carving (Set Subtraction)
        surviving_hits = self.candidate_hits & (~self.free_space)

        # PASS 4: C-Space Expansion on surviving confirmed obstacle pixels
        surviving_2d = surviving_hits.reshape(self.shape)
        if self.inflation_kernel is not None:
            try:
                from scipy.ndimage import binary_dilation
                hazard_mask = binary_dilation(surviving_2d, structure=self.inflation_kernel)
            except Exception:
                hazard_mask = surviving_2d
        else:
            hazard_mask = surviving_2d

        # MPPI-Ready Posterior Probability Map [0.03 clear, 0.50 unknown, 0.95 occupied]
        prob_map = np.full(self.shape, 0.50, dtype=np.float32)
        prob_map[self.free_space.reshape(self.shape)] = 0.03
        prob_map[surviving_2d] = 0.95
        if self.inflation_kernel is not None:
            prob_map[hazard_mask & (~surviving_2d)] = 0.85

        exec_time_ms = (time.perf_counter() - t0) * 1000.0
        return prob_map, hazard_mask.astype(np.uint8), exec_time_ms


# ==============================================================================
# 2. ARCHITECTURE 2: BAYESIAN INVERSE SENSOR MODEL WITH ANGULAR DECAY
# ==============================================================================
class BayesianAngularDecayToFCostmap:
    """
    Bayesian Inverse Sensor Model with Angular Decay:
    Continuous angular falloff across the 11.25 deg cone:
    P(occ | z, theta) = f(r) * exp(- (theta - theta_center)^2 / (2 * sigma_theta^2))
    Center log-odds: +5.0
    Edge log-odds: +1.5
    Free space log-odds: -2.0
    All weights precomputed in LUT for sub-millisecond execution.
    """

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.0, 1.0, 0.0, 2.0),
        resolution: float = 0.02,
        range_step: float = 0.01,
        l_occ_center: float = 5.0,
        l_occ_edge: float = 1.5,
        l_free: float = -2.0,
        r_inflation_m: float = 0.05,
        fov_rad: float = 0.19634954,
        max_range: float = 1.80,
    ):
        self.x_min, self.x_max, self.y_min, self.y_max = grid_bounds
        self.res = resolution
        self.range_step = range_step
        self.l_occ_center = float(l_occ_center)
        self.l_occ_edge = float(l_occ_edge)
        self.l_free = float(l_free)
        self.r_inflation_m = r_inflation_m
        self.fov_rad = fov_rad
        self.max_range = max_range

        self.xs = np.arange(self.x_min, self.x_max + self.res / 2.0, self.res, dtype=np.float32)
        self.ys = np.arange(self.y_min, self.y_max + self.res / 2.0, self.res, dtype=np.float32)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)
        self.shape = self.X.shape
        self.num_cells = self.shape[0] * self.shape[1]

        self.num_buckets = int(self.max_range / self.range_step) + 1

        self.lut_free_flat: Dict[str, List[np.ndarray]] = {}
        # Stores tuples: (flat_indices, log_odds_weights)
        self.lut_hit_flat: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {}
        self._build_lut()

        self._build_inflation_kernel()
        self.log_odds_buffer = np.zeros(self.num_cells, dtype=np.float32)

    def _build_inflation_kernel(self):
        rad_cells = int(round(self.r_inflation_m / self.res))
        if rad_cells > 0:
            y, x = np.ogrid[-rad_cells:rad_cells + 1, -rad_cells:rad_cells + 1]
            self.inflation_kernel = (x * x + y * y <= rad_cells * rad_cells)
        else:
            self.inflation_kernel = None

    def _build_lut(self):
        sigma_theta = (self.fov_rad / 2.0) / 2.145966  # 50% decay at cone half-angle

        for name, x0, y0, heading in SENSOR_RIG:
            dx = self.X - x0
            dy = self.Y - y0
            r = np.hypot(dx, dy)
            phi = np.arctan2(dy, dx)
            delta_theta = np.abs((phi - heading + np.pi) % (2 * np.pi) - np.pi)

            cone = (delta_theta <= (self.fov_rad / 2.0)) & (r >= 0.02) & (r <= self.max_range)

            # Angular decay weight [0.0 to 1.0]
            angular_factor = np.exp(-0.5 * (delta_theta / sigma_theta) ** 2)
            log_odds_hit_grid = (
                self.l_occ_edge + (self.l_occ_center - self.l_occ_edge) * angular_factor
            ).astype(np.float32)

            self.lut_free_flat[name] = []
            self.lut_hit_flat[name] = []

            for b in range(self.num_buckets):
                z = b * self.range_step
                sigma_r = max(0.04 * z, self.res)

                if z >= (self.max_range - 1e-3):
                    free_idx = np.flatnonzero(cone).astype(np.int32)
                    self.lut_free_flat[name].append(free_idx)
                    self.lut_hit_flat[name].append((np.array([], dtype=np.int32), np.array([], dtype=np.float32)))
                else:
                    free_mask = cone & (r < (z - 0.5 * sigma_r))
                    free_idx = np.flatnonzero(free_mask).astype(np.int32)
                    self.lut_free_flat[name].append(free_idx)

                    hit_mask = cone & (r >= (z - 0.5 * sigma_r)) & (r <= (z + 1.2 * sigma_r))
                    hit_idx = np.flatnonzero(hit_mask).astype(np.int32)
                    hit_weights = log_odds_hit_grid.flat[hit_idx]
                    self.lut_hit_flat[name].append((hit_idx, hit_weights))

    def build_costmap(self, readings: Dict[str, float], noise_pct: float = 0.0) -> np.ndarray:
        prob_map, _, _ = self.process_tof_frame(readings, noise_pct=noise_pct)
        return prob_map

    def process_tof_frame(
        self, readings: Dict[str, float], noise_pct: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        t0 = time.perf_counter()

        self.log_odds_buffer.fill(0.0)

        inv_step = 1.0 / self.range_step
        max_b = self.num_buckets - 1

        # 1. Accumulate hit arcs with angular decay
        for name, _, _, _ in SENSOR_RIG:
            raw_z = readings.get(name, self.max_range)
            if noise_pct > 0.0 and raw_z < (self.max_range - 1e-3):
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.02, self.max_range))
            else:
                z = raw_z

            if z < (self.max_range - 1e-3):
                b = min(max_b, max(0, int(round(z * inv_step))))
                h_idx, h_weights = self.lut_hit_flat[name][b]
                if len(h_idx) > 0:
                    self.log_odds_buffer[h_idx] += h_weights

        # 2. Accumulate free-space cones (negative log-odds)
        for name, _, _, _ in SENSOR_RIG:
            raw_z = readings.get(name, self.max_range)
            if noise_pct > 0.0 and raw_z < (self.max_range - 1e-3):
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.02, self.max_range))
            else:
                z = raw_z

            b = min(max_b, max(0, int(round(z * inv_step))))
            f_idx = self.lut_free_flat[name][b]
            if len(f_idx) > 0:
                self.log_odds_buffer[f_idx] += self.l_free

        # Sigmoid posterior probability mapping
        prob_flat = 1.0 / (1.0 + np.exp(-np.clip(self.log_odds_buffer, -10.0, 10.0)))
        prob_map = prob_flat.reshape(self.shape)

        raw_hazard = prob_map > 0.55
        if self.inflation_kernel is not None:
            try:
                from scipy.ndimage import binary_dilation
                hazard_mask = binary_dilation(raw_hazard, structure=self.inflation_kernel)
            except Exception:
                hazard_mask = raw_hazard
        else:
            hazard_mask = raw_hazard

        exec_time_ms = (time.perf_counter() - t0) * 1000.0
        return prob_map, hazard_mask.astype(np.uint8), exec_time_ms


# ==============================================================================
# 3. ARCHITECTURE 3: CROSS-BEAM ARC TRIANGULATION (GEOMETRIC PEAK FUSION)
# ==============================================================================
class TriangulationToFCostmap:
    """
    Cross-Beam Arc Triangulation:
    For sensor pairs with intersecting circular arcs (e.g. cross-pointing front arrays),
    analytically calculates the circle-circle intersection points.
    Validates points inside both sensors' FOV cones to collapse the 11.25 deg uncertainty
    arc into high-confidence pinpoint coordinates (x_obs, y_obs).
    Sensors without crossing intersections fall back to carved arcs.
    Free cones carve out clear corridors.
    """

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.0, 1.0, 0.0, 2.0),
        resolution: float = 0.02,
        range_step: float = 0.01,
        r_inflation_m: float = 0.05,
        fov_rad: float = 0.19634954,
        max_range: float = 1.80,
    ):
        self.x_min, self.x_max, self.y_min, self.y_max = grid_bounds
        self.res = resolution
        self.range_step = range_step
        self.r_inflation_m = r_inflation_m
        self.fov_rad = fov_rad
        self.max_range = max_range

        self.xs = np.arange(self.x_min, self.x_max + self.res / 2.0, self.res, dtype=np.float32)
        self.ys = np.arange(self.y_min, self.y_max + self.res / 2.0, self.res, dtype=np.float32)
        self.X, self.Y = np.meshgrid(self.xs, self.ys)
        self.shape = self.X.shape
        self.num_cells = self.shape[0] * self.shape[1]

        self.num_buckets = int(self.max_range / self.range_step) + 1

        # Sensor lookup dictionary
        self.sensors = {s[0]: s for s in SENSOR_RIG}

        # Candidate cross-beam pairs (Front-Left cross-pointing with Front-Right)
        self.cross_pairs = []
        fl_names = ["tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3"]
        fr_names = ["tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3"]
        for s1 in fl_names:
            for s2 in fr_names:
                self.cross_pairs.append((s1, s2))

        # Precompute flat LUT for free cones and baseline hit arcs
        self.lut_free_flat: Dict[str, List[np.ndarray]] = {}
        self.lut_hit_flat: Dict[str, List[np.ndarray]] = {}
        self._build_lut()

        self._build_inflation_kernel()

        # Precompute small stamp kernel for pinpoint triangulation spots (~3cm radius)
        stamp_rad = max(1, int(round(0.03 / self.res)))
        sy, sx = np.ogrid[-stamp_rad:stamp_rad + 1, -stamp_rad:stamp_rad + 1]
        self.triangulation_stamp = (sx * sx + sy * sy <= stamp_rad * stamp_rad)
        self.stamp_rad = stamp_rad

        self.candidate_hits = np.zeros(self.num_cells, dtype=bool)
        self.free_space = np.zeros(self.num_cells, dtype=bool)

    def _build_inflation_kernel(self):
        rad_cells = int(round(self.r_inflation_m / self.res))
        if rad_cells > 0:
            y, x = np.ogrid[-rad_cells:rad_cells + 1, -rad_cells:rad_cells + 1]
            self.inflation_kernel = (x * x + y * y <= rad_cells * rad_cells)
        else:
            self.inflation_kernel = None

    def _build_lut(self):
        for name, x0, y0, heading in SENSOR_RIG:
            dx = self.X - x0
            dy = self.Y - y0
            r = np.hypot(dx, dy)
            phi = np.arctan2(dy, dx)
            delta_theta = np.abs((phi - heading + np.pi) % (2 * np.pi) - np.pi)

            cone = (delta_theta <= (self.fov_rad / 2.0)) & (r >= 0.02) & (r <= self.max_range)

            self.lut_free_flat[name] = []
            self.lut_hit_flat[name] = []

            for b in range(self.num_buckets):
                z = b * self.range_step
                sigma_r = max(0.04 * z, self.res)

                if z >= (self.max_range - 1e-3):
                    free_idx = np.flatnonzero(cone).astype(np.int32)
                    hit_idx = np.array([], dtype=np.int32)
                else:
                    free_mask = cone & (r < (z - 0.5 * sigma_r))
                    free_idx = np.flatnonzero(free_mask).astype(np.int32)

                    hit_mask = cone & (r >= (z - 0.5 * sigma_r)) & (r <= (z + 1.2 * sigma_r))
                    hit_idx = np.flatnonzero(hit_mask).astype(np.int32)

                self.lut_free_flat[name].append(free_idx)
                self.lut_hit_flat[name].append(hit_idx)

    def _intersect_circles(
        self, s1_data, r1: float, s2_data, r2: float
    ) -> Optional[Tuple[float, float]]:
        """Closed-form circle-circle intersection filtered by sensor FOV cone."""
        _, x1, y1, head1 = s1_data
        _, x2, y2, head2 = s2_data

        dx = x2 - x1
        dy = y2 - y1
        d = math.hypot(dx, dy)

        if d > (r1 + r2) or d < abs(r1 - r2) or d < 1e-6:
            return None

        a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
        h_sq = r1 * r1 - a * a
        if h_sq < 0.0:
            return None
        h = math.sqrt(h_sq)

        # Midpoint on line of centers
        xm = x1 + a * (dx / d)
        ym = y1 + a * (dy / d)

        # Orthogonal components
        rx = -dy * (h / d)
        ry = dx * (h / d)

        half_fov = self.fov_rad / 2.0 + 0.03  # slight tolerance

        # Test both candidate intersections
        candidates = [(xm + rx, ym + ry), (xm - rx, ym - ry)]
        for px, py in candidates:
            # Check in s1 FOV
            th1 = math.atan2(py - y1, px - x1)
            dth1 = abs((th1 - head1 + math.pi) % (2.0 * math.pi) - math.pi)
            if dth1 > half_fov:
                continue

            # Check in s2 FOV
            th2 = math.atan2(py - y2, px - x2)
            dth2 = abs((th2 - head2 + math.pi) % (2.0 * math.pi) - math.pi)
            if dth2 <= half_fov:
                return (px, py)

        return None

    def build_costmap(self, readings: Dict[str, float], noise_pct: float = 0.0) -> np.ndarray:
        prob_map, _, _ = self.process_tof_frame(readings, noise_pct=noise_pct)
        return prob_map

    def process_tof_frame(
        self, readings: Dict[str, float], noise_pct: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        t0 = time.perf_counter()

        self.candidate_hits.fill(False)
        self.free_space.fill(False)

        inv_step = 1.0 / self.range_step
        max_b = self.num_buckets - 1

        active_ranges: Dict[str, float] = {}
        for name, _, _, _ in SENSOR_RIG:
            raw_z = readings.get(name, self.max_range)
            if noise_pct > 0.0 and raw_z < (self.max_range - 1e-3):
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.02, self.max_range))
            else:
                z = raw_z

            if z < (self.max_range - 1e-3):
                active_ranges[name] = z

        # STEP 1: Cross-Beam Triangulation
        triangulated_points: List[Tuple[float, float]] = []
        triangulated_sensors = set()

        for s1_name, s2_name in self.cross_pairs:
            if s1_name in active_ranges and s2_name in active_ranges:
                r1 = active_ranges[s1_name]
                r2 = active_ranges[s2_name]
                pt = self._intersect_circles(
                    self.sensors[s1_name], r1, self.sensors[s2_name], r2
                )
                if pt is not None:
                    triangulated_points.append(pt)
                    triangulated_sensors.add(s1_name)
                    triangulated_sensors.add(s2_name)

        # STEP 2: Candidate Hits
        # For non-triangulated active sensors, place their candidate arc
        for name, z in active_ranges.items():
            if name not in triangulated_sensors:
                b = min(max_b, max(0, int(round(z * inv_step))))
                h_idx = self.lut_hit_flat[name][b]
                if len(h_idx) > 0:
                    self.candidate_hits[h_idx] = True

        # Stamp pinpoint spot for triangulated points
        H, W = self.shape
        cand_2d = self.candidate_hits.reshape(self.shape)
        for px, py in triangulated_points:
            col = int(round((px - self.x_min) / self.res))
            row = int(round((py - self.y_min) / self.res))
            if 0 <= col < W and 0 <= row < H:
                r_min = max(0, row - self.stamp_rad)
                r_max = min(H, row + self.stamp_rad + 1)
                c_min = max(0, col - self.stamp_rad)
                c_max = min(W, col + self.stamp_rad + 1)
                cand_2d[r_min:r_max, c_min:c_max] = True

        # STEP 3: Accumulate Free Space Cones
        for name, _, _, _ in SENSOR_RIG:
            z = active_ranges.get(name, self.max_range)
            b = min(max_b, max(0, int(round(z * inv_step))))
            f_idx = self.lut_free_flat[name][b]
            if len(f_idx) > 0:
                self.free_space[f_idx] = True

        # STEP 4: Set Subtraction (Carve)
        surviving_hits = self.candidate_hits & (~self.free_space)

        # STEP 5: C-Space Expansion
        surviving_2d = surviving_hits.reshape(self.shape)
        if self.inflation_kernel is not None:
            try:
                from scipy.ndimage import binary_dilation
                hazard_mask = binary_dilation(surviving_2d, structure=self.inflation_kernel)
            except Exception:
                hazard_mask = surviving_2d
        else:
            hazard_mask = surviving_2d

        # MPPI Posterior Probability Map
        prob_map = np.full(self.shape, 0.50, dtype=np.float32)
        prob_map[self.free_space.reshape(self.shape)] = 0.03
        prob_map[surviving_2d] = 0.95
        if self.inflation_kernel is not None:
            prob_map[hazard_mask & (~surviving_2d)] = 0.85

        exec_time_ms = (time.perf_counter() - t0) * 1000.0
        return prob_map, hazard_mask.astype(np.uint8), exec_time_ms


# ==============================================================================
# 4. ARCHITECTURE 4: SMART HYBRID (TRIANGULATION + BAYESIAN ANGULAR DECAY)
# ==============================================================================
class HybridSmartToFCostmap(BayesianAngularDecayToFCostmap):
    """
    Smart Hybrid Costmap:
    Combines:
    1. Cross-Beam Arc Triangulation: Collapses 11.25 deg angular smearing to pinpoint
       coordinates (x_obs, y_obs) when crossing sensor pairs both observe an obstacle.
    2. Bayesian Angular Decay: Soft continuous log-odds update with Gaussian falloff
       across the 11.25 deg cone. Protects obstacle cores (+5.0) from noise-induced
       erasure while automatically clearing false lateral wings (+1.5 - 2.0 = -0.5).
    3. C-space dilation applied to confirmed hazard boundaries.
    """

    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.0, 1.0, 0.0, 2.0),
        resolution: float = 0.02,
        range_step: float = 0.01,
        l_occ_center: float = 5.0,
        l_occ_edge: float = 1.5,
        l_free: float = -2.0,
        r_inflation_m: float = 0.05,
        fov_rad: float = 0.19634954,
        max_range: float = 1.80,
    ):
        super().__init__(
            grid_bounds=grid_bounds,
            resolution=resolution,
            range_step=range_step,
            l_occ_center=l_occ_center,
            l_occ_edge=l_occ_edge,
            l_free=l_free,
            r_inflation_m=r_inflation_m,
            fov_rad=fov_rad,
            max_range=max_range,
        )
        self.triangulator = TriangulationToFCostmap(
            grid_bounds=grid_bounds,
            resolution=resolution,
            range_step=range_step,
            r_inflation_m=r_inflation_m,
            fov_rad=fov_rad,
            max_range=max_range,
        )
        self.stamp_rad = max(1, int(round(0.03 / self.res)))

    def process_tof_frame(
        self, readings: Dict[str, float], noise_pct: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        t0 = time.perf_counter()

        # 1. Continuous Bayesian Angular Decay map
        prob_map, raw_hazard, _ = super().process_tof_frame(readings, noise_pct=noise_pct)

        # 2. Geometric Peak Triangulation for crossing beams
        active: Dict[str, float] = {}
        for name, _, _, _ in SENSOR_RIG:
            raw_z = readings.get(name, self.max_range)
            if noise_pct > 0.0 and raw_z < (self.max_range - 1e-3):
                noise = np.random.uniform(-noise_pct, noise_pct) * raw_z
                z = float(np.clip(raw_z + noise, 0.02, self.max_range))
            else:
                z = raw_z
            if z < (self.max_range - 1e-3):
                active[name] = z

        H, W = self.shape
        for s1, s2 in self.triangulator.cross_pairs:
            if s1 in active and s2 in active:
                pt = self.triangulator._intersect_circles(
                    self.triangulator.sensors[s1],
                    active[s1],
                    self.triangulator.sensors[s2],
                    active[s2],
                )
                if pt is not None:
                    col = int(round((pt[0] - self.x_min) / self.res))
                    row = int(round((pt[1] - self.y_min) / self.res))
                    if 0 <= col < W and 0 <= row < H:
                        r0 = max(0, row - self.stamp_rad)
                        r1 = min(H, row + self.stamp_rad + 1)
                        c0 = max(0, col - self.stamp_rad)
                        c1 = min(W, col + self.stamp_rad + 1)
                        prob_map[r0:r1, c0:c1] = 0.99

        raw_hazard = prob_map > 0.55
        if self.inflation_kernel is not None:
            try:
                from scipy.ndimage import binary_dilation
                hazard_mask = binary_dilation(raw_hazard, structure=self.inflation_kernel)
            except Exception:
                hazard_mask = raw_hazard
        else:
            hazard_mask = raw_hazard

        exec_time_ms = (time.perf_counter() - t0) * 1000.0
        return prob_map, hazard_mask.astype(np.uint8), exec_time_ms

