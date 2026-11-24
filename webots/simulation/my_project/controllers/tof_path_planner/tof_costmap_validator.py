import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Literal, Optional, Tuple

import numpy as np

# ==============================================================================
# ==============================================================================
# SENSOR RIG DEFINITIONS (16-Channel Front-Only Rig from only_front_tofs.proto)
# Robot Body Frame: +Y is Forward, +X is Right, -X is Left
# ==============================================================================
@dataclass(frozen=True)
class ToFSensor:
    name: str
    x0: float
    y0: float
    heading_rad: float
    fov_rad: float = 0.19634954  # 11.25 degrees
    min_range: float = 0.001
    max_range: float = 2.40

SENSOR_RIG = [
    # Front-Left Array (cross-pointing Right, rotated 3 deg outwards)
    # Leftmost ToF (pointing straight and right, zones 1.1 deg to 34.9 deg)
    ToFSensor("tof_front_left_4.1deg",  -0.058, 0.092, 1.5512),
    ToFSensor("tof_front_left_15.4deg", -0.058, 0.092, 1.3548),
    ToFSensor("tof_front_left_26.6deg", -0.058, 0.092, 1.1585),
    ToFSensor("tof_front_left_37.9deg", -0.058, 0.092, 0.9621),
    # Left ToF (pointing right, zones 46.1 deg to 79.9 deg)
    ToFSensor("tof_front_left_49.1deg", -0.057, 0.090, 0.7658),
    ToFSensor("tof_front_left_60.4deg", -0.057, 0.090, 0.5694),
    ToFSensor("tof_front_left_71.6deg", -0.057, 0.090, 0.3731),
    ToFSensor("tof_front_left_82.9deg", -0.057, 0.090, 0.1767),
    # Front-Right Array (cross-pointing Left, mirrored along Y axis, rotated 3 deg outwards)
    # Rightmost ToF (pointing straight and left, zones 1.1 deg to 34.9 deg)
    ToFSensor("tof_front_right_4.1deg",   0.058, 0.092, 1.5904),
    ToFSensor("tof_front_right_15.4deg",  0.058, 0.092, 1.7868),
    ToFSensor("tof_front_right_26.6deg",  0.058, 0.092, 1.9831),
    ToFSensor("tof_front_right_37.9deg",  0.058, 0.092, 2.1795),
    # Right ToF (pointing left, zones 46.1 deg to 79.9 deg)
    ToFSensor("tof_front_right_49.1deg",  0.057, 0.090, 2.3758),
    ToFSensor("tof_front_right_60.4deg",  0.057, 0.090, 2.5722),
    ToFSensor("tof_front_right_71.6deg",  0.057, 0.090, 2.7685),
    ToFSensor("tof_front_right_82.9deg",  0.057, 0.090, 2.9649),
]


# ==============================================================================
# COSTMAP GENERATOR (Wrapper over C++ DLL or Python Implementation)
# ==============================================================================
class CppCostmapAdapter:
    def __init__(
        self,
        grid_bounds: Tuple[float, float, float, float] = (-1.25, 1.25, 0.0, 2.60),
        resolution: float = 0.015,
        r_inflation_m: float = 0.075,
    ):
        from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
        self.engine = CppSmartHybridCostmap(
            grid_bounds=grid_bounds,
            resolution=resolution,
            r_inflation_m=r_inflation_m,
        )
        self.X = self.engine.X
        self.Y = self.engine.Y
        self.res = resolution

    def build_costmap(self, readings: Dict[str, float], noise_pct: float = 0.0) -> np.ndarray:
        prob_map, _, _ = self.engine.process_tof_frame(readings, noise_pct=noise_pct)
        return prob_map


# ==============================================================================
# GEOMETRY HELPERS FOR PILLARS (CYLINDERS) & CUBES (BOXES)
# ==============================================================================
def compute_surface_error(
    x_est: float,
    y_est: float,
    x_gt: float,
    y_gt: float,
    shape_type: Literal["pillar", "cylinder", "cube", "box"],
    half_width_m: float,
) -> float:
    """Computes the Euclidean deviation from estimated centroid to object boundary."""
    dx = abs(x_est - x_gt)
    dy = abs(y_est - y_gt)

    if shape_type in ("pillar", "cylinder"):
        # Cylinder: distance from center to surface is constant radius R
        dist_to_center = math.hypot(x_est - x_gt, y_est - y_gt)
        return abs(dist_to_center - half_width_m)
    else:
        # Cube / Box (aligned): shortest distance to box boundary perimeter
        if dx >= half_width_m and dy >= half_width_m:
            # Diagonal corner region
            return math.hypot(dx - half_width_m, dy - half_width_m)
        elif dx >= half_width_m:
            # Lateral X edge
            return dx - half_width_m
        elif dy >= half_width_m:
            # Lateral Y edge
            return dy - half_width_m
        else:
            # Inside the box: distance to closest boundary edge
            return min(half_width_m - dx, half_width_m - dy)


def check_containment(
    prob_map: np.ndarray,
    X: np.ndarray,
    Y: np.ndarray,
    x_gt: float,
    y_gt: float,
    shape_type: Literal["pillar", "cylinder", "cube", "box"],
    half_width_m: float,
    grid_res: float,
    threshold: float = 0.55,
) -> bool:
    """Verifies if high-probability cells exist inside the physical bounding shape."""
    if shape_type in ("pillar", "cylinder"):
        dist = np.hypot(X - x_gt, Y - y_gt)
        mask = dist <= (half_width_m + grid_res)
    else:
        dx = np.abs(X - x_gt)
        dy = np.abs(Y - y_gt)
        mask = (dx <= (half_width_m + grid_res)) & (dy <= (half_width_m + grid_res))

    return bool(np.any(prob_map[mask] > threshold))


# ==============================================================================
# ANALYTICAL RAY-TRACER (Webots 16-Channel Sensor Rig Matching)
# ==============================================================================
def _intersect_ray_cylinder(
    x0: float, y0: float, angle: float, cx: float, cy: float, radius: float
) -> float:
    dx = math.cos(angle)
    dy = math.sin(angle)
    fx = x0 - cx
    fy = y0 - cy
    a = dx * dx + dy * dy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return 2.40
    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)
    hits = [t for t in (t1, t2) if 0.02 <= t <= 2.40]
    return min(hits) if hits else 2.40


def _intersect_ray_box(
    x0: float, y0: float, angle: float, cx: float, cy: float, half_w: float
) -> float:
    dx = math.cos(angle)
    dy = math.sin(angle)
    x_min, x_max = cx - half_w, cx + half_w
    y_min, y_max = cy - half_w, cy + half_w

    t_min, t_max = 0.02, 2.40
    if abs(dx) > 1e-6:
        tx1 = (x_min - x0) / dx
        tx2 = (x_max - x0) / dx
        t_min = max(t_min, min(tx1, tx2))
        t_max = min(t_max, max(tx1, tx2))
    else:
        if x0 < x_min or x0 > x_max:
            return 2.40

    if abs(dy) > 1e-6:
        ty1 = (y_min - y0) / dy
        ty2 = (y_max - y0) / dy
        t_min = max(t_min, min(ty1, ty2))
        t_max = min(t_max, max(ty1, ty2))
    else:
        if y0 < y_min or y0 > y_max:
            return 2.40

    if t_min <= t_max and t_max >= 0.02:
        return max(t_min, 0.02)
    return 2.40


def generate_simulated_dataset(
    output_csv: str,
    shape_type: Literal["pillar", "cube"],
    size_m: float,
    grid_step_m: Optional[float] = None,
    x_range: Tuple[float, float] = (-0.925, 0.925),
    y_range: Tuple[float, float] = (0.150, 2.400),
    num_sub_rays: int = 10,
):
    """Generates synthetic ToF range readings with exact Webots optical characteristics."""
    if grid_step_m is None:
        grid_step_m = size_m / 2.0  # Grid size = half width of object

    half_w = size_m / 2.0
    x_points = np.arange(x_range[0], x_range[1] + grid_step_m / 2.0, grid_step_m)
    y_points = np.arange(y_range[0], y_range[1] + grid_step_m / 2.0, grid_step_m)

    sensor_names = [s.name for s in SENSOR_RIG]
    records = []

    for x in x_points:
        for y in y_points:
            row = {"x_m": round(float(x), 4), "y_m": round(float(y), 4)}
            for s in SENSOR_RIG:
                sub_angles = np.linspace(
                    s.heading_rad - s.fov_rad / 2.0,
                    s.heading_rad + s.fov_rad / 2.0,
                    num_sub_rays,
                )
                if shape_type == "pillar":
                    ray_dists = [
                        _intersect_ray_cylinder(s.x0, s.y0, a, float(x), float(y), half_w)
                        for a in sub_angles
                    ]
                else:
                    ray_dists = [
                        _intersect_ray_box(s.x0, s.y0, a, float(x), float(y), half_w)
                        for a in sub_angles
                    ]
                min_dist = min(ray_dists)
                row[s.name] = round(float(min_dist), 4)
            records.append(row)

    with open(output_csv, mode="w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["x_m", "y_m", *sensor_names])
        writer.writeheader()
        writer.writerows(records)

    return output_csv


# ==============================================================================
# FACT-CHECKING ENGINE
# ==============================================================================
def fact_check_dataset(
    csv_input: str,
    csv_output: str,
    shape_type: Literal["pillar", "cylinder", "cube", "box"] = "pillar",
    size_m: float = 0.05,
    grid_resolution_m: Optional[float] = None,
    noise_pct: float = 0.05,
    sigma_front: float = 0.5,
    sigma_back: float = 1.5,
) -> Dict[str, float]:
    """Evaluates ToF costmap against ground-truth coordinates with radius/box compensation and overextension checks."""
    half_w = size_m / 2.0
    res = grid_resolution_m if grid_resolution_m is not None else (size_m / 2.0)

    engine = CppCostmapAdapter(grid_bounds=(-1.25, 1.25, 0.0, 2.0), resolution=0.015, r_inflation_m=0.075)

    with open(csv_input, mode="r") as f:
        records = list(csv.DictReader(f))

    sensor_keys = [s.name for s in SENSOR_RIG]
    results = []
    errors = []
    contained_count = 0
    total_detected = 0

    close_overexts = []
    med_overexts = []
    far_overexts = []

    for row in records:
        x_gt = float(row["x_m"])
        y_gt = float(row["y_m"])

        readings = {k: float(row[k]) for k in sensor_keys if k in row}
        active_sensors = [k for k, v in readings.items() if v < 2.39]

        if not active_sensors:
            continue

        total_detected += 1
        prob_map = engine.build_costmap(readings, noise_pct=noise_pct)

        # Filter strictly positive evidence (> 0.55 / log-odds > 0.2)
        hit_cells = prob_map > 0.55
        if not np.any(hit_cells):
            results.append({
                "x_gt_m": x_gt,
                "y_gt_m": y_gt,
                "x_est_surf_m": "NaN",
                "y_est_surf_m": "NaN",
                "surface_error_m": "NaN",
                "front_overext_m": "NaN",
                "range_band": "NaN",
                "active_sensors": len(active_sensors),
                "pillar_contained": False,
                "status": "FALSE_NEGATIVE",
            })
            continue

        # Compute estimated surface centroid
        weights = prob_map[hit_cells]
        x_est_surf = float(np.sum(engine.X[hit_cells] * weights) / np.sum(weights))
        y_est_surf = float(np.sum(engine.Y[hit_cells] * weights) / np.sum(weights))

        # Geometric boundary deviation error
        surface_error = compute_surface_error(
            x_est_surf, y_est_surf, x_gt, y_gt, shape_type, half_w
        )
        errors.append(surface_error)

        # Frontal Overextension towards robot origin (0, 0)
        r_hit_cells = np.hypot(engine.X[hit_cells], engine.Y[hit_cells])
        min_r_occ = float(np.min(r_hit_cells))
        r_gt_center = math.hypot(x_gt, y_gt)
        r_true_front = max(0.0, r_gt_center - half_w)
        front_overextension = max(0.0, r_true_front - min_r_occ)

        # Range categorization
        if r_gt_center < 0.6:
            range_band = "CLOSE"
            close_overexts.append(front_overextension)
        elif r_gt_center < 1.2:
            range_band = "MEDIUM"
            med_overexts.append(front_overextension)
        else:
            range_band = "FAR"
            far_overexts.append(front_overextension)

        # Bounding container overlap test
        is_contained = check_containment(
            prob_map, engine.X, engine.Y, x_gt, y_gt, shape_type, half_w, engine.res
        )
        if is_contained:
            contained_count += 1

        results.append({
            "x_gt_m": round(x_gt, 4),
            "y_gt_m": round(y_gt, 4),
            "x_est_surf_m": round(x_est_surf, 4),
            "y_est_surf_m": round(y_est_surf, 4),
            "surface_error_m": round(surface_error, 4),
            "front_overext_m": round(front_overextension, 4),
            "range_band": range_band,
            "active_sensors": len(active_sensors),
            "pillar_contained": is_contained,
            "status": "OK" if surface_error <= max(0.05, res) else "HIGH_ERROR",
        })

    with open(csv_output, mode="w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    stats = {
        "total_positions": len(records),
        "total_detected": total_detected,
        "containment_rate": (contained_count / total_detected * 100.0) if total_detected else 0.0,
        "mean_error_cm": float(np.mean(errors) * 100.0) if errors else 0.0,
        "median_error_cm": float(np.median(errors) * 100.0) if errors else 0.0,
        "close_overext_mean_cm": float(np.mean(close_overexts) * 100.0) if close_overexts else 0.0,
        "close_overext_max_cm": float(np.max(close_overexts) * 100.0) if close_overexts else 0.0,
        "med_overext_mean_cm": float(np.mean(med_overexts) * 100.0) if med_overexts else 0.0,
        "far_overext_mean_cm": float(np.mean(far_overexts) * 100.0) if far_overexts else 0.0,
        "ok_rate": (sum(1 for e in errors if e <= max(0.05, res)) / len(errors) * 100.0) if errors else 0.0,
    }

    return stats


# ==============================================================================
# BENCHMARK SUITE: 5, 10, 15, 20 cm Pillars & Cubes
# ==============================================================================
def run_all_benchmarks():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    datasets_dir = os.path.join(script_dir, "data", "datasets")
    reports_dir = os.path.join(script_dir, "data", "reports")
    os.makedirs(datasets_dir, exist_ok=True)
    os.makedirs(reports_dir, exist_ok=True)

    test_configs = [
        # (Shape, Size in meters, Label)
        ("pillar", 0.05, "5 cm Pillar (Dia 5 cm, Grid 2.5 cm)"),
        ("cube",   0.05, "5 cm Cube (Width 5 cm, Grid 2.5 cm)"),
        ("pillar", 0.10, "10 cm Pillar (Dia 10 cm, Grid 5.0 cm)"),
        ("cube",   0.10, "10 cm Cube (Width 10 cm, Grid 5.0 cm)"),
        ("pillar", 0.15, "15 cm Pillar (Dia 15 cm, Grid 7.5 cm)"),
        ("cube",   0.15, "15 cm Cube (Width 15 cm, Grid 7.5 cm)"),
        ("pillar", 0.20, "20 cm Pillar (Dia 20 cm, Grid 10.0 cm)"),
        ("cube",   0.20, "20 cm Cube (Width 20 cm, Grid 10.0 cm)"),
    ]

    print("\n" + "=" * 80)
    print("RUNNING MULTI-OBJECT TOF COSTMAP BENCHMARK SUITE")
    print("=" * 80)

    summary_rows = []

    for shape, size, label in test_configs:
        grid_res = size / 2.0  # Grid size is half width of object
        input_csv = os.path.join(datasets_dir, f"tof_readings_{shape}_{int(size*100)}cm.csv")
        output_csv = os.path.join(reports_dir, f"tof_validation_{shape}_{int(size*100)}cm.csv")

        # Generate dataset if it's new, or reuse existing 5cm pillar dataset if present
        legacy_5cm_csv = os.path.join(datasets_dir, "tof_16ch_grid_readings.csv")
        if shape == "pillar" and size == 0.05 and os.path.exists(legacy_5cm_csv):
            dataset_file = legacy_5cm_csv
        elif os.path.exists(input_csv):
            dataset_file = input_csv
        else:
            dataset_file = generate_simulated_dataset(
                output_csv=input_csv,
                shape_type=shape,
                size_m=size,
                grid_step_m=grid_res,
            )

        stats = fact_check_dataset(
            csv_input=dataset_file,
            csv_output=output_csv,
            shape_type=shape,
            size_m=size,
            grid_resolution_m=grid_res,
            noise_pct=0.05,
        )

        summary_rows.append((label, shape, size, grid_res, stats))
        print(f"[{label}] -> Containment: {stats['containment_rate']:.1f}%, Mean Err: {stats['mean_error_cm']:.2f} cm, Close Overext (Front): {stats['close_overext_mean_cm']:.2f} cm (Max: {stats['close_overext_max_cm']:.2f} cm), Med Overext: {stats['med_overext_mean_cm']:.2f} cm")

    print("\n" + "=" * 80)
    print("BENCHMARK EXECUTION COMPLETE")
    print("=" * 80 + "\n")
    return summary_rows


if __name__ == "__main__":
    run_all_benchmarks()
