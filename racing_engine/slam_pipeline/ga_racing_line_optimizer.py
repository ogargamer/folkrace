#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - GENETIC ALGORITHM RACING LINE OPTIMIZER
   Evolves pipeline hyper-parameters to find the most perfect yellow racing line
   on 45 degree Webots SLAM tracks.
================================================================================

Genome (11 genes):
  0  alpha_disc_radius     : Rolling Disc Alpha-Shape radius [0.15 .. 0.40] m
  1  voronoi_min_clearance  : Obstacle-Aware Voronoi clearance [0.06 .. 0.25] m
  2  voronoi_occ_stride     : Occupied point stride for Voronoi [400 .. 2000]
  3  spline_sigma_macro     : Gaussian sigma for macro centerline [0.5 .. 6.0]
  4  spline_sigma_voronoi   : Gaussian sigma for obstacle voronoi [1.0 .. 8.0]
  5  wall_offset            : Reconstructed wall half-width [0.40 .. 0.55] m
  6  qp_margin              : TUM QP collision margin [0.04 .. 0.18] m
  7  qp_weight_smooth       : QP curvature-smoothness weight [0.01 .. 0.40]
  8  qp_ray_max             : Max lateral ray length [0.40 .. 0.80] m
  9  qp_ray_steps           : Number of ray discretisation steps [15 .. 60]
  10 num_centerline_pts     : Centerline resampling resolution [200 .. 600]

Fitness (higher = better):
  - REWARD   shorter estimated lap time (friction-limited velocity profile)
  - REWARD   higher minimum clearance to walls/obstacles
  - PENALTY  high peak curvature (limits achievable speed)
  - PENALTY  deviation from the pre-computed CAD racing line (not just centerline)
  - DEATH_PENALTY  if any point physically collides with an obstacle

CAD Racing Line Reference:
  Computed once per track from perfect CAD geometry via the same TUM QP solver
  applied to the clean CAD walls, before any SLAM noise is introduced.
  This is the gold-standard line that the GA tries to match.

Velocity Profile:
  v_corner = sqrt(mu * g / kappa)  -- friction-limited cornering speed
  v_max_straight = 2.5 m/s         -- physical top speed of the robot
  lap_time = sum(ds / v_effective)  -- estimated by integrating along arc

Only saves SVG snapshots when a new global best fitness is found.
================================================================================"""

import os
import sys
import time
import shutil
import random
import json
import numpy as np
import scipy.spatial as spatial
import scipy.optimize as opt
import scipy.ndimage as ndimage
from scipy.interpolate import splprep, splev
import cv2

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import (
    get_track_1_chicane_with_pillar,
    get_track_2_bottleneck_circuit,
    get_track_3_slalom_pillars,
    get_track_4_switchback_arena,
    get_track_5_dual_choke_chicane_ring,
    get_track_6_quad_pillar_loop,
    get_track_7_multi_bottleneck_slalom,
    get_track_8_dual_chicane_arena,
    get_track_9_super_speedway,
    get_track_10_switchback_obstacle_maze,
)
from racing_engine.centerline_finder import clean_and_resample_spline
from webots_simulation.simulate_cartographer_2cm_grid import (
    simulate_cartographer_slam,
    rotate_points_2d,
    OUT_SLAM_DIR,
)
from racing_engine.slam_pipeline.benchmark_all_wall_reconstruction_methods import (
    get_raw_corridor_contours,
    method_3_rolling_disc_alpha_shape,
)
from racing_engine.slam_pipeline.run_track3_track4_slam_voronoi import (
    generate_obstacle_aware_voronoi_centerline,
    compute_500mm_reconstructed_walls,
)

# ──────────────────────────────────────────────────────────────────────────────
# OUTPUT DIRECTORIES
# ──────────────────────────────────────────────────────────────────────────────
GA_OUT_DIR = os.path.join(BASE_DIR, "racing_results", "ga_optimizer")
os.makedirs(GA_OUT_DIR, exist_ok=True)

ART_DIR = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"

# ──────────────────────────────────────────────────────────────────────────────
# GENE BOUNDS (min, max) for each parameter
# ──────────────────────────────────────────────────────────────────────────────
GENE_BOUNDS = [
    (0.15, 0.40),   # 0  alpha_disc_radius
    (0.06, 0.25),   # 1  voronoi_min_clearance
    (400, 2000),     # 2  voronoi_occ_stride (int)
    (0.5, 6.0),      # 3  spline_sigma_macro
    (1.0, 8.0),      # 4  spline_sigma_voronoi
    (0.40, 0.55),    # 5  wall_offset
    (0.04, 0.18),    # 6  qp_margin
    (0.01, 0.40),    # 7  qp_weight_smooth
    (0.40, 0.80),    # 8  qp_ray_max
    (15, 60),        # 9  qp_ray_steps (int)
    (200, 600),      # 10 num_centerline_pts (int)
]

GENE_NAMES = [
    "alpha_disc_radius",
    "voronoi_min_clearance",
    "voronoi_occ_stride",
    "spline_sigma_macro",
    "spline_sigma_voronoi",
    "wall_offset",
    "qp_margin",
    "qp_weight_smooth",
    "qp_ray_max",
    "qp_ray_steps",
    "num_centerline_pts",
]

INT_GENES = {2, 9, 10}  # indices of integer-valued genes

# ──────────────────────────────────────────────────────────────────────────────
# GA CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
POP_SIZE = 40
ELITE_COUNT = 6
TOURNAMENT_K = 4
MUTATION_RATE = 0.25       # probability of mutating each gene
MUTATION_SIGMA = 0.15      # std-dev as fraction of gene range
CROSSOVER_RATE = 0.80
MAX_GENERATIONS = 200
STAGNATION_LIMIT = 20      # increase mutation after this many gens without improvement

# ──────────────────────────────────────────────────────────────────────────────
# FITNESS WEIGHTS
# ──────────────────────────────────────────────────────────────────────────────
W_LAP_TIME   = 80.0        # reward shorter lap time (scaled: 1/lap_time_s)
W_MIN_CLEAR  = 120.0       # reward higher minimum clearance to walls
W_MAX_CURV   = 20.0        # penalise peak curvature
W_CAD_ERR    = 0.005       # penalise deviation from CAD racing line (per mm)
DEATH_PENALTY = -1e6       # collision penalty

# Robot physical parameters for velocity profile
MU_LATERAL   = 0.65        # lateral friction coefficient (rubber on carpet/tile)
G_GRAVITY    = 9.81        # m/s^2
V_MAX        = 2.5         # m/s -- robot top speed on a straight
V_MIN        = 0.20        # m/s -- minimum speed through tightest corners

# ──────────────────────────────────────────────────────────────────────────────
# TRACK POOL - user can pick which tracks to optimise
# ──────────────────────────────────────────────────────────────────────────────
ALL_TRACKS = [
    (get_track_3_slalom_pillars, 3, "Track 3"),
    (get_track_4_switchback_arena, 4, "Track 4"),
]

# ──────────────────────────────────────────────────────────────────────────────
# VELOCITY PROFILE & LAP TIME ESTIMATOR
# ──────────────────────────────────────────────────────────────────────────────
def estimate_lap_time(line_pts):
    """
    Estimates lap time using a friction-limited forward/backward velocity profile.

    Steps:
      1. Compute curvature kappa at every point.
      2. Cornering speed limit: v_corner = sqrt(mu * g / kappa).
      3. Clamp to [V_MIN, V_MAX].
      4. Forward pass: can't accelerate faster than a simple friction-accel model.
      5. Backward pass: mirror for braking.
      6. Integrate arc-length / velocity to get total lap time.
    """
    dx = np.gradient(line_pts[:, 0])
    dy = np.gradient(line_pts[:, 1])
    ds = np.hypot(dx, dy)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    kappa = np.abs(dx * ddy - dy * ddx) / (ds**3 + 1e-8)

    # Friction-limited cornering speed at every point
    v_corner = np.sqrt(MU_LATERAL * G_GRAVITY / (kappa + 1e-6))
    v_profile = np.clip(v_corner, V_MIN, V_MAX)

    # Longitudinal friction accel/decel limit (assume same mu * g)
    a_max = MU_LATERAL * G_GRAVITY   # ~6.4 m/s^2 for mu=0.65

    # Forward pass (acceleration limited)
    N = len(v_profile)
    v_fwd = v_profile.copy()
    for i in range(1, N):
        seg_ds = ds[i] if ds[i] > 1e-6 else 1e-6
        v_fwd[i] = min(v_profile[i], np.sqrt(v_fwd[i-1]**2 + 2.0 * a_max * seg_ds))

    # Backward pass (braking limited)
    v_bwd = v_profile.copy()
    for i in range(N - 2, -1, -1):
        seg_ds = ds[i] if ds[i] > 1e-6 else 1e-6
        v_bwd[i] = min(v_profile[i], np.sqrt(v_bwd[i+1]**2 + 2.0 * a_max * seg_ds))

    # Effective velocity is the minimum of forward and backward passes
    v_eff = np.minimum(v_fwd, v_bwd)
    v_eff = np.clip(v_eff, V_MIN, V_MAX)

    # Integrate to get lap time
    lap_time = np.sum(ds / (v_eff + 1e-8))
    avg_speed = np.sum(ds) / (lap_time + 1e-8)
    return float(lap_time), float(avg_speed), v_eff


# ──────────────────────────────────────────────────────────────────────────────
# HELPER: Pre-compute CAD racing line (gold standard, no SLAM noise)
# ──────────────────────────────────────────────────────────────────────────────
def compute_cad_racing_line(track, rot_rad, anchor):
    """
    Computes the gold-standard racing line directly from perfect CAD geometry
    (no SLAM noise). Uses the raycast gate centerline as QP seed, then runs
    the same TUM minimum-curvature QP against clean CAD walls.
    Returns the racing line in the 45-degree rotated frame.
    """
    from racing_engine.centerline_finder import find_centerline_raycast_gates

    # Get obstacle-aware centerline from clean CAD data
    cl_result = find_centerline_raycast_gates(track, num_points=400)
    cl_cad = cl_result["centerline"]

    # Build clean CAD walls from raw track data
    raw_left = np.array(track["left_wall"])
    raw_right = np.array(track["right_wall"])

    # Compute normals and offset walls at +-500mm
    N = len(cl_cad)
    pts_wrap = np.vstack([cl_cad[-1:], cl_cad, cl_cad[:2]])
    dx = (pts_wrap[2:, 0] - pts_wrap[:-2, 0]) * 0.5
    dy = (pts_wrap[2:, 1] - pts_wrap[:-2, 1]) * 0.5
    norms = np.hypot(dx, dy) + 1e-6
    tangents = np.column_stack([dx / norms, dy / norms])
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    n_pts = normals[1:N+1]
    cad_syn_left  = cl_cad + n_pts * 0.50
    cad_syn_right = cl_cad - n_pts * 0.50

    # Build a synthetic SLAM-like occupied_pts from the CAD walls for the QP
    wall_kdt_pts = np.vstack([raw_left, raw_right])

    # TUM QP on CAD geometry (simplified - single curvature pass)
    dx2 = np.gradient(cl_cad[:, 0])
    dy2 = np.gradient(cl_cad[:, 1])
    headings = np.arctan2(dy2, dx2)
    normals_left = np.column_stack([-np.sin(headings), np.cos(headings)])

    kdt_cad_walls = spatial.cKDTree(np.vstack([cad_syn_left, cad_syn_right]))
    kdt_cad_obs   = spatial.cKDTree(wall_kdt_pts)

    steps = np.linspace(0.03, 0.65, 30)
    ray_left  = cl_cad[:, None, :] + normals_left[:, None, :] * steps[None, :, None]
    ray_right = cl_cad[:, None, :] - normals_left[:, None, :] * steps[None, :, None]

    d_wl, _ = kdt_cad_walls.query(ray_left)
    d_ol, _ = kdt_cad_obs.query(ray_left)
    d_wr, _ = kdt_cad_walls.query(ray_right)
    d_or, _ = kdt_cad_obs.query(ray_right)

    margin = 0.08
    coll_left  = (d_wl < margin) | (d_ol < margin)
    coll_right = (d_wr < margin) | (d_or < margin)

    w_left_list, w_right_list = [], []
    for i in range(N):
        hits_l = np.where(coll_left[i])[0]
        w_L = steps[hits_l[0]] - margin if len(hits_l) > 0 else 0.42
        w_left_list.append(max(0.04, min(0.42, w_L)))
        hits_r = np.where(coll_right[i])[0]
        w_R = steps[hits_r[0]] - margin if len(hits_r) > 0 else 0.42
        w_right_list.append(max(0.04, min(0.42, w_R)))

    w_left  = np.array(w_left_list)
    w_right = np.array(w_right_list)

    ds = np.hypot(dx2, dy2)
    ddx = np.gradient(dx2)
    ddy = np.gradient(dy2)
    kappa_c = (dx2 * ddy - dy2 * ddx) / (ds**3 + 1e-6)

    D2 = np.zeros((N, N))
    for i in range(N):
        D2[i, (i-1)%N] =  1.0 / (ds[i]**2)
        D2[i,  i      ] = -2.0 / (ds[i]**2)
        D2[i, (i+1)%N] =  1.0 / (ds[i]**2)

    H = D2.T @ D2 + 0.06 * np.eye(N)
    f_vec = D2.T @ kappa_c
    bounds = opt.Bounds(-w_right, w_left)
    res_opt = opt.minimize(
        fun=lambda a: 0.5 * a.T @ H @ a + f_vec.T @ a,
        x0=np.zeros(N),
        jac=lambda a: H @ a + f_vec,
        bounds=bounds,
        method='L-BFGS-B',
        options={'maxiter': 300},
    )
    alpha_opt = res_opt.x
    cad_yellow = cl_cad + alpha_opt[:, None] * normals_left
    cad_yellow = clean_and_resample_spline(cad_yellow, num_output_points=400, sigma=1.2)

    # Rotate into the 45-degree SLAM frame
    return rotate_points_2d(cad_yellow, rot_rad, anchor)


# ──────────────────────────────────────────────────────────────────────────────
# HELPER: Pre-compute SLAM grid for each track (expensive, do once)
# ──────────────────────────────────────────────────────────────────────────────
def precompute_slam(track_getter, track_idx):
    """Run Webots SLAM simulation once and cache the result."""
    track = track_getter()
    rot_rad = np.radians(45.0)
    raw_left = np.array(track["left_wall"])
    anchor = np.mean(raw_left, axis=0)

    temp_traj = os.path.join(BASE_DIR, "webots_simulation", f"temp_ga_t{track_idx}_traj.csv")
    cl = track["centerline"]
    np.savetxt(
        temp_traj,
        np.column_stack([cl, np.zeros(len(cl)), np.zeros(len(cl))]),
        delimiter=",",
        header="x,y,z,yaw",
        comments="",
    )
    slam_res = simulate_cartographer_slam(track, temp_traj, angle_deg=45.0, resolution=0.02)

    # Pre-extract corridor contours
    drivable, outer_c, inner_c = get_raw_corridor_contours(slam_res["occupancy_grid"], 0.02)

    # CAD references rotated into 45-deg SLAM frame
    cad_left_rot  = rotate_points_2d(raw_left, rot_rad, anchor)
    cad_right_rot = rotate_points_2d(np.array(track["right_wall"]), rot_rad, anchor)
    macro_cl_rot  = rotate_points_2d(track["centerline"], rot_rad, anchor)

    # Gold-standard CAD racing line (run TUM QP on perfect CAD geometry)
    print(f"  [INIT] Computing CAD gold-standard racing line for track {track_idx}...")
    cad_racing_line_rot = compute_cad_racing_line(track, rot_rad, anchor)
    cad_lap_time, cad_avg_speed, _ = estimate_lap_time(cad_racing_line_rot)
    print(f"  [INIT] CAD reference lap time: {cad_lap_time:.2f}s  avg speed: {cad_avg_speed:.2f} m/s")

    # Clean up temp file
    if os.path.exists(temp_traj):
        os.remove(temp_traj)

    return {
        "track": track,
        "slam_res": slam_res,
        "drivable": drivable,
        "outer_c": outer_c,
        "inner_c": inner_c,
        "cad_left_rot": cad_left_rot,
        "cad_right_rot": cad_right_rot,
        "macro_cl_rot": macro_cl_rot,
        "cad_racing_line_rot": cad_racing_line_rot,    # gold standard racing line
        "cad_lap_time": cad_lap_time,                  # reference lap time to beat
        "cad_avg_speed": cad_avg_speed,
    }


# ──────────────────────────────────────────────────────────────────────────────
# MODIFIED PIPELINE FUNCTIONS THAT ACCEPT GENOME PARAMS
# ──────────────────────────────────────────────────────────────────────────────
def run_alpha_shape_with_params(slam_res, outer_c, inner_c, disc_radius, sigma_macro, num_pts):
    """Rolling Disc Alpha-Shape with tuneable disc radius."""
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    grid = slam_res["occupancy_grid"]
    h, w = grid.shape

    kernel_r = max(3, int(disc_radius / res))
    y, x = np.ogrid[-kernel_r : kernel_r + 1, -kernel_r : kernel_r + 1]
    disc = (x * x + y * y <= kernel_r * kernel_r).astype(np.uint8)

    outer_filled = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(outer_filled, [outer_c], -1, 255, thickness=cv2.FILLED)
    inner_filled = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(inner_filled, [inner_c], -1, 255, thickness=cv2.FILLED)

    outer_closed = cv2.morphologyEx(outer_filled, cv2.MORPH_CLOSE, disc)
    inner_closed = cv2.morphologyEx(inner_filled, cv2.MORPH_CLOSE, disc)

    c_out, _ = cv2.findContours(outer_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c_in, _ = cv2.findContours(inner_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if not c_out or not c_in:
        return None

    pts_out = c_out[0].squeeze()
    pts_in = c_in[0].squeeze()

    out_w = np.column_stack([min_x + pts_out[:, 0] * res, min_y + pts_out[:, 1] * res])
    in_w = np.column_stack([min_x + pts_in[:, 0] * res, min_y + pts_in[:, 1] * res])

    stride_out = max(1, len(out_w) // 300)
    stride_in = max(1, len(in_w) // 300)
    wall_pts = np.vstack([out_w[::stride_out], in_w[::stride_in]])

    if len(wall_pts) < 10:
        return None

    vor = spatial.Voronoi(wall_pts)
    corridor_clean = cv2.bitwise_and(outer_closed, cv2.bitwise_not(inner_closed))
    dist_clean = cv2.distanceTransform(corridor_clean, cv2.DIST_L2, 5) * res

    valid_v = {}
    for i, v in enumerate(vor.vertices):
        gx = int((v[0] - min_x) / res)
        gy = int((v[1] - min_y) / res)
        if 0 <= gx < w and 0 <= gy < h:
            if corridor_clean[gy, gx] == 255 and dist_clean[gy, gx] >= 0.18:
                valid_v[i] = v

    adj = {i: set() for i in valid_v}
    for v1, v2 in vor.ridge_vertices:
        if v1 in valid_v and v2 in valid_v:
            adj[v1].add(v2)
            adj[v2].add(v1)

    # Prune degree-1
    pruned = True
    while pruned:
        pruned = False
        deg1 = [n for n in adj if len(adj[n]) <= 1]
        if deg1:
            pruned = True
            for n in deg1:
                for nbr in adj[n]:
                    adj[nbr].discard(n)
                del adj[n]

    if len(adj) < 10:
        return None

    start_node = max(
        adj.keys(),
        key=lambda n: dist_clean[
            int((valid_v[n][1] - min_y) / res), int((valid_v[n][0] - min_x) / res)
        ],
    )
    visited = {start_node}
    ordered = [start_node]
    curr = start_node
    prev_dir = None

    while True:
        unvis = [n for n in adj[curr] if n not in visited]
        if not unvis:
            break
        if len(unvis) == 1:
            nxt = unvis[0]
        else:
            if prev_dir is not None:
                def score(n):
                    vec = valid_v[n] - valid_v[curr]
                    v_len = np.linalg.norm(vec)
                    if v_len < 1e-4:
                        return -1.0
                    cos_sim = np.dot(vec / v_len, prev_dir)
                    gy_n = int((valid_v[n][1] - min_y) / res)
                    gx_n = int((valid_v[n][0] - min_x) / res)
                    cl_n = dist_clean[gy_n, gx_n] if (0 <= gx_n < w and 0 <= gy_n < h) else 0.0
                    return cos_sim * 2.0 + cl_n * 1.5
                nxt = max(unvis, key=score)
            else:
                nxt = unvis[0]
        vec = valid_v[nxt] - valid_v[curr]
        if np.linalg.norm(vec) > 1e-4:
            prev_dir = vec / np.linalg.norm(vec)
        visited.add(nxt)
        ordered.append(nxt)
        curr = nxt

    raw_coords = np.array([valid_v[n] for n in ordered])
    return clean_and_resample_spline(raw_coords, num_output_points=num_pts, sigma=sigma_macro)


def run_obstacle_voronoi_with_params(slam_res, min_clearance, occ_stride, sigma_vor, num_pts):
    """Obstacle-Aware Voronoi with tuneable parameters."""
    occ_pts = slam_res["occupied_pts"]
    grid = slam_res["occupancy_grid"]
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    h, w = grid.shape

    free_mask = np.zeros((h, w), dtype=np.uint8)
    free_mask[grid == 0] = 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(closed_mask)
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    drivable = np.zeros((h, w), dtype=np.uint8)
    drivable[labels == largest_label] = 255
    dist_map = ndimage.distance_transform_edt(drivable == 255) * res

    stride = max(1, len(occ_pts) // int(occ_stride))
    sample_occ = occ_pts[::stride]

    if len(sample_occ) < 10:
        return None

    vor = spatial.Voronoi(sample_occ)

    valid_vertices = {}
    for i, v in enumerate(vor.vertices):
        gx = int((v[0] - min_x) / res)
        gy = int((v[1] - min_y) / res)
        if 0 <= gx < w and 0 <= gy < h:
            if drivable[gy, gx] == 255 and dist_map[gy, gx] >= min_clearance:
                valid_vertices[i] = v

    adj = {i: set() for i in valid_vertices}
    for v1, v2 in vor.ridge_vertices:
        if v1 in valid_vertices and v2 in valid_vertices:
            adj[v1].add(v2)
            adj[v2].add(v1)

    pruned = True
    while pruned:
        pruned = False
        deg1 = [n for n in adj if len(adj[n]) <= 1]
        if deg1:
            pruned = True
            for n in deg1:
                for nbr in adj[n]:
                    adj[nbr].discard(n)
                del adj[n]

    if len(adj) < 10:
        return None

    start_node = max(
        adj.keys(),
        key=lambda n: dist_map[
            int((valid_vertices[n][1] - min_y) / res),
            int((valid_vertices[n][0] - min_x) / res),
        ],
    )
    visited = {start_node}
    ordered = [start_node]
    curr = start_node
    prev_dir = None

    while True:
        unvis = [n for n in adj[curr] if n not in visited]
        if not unvis:
            break
        if len(unvis) == 1:
            nxt = unvis[0]
        else:
            if prev_dir is not None:
                def score(n):
                    vec = valid_vertices[n] - valid_vertices[curr]
                    v_len = np.linalg.norm(vec)
                    if v_len < 1e-4:
                        return -1.0
                    cos_sim = np.dot(vec / v_len, prev_dir)
                    gy_n = int((valid_vertices[n][1] - min_y) / res)
                    gx_n = int((valid_vertices[n][0] - min_x) / res)
                    cl_n = (
                        dist_map[gy_n, gx_n]
                        if (0 <= gx_n < w and 0 <= gy_n < h)
                        else 0.0
                    )
                    return cos_sim * 2.5 + cl_n * 1.5
                nxt = max(unvis, key=score)
            else:
                nxt = unvis[0]
        vec = valid_vertices[nxt] - valid_vertices[curr]
        if np.linalg.norm(vec) > 1e-4:
            prev_dir = vec / np.linalg.norm(vec)
        visited.add(nxt)
        ordered.append(nxt)
        curr = nxt

    raw_coords = np.array([valid_vertices[n] for n in ordered])
    return clean_and_resample_spline(raw_coords, num_output_points=num_pts, sigma=sigma_vor)


def compute_walls_with_offset(centerline_points, wall_offset):
    """Offset walls at a tuneable half-width."""
    N = len(centerline_points)
    pts_wrap = np.vstack([centerline_points[-1:], centerline_points, centerline_points[:2]])
    dx = (pts_wrap[2:, 0] - pts_wrap[:-2, 0]) * 0.5
    dy = (pts_wrap[2:, 1] - pts_wrap[:-2, 1]) * 0.5
    norms = np.hypot(dx, dy) + 1e-6
    tangents = np.column_stack([dx / norms, dy / norms])
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    n_pts = normals[1 : N + 1]
    syn_left = centerline_points + n_pts * wall_offset
    syn_right = centerline_points - n_pts * wall_offset
    return syn_left, syn_right


def optimize_yellow_line_with_params(
    cl_obs_vor, syn_left, syn_right, slam_res,
    margin, weight_smoothness, ray_max, ray_steps
):
    """TUM Minimum Curvature QP with tuneable parameters."""
    N = len(cl_obs_vor)
    dx = np.gradient(cl_obs_vor[:, 0])
    dy = np.gradient(cl_obs_vor[:, 1])
    headings = np.arctan2(dy, dx)
    normals_left = np.column_stack([-np.sin(headings), np.cos(headings)])

    kdt_walls = spatial.cKDTree(np.vstack([syn_left, syn_right]))
    kdt_occ = spatial.cKDTree(slam_res["occupied_pts"])

    steps = np.linspace(0.03, ray_max, int(ray_steps))
    ray_left = cl_obs_vor[:, None, :] + normals_left[:, None, :] * steps[None, :, None]
    ray_right = cl_obs_vor[:, None, :] - normals_left[:, None, :] * steps[None, :, None]

    d_wl, _ = kdt_walls.query(ray_left)
    d_ol, _ = kdt_occ.query(ray_left)
    d_wr, _ = kdt_walls.query(ray_right)
    d_or, _ = kdt_occ.query(ray_right)

    coll_left = (d_wl < margin) | (d_ol < margin)
    coll_right = (d_wr < margin) | (d_or < margin)

    w_left_list = []
    w_right_list = []

    for i in range(N):
        hits_l = np.where(coll_left[i])[0]
        w_L = steps[hits_l[0]] - margin if len(hits_l) > 0 else 0.42
        w_left_list.append(max(0.04, min(0.42, w_L)))

        hits_r = np.where(coll_right[i])[0]
        w_R = steps[hits_r[0]] - margin if len(hits_r) > 0 else 0.42
        w_right_list.append(max(0.04, min(0.42, w_R)))

    w_left = np.array(w_left_list)
    w_right = np.array(w_right_list)

    ds = np.hypot(dx, dy)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    kappa_c = (dx * ddy - dy * ddx) / (ds**3 + 1e-6)

    D2 = np.zeros((N, N))
    for i in range(N):
        D2[i, (i - 1) % N] = 1.0 / (ds[i] ** 2)
        D2[i, i] = -2.0 / (ds[i] ** 2)
        D2[i, (i + 1) % N] = 1.0 / (ds[i] ** 2)

    H = D2.T @ D2 + weight_smoothness * np.eye(N)
    f = D2.T @ kappa_c
    bounds = opt.Bounds(-w_right, w_left)

    res_opt = opt.minimize(
        fun=lambda a: 0.5 * a.T @ H @ a + f.T @ a,
        x0=np.zeros(N),
        jac=lambda a: H @ a + f,
        bounds=bounds,
        method="L-BFGS-B",
        options={"maxiter": 200},
    )

    alpha_opt = res_opt.x
    yellow_pts = cl_obs_vor + alpha_opt[:, None] * normals_left
    return clean_and_resample_spline(yellow_pts, num_output_points=N, sigma=1.2)


# ──────────────────────────────────────────────────────────────────────────────
# FITNESS EVALUATION
# ──────────────────────────────────────────────────────────────────────────────
def evaluate_fitness(genome, cached_slam):
    """
    Runs the full pipeline with genome parameters and returns a fitness scalar.
    Higher is better.
    """
    (
        disc_r, vor_clear, occ_stride, sigma_macro, sigma_vor,
        wall_offset, qp_margin, qp_weight, qp_ray_max, qp_ray_steps, num_pts
    ) = genome

    occ_stride = int(occ_stride)
    qp_ray_steps = int(qp_ray_steps)
    num_pts = int(num_pts)

    slam_res = cached_slam["slam_res"]
    outer_c = cached_slam["outer_c"]
    inner_c = cached_slam["inner_c"]

    try:
        # Stage 1: Alpha-Shape Macro Centerline
        cl_macro = run_alpha_shape_with_params(
            slam_res, outer_c, inner_c, disc_r, sigma_macro, num_pts
        )
        if cl_macro is None:
            return DEATH_PENALTY, None

        # Stage 2: Reconstructed walls
        syn_left, syn_right = compute_walls_with_offset(cl_macro, wall_offset)

        # Stage 3: Obstacle-Aware Voronoi seed
        cl_obs_vor = run_obstacle_voronoi_with_params(
            slam_res, vor_clear, occ_stride, sigma_vor, num_pts
        )
        if cl_obs_vor is None:
            return DEATH_PENALTY, None

        # Stage 4: TUM QP Yellow Racing Line
        yellow = optimize_yellow_line_with_params(
            cl_obs_vor, syn_left, syn_right, slam_res,
            qp_margin, qp_weight, qp_ray_max, qp_ray_steps
        )
        if yellow is None or len(yellow) < 20:
            return DEATH_PENALTY, None

        # ── Compute fitness metrics ──
        dx = np.gradient(yellow[:, 0])
        dy = np.gradient(yellow[:, 1])
        ds = np.hypot(dx, dy)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        kappa = np.abs(dx * ddy - dy * ddx) / (ds**3 + 1e-6)

        total_curvature = np.sum(kappa * ds)
        max_curvature = np.max(kappa)

        # Minimum clearance to any occupied point
        kdt_occ = spatial.cKDTree(slam_res["occupied_pts"])
        dists_occ, _ = kdt_occ.query(yellow)
        min_clearance = np.min(dists_occ)

        # Collision check: if any point is closer than 0.03m to an obstacle
        if min_clearance < 0.03:
            return DEATH_PENALTY, None

        # Velocity profile and lap time
        lap_time, avg_speed, v_profile = estimate_lap_time(yellow)

        # CAD error: compare against the gold-standard CAD RACING LINE
        # (NOT the geometric centerline -- the actual optimal TUM QP line on perfect CAD)
        cad_racing_line = cached_slam["cad_racing_line_rot"]
        kdt_cad = spatial.cKDTree(cad_racing_line)
        dists_cad, _ = kdt_cad.query(yellow)
        cad_error_mm = np.mean(dists_cad) * 1000.0

        # Fitness: reward short lap time, safe clearance, penalise peak curvature & CAD deviation
        fitness = (
            W_LAP_TIME  * (1.0 / (lap_time + 1e-6))
            + W_MIN_CLEAR * min_clearance
            - W_MAX_CURV  * max_curvature
            - W_CAD_ERR   * cad_error_mm
        )

        result_data = {
            "yellow": yellow,
            "cl_macro": cl_macro,
            "cl_obs_vor": cl_obs_vor,
            "syn_left": syn_left,
            "syn_right": syn_right,
            "total_curvature": float(total_curvature),
            "max_curvature": float(max_curvature),
            "min_clearance": float(min_clearance),
            "cad_error_mm": float(cad_error_mm),
            "lap_time": float(lap_time),
            "avg_speed": float(avg_speed),
        }

        return float(fitness), result_data

    except Exception as e:
        return DEATH_PENALTY, None


# ──────────────────────────────────────────────────────────────────────────────
# GENETIC OPERATORS
# ──────────────────────────────────────────────────────────────────────────────
def random_genome():
    """Create a random genome within bounds."""
    g = []
    for i, (lo, hi) in enumerate(GENE_BOUNDS):
        val = random.uniform(lo, hi)
        if i in INT_GENES:
            val = round(val)
        g.append(val)
    return g


def clamp_genome(genome):
    """Clamp each gene to its valid range."""
    clamped = []
    for i, (lo, hi) in enumerate(GENE_BOUNDS):
        val = max(lo, min(hi, genome[i]))
        if i in INT_GENES:
            val = round(val)
        clamped.append(val)
    return clamped


def crossover_blend(parent_a, parent_b):
    """BLX-alpha crossover: blend parents with some exploration."""
    alpha = 0.3
    child = []
    for i in range(len(parent_a)):
        lo = min(parent_a[i], parent_b[i])
        hi = max(parent_a[i], parent_b[i])
        span = hi - lo
        val = random.uniform(lo - alpha * span, hi + alpha * span)
        child.append(val)
    return clamp_genome(child)


def mutate(genome, generation, stagnation_count):
    """Gaussian mutation with adaptive sigma that increases on stagnation."""
    sigma_mult = 1.0 + 0.5 * min(stagnation_count / STAGNATION_LIMIT, 3.0)
    mutated = list(genome)
    for i in range(len(mutated)):
        if random.random() < MUTATION_RATE:
            lo, hi = GENE_BOUNDS[i]
            sigma = MUTATION_SIGMA * (hi - lo) * sigma_mult
            mutated[i] += random.gauss(0, sigma)
    return clamp_genome(mutated)


def tournament_select(population, fitnesses, k=TOURNAMENT_K):
    """Tournament selection."""
    indices = random.sample(range(len(population)), min(k, len(population)))
    best_idx = max(indices, key=lambda i: fitnesses[i])
    return population[best_idx]


# ──────────────────────────────────────────────────────────────────────────────
# SNAPSHOT SVG EXPORTER (Lightweight, for GA progress snapshots)
# ──────────────────────────────────────────────────────────────────────────────
def export_snapshot_svg(
    filename, track_idx, generation, fitness, genome, result_data, cached_slam
):
    """Export a compact SVG snapshot of the current best racing line."""
    slam_res = cached_slam["slam_res"]
    occ_pts = slam_res["occupied_pts"]
    yellow = result_data["yellow"]
    cl_macro = result_data["cl_macro"]
    cl_obs_vor = result_data["cl_obs_vor"]
    syn_left = result_data["syn_left"]
    syn_right = result_data["syn_right"]
    cad_left_rot = cached_slam["cad_left_rot"]
    cad_right_rot = cached_slam["cad_right_rot"]

    all_pts = np.vstack([occ_pts, yellow, syn_left, syn_right, cad_left_rot, cad_right_rot])
    min_x, min_y = np.min(all_pts, axis=0) - 0.3
    max_x, max_y = np.max(all_pts, axis=0) + 0.3
    scale = 100.0

    track_w = int((max_x - min_x) * scale)
    track_h = int((max_y - min_y) * scale)
    sidebar_w = 460
    svg_w = track_w + sidebar_w
    svg_h = max(track_h, 650)

    def tx(x):
        return (x - min_x) * scale

    def ty(y):
        return track_h - (y - min_y) * scale + (svg_h - track_h) / 2

    svg = []
    svg.append(
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {svg_w} {svg_h}" '
        f'width="{svg_w}" height="{svg_h}" '
        f'style="background:#080c14; font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;">'
    )

    # Grid
    svg.append("<defs>")
    svg.append('  <pattern id="gm" width="100" height="100" patternUnits="userSpaceOnUse">')
    svg.append(
        '    <path d="M 100 0 L 0 0 0 100" fill="none" stroke="#23324d" stroke-width="1" stroke-opacity="0.5"/>'
    )
    svg.append("  </pattern>")
    svg.append("</defs>")
    svg.append(f'<rect width="{track_w}" height="{svg_h}" fill="url(#gm)" />')

    # SLAM voxels
    svg.append('<g opacity="0.7">')
    # Sample every 3rd point to keep SVG small
    for pt in occ_pts[::3]:
        svg.append(
            f'<rect x="{tx(pt[0])-1:.0f}" y="{ty(pt[1])-1:.0f}" '
            f'width="2" height="2" fill="#00f0ff" opacity="0.6"/>'
        )
    svg.append("</g>")

    # Obstacles
    for obs in slam_res["obstacles"]:
        if obs["type"] == "circle":
            cx, cy, r = tx(obs["cx"]), ty(obs["cy"]), obs["radius"] * scale
            svg.append(
                f'<circle cx="{cx:.0f}" cy="{cy:.0f}" r="{r:.0f}" '
                f'fill="#fbbf24" stroke="#fff" stroke-width="2" opacity="0.9"/>'
            )
        elif obs["type"] == "wall_rectangle":
            poly = obs.get("polygon")
            if poly is not None:
                p_str = " ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in poly])
                svg.append(
                    f'<polygon points="{p_str}" fill="#fbbf24" stroke="#fff" '
                    f'stroke-width="2" opacity="0.9"/>'
                )

    # CAD walls dotted
    lw = "M " + " L ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in cad_left_rot]) + " Z"
    rw = "M " + " L ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in cad_right_rot]) + " Z"
    svg.append(f'<path d="{lw}" fill="none" stroke="#38bdf8" stroke-width="1.5" stroke-dasharray="4,4" opacity="0.5"/>')
    svg.append(f'<path d="{rw}" fill="none" stroke="#f43f5e" stroke-width="1.5" stroke-dasharray="4,4" opacity="0.5"/>')

    # Reconstructed walls
    sl = "M " + " L ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in syn_left]) + " Z"
    sr = "M " + " L ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in syn_right]) + " Z"
    svg.append(f'<path d="{sl}" fill="none" stroke="#06b6d4" stroke-width="2" opacity="0.7"/>')
    svg.append(f'<path d="{sr}" fill="none" stroke="#fb7185" stroke-width="2" opacity="0.7"/>')

    # Voronoi seed
    ov = "M " + " L ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in cl_obs_vor]) + " Z"
    svg.append(f'<path d="{ov}" fill="none" stroke="#38bdf8" stroke-width="1.8" stroke-dasharray="4,3" opacity="0.5"/>')

    # Alpha-shape centerline
    mc = "M " + " L ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in cl_macro]) + " Z"
    svg.append(f'<path d="{mc}" fill="none" stroke="#c084fc" stroke-width="2" opacity="0.6"/>')

    # CAD racing line reference (gold dotted overlay)
    cad_rl = cached_slam["cad_racing_line_rot"]
    cr = "M " + " L ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in cad_rl]) + " Z"
    svg.append(f'<path d="{cr}" fill="none" stroke="#ffd700" stroke-width="2.5" stroke-dasharray="5,3" opacity="0.70"/>')

    # YELLOW RACING LINE (the star)
    yl = "M " + " L ".join([f"{tx(p[0]):.0f},{ty(p[1]):.0f}" for p in yellow]) + " Z"
    svg.append(
        f'<path d="{yl}" fill="none" stroke="#facc15" stroke-width="4" '
        f'stroke-linecap="round" stroke-linejoin="round" opacity="0.98"/>'
    )

    # ── SIDEBAR ──
    sb = track_w
    svg.append(f'<rect x="{sb}" y="0" width="{sidebar_w}" height="{svg_h}" fill="#0f172a" stroke="#1e293b" stroke-width="1"/>')

    svg.append(f'<text x="{sb+20}" y="36" fill="#f8fafc" font-size="18" font-weight="700">GA Optimizer</text>')
    svg.append(f'<text x="{sb+20}" y="58" fill="#94a3b8" font-size="13">Track {track_idx} | Gen {generation}</text>')
    svg.append(f'<line x1="{sb+20}" y1="72" x2="{sb+sidebar_w-20}" y2="72" stroke="#334155" stroke-width="1"/>')

    # Fitness
    svg.append(f'<text x="{sb+20}" y="100" fill="#4ade80" font-size="16" font-weight="700">Fitness: {fitness:.4f}</text>')

    # CAD reference lap time (gold standard)
    cad_lt  = cached_slam["cad_lap_time"]
    cad_spd = cached_slam["cad_avg_speed"]
    svg.append(f'<text x="{sb+20}" y="120" fill="#64748b" font-size="11">CAD ref: {cad_lt:.2f}s @ {cad_spd:.2f} m/s</text>')

    # Metrics
    y_cur = 148
    lap_t = result_data.get("lap_time", 0.0)
    avg_s = result_data.get("avg_speed", 0.0)
    delta_t = lap_t - cad_lt
    delta_str = f"+{delta_t:.2f}s slower" if delta_t > 0 else f"{-delta_t:.2f}s faster"
    delta_col = "#f87171" if delta_t > 0 else "#4ade80"
    metrics = [
        ("Lap Time",      f"{lap_t:.2f}s",                                  "#f1f5f9"),
        ("vs CAD Line",   delta_str,                                         delta_col),
        ("Avg Speed",     f"{avg_s:.2f} m/s",                               "#f1f5f9"),
        ("Min Clearance", f'{result_data["min_clearance"]*1000:.1f} mm',    "#f1f5f9"),
        ("Peak Kappa",    f'{result_data["max_curvature"]:.2f} rad/m',      "#f1f5f9"),
        ("CAD Line Err",  f'{result_data["cad_error_mm"]:.1f} mm',          "#f1f5f9"),
    ]
    for label, val, col in metrics:
        svg.append(f'<text x="{sb+20}" y="{y_cur}" fill="#94a3b8" font-size="12">{label}:</text>')
        svg.append(f'<text x="{sb+200}" y="{y_cur}" fill="{col}" font-size="12" font-weight="600">{val}</text>')
        y_cur += 22

    # Genome
    y_cur += 12
    svg.append(f'<text x="{sb+20}" y="{y_cur}" fill="#818cf8" font-size="13" font-weight="700">Genome Parameters</text>')
    y_cur += 6
    svg.append(f'<line x1="{sb+20}" y1="{y_cur}" x2="{sb+sidebar_w-20}" y2="{y_cur}" stroke="#334155" stroke-width="1"/>')
    y_cur += 18

    for i, (name, val) in enumerate(zip(GENE_NAMES, genome)):
        if i in INT_GENES:
            val_str = str(int(val))
        else:
            val_str = f"{val:.4f}"
        svg.append(f'<text x="{sb+20}" y="{y_cur}" fill="#94a3b8" font-size="11">{name}:</text>')
        svg.append(f'<text x="{sb+250}" y="{y_cur}" fill="#e2e8f0" font-size="11" font-weight="600">{val_str}</text>')
        y_cur += 18

    svg.append("</svg>")

    with open(filename, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


# ──────────────────────────────────────────────────────────────────────────────
# MAIN GA LOOP
# ──────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 100)
    print("   DA WINNA - GENETIC ALGORITHM RACING LINE OPTIMIZER")
    print("=" * 100)

    # ── Pre-compute SLAM grids (do once) ──
    print("\n[INIT] Pre-computing SLAM grids for all tracks...")
    cached_slams = {}
    for getter, idx, name in ALL_TRACKS:
        t0 = time.time()
        cached_slams[idx] = precompute_slam(getter, idx)
        print(f"  [OK] {name} SLAM cached in {time.time()-t0:.1f}s")

    # Run GA separately per track
    for getter, track_idx, track_name in ALL_TRACKS:
        print(f"\n{'='*100}")
        print(f"   OPTIMISING: {track_name}")
        print(f"{'='*100}")

        cached = cached_slams[track_idx]
        track_out_dir = os.path.join(GA_OUT_DIR, f"track{track_idx:02d}")
        os.makedirs(track_out_dir, exist_ok=True)

        # Seed the known-good default genome into population
        default_genome = [0.26, 0.14, 1200, 2.0, 3.0, 0.50, 0.10, 0.08, 0.70, 35, 400]
        population = [default_genome] + [random_genome() for _ in range(POP_SIZE - 1)]

        global_best_fitness = -1e9
        global_best_genome = None
        global_best_result = None
        stagnation_count = 0
        snapshot_count = 0

        log_path = os.path.join(track_out_dir, "ga_log.jsonl")
        log_file = open(log_path, "w", encoding="utf-8")

        for gen in range(MAX_GENERATIONS):
            gen_start = time.time()

            # ── Evaluate population ──
            fitnesses = []
            results = []
            for genome in population:
                fit, res_data = evaluate_fitness(genome, cached)
                fitnesses.append(fit)
                results.append(res_data)

            # ── Track best in this generation ──
            gen_best_idx = max(range(len(fitnesses)), key=lambda i: fitnesses[i])
            gen_best_fit = fitnesses[gen_best_idx]
            gen_best_genome = population[gen_best_idx]
            gen_best_result = results[gen_best_idx]

            alive_count = sum(1 for f in fitnesses if f > DEATH_PENALTY)
            gen_time = time.time() - gen_start

            # ── Check for new global best ──
            improved = False
            if gen_best_fit > global_best_fitness and gen_best_result is not None:
                global_best_fitness = gen_best_fit
                global_best_genome = list(gen_best_genome)
                global_best_result = gen_best_result
                stagnation_count = 0
                improved = True

                # ── SNAPSHOT: Save SVG only on improvement ──
                snapshot_count += 1
                svg_name = f"ga_best_gen{gen:04d}_fit{gen_best_fit:.3f}.svg"
                svg_path = os.path.join(track_out_dir, svg_name)
                export_snapshot_svg(
                    svg_path, track_idx, gen, gen_best_fit,
                    gen_best_genome, gen_best_result, cached
                )
                print(
                    f"  [GEN {gen:4d}] ** NEW BEST  fitness={gen_best_fit:10.4f}  "
                    f"curv={gen_best_result['total_curvature']:.2f}  "
                    f"clr={gen_best_result['min_clearance']*1000:.0f}mm  "
                    f"cad={gen_best_result['cad_error_mm']:.1f}mm  "
                    f"alive={alive_count}/{POP_SIZE}  "
                    f"t={gen_time:.1f}s  -> {svg_name}"
                )
            else:
                stagnation_count += 1
                if gen % 5 == 0 or gen < 5:
                    print(
                        f"  [GEN {gen:4d}]   best={gen_best_fit:10.4f}  "
                        f"global={global_best_fitness:10.4f}  "
                        f"alive={alive_count}/{POP_SIZE}  "
                        f"stag={stagnation_count}  "
                        f"t={gen_time:.1f}s"
                    )

            # ── Log to JSONL ──
            log_entry = {
                "gen": gen,
                "best_fitness": float(gen_best_fit),
                "global_best": float(global_best_fitness),
                "alive": alive_count,
                "stagnation": stagnation_count,
                "time_s": round(gen_time, 2),
                "improved": improved,
                "best_genome": [round(g, 5) for g in gen_best_genome],
            }
            if gen_best_result:
                log_entry["total_curvature"] = round(gen_best_result["total_curvature"], 3)
                log_entry["max_curvature"] = round(gen_best_result["max_curvature"], 3)
                log_entry["min_clearance_mm"] = round(gen_best_result["min_clearance"] * 1000, 1)
                log_entry["cad_error_mm"] = round(gen_best_result["cad_error_mm"], 1)
            log_file.write(json.dumps(log_entry) + "\n")
            log_file.flush()

            # ── Build next generation ──
            # Sort by fitness (descending)
            ranked = sorted(range(POP_SIZE), key=lambda i: fitnesses[i], reverse=True)

            # Elitism: keep top ELITE_COUNT
            new_pop = [list(population[ranked[i]]) for i in range(ELITE_COUNT)]

            # Always keep global best in population
            if global_best_genome is not None:
                new_pop[0] = list(global_best_genome)

            # Fill remaining via crossover + mutation
            while len(new_pop) < POP_SIZE:
                if random.random() < CROSSOVER_RATE:
                    p1 = tournament_select(population, fitnesses)
                    p2 = tournament_select(population, fitnesses)
                    child = crossover_blend(p1, p2)
                else:
                    child = list(tournament_select(population, fitnesses))
                child = mutate(child, gen, stagnation_count)
                new_pop.append(child)

            # Inject random immigrants on heavy stagnation
            if stagnation_count > STAGNATION_LIMIT * 2:
                n_immigrants = POP_SIZE // 4
                for i in range(n_immigrants):
                    new_pop[-(i + 1)] = random_genome()
                print(f"  [GEN {gen:4d}]   >> Injected {n_immigrants} random immigrants (heavy stagnation)")

            population = new_pop

        log_file.close()

        # -- Final report --
        print(f"\n{'-'*80}")
        print(f"  TRACK {track_idx} OPTIMIZATION COMPLETE")
        print(f"  Best Fitness:       {global_best_fitness:.4f}")
        if global_best_result:
            print(f"  Lap Time:           {global_best_result['lap_time']:.2f} s  (CAD Ref: {cached['cad_lap_time']:.2f} s)")
            print(f"  Avg Speed:          {global_best_result['avg_speed']:.2f} m/s (CAD Ref: {cached['cad_avg_speed']:.2f} m/s)")
            print(f"  CAD Racing Line Err:{global_best_result['cad_error_mm']:.1f} mm")
            print(f"  Min Clearance:      {global_best_result['min_clearance']*1000:.1f} mm")
            print(f"  Max Curvature:      {global_best_result['max_curvature']:.3f} rad/m")
            print(f"  Total Curvature:    {global_best_result['total_curvature']:.3f} rad")
        print(f"  Total Snapshots:    {snapshot_count}")
        print(f"  Genome:")
        for name, val in zip(GENE_NAMES, global_best_genome):
            print(f"    {name:30s} = {val}")
        print(f"{'-'*80}")

        # Save final best genome as JSON
        final_path = os.path.join(track_out_dir, "best_genome.json")
        with open(final_path, "w") as f:
            json.dump(
                {
                    "track": track_idx,
                    "fitness": global_best_fitness,
                    "genome": dict(zip(GENE_NAMES, global_best_genome)),
                    "metrics": {
                        "lap_time_s": global_best_result["lap_time"],
                        "avg_speed_mps": global_best_result["avg_speed"],
                        "cad_racing_line_error_mm": global_best_result["cad_error_mm"],
                        "min_clearance_mm": global_best_result["min_clearance"] * 1000,
                        "max_curvature_rad_m": global_best_result["max_curvature"],
                        "total_curvature_rad": global_best_result["total_curvature"],
                    } if global_best_result else {},
                },
                f,
                indent=2,
            )

        # Export final best SVG to main artifacts
        if global_best_result:
            final_svg = os.path.join(track_out_dir, f"track{track_idx:02d}_ga_best_final.svg")
            export_snapshot_svg(
                final_svg, track_idx, MAX_GENERATIONS, global_best_fitness,
                global_best_genome, global_best_result, cached
            )
            print(f"  [SAVED] Final best -> {final_svg}")

    print("\n" + "=" * 100)
    print("   GA OPTIMIZATION COMPLETE FOR ALL TRACKS")
    print("=" * 100)


if __name__ == "__main__":
    main()
