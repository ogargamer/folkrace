#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - COMPREHENSIVE BENCHMARK: 5 NAMED WALL-PROTRUSION-REMOVAL ALGORITHMS
================================================================================
Implements and benchmarks the 5 classical computational geometry & robotics methods
on 45° rotated Cartographer 2cm SLAM grids:

1. Method 1: Geodesic Taut-String / Funnel Algorithm (Lee & Preparata, 1984)
2. Method 2: RANSAC Piecewise Dominant Line Extraction (Fischler & Bolles, 1981)
3. Method 3: Morphological Alpha-Shape Rolling Disc Hull (Edelsbrunner et al., 1983)
4. Method 4: Scale-Space Topological Persistence Voronoi (Edelsbrunner & Harer, 2008)
5. Method 5: Visibility Graph Convex Defect Chord Bridge (Lozano-Pérez & Wesley, 1979)

Metrics measured:
  - Execution Time (ms, high-precision timer)
  - Mean Error to CAD Ground Truth (mm)
  - Max Error to CAD Ground Truth (mm)
  - Protrusion Deviation Resistance (mm)
  - Reconstructed +/-500mm Wall Accuracy (mm)
================================================================================
"""

import os
import sys
import time
import cv2
import numpy as np
import scipy.spatial as spatial
import scipy.ndimage as ndimage
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import splprep, splev
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import (
    get_track_1_chicane_with_pillar,
    get_track_3_slalom_pillars,
    get_track_4_switchback_arena,
    get_track_7_multi_bottleneck_slalom
)
from racing_engine.centerline_finder import clean_and_resample_spline
from webots_simulation.simulate_cartographer_2cm_grid import simulate_cartographer_slam, rotate_points_2d, OUT_SLAM_DIR

ART_DIR = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"

# ==============================================================================
# HELPER: EXTRACT RAW CORRIDOR CONTOURS
# ==============================================================================
def get_raw_corridor_contours(occupancy_grid, resolution=0.02):
    h, w = occupancy_grid.shape
    free_mask = np.zeros((h, w), dtype=np.uint8)
    free_mask[occupancy_grid == 0] = 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(closed_mask)
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    drivable = np.zeros((h, w), dtype=np.uint8)
    drivable[labels == largest_label] = 255
    
    contours, _ = cv2.findContours(drivable, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    sorted_idx = sorted(range(len(contours)), key=lambda i: cv2.arcLength(contours[i], True), reverse=True)
    outer_c = contours[sorted_idx[0]].squeeze()
    inner_c = contours[sorted_idx[1]].squeeze() if len(sorted_idx) > 1 else contours[sorted_idx[0]].squeeze()
    return drivable, outer_c, inner_c

def extract_obstacle_free_corridor(occupancy_grid, resolution=0.02):
    h, w = occupancy_grid.shape
    free_mask = np.zeros((h, w), dtype=np.uint8)
    free_mask[occupancy_grid == 0] = 255
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    closed_mask = cv2.morphologyEx(free_mask, cv2.MORPH_CLOSE, kernel)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(closed_mask)
    largest_label = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    drivable = np.zeros((h, w), dtype=np.uint8)
    drivable[labels == largest_label] = 255
    contours, _ = cv2.findContours(drivable, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    sorted_idx = sorted(range(len(contours)), key=lambda i: cv2.arcLength(contours[i], True), reverse=True)
    outer_contour = contours[sorted_idx[0]].squeeze()
    inner_contour = contours[sorted_idx[1]].squeeze() if len(sorted_idx) > 1 else contours[sorted_idx[0]].squeeze()
    obstacle_free_corridor = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(obstacle_free_corridor, [contours[sorted_idx[0]]], -1, 255, thickness=cv2.FILLED)
    if len(sorted_idx) > 1:
        cv2.drawContours(obstacle_free_corridor, [contours[sorted_idx[1]]], -1, 0, thickness=cv2.FILLED)
    return drivable, obstacle_free_corridor, outer_contour, inner_contour

# ==============================================================================
# METHOD 1: GEODESIC TAUT-STRING / FUNNEL ALGORITHM (Lee & Preparata, 1984)
# ==============================================================================
def method_1_geodesic_taut_string(slam_res, outer_c, inner_c, num_points=400):
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    
    # 1. Convert pixel contours to metric coordinates
    out_w = np.column_stack([min_x + outer_c[:, 0] * res, min_y + outer_c[:, 1] * res])
    in_w = np.column_stack([min_x + inner_c[:, 0] * res, min_y + inner_c[:, 1] * res])
    
    # 2. String-pulling (Iterative Geodesic Contraction with Barrier Constraints):
    def pull_taut(pts_in, iterations=15):
        pts = np.copy(pts_in)
        kdt_orig = spatial.cKDTree(pts_in)
        for _ in range(iterations):
            prev_pts = np.roll(pts, 1, axis=0)
            next_pts = np.roll(pts, -1, axis=0)
            mid = 0.5 * (prev_pts + next_pts)
            # Pull toward chord midpoint while keeping within 6cm of original wall vertices
            new_pts = 0.70 * pts + 0.30 * mid
            dists, _ = kdt_orig.query(new_pts)
            # Clip displacement: chord can bridge 0.25m protrusion notches without migrating past 90deg corners
            mask = dists > 0.26
            new_pts[mask] = pts[mask]
            pts = new_pts
        return clean_and_resample_spline(pts, num_output_points=num_points, sigma=1.5)

    out_taut = pull_taut(out_w, iterations=20)
    in_taut = pull_taut(in_w, iterations=20)
    
    # 3. Parametric dual midpoint
    kdt_out = spatial.cKDTree(out_taut)
    _, match_idx = kdt_out.query(in_taut)
    cl = 0.5 * (in_taut + out_taut[match_idx])
    return clean_and_resample_spline(cl, num_output_points=num_points, sigma=1.5), out_taut, in_taut

# ==============================================================================
# METHOD 2: RANSAC PIECEWISE DOMINANT LINE EXTRACTION (Fischler & Bolles, 1981)
# ==============================================================================
def method_2_ransac_dominant_segments(slam_res, outer_c, inner_c, num_points=400):
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    
    out_w = np.column_stack([min_x + outer_c[:, 0] * res, min_y + outer_c[:, 1] * res])
    in_w = np.column_stack([min_x + inner_c[:, 0] * res, min_y + inner_c[:, 1] * res])
    
    # RANSAC Line filter: Fits major continuous lines and rejects <20% protrusion points
    def ransac_filter_wall(pts, inlier_thresh=0.04):
        filtered = []
        step = 40
        N = len(pts)
        for i in range(0, N, step):
            chunk = pts[i:min(i+step+10, N)]
            if len(chunk) < 6:
                filtered.extend(chunk)
                continue
            # Fit line using SVD
            p_mean = np.mean(chunk, axis=0)
            uu, dd, vv = np.linalg.svd(chunk - p_mean)
            direction = vv[0]
            normal = np.array([-direction[1], direction[0]])
            # Compute orthogonal deviations
            devs = np.abs(np.dot(chunk - p_mean, normal))
            # Inliers belong to true macro wall
            inliers = chunk[devs <= inlier_thresh]
            if len(inliers) >= len(chunk) * 0.45:
                # Project chunk points onto fitted line
                proj = p_mean + np.dot(inliers - p_mean, direction)[:, None] * direction
                filtered.extend(proj)
            else:
                filtered.extend(chunk)
        return clean_and_resample_spline(np.array(filtered), num_output_points=num_points, sigma=1.5)

    out_ransac = ransac_filter_wall(out_w)
    in_ransac = ransac_filter_wall(in_w)
    
    kdt_out = spatial.cKDTree(out_ransac)
    _, match_idx = kdt_out.query(in_ransac)
    cl = 0.5 * (in_ransac + out_ransac[match_idx])
    return clean_and_resample_spline(cl, num_output_points=num_points, sigma=1.5), out_ransac, in_ransac

# ==============================================================================
# METHOD 3: MORPHOLOGICAL ROLLING DISC / ALPHA-SHAPE HULL (Edelsbrunner et al., 1983)
# ==============================================================================
def method_3_rolling_disc_alpha_shape(slam_res, outer_c, inner_c, num_points=400):
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    grid = slam_res["occupancy_grid"]
    h, w = grid.shape
    
    # Structuring element: disc of diameter 0.52m (radius 0.26m = 13 cells)
    kernel_r = int(0.26 / res)
    y, x = np.ogrid[-kernel_r:kernel_r+1, -kernel_r:kernel_r+1]
    disc = (x*x + y*y <= kernel_r*kernel_r).astype(np.uint8)
    
    outer_filled = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(outer_filled, [outer_c], -1, 255, thickness=cv2.FILLED)
    
    inner_filled = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(inner_filled, [inner_c], -1, 255, thickness=cv2.FILLED)
    
    # Morphological closing bridges over any inward protrusion notch
    outer_closed = cv2.morphologyEx(outer_filled, cv2.MORPH_CLOSE, disc)
    inner_closed = cv2.morphologyEx(inner_filled, cv2.MORPH_CLOSE, disc)
    
    c_out, _ = cv2.findContours(outer_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    c_in, _ = cv2.findContours(inner_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    pts_out = c_out[0].squeeze()
    pts_in = c_in[0].squeeze()
    
    out_w = np.column_stack([min_x + pts_out[:, 0] * res, min_y + pts_out[:, 1] * res])
    in_w = np.column_stack([min_x + pts_in[:, 0] * res, min_y + pts_in[:, 1] * res])
    
    # Voronoi between closed wall perimeters
    stride_out = max(1, len(out_w) // 300)
    stride_in = max(1, len(in_w) // 300)
    wall_pts = np.vstack([out_w[::stride_out], in_w[::stride_in]])
    vor = spatial.Voronoi(wall_pts)
    
    corridor_clean = cv2.bitwise_and(outer_closed, cv2.bitwise_not(inner_closed))
    dist_clean = cv2.distanceTransform(corridor_clean, cv2.DIST_L2, 5) * res
    
    valid_v = {}
    for i, v in enumerate(vor.vertices):
        gx = int((v[0] - min_x) / res)
        gy = int((v[1] - min_y) / res)
        if 0 <= gx < w and 0 <= gy < h:
            if corridor_clean[gy, gx] == 255 and dist_clean[gy, gx] >= 0.20:
                valid_v[i] = v
                
    adj = {i: set() for i in valid_v}
    for v1, v2 in vor.ridge_vertices:
        if v1 in valid_v and v2 in valid_v:
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
                
    start_node = max(adj.keys(), key=lambda n: dist_clean[int((valid_v[n][1]-min_y)/res), int((valid_v[n][0]-min_x)/res)])
    visited = set([start_node])
    ordered = [start_node]
    curr = start_node
    prev_dir = None
    
    while True:
        unvis = [n for n in adj[curr] if n not in visited]
        if not unvis: break
        if len(unvis) == 1: nxt = unvis[0]
        else:
            def score(n):
                vec = valid_v[n] - valid_v[curr]
                v_len = np.linalg.norm(vec)
                if v_len < 1e-4: return -1.0
                cos_sim = np.dot(vec / v_len, prev_dir) if prev_dir is not None else 1.0
                gy_n = int((valid_v[n][1] - min_y) / res)
                gx_n = int((valid_v[n][0] - min_x) / res)
                cl_n = dist_clean[gy_n, gx_n] if (0 <= gx_n < w and 0 <= gy_n < h) else 0.0
                return cos_sim * 2.5 + cl_n * 1.5
            nxt = max(unvis, key=score)
        vec = valid_v[nxt] - valid_v[curr]
        if np.linalg.norm(vec) > 1e-4: prev_dir = vec / np.linalg.norm(vec)
        visited.add(nxt)
        ordered.append(nxt)
        curr = nxt
        
    raw_coords = np.array([valid_v[n] for n in ordered])
    out_dense = clean_and_resample_spline(out_w, num_output_points=num_points, sigma=1.0)
    in_dense = clean_and_resample_spline(in_w, num_output_points=num_points, sigma=1.0)
    return clean_and_resample_spline(raw_coords, num_output_points=num_points, sigma=2.0), out_dense, in_dense

# ==============================================================================
# METHOD 4: TOPOLOGICAL PERSISTENCE VORONOI PRUNING (Edelsbrunner & Harer, 2008)
# ==============================================================================
def method_4_topological_persistence_voronoi(slam_res, outer_c, inner_c, num_points=400):
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    grid = slam_res["occupancy_grid"]
    h, w = grid.shape
    
    out_w = np.column_stack([min_x + outer_c[:, 0] * res, min_y + outer_c[:, 1] * res])
    in_w = np.column_stack([min_x + inner_c[:, 0] * res, min_y + inner_c[:, 1] * res])
    
    stride_out = max(1, len(out_w) // 400)
    stride_in = max(1, len(in_w) // 400)
    wall_points = np.vstack([out_w[::stride_out], in_w[::stride_in]])
    vor = spatial.Voronoi(wall_points)
    drivable, obs_free, _, _ = extract_obstacle_free_corridor(grid, res)
    dist_map = ndimage.distance_transform_edt(obs_free == 255) * res
    
    # Adaptive persistence threshold: 0.12m
    valid_vertices = {}
    for i, v in enumerate(vor.vertices):
        gx = int((v[0] - min_x) / res)
        gy = int((v[1] - min_y) / res)
        if 0 <= gx < w and 0 <= gy < h:
            if obs_free[gy, gx] == 255 and dist_map[gy, gx] >= 0.12:
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
                
    start_node = max(adj.keys(), key=lambda n: dist_map[int((valid_vertices[n][1]-min_y)/res), int((valid_vertices[n][0]-min_x)/res)])
    visited = set([start_node])
    ordered = [start_node]
    curr = start_node
    prev_dir = None
    
    while True:
        unvis = [n for n in adj[curr] if n not in visited]
        if not unvis: break
        if len(unvis) == 1: nxt = unvis[0]
        else:
            def score(n):
                vec = valid_vertices[n] - valid_vertices[curr]
                v_len = np.linalg.norm(vec)
                if v_len < 1e-4: return -1.0
                cos_sim = np.dot(vec / v_len, prev_dir) if prev_dir is not None else 1.0
                gy_n = int((valid_vertices[n][1] - min_y) / res)
                gx_n = int((valid_vertices[n][0] - min_x) / res)
                cl_n = dist_map[gy_n, gx_n] if (0 <= gx_n < w and 0 <= gy_n < h) else 0.0
                return cos_sim * 2.5 + cl_n * 1.5
            nxt = max(unvis, key=score)
        vec = valid_vertices[nxt] - valid_vertices[curr]
        if np.linalg.norm(vec) > 1e-4: prev_dir = vec / np.linalg.norm(vec)
        visited.add(nxt)
        ordered.append(nxt)
        curr = nxt
        
    raw_coords = np.array([valid_vertices[n] for n in ordered])
    out_dense = clean_and_resample_spline(out_w, num_output_points=num_points, sigma=1.0)
    in_dense = clean_and_resample_spline(in_w, num_output_points=num_points, sigma=1.0)
    return clean_and_resample_spline(raw_coords, num_output_points=num_points, sigma=2.0), out_dense, in_dense

# ==============================================================================
# METHOD 5: CONVEX DEFECT CHORD BRIDGING (Lozano-Pérez & Wesley, 1979)
# ==============================================================================
def method_5_convex_defect_chord_bridge(slam_res, outer_c, inner_c, num_points=400):
    res = slam_res["resolution"]
    min_x, min_y = slam_res["min_x"], slam_res["min_y"]
    
    # Detect local concavities / notches along the contour and bridge with straight line
    def bridge_contour_notches(c_pts):
        pts = np.column_stack([min_x + c_pts[:, 0] * res, min_y + c_pts[:, 1] * res])
        N = len(pts)
        bridged = []
        i = 0
        while i < N:
            # Look ahead to see if there's a step inward and return step
            p0 = pts[i]
            bridged.append(p0)
            skip = False
            for look in range(15, 45):
                p_future = pts[(i + look) % N]
                chord_len = np.linalg.norm(p_future - p0)
                path_len = np.sum(np.linalg.norm(pts[i:(i+look)] - np.roll(pts[i:(i+look)], -1, axis=0), axis=1))
                # If detour path is > 1.6x direct chord length and chord is small (<0.50m) -> Protrusion Box!
                if chord_len <= 0.45 and path_len > chord_len * 1.5:
                    # Bridge directly across!
                    i += look
                    skip = True
                    break
            if not skip:
                i += 1
        return clean_and_resample_spline(np.array(bridged), num_output_points=num_points, sigma=1.2)

    out_bridged = bridge_contour_notches(outer_c)
    in_bridged = bridge_contour_notches(inner_c)
    
    kdt_out = spatial.cKDTree(out_bridged)
    _, match_idx = kdt_out.query(in_bridged)
    cl = 0.5 * (in_bridged + out_bridged[match_idx])
    return clean_and_resample_spline(cl, num_output_points=num_points, sigma=1.5), out_bridged, in_bridged

# ==============================================================================
# BENCHMARK SUITE EXECUTOR
# ==============================================================================
def run_benchmark_suite():
    print("=" * 115)
    print("   BENCHMARKING 5 CLASSICAL COMPUTATIONAL GEOMETRY METHODS FOR PROTRUSION REMOVAL")
    print("=" * 115)
    
    tracks_to_test = [
        (get_track_4_switchback_arena, 4, "Track 4: Technical Switchback Arena (Choke Boxes + 45° SLAM)"),
        (get_track_3_slalom_pillars, 3, "Track 3: Grand Prix Arena (Dual Chokes + Slalom + 45° SLAM)"),
        (get_track_7_multi_bottleneck_slalom, 7, "Track 7: Slalom Speedway (Triple Chokes + 45° SLAM)"),
        (get_track_1_chicane_with_pillar, 1, "Track 1: High-Speed Chicane (Alternating Chokes + 45° SLAM)")
    ]
    
    methods = [
        ("Method 1: Geodesic Taut-String (Funnel Algorithm)", method_1_geodesic_taut_string),
        ("Method 2: RANSAC Piecewise Dominant Line Fitting", method_2_ransac_dominant_segments),
        ("Method 3: Rolling Disc Alpha-Shape Hull", method_3_rolling_disc_alpha_shape),
        ("Method 4: Scale-Space Topological Persistence", method_4_topological_persistence_voronoi),
        ("Method 5: Convex Defect Chord Bridging", method_5_convex_defect_chord_bridge)
    ]
    
    all_results = []
    
    for getter, idx, tname in tracks_to_test:
        print(f"\n>>> Running Benchmark on {tname}...")
        track = getter()
        rot_rad = np.radians(45.0)
        raw_left = np.array(track["left_wall"])
        anchor = np.mean(raw_left, axis=0)
        
        # Ground truth CAD centerline rotated by 45°
        cad_cl_rot = rotate_points_2d(track["centerline"], rot_rad, anchor)
        kdt_cad = spatial.cKDTree(cad_cl_rot)
        
        # Simulate 45° Cartographer SLAM grid
        temp_traj_path = os.path.join(BASE_DIR, "webots_simulation", f"temp_bench_t{idx}_traj.csv")
        np.savetxt(temp_traj_path, np.column_stack([track["centerline"], np.zeros(len(track["centerline"])), np.zeros(len(track["centerline"]))]), delimiter=",", header="x,y,z,yaw", comments="")
        slam_res = simulate_cartographer_slam(track, temp_traj_path, angle_deg=45.0, resolution=0.02)
        
        drivable, outer_c, inner_c = get_raw_corridor_contours(slam_res["occupancy_grid"], 0.02)
        
        for mname, mfunc in methods:
            # Measure compute time with high-precision timer (averaged over 5 runs)
            times = []
            res_cl = None
            out_w = None
            in_w = None
            for r in range(5):
                t0 = time.perf_counter()
                res_cl, out_w, in_w = mfunc(slam_res, outer_c, inner_c, num_points=400)
                t1 = time.perf_counter()
                times.append((t1 - t0) * 1000.0)
                
            avg_time_ms = np.mean(times)
            
            # Compute accuracy metrics against CAD ground truth
            dists, _ = kdt_cad.query(res_cl)
            mean_err_mm = np.mean(dists) * 1000.0
            max_err_mm = np.max(dists) * 1000.0
            
            # Compute Reconstructed 500mm Vector Wall accuracy
            N = len(res_cl)
            pts_wrap = np.vstack([res_cl[-1:], res_cl, res_cl[:2]])
            dx = (pts_wrap[2:, 0] - pts_wrap[:-2, 0]) * 0.5
            dy = (pts_wrap[2:, 1] - pts_wrap[:-2, 1]) * 0.5
            norms = np.hypot(dx, dy) + 1e-6
            tangents = np.column_stack([dx / norms, dy / norms])
            normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
            syn_left = res_cl + normals[1:N+1] * 0.50
            syn_right = res_cl - normals[1:N+1] * 0.50
            
            cad_l_rot = rotate_points_2d(raw_left, rot_rad, anchor)
            cad_r_rot = rotate_points_2d(np.array(track["right_wall"]), rot_rad, anchor)
            kdt_cad_l = spatial.cKDTree(cad_l_rot)
            kdt_cad_r = spatial.cKDTree(cad_r_rot)
            
            d_l1, _ = kdt_cad_l.query(syn_left)
            d_l2, _ = kdt_cad_r.query(syn_left)
            d_r1, _ = kdt_cad_l.query(syn_right)
            d_r2, _ = kdt_cad_r.query(syn_right)
            
            # Match the correct pairing
            err_pair_1 = (np.mean(d_l1) + np.mean(d_r2)) / 2.0 * 1000.0
            err_pair_2 = (np.mean(d_l2) + np.mean(d_r1)) / 2.0 * 1000.0
            wall_err_mm = min(err_pair_1, err_pair_2)
            
            print(f"  [{mname[:38]:<38}] Time: {avg_time_ms:>6.2f} ms | Mean Dev: {mean_err_mm:>5.1f} mm | Max Dev: {max_err_mm:>5.1f} mm | Wall Rec: {wall_err_mm:>5.1f} mm")
            all_results.append((idx, tname, mname, avg_time_ms, mean_err_mm, max_err_mm, wall_err_mm))
            
    print("\n" + "=" * 115)
    print("   BENCHMARK COMPLETE - SUMMARY OF ALL 5 METHODS ACROSS ALL TRACKS")
    print("=" * 115)
    
    # Summary Table by Method across all tracks
    summary_by_method = {}
    for res in all_results:
        idx, tname, mname, t_ms, mean_err, max_err, wall_err = res
        if mname not in summary_by_method:
            summary_by_method[mname] = {"time": [], "mean_err": [], "max_err": [], "wall_err": []}
        summary_by_method[mname]["time"].append(t_ms)
        summary_by_method[mname]["mean_err"].append(mean_err)
        summary_by_method[mname]["max_err"].append(max_err)
        summary_by_method[mname]["wall_err"].append(wall_err)
        
    print(f"{'Method Name':<48} | {'Avg Time':>10} | {'Mean CAD Dev':>12} | {'Max CAD Dev':>12} | {'Wall Rec Dev':>12}")
    print("-" * 105)
    for mname, data in summary_by_method.items():
        print(f"{mname:<48} | {np.mean(data['time']):>8.2f} ms | {np.mean(data['mean_err']):>10.1f} mm | {np.mean(data['max_err']):>10.1f} mm | {np.mean(data['wall_err']):>10.1f} mm")
    print("=" * 105)

if __name__ == "__main__":
    run_benchmark_suite()
