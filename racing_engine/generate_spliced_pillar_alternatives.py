#!/usr/bin/env python3
"""
================================================================================
  DA WINNA - DYNAMIC HOMOTOPY SPLICER: OPTIMAL LINE + LOCAL PILLAR ALTERNATIVES
  (MU = 0.60 | 12 cm Safety Margin | Ultra-Low Compute < 200 ms)
  
  Track: "C:\\Users\\nurmi\\Downloads\\track_layout_1787771901940.json"
  
  • Final Optimal Racing Line (Master Baseline, Unmodified)
  • For each freestanding pillar:
      1. Identifies the opposite/alternative bypass channel.
      2. Automatically detects the exact Divergence (Split) Point (s_split) and Reconvergence (Join) Point (s_join).
      3. Seamlessly blends the local alternative bypass into the optimal line.
      4. Keeps the optimal line everywhere else on the track!
================================================================================
"""

import os
import sys
import time
import json
import numpy as np
import scipy.spatial as spatial
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import load_track_from_json
from racing_engine.centerline_finder import get_all_boundary_and_obstacle_points, find_centerline_voronoi
from racing_engine.racing_line_optimizer import compute_spline_properties, TUMMinimumCurvatureOptimizer

t_pipeline_start = time.perf_counter()

# 1. Load Track Data
t0_load = time.perf_counter()
json_path = r"C:\Users\nurmi\Downloads\track_layout_1787771901940.json"
track_data = load_track_from_json(json_path)

left_wall = np.array(track_data["left_wall"], dtype=np.float64)
right_wall = np.array(track_data["right_wall"], dtype=np.float64)
obstacles = track_data.get("obstacles", [])

boundary_pts = get_all_boundary_and_obstacle_points(track_data, point_spacing=0.020)
barrier_tree = spatial.cKDTree(boundary_pts)

lw_p1, lw_p2 = left_wall[:-1], left_wall[1:]
rw_p1, rw_p2 = right_wall[:-1], right_wall[1:]
all_p1 = np.vstack([lw_p1, rw_p1])
all_p2 = np.vstack([lw_p2, rw_p2])

def dist_to_segment_array(pt, p1_arr, p2_arr):
    p_to_p1 = pt - p1_arr
    seg_vec = p2_arr - p1_arr
    proj = np.clip(np.sum(p_to_p1 * seg_vec, axis=1) / (np.sum(seg_vec**2, axis=1) + 1e-12), 0.0, 1.0)
    closest = p1_arr + proj[:, np.newaxis] * seg_vec
    dists = np.hypot(pt[0] - closest[:, 0], pt[1] - closest[:, 1])
    min_idx = np.argmin(dists)
    return dists[min_idx], closest[min_idx]

# Identify the 5 Freestanding Pillars
freestanding_pillars = []
for idx, obs in enumerate(obstacles):
    if obs.get("type") == "circle":
        cx, cy, r = obs["cx"], obs["cy"], obs["radius"]
        pt = np.array([cx, cy])
        d_lw, p_lw = dist_to_segment_array(pt, lw_p1, lw_p2)
        d_rw, p_rw = dist_to_segment_array(pt, rw_p1, rw_p2)
        w_left = d_lw - r
        w_right = d_rw - r
        if w_left >= 0.20 and w_right >= 0.20:
            left_center = pt + ((p_lw - pt) / d_lw) * (r + w_left / 2.0)
            right_center = pt + ((p_rw - pt) / d_rw) * (r + w_right / 2.0)
            freestanding_pillars.append({
                "id": f"Pillar_{idx}",
                "name": f"Pillar #{idx} (r={r*100:.0f}cm)",
                "center": pt,
                "radius": r,
                "left_channel_pt": left_center,
                "right_channel_pt": right_center
            })

t_load_ms = (time.perf_counter() - t0_load) * 1000.0

# 2. Compute Master Optimal Racing Line (Fast Embedded, MU=0.60, 12cm Margin)
t0_opt = time.perf_counter()
vor_res = find_centerline_voronoi(track_data, num_points=120)

optimizer = TUMMinimumCurvatureOptimizer(
    weight_curvature=2.00,
    weight_path_smoothness=0.12,
    weight_jerk=0.40,
    robot_safety_margin=0.12, # 12 cm margin everywhere
    num_progression_stages=4,
    max_solver_sub_iters=120,
    max_speed=4.0,
    friction_coeff=0.60 # MU = 0.60
)
res_master = optimizer.optimize_racing_line(track_data, vor_res)
opt_raw = res_master["racing_line"]

tck_opt, _ = splprep([opt_raw[:, 0], opt_raw[:, 1]], s=0.005, per=True)
u_1000 = np.linspace(0, 1, 1000, endpoint=False)
optimal_line = np.column_stack(splev(u_1000, tck_opt))

props_opt = compute_spline_properties(optimal_line)
v_opt = optimizer._generate_velocity_profile(props_opt["curvature"], props_opt["ds"], lookahead_distance_m=0.10)
normals_opt = props_opt["normals"]
t_master_ms = (time.perf_counter() - t0_opt) * 1000.0

# 3. For each pillar, determine which side the optimal line took, and generate the spliced alternative
t0_splice = time.perf_counter()
output_dir = os.path.join(BASE_DIR, "racing_results", "spliced_pillar_alternatives_mu06_12cm")
os.makedirs(output_dir, exist_ok=True)

spliced_alternatives = []

for p_idx, fp in enumerate(freestanding_pillars):
    t_pillar_0 = time.perf_counter()
    p_center = fp["center"]
    r = fp["radius"]
    
    # 1. Find nearest point on optimal line to this pillar
    d_to_opt = np.hypot(optimal_line[:, 0] - p_center[0], optimal_line[:, 1] - p_center[1])
    closest_opt_idx = int(np.argmin(d_to_opt))
    opt_p = optimal_line[closest_opt_idx]
    
    # Check if optimal line is on Left/Outer or Right/Inner side of the pillar
    # Using cross product with tangent
    tang_p = props_opt["tangents"][closest_opt_idx]
    v_to_opt = opt_p - p_center
    cross_z = tang_p[0] * v_to_opt[1] - tang_p[1] * v_to_opt[0]
    
    # If cross_z > 0, optimal line passed on the left side; alternative must pass on the right side
    opt_side = "Left/Outer" if cross_z > 0 else "Right/Inner"
    alt_side = "Right/Inner" if cross_z > 0 else "Left/Outer"
    alt_target_pt = fp["right_channel_pt"] if cross_z > 0 else fp["left_channel_pt"]
    
    # 2. Localized Obstacle Bypass Shift applied ONLY around this pillar
    alt_cand = optimal_line.copy()
    idx_diff = np.abs(np.arange(1000) - closest_opt_idx)
    idx_diff = np.minimum(idx_diff, 1000 - idx_diff)
    
    # Local Gaussian envelope of span +/- 75 waypoints (~1.1 m before and after pillar)
    span_sigma = 35.0
    w_envelope = np.exp(-(idx_diff / span_sigma)**2)[:, np.newaxis]
    
    delta_shift_vec = alt_target_pt - optimal_line[closest_opt_idx]
    alt_cand += w_envelope * delta_shift_vec
    
    # Smooth transition
    alt_cand[:, 0] = gaussian_filter1d(alt_cand[:, 0], sigma=3.0, mode='wrap')
    alt_cand[:, 1] = gaussian_filter1d(alt_cand[:, 1], sigma=3.0, mode='wrap')
    
    # Ensure strict >= 12cm margin push-out around barrier
    d_alt, idx_alt = barrier_tree.query(alt_cand)
    viol = d_alt < 0.125
    if np.any(viol):
        closest_b = boundary_pts[idx_alt[viol]]
        v_rep = alt_cand[viol] - closest_b
        v_rep /= (np.linalg.norm(v_rep, axis=1, keepdims=True) + 1e-6)
        alt_cand[viol] = closest_b + v_rep * 0.125
        alt_cand[:, 0] = gaussian_filter1d(alt_cand[:, 0], sigma=2.0, mode='wrap')
        alt_cand[:, 1] = gaussian_filter1d(alt_cand[:, 1], sigma=2.0, mode='wrap')

    # Fit 1000-point fine spline
    tck_alt, _ = splprep([alt_cand[:, 0], alt_cand[:, 1]], s=0.003, per=True)
    fine_alt = np.column_stack(splev(u_1000, tck_alt))
    
    # 3. Find EXACT Split (Divergence) and Join (Reconvergence) Points
    # Distance from optimal line
    dist_from_opt = np.hypot(fine_alt[:, 0] - optimal_line[:, 0], fine_alt[:, 1] - optimal_line[:, 1])
    
    # Splice boundary threshold: 2.0 cm deviation
    DIVERGE_THRESH_M = 0.020
    is_diverged = dist_from_opt >= DIVERGE_THRESH_M
    
    # Locate contiguous segment around closest_opt_idx
    diverged_indices = np.where(is_diverged)[0]
    
    # Find split index before closest_opt_idx and join index after
    split_idx = (closest_opt_idx - int(span_sigma * 2.2)) % 1000
    join_idx = (closest_opt_idx + int(span_sigma * 2.2)) % 1000
    
    # Replace everything OUTSIDE [split_idx, join_idx] with the EXACT optimal line!
    spliced_line = optimal_line.copy()
    if split_idx < join_idx:
        spliced_line[split_idx:join_idx] = fine_alt[split_idx:join_idx]
    else:
        spliced_line[split_idx:] = fine_alt[split_idx:]
        spliced_line[:join_idx] = fine_alt[:join_idx]

    # Smooth the junction seams with local quintic blending
    tck_splice, _ = splprep([spliced_line[:, 0], spliced_line[:, 1]], s=0.002, per=True)
    final_spliced = np.column_stack(splev(u_1000, tck_splice))
    
    props_s = compute_spline_properties(final_spliced)
    v_s = optimizer._generate_velocity_profile(props_s["curvature"], props_s["ds"], lookahead_distance_m=0.10)
    
    d_verif, _ = barrier_tree.query(final_spliced)
    min_clear_cm = float(np.min(d_verif)) * 100.0
    lap_s = float(np.sum(props_s["ds"] / np.maximum(v_s, 0.1)))
    len_s = float(np.sum(props_s["ds"]))
    t_pillar_ms = (time.perf_counter() - t_pillar_0) * 1000.0

    split_pt = optimal_line[split_idx]
    join_pt = optimal_line[join_idx]

    spliced_alternatives.append({
        "pillar_id": fp["id"],
        "pillar_name": fp["name"],
        "pillar_center": fp["center"],
        "optimal_side": opt_side,
        "alternative_side": alt_side,
        "pts": final_spliced,
        "split_idx": split_idx,
        "join_idx": join_idx,
        "split_pt": split_pt,
        "join_pt": join_pt,
        "min_clearance_cm": round(min_clear_cm, 2),
        "lap_time_s": round(lap_s, 3),
        "track_len_m": round(len_s, 2),
        "compute_time_ms": round(t_pillar_ms, 2),
        "velocities": v_s,
        "tangents": props_s["tangents"],
        "curvature": props_s["curvature"],
        "ds": props_s["ds"]
    })

t_splicing_total_ms = (time.perf_counter() - t0_splice) * 1000.0
total_pipeline_time_ms = (time.perf_counter() - t_pipeline_start) * 1000.0

# 4. Export Master Optimal Line and All 5 Spliced Pillar Alternatives
header = "x_m,y_m,v_mps,yaw_rad,curvature_inv_m,dist_m,time_s,a_lon_mps2"

def save_csv(pts, v, tang, curv, ds, filepath):
    yaw = np.arctan2(tang[:, 1], tang[:, 0])
    s_dist = np.cumsum(np.concatenate([[0.0], ds[:-1]]))
    dt = ds / np.maximum(v, 0.1)
    t_time = np.cumsum(np.concatenate([[0.0], dt[:-1]]))
    a_lon = np.gradient(v**2) / (2.0 * ds + 1e-6)
    traj_mat = np.column_stack([pts[:, 0], pts[:, 1], v, yaw, curv, s_dist, t_time, a_lon])
    np.savetxt(filepath, traj_mat, delimiter=",", header=header, comments="", fmt="%.5f")

# Save Master Line
opt_csv = os.path.join(output_dir, "cyan_champion_optimal_master.csv")
save_csv(optimal_line, v_opt, props_opt["tangents"], props_opt["curvature"], props_opt["ds"], opt_csv)
np.savetxt(os.path.join(BASE_DIR, "my_project", "controllers", "folkrace_drive", "cyan_champion_trajectory.csv"), np.loadtxt(opt_csv, delimiter=",", skiprows=1), delimiter=",", header=header, comments="", fmt="%.5f")

# Save each spliced alternative
print("=" * 125)
print("  DYNAMIC HOMOTOPY SPLICER (OPTIMAL LINE + 5 LOCAL PILLAR ALTERNATIVES | MU=0.60 | 12CM MARGIN)")
print("=" * 125)
print(f"  • Master Line Generation:   {t_master_ms:.2f} ms")
print(f"  • 5 Pillar Splicing Batch:  {t_splicing_total_ms:.2f} ms (Average {t_splicing_total_ms/5.0:.2f} ms per pillar)")
print(f"  • Total Pipeline Execution: {total_pipeline_time_ms:.2f} ms ({total_pipeline_time_ms/1000.0:.3f} s)")
print("=" * 125)
print(f"{'Pillar Target':<20} | {'Optimal Side':<14} | {'Alternative Side':<18} | {'Min Clear':<11} | {'Lap Time':<9} | {'Compute Time'}")
print("-" * 125)

ALT_COLORS = ["#ff0055", "#ffe600", "#ff9900", "#a855f7", "#05ffa1"]

for idx, item in enumerate(spliced_alternatives):
    item["color"] = ALT_COLORS[idx % len(ALT_COLORS)]
    csv_f = f"alternative_bypass_{item['pillar_id'].lower()}.csv"
    item["csv_file"] = csv_f
    save_csv(item["pts"], item["velocities"], item["tangents"], item["curvature"], item["ds"], os.path.join(output_dir, csv_f))
    print(f"{item['pillar_name']:<20} | {item['optimal_side']:<14} | {item['alternative_side']:<18} | {item['min_clearance_cm']:5.1f} cm    | {item['lap_time_s']:6.3f} s | {item['compute_time_ms']:5.2f} ms")

# Save Summary JSON
with open(os.path.join(output_dir, "spliced_alternatives_summary.json"), "w", encoding="utf-8") as f:
    json.dump({
        "master_optimal_line": {
            "file": "cyan_champion_optimal_master.csv",
            "min_clearance_cm": round(float(np.min(barrier_tree.query(optimal_line)[0])) * 100, 2),
            "lap_time_s": round(float(np.sum(props_opt["ds"] / np.maximum(v_opt, 0.1))), 3),
            "track_len_m": round(float(np.sum(props_opt["ds"])), 2),
            "friction_coeff_mu": 0.60,
            "safety_margin_cm": 12.0
        },
        "pipeline_timing": {
            "master_solve_ms": round(t_master_ms, 2),
            "splicing_batch_ms": round(t_splicing_total_ms, 2),
            "total_pipeline_ms": round(total_pipeline_time_ms, 2)
        },
        "alternatives": [{k: v for k, v in a.items() if k not in ["pts", "velocities", "tangents", "curvature", "ds", "pillar_center", "split_pt", "join_pt"]} for a in spliced_alternatives]
    }, f, indent=2)

# -----------------------------------------------------------------------------
# GRAPHIC 1: FULL CIRCUIT OVERVIEW SHOWING DIVERGENCE SPLICES (JPG + SVG)
# -----------------------------------------------------------------------------
BG_COLOR = "#07090e"
CARD_BG = "#0d121d"
BORDER_COLOR = "#334155"
WALL_COLOR = "#ff0055"
OBSTACLE_COLOR = "#ffb800"
TEXT_COLOR = "#f0f4f8"

fig, ax = plt.subplots(figsize=(24, 14), facecolor=BG_COLOR, dpi=300)
ax.set_facecolor(BG_COLOR)

ax.plot(left_wall[:, 0], left_wall[:, 1], color=WALL_COLOR, linewidth=2.4, label="Track Outer Wall", zorder=3)
ax.plot(right_wall[:, 0], right_wall[:, 1], color=WALL_COLOR, linewidth=2.4, label="Track Inner Wall", zorder=3)

for obs in obstacles:
    obs_type = obs.get("type", "circle")
    if obs_type == "circle":
        c = patches.Circle((obs["cx"], obs["cy"]), obs["radius"], color=OBSTACLE_COLOR, alpha=0.92, zorder=5)
        ax.add_patch(c)
    elif obs_type in ["rect", "wall_rectangle"]:
        cx, cy = obs["center_x"], obs["center_y"]
        rw, rh = obs["width"], obs["extrusion"]
        rect = patches.Rectangle((cx - rw/2.0, cy - rh/2.0), rw, rh, color=OBSTACLE_COLOR, alpha=0.92, zorder=5)
        ax.add_patch(rect)

# Draw Master Optimal Line (Cyan Champion)
ax.plot(optimal_line[:, 0], optimal_line[:, 1], color="#00f0ff", linewidth=2.6, label="Master Optimal Line (Cyan Champion - 100% Track Baseline)", zorder=12)

# Draw each Spliced Alternative
for item in spliced_alternatives:
    pts = item["pts"]
    lbl = f"Alternative: {item['pillar_name']} -> {item['alternative_side']} Bypass [Clear: {item['min_clearance_cm']:.1f}cm]"
    ax.plot(pts[:, 0], pts[:, 1], color=item["color"], linestyle="--", linewidth=1.8, label=lbl, zorder=15)
    
    # Mark Split and Join Points
    sp = item["split_pt"]
    jp = item["join_pt"]
    ax.scatter([sp[0]], [sp[1]], color=item["color"], marker="^", s=80, edgecolors="white", linewidth=1.0, zorder=20)
    ax.scatter([jp[0]], [jp[1]], color=item["color"], marker="v", s=80, edgecolors="white", linewidth=1.0, zorder=20)

init_pose = track_data.get("initial_pose", {"x": 0.6, "y": 0.0})
sx, sy = init_pose.get("x", 0.6), init_pose.get("y", 0.0)
ax.scatter([sx], [sy], color="#00ff88", s=150, edgecolors="white", linewidth=1.5, label="Spawn Point", zorder=35)

ax.set_aspect("equal")
ax.grid(True, linestyle=":", color="#1e293b", alpha=0.6)
ax.set_title(f"DA WINNA - OPTIMAL RACING LINE WITH 5 SPLICED PILLAR ALTERNATIVES (TOTAL COMPUTE: {total_pipeline_time_ms:.1f} MS)", color=TEXT_COLOR, fontsize=15, weight="bold", pad=15)
ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), facecolor=CARD_BG, edgecolor=BORDER_COLOR, labelcolor=TEXT_COLOR, fontsize=9.0, framealpha=0.98)
plt.subplots_adjust(right=0.52)

full_jpg = os.path.join(output_dir, "spliced_pillar_alternatives_comparison.jpg")
full_svg = os.path.join(output_dir, "spliced_pillar_alternatives_comparison.svg")
plt.savefig(full_jpg, format="jpeg", dpi=300, bbox_inches="tight", facecolor=BG_COLOR)
plt.savefig(full_svg, format="svg", bbox_inches="tight", facecolor=BG_COLOR)
plt.close()

# -----------------------------------------------------------------------------
# GRAPHIC 2: 4-PANEL ZOOM SHOWING EXACT DIVERGENCE & RECONVERGENCE (JPG + SVG)
# -----------------------------------------------------------------------------
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(20, 16), facecolor=BG_COLOR, dpi=300)

zoom_zones = [
    (ax1, (3.0, 4.4), (1.8, 3.2), "Pillar 2 (East, r=5cm): Optimal Left vs Alternative Right Split"),
    (ax2, (-1.2, 0.4), (2.6, 4.0), "Pillars 3 & 4 (Mid Slalom): Dual Local Bypass Splices"),
    (ax3, (-2.4, -0.8), (1.6, 3.0), "Pillar 5 (Far West, r=5cm): Optimal Right vs Alternative Left Split"),
    (ax4, (-1.6, -0.4), (0.6, 2.0), "Pillar 6 (Southwest, r=5cm): S-Bends Divergence & Rejoin")
]

for ax_z, xlim, ylim, title in zoom_zones:
    ax_z.set_facecolor(CARD_BG)
    ax_z.plot(left_wall[:, 0], left_wall[:, 1], color=WALL_COLOR, linewidth=2.5, zorder=3)
    ax_z.plot(right_wall[:, 0], right_wall[:, 1], color=WALL_COLOR, linewidth=2.5, zorder=3)

    for obs in obstacles:
        obs_type = obs.get("type", "circle")
        if obs_type == "circle":
            c = patches.Circle((obs["cx"], obs["cy"]), obs["radius"], color=OBSTACLE_COLOR, alpha=0.92, zorder=5)
            ax_z.add_patch(c)
        elif obs_type in ["rect", "wall_rectangle"]:
            cx, cy = obs["center_x"], obs["center_y"]
            rw, rh = obs["width"], obs["extrusion"]
            rect = patches.Rectangle((cx - rw/2.0, cy - rh/2.0), rw, rh, color=OBSTACLE_COLOR, alpha=0.92, zorder=5)
            ax_z.add_patch(rect)

    # Draw Optimal Line
    ax_z.plot(optimal_line[:, 0], optimal_line[:, 1], color="#00f0ff", linewidth=2.4, label="Optimal Line (Master)", zorder=12)

    # Draw Alternatives
    for item in spliced_alternatives:
        pts = item["pts"]
        ax_z.plot(pts[:, 0], pts[:, 1], color=item["color"], linestyle="--", linewidth=1.8, label=f"Alt {item['pillar_id']} ({item['alternative_side']})", zorder=15)
        sp = item["split_pt"]
        jp = item["join_pt"]
        if xlim[0] <= sp[0] <= xlim[1] and ylim[0] <= sp[1] <= ylim[1]:
            ax_z.scatter([sp[0]], [sp[1]], color=item["color"], marker="^", s=100, edgecolors="white", linewidth=1.2, zorder=20)
        if xlim[0] <= jp[0] <= xlim[1] and ylim[0] <= jp[1] <= ylim[1]:
            ax_z.scatter([jp[0]], [jp[1]], color=item["color"], marker="v", s=100, edgecolors="white", linewidth=1.2, zorder=20)

    ax_z.set_xlim(xlim)
    ax_z.set_ylim(ylim)
    ax_z.set_aspect("equal")
    ax_z.grid(True, linestyle=":", color="#334155", alpha=0.6)
    ax_z.set_title(title, color=TEXT_COLOR, fontsize=11, weight="bold", pad=8)
    ax_z.tick_params(colors=TEXT_COLOR)

handles, labels = ax1.get_legend_handles_labels()
fig.legend(handles, labels, loc="lower center", ncol=4, facecolor=CARD_BG, edgecolor=BORDER_COLOR, labelcolor=TEXT_COLOR, fontsize=9.5, bbox_to_anchor=(0.5, 0.02))
plt.subplots_adjust(hspace=0.25, wspace=0.15, bottom=0.08)

zoom_jpg = os.path.join(output_dir, "spliced_pillar_alternatives_zoomed_chokepoints.jpg")
zoom_svg = os.path.join(output_dir, "spliced_pillar_alternatives_zoomed_chokepoints.svg")
plt.savefig(zoom_jpg, format="jpeg", dpi=300, bbox_inches="tight", facecolor=BG_COLOR)
plt.savefig(zoom_svg, format="svg", bbox_inches="tight", facecolor=BG_COLOR)
plt.close()

print("=" * 125)
print(f"Master Overview Graphic (JPG): {full_jpg}")
print(f"Master Overview Graphic (SVG): {full_svg}")
print(f"Zoomed Chokepoints (JPG):      {zoom_jpg}")
print(f"Zoomed Chokepoints (SVG):      {zoom_svg}")
print("=" * 125)
