#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - GOOGLE CARTOGRAPHER 2cm OCCUPANCY GRID SIMULATOR (45° ROTATED)
================================================================================
Simulates exact Google Cartographer Log-Odds 2D Occupancy Grid Mapping:
  - 45-degree rotated track (creates realistic staircase aliasing on straight walls)
  - 2cm x 2cm discrete cell resolution (res = 0.02m)
  - Bresenham ray carving (Free space = 0, Occupied barrier = 100, Unknown = -1)
  - Full 1-lap LiDAR sweep integration with sensor noise & vehicle dynamics
  - Exports ROS2 standard .pgm + .yaml and numpy grid array
================================================================================
"""

import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEBOTS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)
sys.path.insert(0, WEBOTS_DIR)

from racing_engine.track_generator import get_track_7_multi_bottleneck_slalom, get_track_4_switchback_arena
from racing_engine.centerline_finder import find_centerline_voronoi
from exact_2d_lidar_simulator import cast_lidar_rays_vectorized

OUT_SLAM_DIR = os.path.join(BASE_DIR, "racing_results", "cartographer_slam_2cm")
os.makedirs(OUT_SLAM_DIR, exist_ok=True)

def rotate_points_2d(pts, angle_rad, origin=np.array([0.0, 0.0])):
    """Rotates an array of (N, 2) points by angle_rad around origin."""
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    R = np.array([[c, -s], [s, c]])
    return (pts - origin) @ R.T + origin

def bresenham_ray_cells(x0, y0, x1, y1):
    """Generates all discrete 2D grid cell indices along a line segment."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 > x0 else -1
    sy = 1 if y1 > y0 else -1
    
    cells = []
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            cells.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            cells.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    cells.append((x1, y1))
    return cells

def simulate_cartographer_slam(track_data, yellow_csv_path, angle_deg=45.0, resolution=0.02, num_scans=90, num_rays=450, noise_std=0.010):
    """
    Builds a 2cm Occupancy Grid Map from LiDAR sweeps on a 45-degree rotated track.
    """
    rot_rad = np.radians(angle_deg)
    
    # 1. Rotate Track Geometry by 45 degrees
    raw_left = np.array(track_data["left_wall"])
    raw_right = np.array(track_data["right_wall"])
    
    center_anchor = np.mean(raw_left, axis=0)
    left_wall = rotate_points_2d(raw_left, rot_rad, center_anchor)
    right_wall = rotate_points_2d(raw_right, rot_rad, center_anchor)
    
    obstacles = []
    seg_list = []
    for i in range(len(left_wall)):
        seg_list.append([left_wall[i], left_wall[(i + 1) % len(left_wall)]])
    for i in range(len(right_wall)):
        seg_list.append([right_wall[i], right_wall[(i + 1) % len(right_wall)]])
        
    for obs in track_data.get("obstacles", []):
        if obs.get("type") == "circle":
            c_orig = np.array([obs["cx"], obs["cy"]])
            c_rot = rotate_points_2d(c_orig[None, :], rot_rad, center_anchor)[0]
            obstacles.append({"type": "circle", "cx": c_rot[0], "cy": c_rot[1], "radius": obs["radius"]})
        elif obs.get("type") == "wall_rectangle":
            cx, cy = obs["center_x"], obs["center_y"]
            w, ext = obs["width"], obs["extrusion"]
            rect_raw = np.array([
                [cx - w/2, cy - ext/2],
                [cx + w/2, cy - ext/2],
                [cx + w/2, cy + ext/2],
                [cx - w/2, cy + ext/2]
            ])
            rect_rot = rotate_points_2d(rect_raw, rot_rad, center_anchor)
            obstacles.append({"type": "wall_rectangle", "polygon": rect_rot, "center_x": np.mean(rect_rot[:, 0]), "center_y": np.mean(rect_rot[:, 1]), "width": w, "extrusion": ext})
            for j in range(4):
                seg_list.append([rect_rot[j], rect_rot[(j + 1) % 4]])
                
    segments = np.array(seg_list)
    circles = [(np.array([o["cx"], o["cy"]]), o["radius"]) for o in obstacles if o.get("type") == "circle"]
    
    # 2. Rotate Yellow Trajectory
    raw_traj = np.loadtxt(yellow_csv_path, delimiter=",", skiprows=1)
    pts_rot = rotate_points_2d(raw_traj[:, :2], rot_rad, center_anchor)
    
    # 3. Setup 2cm Discrete Grid Bounds
    all_bounds = np.vstack([left_wall, right_wall])
    min_x, min_y = np.min(all_bounds, axis=0) - 1.50
    max_x, max_y = np.max(all_bounds, axis=0) + 1.50
    
    grid_w = int(np.ceil((max_x - min_x) / resolution))
    grid_h = int(np.ceil((max_y - min_y) / resolution))
    
    # Log-Odds Probability Grid (0.0 = Unknown)
    # Positive = Occupied, Negative = Free
    log_odds_grid = np.zeros((grid_h, grid_w), dtype=np.float32)
    
    N = len(pts_rot)
    scan_indices = np.linspace(0, N - 1, num_scans, dtype=int)
    ray_rel_angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False)
    
    print(f"[INFO] Simulating Cartographer SLAM on {angle_deg}° rotated track...")
    print(f"[INFO] Grid size: {grid_w} x {grid_h} cells ({grid_w*resolution:.2f}m x {grid_h*resolution:.2f}m @ {resolution*100:.0f}cm resolution)")
    
    LO_FREE = -0.45   # Log-odds decrement for free space
    LO_OCCUPIED = 0.90 # Log-odds increment for occupied barrier
    
    for scan_step, idx in enumerate(scan_indices):
        rx, ry = pts_rot[idx]
        next_idx = (idx + 1) % N
        heading = np.arctan2(pts_rot[next_idx, 1] - ry, pts_rot[next_idx, 0] - rx)
        
        global_headings = heading + ray_rel_angles
        origin = np.array([rx, ry])
        
        # Raycast all 450 beams
        ray_dirs = np.column_stack([np.cos(global_headings), np.sin(global_headings)])
        
        # Find exact intersections
        p1 = segments[:, 0, :]
        p2 = segments[:, 1, :]
        v2 = p2 - p1
        v1 = origin[None, :] - p1
        v3 = np.column_stack([-ray_dirs[:, 1], ray_dirs[:, 0]])
        dot = np.dot(v2, v3.T)
        cross_num = (v2[:, 0, None] * v1[:, 1, None] - v2[:, 1, None] * v1[:, 0, None])
        t2_num = np.dot(v1, v3.T)
        valid_dot = np.abs(dot) > 1e-8
        t1 = np.where(valid_dot, cross_num / dot, np.inf)
        t2 = np.where(valid_dot, t2_num / dot, -1.0)
        valid_mask = (t1 >= 0.05) & (t1 <= 4.0) & (t2 >= 0.0) & (t2 <= 1.0)
        t1_filtered = np.where(valid_mask, t1, np.inf)
        min_dists = np.min(t1_filtered, axis=0)
        
        for c_pos, r_val in circles:
            oc = origin - c_pos
            b = 2.0 * np.dot(ray_dirs, oc)
            c = np.dot(oc, oc) - r_val * r_val
            disc = b * b - 4.0 * c
            valid_disc = disc >= 0.0
            sq = np.sqrt(np.maximum(0.0, disc))
            t_c1 = (-b - sq) * 0.5
            c_mask1 = valid_disc & (t_c1 >= 0.05) & (t_c1 <= 4.0)
            t_circ = np.where(c_mask1, t_c1, np.inf)
            min_dists = np.minimum(min_dists, t_circ)
            
        robot_gx = int((rx - min_x) / resolution)
        robot_gy = int((ry - min_y) / resolution)
        
        for k in range(num_rays):
            d = min_dists[k]
            if d < 4.0:
                if noise_std > 0:
                    d += np.random.normal(0, noise_std)
                hit_x = rx + d * ray_dirs[k, 0]
                hit_y = ry + d * ray_dirs[k, 1]
                
                hit_gx = int((hit_x - min_x) / resolution)
                hit_gy = int((hit_y - min_y) / resolution)
                
                if 0 <= hit_gx < grid_w and 0 <= hit_gy < grid_h:
                    # Trace Bresenham ray cells (Free space)
                    ray_cells = bresenham_ray_cells(robot_gx, robot_gy, hit_gx, hit_gy)
                    for cx, cy in ray_cells[:-1]:
                        if 0 <= cx < grid_w and 0 <= cy < grid_h:
                            log_odds_grid[cy, cx] = max(log_odds_grid[cy, cx] + LO_FREE, -3.5)
                    # Occupied hit cell
                    log_odds_grid[hit_gy, hit_gx] = min(log_odds_grid[hit_gy, hit_gx] + LO_OCCUPIED, 4.0)

    # 4. Convert Log-Odds to Standard ROS2 Occupancy Grid Format
    # 0 = Free, 100 = Occupied, -1 = Unknown
    occupancy_grid = np.full((grid_h, grid_w), -1, dtype=np.int8)
    occupancy_grid[log_odds_grid < -0.3] = 0   # Free space
    occupancy_grid[log_odds_grid > 0.6] = 100  # Occupied barrier
    
    # 5. Extract 2D Coordinates of Occupied 2cm Pixels
    occ_y, occ_x = np.where(occupancy_grid == 100)
    occupied_world_pts = np.column_stack([
        min_x + occ_x * resolution + resolution / 2.0,
        min_y + occ_y * resolution + resolution / 2.0
    ])
    
    print(f"[OK] SLAM Grid Generated: {len(occupied_world_pts)} occupied 2cm barrier boxes.")
    print(f"[OK] Free corridor space: {np.sum(occupancy_grid == 0)} cells.")
    
    return {
        "occupancy_grid": occupancy_grid,
        "occupied_pts": occupied_world_pts,
        "left_wall": left_wall,
        "right_wall": right_wall,
        "obstacles": obstacles,
        "yellow_pts": pts_rot,
        "min_x": min_x,
        "min_y": min_y,
        "resolution": resolution,
        "grid_w": grid_w,
        "grid_h": grid_h
    }

def visualize_and_save_cartographer_grid(slam_res, track_name, prefix):
    grid = slam_res["occupancy_grid"]
    occ_pts = slam_res["occupied_pts"]
    lw = slam_res["left_wall"]
    rw = slam_res["right_wall"]
    yellow_pts = slam_res["yellow_pts"]
    res = slam_res["resolution"]
    
    # 1. Save standard ROS2 .yaml + .pgm
    pgm_img = np.zeros_like(grid, dtype=np.uint8)
    pgm_img[grid == -1] = 205 # Unknown gray
    pgm_img[grid == 0] = 254  # Free white
    pgm_img[grid == 100] = 0  # Occupied black
    
    # Flip vertically for standard image / ROS coordinate origin (lower-left)
    pgm_flipped = np.flipud(pgm_img)
    pgm_path = os.path.join(OUT_SLAM_DIR, f"{prefix}_cartographer_2cm_map.pgm")
    
    # Save standard Netpbm P5 Binary PGM
    h_pgm, w_pgm = pgm_flipped.shape
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{w_pgm} {h_pgm}\n255\n".encode("ascii"))
        f.write(pgm_flipped.tobytes())
    
    yaml_content = f"""image: {prefix}_cartographer_2cm_map.pgm
mode: trinary
resolution: {res:.3f}
origin: [{slam_res['min_x']:.3f}, {slam_res['min_y']:.3f}, 0.000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    yaml_path = os.path.join(OUT_SLAM_DIR, f"{prefix}_cartographer_2cm_map.yaml")
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
        
    np.save(os.path.join(OUT_SLAM_DIR, f"{prefix}_occupied_cells_2cm.npy"), occ_pts)
    
    # 2. Render High-Resolution Comparison Visualization
    fig, ax = plt.subplots(figsize=(18, 14), facecolor="#080c14")
    ax.set_facecolor("#080c14")
    
    extent = [
        slam_res["min_x"],
        slam_res["min_x"] + slam_res["grid_w"] * res,
        slam_res["min_y"],
        slam_res["min_y"] + slam_res["grid_h"] * res
    ]
    
    # Render discrete occupancy grid bitmap
    # Colormap: Unknown=Dark Gray (#151b28), Free Space=Near Black (#0c101a), Occupied=Bright Cyan/White (#00f0ff)
    cmap = matplotlib.colors.ListedColormap(['#151b28', '#070a12', '#00f0ff'])
    bounds_map = [-1.5, -0.5, 50, 150]
    norm = matplotlib.colors.BoundaryNorm(bounds_map, cmap.N)
    
    ax.imshow(grid, origin='lower', extent=extent, cmap=cmap, norm=norm, interpolation='nearest')
    
    # Plot Ground-Truth Rotated CAD Boundaries (subtle dashed outline to show aliased rasterization)
    lw_w = np.vstack([lw, lw[:1]])
    rw_w = np.vstack([rw, rw[:1]])
    ax.plot(lw_w[:, 0], lw_w[:, 1], color="#f43f5e", linewidth=1.5, linestyle="--", alpha=0.7, label="Ground Truth Left Wall (45° Rotated)")
    ax.plot(rw_w[:, 0], rw_w[:, 1], color="#38bdf8", linewidth=1.5, linestyle="--", alpha=0.7, label="Ground Truth Right Wall (45° Rotated)")
    
    for obs in slam_res["obstacles"]:
        circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], color="#fbbf24", fill=False, linewidth=1.5, linestyle="--", alpha=0.9)
        ax.add_patch(circle)
    ax.plot([], [], color="#fbbf24", linewidth=1.5, linestyle="--", label="Ground Truth Obstacle Pillars (45° Rotated)")
    
    # Plot Yellow Racing Line
    y_w = np.vstack([yellow_pts, yellow_pts[:1]])
    ax.plot(y_w[:, 0], y_w[:, 1], color="#ffd700", linewidth=2.8, label="Optimal Yellow Racing Line (45° Rotated)")
    ax.scatter([yellow_pts[0, 0]], [yellow_pts[0, 1]], color="#ffffff", s=130, edgecolors="#ffd700", linewidth=2, label="Robot Start", zorder=7)
    
    ax.set_title(f"{track_name} - Google Cartographer 2cm 2D Occupancy Grid Map (45° Rotated Aliasing)",
                 color="#f8fafc", fontsize=13, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.15, color="#ffffff")
    ax.tick_params(colors="#94a3b8")
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748",
              labelcolor="#f8fafc", framealpha=1.0, fontsize=9.5, borderpad=1.0, labelspacing=0.8)
    
    png_path = os.path.join(OUT_SLAM_DIR, f"{prefix}_cartographer_2cm_visualization.png")
    plt.savefig(png_path, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    
    print(f"[OK] Saved ROS2 PGM Map: {pgm_path}")
    print(f"[OK] Saved ROS2 YAML Meta: {yaml_path}")
    print(f"[OK] Saved High-Res PNG: {png_path}")

def main():
    print("=" * 100)
    print("   GOOGLE CARTOGRAPHER 2cm OCCUPANCY GRID SIMULATION (45° ROTATED)")
    print("=" * 100)

    # 1. Track 7: Slalom Speedway (45° Rotated)
    track7 = get_track_7_multi_bottleneck_slalom()
    yellow7_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
    slam7 = simulate_cartographer_slam(track7, yellow7_csv, angle_deg=45.0, resolution=0.02, num_scans=90, num_rays=450, noise_std=0.010)
    visualize_and_save_cartographer_grid(slam7, "Track 7 (Slalom Speedway)", "track7_45deg")

    # 2. Track 4: Technical Switchback (45° Rotated)
    track4 = get_track_4_switchback_arena()
    yellow4_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "04_track_4_yellow_trajectory.csv")
    slam4 = simulate_cartographer_slam(track4, yellow4_csv, angle_deg=45.0, resolution=0.02, num_scans=90, num_rays=450, noise_std=0.010)
    visualize_and_save_cartographer_grid(slam4, "Track 4 (Technical Switchback)", "track4_45deg")

    print("\n" + "=" * 100)

if __name__ == "__main__":
    main()
