import os, sys
import numpy as np
BASE_DIR = r"c:\Users\nurmi\Documents\Da Winna"
sys.path.insert(0, BASE_DIR)
from racing_engine.track_generator import get_track_4_switchback_arena
from simulate_cartographer_2cm_grid import simulate_cartographer_slam, rotate_points_2d
import scipy.spatial as spatial
import cv2

track4 = get_track_4_switchback_arena()
yellow4_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "04_track_4_yellow_trajectory.csv")
slam_res = simulate_cartographer_slam(track4, yellow4_csv, angle_deg=45.0, resolution=0.02)

rot_rad = np.radians(45.0)
raw_left = np.array(track4["left_wall"])
center_anchor = np.mean(raw_left, axis=0)
cad_traj = np.loadtxt(yellow4_csv, delimiter=",", skiprows=1)
cad_pts_rot = rotate_points_2d(cad_traj[:, :2], rot_rad, center_anchor)

obs_rot = slam_res["obstacles"][0]
print("Track 4 Obstacle center:", obs_rot["cx"], obs_rot["cy"], "radius:", obs_rot["radius"])

# Let's inspect Voronoi vertices within 1.0m of the obstacle
stride = max(1, len(slam_res["occupied_pts"]) // 1200)
sample_occ = slam_res["occupied_pts"][::stride]
vor = spatial.Voronoi(sample_occ)

res = slam_res["resolution"]
min_x, min_y = slam_res["min_x"], slam_res["min_y"]
h, w = slam_res["occupancy_grid"].shape

from racing_engine.slam_complete_pipeline_generator import preprocess_and_smooth_slam_grid
drivable, dist_map = preprocess_and_smooth_slam_grid(slam_res["occupancy_grid"], res)

valid_vertices = {}
for i, v in enumerate(vor.vertices):
    gx = int((v[0] - min_x) / res)
    gy = int((v[1] - min_y) / res)
    if 0 <= gx < w and 0 <= gy < h:
        if drivable[gy, gx] == 255 and dist_map[gy, gx] >= 0.14:
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

print("Total pruned vertices:", len(adj))
branching_nodes = [n for n in adj if len(adj[n]) > 2]
print("Branching nodes (deg > 2):", len(branching_nodes))
for b in branching_nodes:
    v = valid_vertices[b]
    d_obs = np.hypot(v[0] - obs_rot["cx"], v[1] - obs_rot["cy"])
    print(f"Node {b} at ({v[0]:.2f}, {v[1]:.2f}), dist to obs: {d_obs:.2f}m, neighbors: {len(adj[b])}")
    for nbr in adj[b]:
        vn = valid_vertices[nbr]
        d_nbr_obs = np.hypot(vn[0] - obs_rot["cx"], vn[1] - obs_rot["cy"])
        # Cross product / side of obstacle:
        # vector from obs to node vs vector to neighbor
        vec_obs_n = vn - np.array([obs_rot["cx"], obs_rot["cy"]])
        print(f"   -> Nbr {nbr} at ({vn[0]:.2f}, {vn[1]:.2f}), dist to obs: {d_nbr_obs:.2f}m, vec from obs: ({vec_obs_n[0]:.2f}, {vec_obs_n[1]:.2f})")
