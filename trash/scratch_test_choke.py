import os, sys
import numpy as np
BASE_DIR = r"c:\Users\nurmi\Documents\Da Winna"
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import build_raw_track_corridor

seq1 = ['S', 'S', 'S', 'S', 'L', 'S', 'S', 'L', 'S', 'S', 'S', 'S', 'L', 'S', 'S', 'L']
track1 = build_raw_track_corridor(seq1)
track1["name"] = "Track 1: Alternating Wall Choke Speedway (16 Tiles)"
track1["obstacles"] = [
    {"type": "wall_rectangle", "name": "Main Straight Left Box", "side": "left", "center_x": 1.40, "center_y": 0.375, "width": 0.35, "extrusion": 0.25},
    {"type": "wall_rectangle", "name": "Main Straight Right Box", "side": "right", "center_x": 2.80, "center_y": -0.375, "width": 0.35, "extrusion": 0.25},
    {"type": "circle", "name": "Turn 2 Apex Pillar", "cx": 4.20, "cy": 1.80, "radius": 0.11},
    {"type": "wall_rectangle", "name": "Backstretch Left Box", "side": "left", "center_x": 2.80, "center_y": 3.60 - 0.375, "width": 0.35, "extrusion": 0.24},
    {"type": "circle", "name": "Return Stretch Pillar", "cx": 1.40, "cy": 3.60, "radius": 0.11}
]

from simulate_cartographer_2cm_grid import simulate_cartographer_slam
yellow_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
slam_res = simulate_cartographer_slam(track1, yellow_csv, angle_deg=45.0, resolution=0.02)
print("Obstacles in slam_res:", len(slam_res["obstacles"]))
for obs in slam_res["obstacles"]:
    print("  ->", obs.get("type"), obs.get("center_x", obs.get("cx")), obs.get("center_y", obs.get("cy")))
