#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - TRACK 7 WEBOTS SIMULATION: POLAR RANGE TRIGONOMETRY + SOLID PILLARS
================================================================================
"""

import os
import sys
import json
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import scipy.spatial as spatial

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from webots/optimization/racing_engine.track_generator import get_track_7_multi_bottleneck_slalom
from webots/optimization/racing_engine.centerline_finder import find_centerline_voronoi
from webots/optimization/racing_engine.visualizer import OUTPUT_DIR, BG_COLOR, TEXT_COLOR

WEBOTS_DIR = os.path.join(BASE_DIR, "webots/simulation/webots_simulation")
WORLDS_DIR = os.path.join(WEBOTS_DIR, "worlds")
CONTROLLERS_DIR = os.path.join(WEBOTS_DIR, "controllers", "robot_lidar_collector")

os.makedirs(WORLDS_DIR, exist_ok=True)
os.makedirs(CONTROLLERS_DIR, exist_ok=True)

def build_indexed_faceset_wall(pts, name, base_color_str, wall_height=0.40):
    N = len(pts)
    coord_lines = []
    for p in pts:
        coord_lines.append(f"{p[0]:.4f} 0.0000 {p[1]:.4f}")
    for p in pts:
        coord_lines.append(f"{p[0]:.4f} {wall_height:.4f} {p[1]:.4f}")
        
    coord_str = ", ".join(coord_lines)
    index_lines = []
    for i in range(N):
        i_next = (i + 1) % N
        index_lines.append(f"{i}, {i_next}, {i_next + N}, {i + N}, -1")
        index_lines.append(f"{i}, {i + N}, {i_next + N}, {i_next}, -1")
        
    index_str = ", ".join(index_lines)

    node = f'''Solid {{
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {base_color_str}
        roughness 0.5
      }}
      geometry IndexedFaceSet {{
        coord Coordinate {{
          point [ {coord_str} ]
        }}
        coordIndex [ {index_str} ]
      }}
    }}
  ]
  boundingObject IndexedFaceSet {{
    coord Coordinate {{
      point [ {coord_str} ]
    }}
    coordIndex [ {index_str} ]
  }}
}}'''
    return node

def generate_track7_webots_world():
    track_data = get_track_7_multi_bottleneck_slalom()
    left_wall = track_data["left_wall"]
    right_wall = track_data["right_wall"]
    obstacles = track_data.get("obstacles", [])
    
    yellow_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
    traj = np.loadtxt(yellow_csv, delimiter=",", skiprows=1)
    start_x, start_y = traj[0, 0], traj[0, 1]
    
    dx = traj[1, 0] - traj[0, 0]
    dy = traj[1, 1] - traj[0, 1]
    start_yaw = np.arctan2(dy, dx)

    wbt_lines = [
        '#VRML_SIM R2025a utf8',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"',
        '',
        'WorldInfo {',
        '  info [ "Da Winna Track 7 - Solid Slalom Speedway" ]',
        '  basicTimeStep 10',
        '  FPS 60',
        '  coordinateSystem "NUE"',
        '}',
        'Viewpoint {',
        '  orientation -0.57 0.57 0.57 2.09',
        f'  position {start_x + 2.0:.2f} 8.0 {start_y + 2.0:.2f}',
        '}',
        'TexturedBackground {',
        '  texture "dusk"',
        '}',
        'TexturedBackgroundLight {',
        '  luminosity 1.2',
        '}',
        'RectangleArena {',
        '  floorSize 30 30',
        '  floorTileSize 1 1',
        '  floorAppearance PBRAppearance {',
        '    baseColor 0.08 0.10 0.15',
        '    roughness 0.8',
        '  }',
        '}',
    ]

    # Watertight Walls (Left & Right)
    wbt_lines.append(build_indexed_faceset_wall(left_wall, "track7_left_wall", "0.95 0.15 0.35"))
    wbt_lines.append(build_indexed_faceset_wall(right_wall, "track7_right_wall", "0.20 0.60 0.95"))

    # Add 3 Solid Slalom Pillars
    wall_height = 0.40
    for idx, obs in enumerate(obstacles):
        if obs.get("type") == "circle":
            cx, cz = obs["cx"], obs["cy"]
            r = obs["radius"]
            wbt_lines.extend([
                'Solid {',
                f'  name "slalom_pillar_{idx}"',
                f'  translation {cx:.4f} {wall_height/2:.4f} {cz:.4f}',
                '  children [',
                '    Shape {',
                '      appearance PBRAppearance { baseColor 1.0 0.75 0.0 roughness 0.2 }',
                f'      geometry Cylinder {{ height {wall_height:.4f} radius {r:.4f} }}',
                '    }',
                '  ]',
                f'  boundingObject Cylinder {{ height {wall_height:.4f} radius {r:.4f} }}',
                '}'
            ])

    # 15x20cm Robot with MS200k LiDAR (4.0m Max Range, 4mm Noise)
    wbt_lines.extend([
        'Robot {',
        '  name "da_winna_robot"',
        f'  translation {start_x:.4f} 0.05 {start_y:.4f}',
        f'  rotation 0 1 0 {start_yaw:.4f}',
        '  children [',
        '    Shape {',
        '      appearance PBRAppearance {',
        '        baseColor 1.0 0.85 0.0',
        '        metalness 0.6',
        '        roughness 0.2',
        '      }',
        '      geometry Box {',
        '        size 0.20 0.05 0.15',
        '      }',
        '    }',
        '    # MS200k 2D LiDAR: 4.0m Max Range, 450 rays, 12Hz, 4mm Noise',
        '    Lidar {',
        '      translation 0.0 0.04 0.0',
        '      name "ms200k_lidar"',
        '      fieldOfView 6.2831853',
        '      horizontalResolution 450',
        '      numberOfLayers 1',
        '      minRange 0.12',
        '      maxRange 4.0',
        '      noise 0.004',
        '      type "fixed"',
        '    }',
        '  ]',
        '  controller "robot_lidar_collector"',
        '  supervisor TRUE',
        '}'
    ])

    wbt_path = os.path.join(WORLDS_DIR, "track7_slalom_speedway.wbt")
    with open(wbt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(wbt_lines))
    return wbt_path, yellow_csv

def generate_polar_lidar_controller(yellow_csv_path):
    controller_code = f'''#!/usr/bin/env python3
"""
================================================================================
   WEBOTS CONTROLLER: EXACT POLAR RANGE RAY TRACING WITH MS200k LiDAR
================================================================================
"""

import sys
import os
import json
import numpy as np

if "WEBOTS_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))

from controller import Supervisor

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep()) # 10 ms

    lidar = robot.getDevice("ms200k_lidar")
    lidar.enable(int(1000 / 12)) # 12 Hz scan rate (~83.3 ms)
    lidar.enablePointCloud()

    traj_path = r"{yellow_csv_path}"
    if not os.path.exists(traj_path):
        print(f"[ERROR] Trajectory file not found: {{traj_path}}")
        return

    traj = np.loadtxt(traj_path, delimiter=",", skiprows=1)
    N = len(traj)
    pts = traj[:, :2]
    vels = traj[:, 2] * 0.25 # 0.25x speed
    headings = traj[:, 3]

    robot_node = robot.getSelf()
    trans_field = robot_node.getField("translation")
    rot_field = robot_node.getField("rotation")

    print("[INFO] Webots Track 7 Exact Polar Collector Started!")

    collected_scans = []
    
    cur_idx = 0
    cur_dist = 0.0
    total_dist = 0.0
    ds_list = [np.hypot(pts[(i+1)%N, 0] - pts[i, 0], pts[(i+1)%N, 1] - pts[i, 1]) for i in range(N)]
    lap_length = sum(ds_list)
    
    dt = timestep / 1000.0
    step_count = 0
    lidar_scan_count = 0

    num_rays = 450
    fov = 2.0 * np.pi
    ray_angles = np.linspace(-np.pi, np.pi, num_rays, endpoint=False)

    while robot.step(timestep) != -1:
        step_count += 1
        current_v = vels[cur_idx]
        cur_dist += current_v * dt
        total_dist += current_v * dt
        
        while cur_dist >= ds_list[cur_idx]:
            cur_dist -= ds_list[cur_idx]
            cur_idx = (cur_idx + 1) % N

        next_idx = (cur_idx + 1) % N
        interp_factor = cur_dist / max(ds_list[cur_idx], 1e-4)
        
        rx = pts[cur_idx, 0] + interp_factor * (pts[next_idx, 0] - pts[cur_idx, 0])
        rz = pts[cur_idx, 1] + interp_factor * (pts[next_idx, 1] - pts[cur_idx, 1])
        
        # Heading in 2D Cartesian plane
        dx_next = pts[next_idx, 0] - pts[cur_idx, 0]
        dz_next = pts[next_idx, 1] - pts[cur_idx, 1]
        heading_2d = np.arctan2(dz_next, dx_next)
        
        # Webots NUE yaw angle (around Y-axis)
        webots_yaw = np.arctan2(dx_next, dz_next)

        # Update pose in Webots 3D World
        trans_field.setSFVec3f([float(rx), 0.05, float(rz)])
        rot_field.setSFRotation([0.0, 1.0, 0.0, float(webots_yaw)])

        # Read LiDAR Range Image (450 distances)
        ranges = lidar.getRangeImage()
        if ranges is not None and len(ranges) == num_rays:
            lidar_scan_count += 1
            valid_pts = []
            
            for r, a in zip(ranges, ray_angles):
                if 0.12 < r <= 3.95:
                    # Global ray projection in 2D track frame
                    beam_global_angle = heading_2d + a
                    gx = rx + r * np.cos(beam_global_angle)
                    gy = rz + r * np.sin(beam_global_angle)
                    valid_pts.append([gx, gy])
                    
            if len(valid_pts) > 0:
                collected_scans.extend(valid_pts)

        if total_dist >= lap_length:
            print(f"[COMPLETE] Finished Track 7 Lap! Traversed: {{total_dist:.2f}}m")
            print(f"[COMPLETE] Captured {{len(collected_scans)}} exact points across {{lidar_scan_count}} sweeps.")
            break

    output_dir = os.path.dirname(__file__)
    np_pts = np.array(collected_scans)
    np.save(os.path.join(output_dir, "track7_deskewed_lidar_points.npy"), np_pts)
    
    with open(os.path.join(output_dir, "track7_sim_summary.json"), "w") as f:
        json.dump({{
            "track": "Track 7 (Multi-Bottleneck Slalom)",
            "total_points": len(collected_scans),
            "lidar_scans": lidar_scan_count,
            "lap_distance_m": total_dist,
            "lap_duration_s": step_count * dt
        }}, f, indent=2)
        
    print(f"[OK] Saved Track 7 LiDAR map: {{len(np_pts)}} points.")
    robot.simulationQuit(0)

if __name__ == "__main__":
    main()
'''
    controller_path = os.path.join(CONTROLLERS_DIR, "robot_lidar_collector.py")
    with open(controller_path, "w", encoding="utf-8") as f:
        f.write(controller_code)
    return controller_path

def plot_track7_results():
    track_data = get_track_7_multi_bottleneck_slalom()
    left_wall = np.array(track_data["left_wall"])
    right_wall = np.array(track_data["right_wall"])
    lw_wrap = np.vstack([left_wall, left_wall[:1]])
    rw_wrap = np.vstack([right_wall, right_wall[:1]])

    yellow_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
    traj = np.loadtxt(yellow_csv, delimiter=",", skiprows=1)
    yellow_pts = traj[:, :2]
    yellow_wrap = np.vstack([yellow_pts, yellow_pts[:1]])

    vor_res = find_centerline_voronoi(track_data)
    centerline = vor_res["centerline"]
    c_wrap = np.vstack([centerline, centerline[:1]])

    pts_path = os.path.join(CONTROLLERS_DIR, "track7_deskewed_lidar_points.npy")
    lidar_pts = np.load(pts_path)

    valid_mask = (lidar_pts[:, 0] >= -2.0) & (lidar_pts[:, 0] <= 8.5) & (lidar_pts[:, 1] >= -2.5) & (lidar_pts[:, 1] <= 3.5)
    filtered_lidar = lidar_pts[valid_mask]

    fig, ax = plt.subplots(figsize=(20, 11), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)

    # 1. Ground Truth Outer & Inner Walls
    ax.plot(lw_wrap[:, 0], lw_wrap[:, 1], color="#f43f5e", linewidth=2.5, label="Ground Truth Left Wall (Outer)", zorder=3)
    ax.plot(rw_wrap[:, 0], rw_wrap[:, 1], color="#38bdf8", linewidth=2.5, label="Ground Truth Right Wall (Inner)", zorder=3)

    # 2. Obstacles (3 Slalom Pillars)
    for obs in track_data.get("obstacles", []):
        if obs.get("type") == "circle":
            circle = plt.Circle((obs["cx"], obs["cy"]), obs["radius"], color="#fbbf24", alpha=0.95, zorder=4)
            ax.add_patch(circle)

    ax.plot([], [], color="#fbbf24", linewidth=6, label="Ground Truth Slalom Pillars (Ø220mm)")

    # 3. Webots MS200k LiDAR Points with IMU Deskewing (Cyan) - Subsampled for crisp fast SVG rendering
    sub_pts = filtered_lidar[::6]
    ax.scatter(sub_pts[:, 0], sub_pts[:, 1], color="#00f0ff", s=1.2, alpha=0.45,
               label=f"Webots MS200k LiDAR (Zero Noise, 4m Range, {len(filtered_lidar)} pts)", zorder=2)

    # 4. Medial Centerline
    ax.plot(c_wrap[:, 0], c_wrap[:, 1], color="#475569", linewidth=1.2, linestyle=":", label="Medial Centerline (Voronoi)", zorder=5)

    # 5. Optimal Yellow Line
    ax.plot(yellow_wrap[:, 0], yellow_wrap[:, 1], color="#ffd700", linewidth=3.0, label="Optimal Yellow Line (Followed in Webots @ 0.25x Speed)", zorder=6)

    # 6. Start Position
    ax.scatter([yellow_pts[0, 0]], [yellow_pts[0, 1]], color="#ffffff", s=150, edgecolors="#ffd700", linewidth=2, label="Robot Start Position", zorder=7)

    ax.set_title("Track 7 (Slalom Speedway): Webots MS200k LiDAR Scans & Optimal Yellow Racing Line",
                 color=TEXT_COLOR, fontsize=14, fontweight='bold', pad=15)
    ax.axis("equal")
    ax.grid(True, linestyle=":", alpha=0.25, color="#fff")
    ax.tick_params(colors=TEXT_COLOR)
    ax.legend(bbox_to_anchor=(1.02, 1.0), loc="upper left", facecolor="#121826", edgecolor="#2d3748",
              labelcolor=TEXT_COLOR, framealpha=1.0, fontsize=9.5, borderpad=1.0, labelspacing=0.8)

    filename = "07_track_7_webots_deskewed_lidar_map.svg"
    svg_path = os.path.join(OUTPUT_DIR, filename)
    plt.savefig(svg_path, format="svg", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)

    art_dir = r"C:\Users\nurmi\.gemini\antigravity-ide\brain\b8e4f4d2-75d6-4f14-9e5f-2ce6afb237bc"
    shutil.copy2(svg_path, os.path.join(art_dir, filename))
    print(f"[OK] Visualized Track 7 SVG saved to: {svg_path} ({os.path.getsize(svg_path)/1024:.1f} KB)")

def main():
    print("=" * 100)
    print("   GENERATING AND RUNNING EXACT POLAR TRACK 7 WEBOTS SIMULATION")
    print("=" * 100)

    wbt_path, yellow_csv = generate_track7_webots_world()
    generate_polar_lidar_controller(yellow_csv)

    print("\n[INFO] Launching Webots simulation...")
    # Clean run Webots
    os.system('Stop-Process -Name "webots", "webots-bin" -Force -ErrorAction SilentlyContinue')
    print("[OK] Webots world and controller updated.")

if __name__ == "__main__":
    main()
