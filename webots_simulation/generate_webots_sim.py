#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - HIGH-PRECISION WEBOTS SIMULATION GENERATOR
================================================================================
- Watertight continuous walls via IndexedFaceSet (Zero gaps, exact track geometry)
- 4.0m MS200k LiDAR range limit
- Exact 3D transformation via Supervisor Matrix (R * p_local + pos)
================================================================================
"""

import os
import sys
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import (
    get_track_4_switchback_arena,
    get_track_11_quad_chicane_slalom
)

WEBOTS_DIR = os.path.join(BASE_DIR, "webots_simulation")
WORLDS_DIR = os.path.join(WEBOTS_DIR, "worlds")
CONTROLLERS_DIR = os.path.join(WEBOTS_DIR, "controllers", "robot_lidar_collector")

os.makedirs(WORLDS_DIR, exist_ok=True)
os.makedirs(CONTROLLERS_DIR, exist_ok=True)

def build_indexed_faceset_wall(pts, name, base_color_str, wall_height=0.30):
    """
    Builds a continuous, watertight 3D wall mesh from a 2D closed polygon.
    """
    N = len(pts)
    coord_lines = []
    # Bottom vertices (y=0)
    for p in pts:
        coord_lines.append(f"{p[0]:.4f} 0.0000 {p[1]:.4f}")
    # Top vertices (y=wall_height)
    for p in pts:
        coord_lines.append(f"{p[0]:.4f} {wall_height:.4f} {p[1]:.4f}")
        
    coord_str = ", ".join(coord_lines)
    
    # Generate quad indices for each wall segment
    index_lines = []
    for i in range(N):
        i_next = (i + 1) % N
        # Quad face: (bottom_i, bottom_next, top_next, top_i)
        index_lines.append(f"{i}, {i_next}, {i_next + N}, {i + N}, -1")
        # Reverse face for two-sided LiDAR collision
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

def generate_webots_world_wbt(track_data, world_filename, yellow_csv_path):
    left_wall = track_data["left_wall"]
    right_wall = track_data["right_wall"]
    obstacles = track_data.get("obstacles", [])
    
    start_x, start_y, start_yaw = 0.0, 0.0, 0.0
    if os.path.exists(yellow_csv_path):
        traj = np.loadtxt(yellow_csv_path, delimiter=",", skiprows=1)
        start_x, start_y = traj[0, 0], traj[0, 1]
        start_yaw = traj[0, 3]

    wbt_lines = [
        '#VRML_SIM R2025a utf8',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"',
        '',
        'WorldInfo {',
        '  info [ "Da Winna Autonomous Racing Track Simulation - Watertight Walls & MS200k LiDAR" ]',
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
        '  floorSize 25 25',
        '  floorTileSize 1 1',
        '  floorAppearance PBRAppearance {',
        '    baseColor 0.08 0.10 0.15',
        '    roughness 0.8',
        '  }',
        '}',
    ]

    # Continuous Watertight Outer & Inner Track Walls
    wbt_lines.append(build_indexed_faceset_wall(left_wall, "track_left_wall", "0.95 0.15 0.35"))
    wbt_lines.append(build_indexed_faceset_wall(right_wall, "track_right_wall", "0.20 0.60 0.95"))

    # Add Obstacle Pillars
    wall_height = 0.30
    for idx, obs in enumerate(obstacles):
        if obs.get("type") == "circle":
            cx, cz = obs["cx"], obs["cy"]
            r = obs["radius"]
            wbt_lines.extend([
                'Solid {',
                f'  name "pillar_{idx}"',
                f'  translation {cx:.4f} {wall_height/2:.4f} {cz:.4f}',
                '  children [',
                '    Shape {',
                '      appearance PBRAppearance { baseColor 1.0 0.7 0.0 roughness 0.3 }',
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
        '    # MS200k 2D LiDAR: 4.0m Max Distance, 450 rays, 12Hz, 4mm noise',
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
        '    InertialUnit {',
        '      name "bno08x_imu"',
        '      noise 0.001',
        '    }',
        '  ]',
        '  controller "robot_lidar_collector"',
        '  supervisor TRUE',
        '}'
    ])

    wbt_path = os.path.join(WORLDS_DIR, world_filename)
    with open(wbt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(wbt_lines))
    return wbt_path

def generate_robot_controller_script(yellow_csv_path):
    controller_code = f'''#!/usr/bin/env python3
"""
================================================================================
   WEBOTS CONTROLLER: EXACT 3D MATRIX POSE & MS200k LiDAR RAY TRANSFORM
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
    lidar.enable(int(1000 / 12)) # 12 Hz scan rate
    lidar.enablePointCloud()

    traj_path = r"{yellow_csv_path}"
    if not os.path.exists(traj_path):
        print(f"[ERROR] Trajectory file not found: {{traj_path}}")
        return

    traj = np.loadtxt(traj_path, delimiter=",", skiprows=1)
    N = len(traj)
    pts = traj[:, :2]
    vels = traj[:, 2] * 0.50 # Exact 0.5x speed
    headings = traj[:, 3]

    robot_node = robot.getSelf()
    trans_field = robot_node.getField("translation")
    rot_field = robot_node.getField("rotation")

    print("[INFO] Webots Precision 0.5x Kinematic Collector Started!")

    collected_scans = []
    
    cur_idx = 0
    cur_dist = 0.0
    total_dist = 0.0
    ds_list = [np.hypot(pts[(i+1)%N, 0] - pts[i, 0], pts[(i+1)%N, 1] - pts[i, 1]) for i in range(N)]
    lap_length = sum(ds_list)
    
    dt = timestep / 1000.0
    step_count = 0
    lidar_scan_count = 0

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
        
        # Calculate true continuous forward heading tangent
        dx_next = pts[next_idx, 0] - pts[cur_idx, 0]
        dz_next = pts[next_idx, 1] - pts[cur_idx, 1]
        
        # In Webots NUE: Angle around Y-axis is atan2(dx, dz)
        webots_yaw = np.arctan2(dx_next, dz_next)

        # Update pose in Webots 3D World
        trans_field.setSFVec3f([float(rx), 0.05, float(rz)])
        rot_field.setSFRotation([0.0, 1.0, 0.0, float(webots_yaw)])

        # Read LiDAR 3D Point Cloud directly in body frame
        raw_cloud = lidar.getPointCloud()
        if raw_cloud is not None and len(raw_cloud) > 0:
            lidar_scan_count += 1
            
            # Get exact robot 3D transform from Supervisor
            pos = robot_node.getPosition()
            rot_mat = np.array(robot_node.getOrientation()).reshape(3, 3)
            
            valid_pts = []
            for p in raw_cloud:
                # Local coords (x=right, y=height, z=forward in Webots Lidar frame)
                p_local = np.array([p.x, p.y, p.z])
                dist = np.linalg.norm(p_local)
                if 0.12 < dist <= 4.00:
                    # Global transform: P_global = R * P_local + Pos
                    p_global = rot_mat @ p_local + pos
                    # Store (X, Z) as 2D track coordinates
                    valid_pts.append([p_global[0], p_global[2]])
                    
            if len(valid_pts) > 0:
                collected_scans.extend(valid_pts)

        if total_dist >= lap_length:
            print(f"[COMPLETE] Finished 1 Lap! Traversed: {{total_dist:.2f}}m")
            print(f"[COMPLETE] Scanned {{len(collected_scans)}} high-precision points across {{lidar_scan_count}} sweeps.")
            break

    output_dir = os.path.dirname(__file__)
    np_pts = np.array(collected_scans)
    np.save(os.path.join(output_dir, "noisy_lidar_map_points.npy"), np_pts)
    
    with open(os.path.join(output_dir, "simulation_summary.json"), "w") as f:
        json.dump({{
            "total_points": len(collected_scans),
            "lidar_scans": lidar_scan_count,
            "lap_distance_m": total_dist,
            "lap_duration_s": step_count * dt
        }}, f, indent=2)
        
    print(f"[OK] Saved precision LiDAR map: {{len(np_pts)}} points.")
    robot.simulationQuit(0)

if __name__ == "__main__":
    main()
'''
    controller_path = os.path.join(CONTROLLERS_DIR, "robot_lidar_collector.py")
    with open(controller_path, "w", encoding="utf-8") as f:
        f.write(controller_code)
    return controller_path

def main():
    track = get_track_4_switchback_arena()
    yellow_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "04_track_4_yellow_trajectory.csv")
    generate_robot_controller_script(yellow_csv)
    generate_webots_world_wbt(track, "track4_switchback_arena.wbt", yellow_csv)

    track11 = get_track_11_quad_chicane_slalom()
    yellow11_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "11_track_11_yellow_trajectory.csv")
    generate_webots_world_wbt(track11, "track11_quad_chicane_arena.wbt", yellow11_csv)
    print("[OK] Watertight Webots worlds (4m range, exact R*p+t transform) generated cleanly!")

if __name__ == "__main__":
    main()
