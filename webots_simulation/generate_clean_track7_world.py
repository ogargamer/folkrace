#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - ZERO-NOISE HORIZONTAL PLANAR LIDAR TEST ON TRACK 7
================================================================================
"""

import os
import sys
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE_DIR)

from racing_engine.track_generator import get_track_7_multi_bottleneck_slalom
from run_track7_webots_sim import build_indexed_faceset_wall, WORLDS_DIR, CONTROLLERS_DIR

def generate_clean_track7_world():
    track_data = get_track_7_multi_bottleneck_slalom()
    left_wall = track_data["left_wall"]
    right_wall = track_data["right_wall"]
    obstacles = track_data.get("obstacles", [])
    
    yellow_csv = os.path.join(BASE_DIR, "racing_results", "yellow_line_backup", "07_track_7_yellow_trajectory.csv")
    traj = np.loadtxt(yellow_csv, delimiter=",", skiprows=1)
    start_x, start_y = traj[0, 0], traj[0, 1]
    
    dx = traj[1, 0] - traj[0, 0]
    dy = traj[1, 1] - traj[0, 1]
    start_yaw = np.arctan2(dx, dy)

    wbt_lines = [
        '#VRML_SIM R2025a utf8',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"',
        '',
        'WorldInfo {',
        '  info [ "Da Winna Track 7 - Zero Noise Horizontal Planar Lidar" ]',
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

    # Watertight Walls
    wbt_lines.append(build_indexed_faceset_wall(left_wall, "track7_left_wall", "0.95 0.15 0.35"))
    wbt_lines.append(build_indexed_faceset_wall(right_wall, "track7_right_wall", "0.20 0.60 0.95"))

    # 3 Slalom Pillars
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

    # 15x20cm Robot with HORIZONTAL Planar LiDAR (Zero Noise)
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
        '        size 0.15 0.05 0.20',
        '      }',
        '    }',
        '    # HORIZONTAL 2D LiDAR (Rotated so scan plane is parallel to ground floor in X-Z plane)',
        '    Solid {',
        '      translation 0.0 0.04 0.0',
        '      rotation 1 0 0 1.57079632679',
        '      children [',
        '        Lidar {',
        '          name "ms200k_lidar"',
        '          fieldOfView 6.2831853',
        '          horizontalResolution 450',
        '          numberOfLayers 1',
        '          minRange 0.12',
        '          maxRange 4.0',
        '          noise 0.0',
        '          type "fixed"',
        '        }',
        '      ]',
        '    }',
        '  ]',
        '  controller "robot_lidar_collector"',
        '  supervisor TRUE',
        '}'
    ])

    wbt_path = os.path.join(WORLDS_DIR, "track7_slalom_speedway.wbt")
    with open(wbt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(wbt_lines))
    print(f"[OK] Generated Clean Horizontal LiDAR World: {wbt_path}")
    return wbt_path, yellow_csv

if __name__ == "__main__":
    generate_clean_track7_world()
