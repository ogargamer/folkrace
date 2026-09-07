#!/usr/bin/env python3
"""
================================================================================
   DA WINNA - TRACK BUILDER JSON TO WEBOTS WORLD EXPORTER (.wbt)
================================================================================
Converts track layout JSON from Modular Track Studio into a fully playable,
watertight Webots 3D Simulation World with ENU Standard Coordinates (Z is Up).
================================================================================
"""

import os
import sys
import json
import argparse
import math
import numpy as np

# Direction & Left Vectors (East, North, West, South)
DIR_VECTORS = [
    np.array([1.0, 0.0]),   # 0: East (+X)
    np.array([0.0, 1.0]),   # 1: North (+Y)
    np.array([-1.0, 0.0]),  # 2: West (-X)
    np.array([0.0, -1.0])   # 3: South (-Y)
]

LEFT_VECTORS = [
    np.array([0.0, 1.0]),   # 0: North
    np.array([-1.0, 0.0]),  # 1: West
    np.array([0.0, -1.0]),  # 2: South
    np.array([1.0, 0.0])    # 3: East
]

def build_track_wall_polygons(track_data, step_size=0.04):
    """
    Computes left and right wall vertices and centerline points.
    """
    params = track_data.get("parameters", {})
    tile_size = float(params.get("tileSize", 1.2))
    track_width = float(params.get("trackWidth", 1.0))
    blocks = track_data.get("blocks", [])
    
    half_width = track_width / 2.0
    left_wall = []
    right_wall = []
    centerline = []
    
    for b in blocks:
        b_type = b.get("type", "straight")
        start_x = float(b.get("startX", 0.0))
        start_y = float(b.get("startY", 0.0))
        cur_h = int(b.get("startHeading", 0)) % 4
        fwd = DIR_VECTORS[cur_h]
        left = LEFT_VECTORS[cur_h]
        
        if b_type == "straight":
            steps = max(8, int(math.ceil(tile_size / step_size)))
            for i in range(1, steps + 1):
                t = i / steps
                px = start_x + fwd[0] * tile_size * t
                py = start_y + fwd[1] * tile_size * t
                left_wall.append([px + left[0] * half_width, py + left[1] * half_width])
                right_wall.append([px - left[0] * half_width, py - left[1] * half_width])
                centerline.append([px, py])
                
        elif b_type == "turn_left":
            r_center = tile_size / 2.0
            c_data = b.get("center", {})
            cx = float(c_data.get("x", start_x + left[0] * r_center))
            cy = float(c_data.get("y", start_y + left[1] * r_center))
            start_angle = math.atan2(start_y - cy, start_x - cx)
            arc_len = (math.pi / 2.0) * r_center
            steps = max(12, int(math.ceil(arc_len / step_size)))
            
            for i in range(1, steps + 1):
                angle = start_angle + (math.pi / 2.0) * (i / steps)
                cos_a = math.cos(angle)
                sin_a = math.sin(angle)
                left_wall.append([cx + (r_center - half_width) * cos_a, cy + (r_center - half_width) * sin_a])
                right_wall.append([cx + (r_center + half_width) * cos_a, cy + (r_center + half_width) * sin_a])
                centerline.append([cx + r_center * cos_a, cy + r_center * sin_a])
                
        elif b_type == "turn_right":
            r_center = tile_size / 2.0
            c_data = b.get("center", {})
            cx = float(c_data.get("x", start_x - left[0] * r_center))
            cy = float(c_data.get("y", start_y - left[1] * r_center))
            start_angle = math.atan2(start_y - cy, start_x - cx)
            arc_len = (math.pi / 2.0) * r_center
            steps = max(12, int(math.ceil(arc_len / step_size)))
            
            for i in range(1, steps + 1):
                angle = start_angle - (math.pi / 2.0) * (i / steps)
                cos_a = math.cos(angle)
                sin_a = math.sin(angle)
                left_wall.append([cx + (r_center + half_width) * cos_a, cy + (r_center + half_width) * sin_a])
                right_wall.append([cx + (r_center - half_width) * cos_a, cy + (r_center - half_width) * sin_a])
                centerline.append([cx + r_center * cos_a, cy + r_center * sin_a])
                
    return np.array(left_wall), np.array(right_wall), np.array(centerline)

def build_indexed_faceset_wall(pts, name, color_str, wall_height=0.30):
    """
    Builds watertight 3D wall node in ENU coordinates (Z is Up).
    """
    N = len(pts)
    if N < 2:
        return ""
        
    dist_loop = math.hypot(pts[0, 0] - pts[-1, 0], pts[0, 1] - pts[-1, 1])
    is_closed = dist_loop < 0.25

    # Bottom vertices at z=0.0000
    coord_lines = [f"{p[0]:.4f} {p[1]:.4f} 0.0000" for p in pts]
    # Top vertices at z=wall_height
    coord_lines.extend([f"{p[0]:.4f} {p[1]:.4f} {wall_height:.4f}" for p in pts])
    coord_str = ", ".join(coord_lines)

    index_lines = []
    max_i = N if is_closed else (N - 1)
    for i in range(max_i):
        i_next = (i + 1) % N
        # Quad front face: (bottom_i, bottom_next, top_next, top_i)
        index_lines.append(f"{i}, {i_next}, {i_next + N}, {i + N}, -1")
        # Quad reverse face for 2-sided LiDAR collision
        index_lines.append(f"{i}, {i + N}, {i_next + N}, {i_next}, -1")

    index_str = ", ".join(index_lines)

    node = f'''Solid {{
  name "{name}"
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {color_str}
        roughness 0.5
        metalness 0.1
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

def convert_json_to_webots_wbt(json_path, output_wbt_path=None, wall_height=0.30):
    with open(json_path, "r", encoding="utf-8") as f:
        track_data = json.load(f)

    params = track_data.get("parameters", {})
    track_width = float(params.get("trackWidth", 1.0))
    wall_height = float(params.get("wallHeight", wall_height))
    init_pose = track_data.get("initial_pose", {})
    start_x = float(init_pose.get("x", 0.60))
    start_y = float(init_pose.get("y", 0.00))
    start_yaw = float(init_pose.get("yaw_rad", float(init_pose.get("yaw_deg", 0.0)) * math.pi / 180.0))
    
    obstacles = track_data.get("obstacles", [])
    start_line = track_data.get("startLine", None)
    blocks = track_data.get("blocks", [])

    left_wall, right_wall, centerline = build_track_wall_polygons(track_data)

    all_x = [start_x - 2.0, start_x + 2.0]
    all_y = [start_y - 2.0, start_y + 2.0]
    if len(left_wall) > 0:
        all_x.extend(left_wall[:, 0])
        all_y.extend(left_wall[:, 1])
    if len(right_wall) > 0:
        all_x.extend(right_wall[:, 0])
        all_y.extend(right_wall[:, 1])

    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)
    span_x = max(5.0, max_x - min_x)
    span_y = max(5.0, max_y - min_y)
    mid_x = (min_x + max_x) / 2.0
    mid_y = (min_y + max_y) / 2.0

    arena_w = max(16, int(math.ceil(span_x + 8)))
    arena_h = max(16, int(math.ceil(span_y + 8)))
    view_dist = max(7.0, max(span_x, span_y) * 1.35)

    wbt_lines = [
        '#VRML_SIM R2025a utf8',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"',
        '',
        'WorldInfo {',
        '  info [',
        '    "Da Winna Modular Race Track Studio World",',
        '    "Coordinate System: ENU (East-North-Up Standard, Z is Elevation)"',
        '  ]',
        '  title "Da Winna Track World"',
        '  basicTimeStep 10',
        '  FPS 60',
        '  coordinateSystem "ENU"',
        '}',
        'Viewpoint {',
        '  orientation 0.65 0.35 0.67 1.85',
        f'  position {mid_x:.2f} {(mid_y - view_dist * 0.55):.2f} {(view_dist * 0.95):.2f}',
        '}',
        'TexturedBackground {',
        '  texture "dusk"',
        '}',
        'TexturedBackgroundLight {',
        '  luminosity 1.2',
        '}',
        'RectangleArena {',
        f'  floorSize {arena_w} {arena_h}',
        '  floorTileSize 1 1',
        '  floorAppearance PBRAppearance {',
        '    baseColor 0.08 0.10 0.15',
        '    roughness 0.8',
        '  }',
        '}',
        ''
    ]

    # Watertight Walls
    if len(left_wall) > 1:
        wbt_lines.append(build_indexed_faceset_wall(left_wall, "track_left_wall", "0.95 0.15 0.35", wall_height))
    if len(right_wall) > 1:
        wbt_lines.append(build_indexed_faceset_wall(right_wall, "track_right_wall", "0.20 0.60 0.95", wall_height))

    # Obstacles
    for idx, obs in enumerate(obstacles):
        obs_type = obs.get("type", "circle")
        if obs_type == "circle":
            cx = float(obs.get("cx", 0.0))
            cy = float(obs.get("cy", 0.0))
            r = float(obs.get("radius", 0.10))
            wbt_lines.append(f'''Solid {{
  name "slalom_pillar_{idx}"
  translation {cx:.4f} {cy:.4f} {(wall_height / 2.0):.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1.0 0.75 0.0
        roughness 0.25
        metalness 0.1
      }}
      geometry Cylinder {{
        height {wall_height:.4f}
        radius {r:.4f}
      }}
    }}
  ]
  boundingObject Cylinder {{
    height {wall_height:.4f}
    radius {r:.4f}
  }}
}}''')
        elif obs_type == "rect":
            x1 = float(obs.get("x1", 0.0))
            x2 = float(obs.get("x2", 0.0))
            y1 = float(obs.get("y1", 0.0))
            y2 = float(obs.get("y2", 0.0))
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = abs(x2 - x1)
            h = abs(y2 - y1)
            wbt_lines.append(f'''Solid {{
  name "obstacle_box_{idx}"
  translation {cx:.4f} {cy:.4f} {(wall_height / 2.0):.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1.0 0.45 0.0
        roughness 0.35
      }}
      geometry Box {{
        size {w:.4f} {h:.4f} {wall_height:.4f}
      }}
    }}
  ]
  boundingObject Box {{
    size {w:.4f} {h:.4f} {wall_height:.4f}
  }}
}}''')

    # Start Line
    if start_line or len(blocks) > 0:
        sx = float(start_line.get("x", blocks[0].get("startX", 0.0))) if start_line else float(blocks[0].get("startX", 0.0))
        sy = float(start_line.get("y", blocks[0].get("startY", 0.0))) if start_line else float(blocks[0].get("startY", 0.0))
        sh = float(start_line.get("heading", blocks[0].get("startHeading", 0))) if start_line else float(blocks[0].get("startHeading", 0))
        syaw = sh * (math.pi / 2.0)
        wbt_lines.append(f'''Transform {{
  translation {sx:.4f} {sy:.4f} 0.002
  rotation 0 0 1 {syaw:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.0 1.0 0.55
        roughness 0.4
      }}
      geometry Box {{
        size 0.08 {track_width:.4f} 0.003
      }}
    }}
  ]
}}''')

    # Robot with MS200k LiDAR
    wbt_lines.append(f'''Robot {{
  name "da_winna_robot"
  translation {start_x:.4f} {start_y:.4f} 0.05
  rotation 0 0 1 {start_yaw:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1.0 0.85 0.0
        metalness 0.6
        roughness 0.2
      }}
      geometry Box {{
        size 0.20 0.15 0.05
      }}
    }}
    Transform {{
      translation 0.09 0.0 0.015
      children [
        Shape {{
          appearance PBRAppearance {{
            baseColor 1.0 0.0 0.45
            roughness 0.2
          }}
          geometry Box {{
            size 0.03 0.08 0.02
          }}
        }}
      ]
    }}
    Solid {{
      translation 0.0 0.0 0.04
      children [
        Lidar {{
          name "ms200k_lidar"
          fieldOfView 6.2831853
          horizontalResolution 450
          numberOfLayers 1
          minRange 0.12
          maxRange 5.0
          noise 0.0
          type "fixed"
        }}
      ]
    }}
    InertialUnit {{
      name "bno08x_imu"
      noise 0.001
    }}
  ]
  controller "robot_lidar_collector"
  supervisor TRUE
}}''')

    if output_wbt_path is None:
        base, _ = os.path.splitext(json_path)
        output_wbt_path = base + ".wbt"

    with open(output_wbt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(wbt_lines))

    print(f"[OK] Generated Webots 3D World (ENU Standard): {output_wbt_path}")
    return output_wbt_path

def main():
    parser = argparse.ArgumentParser(description="Convert Modular Track JSON to Webots .wbt world file")
    parser.add_argument("json_file", help="Path to track layout JSON file")
    parser.add_argument("-o", "--output", help="Output .wbt file path", default=None)
    parser.add_argument("--wall-height", help="Wall height in meters (default: 0.30)", type=float, default=0.30)
    args = parser.parse_args()

    if not os.path.exists(args.json_file):
        print(f"Error: JSON file not found: {args.json_file}")
        sys.exit(1)

    convert_json_to_webots_wbt(args.json_file, args.output, args.wall_height)

if __name__ == "__main__":
    main()
