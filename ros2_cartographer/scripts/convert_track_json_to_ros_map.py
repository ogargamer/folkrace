#!/usr/bin/env python3
"""
Converts track_layout JSON into:
1. Standard ROS Occupancy Grid (.pgm image + .yaml metadata) for Nav2 Map Server
2. Accurate 2D Wall coordinates for verification and pure localization
"""

import os
import sys
import json
import math
import numpy as np
from PIL import Image, ImageDraw

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

def build_polygons(track_data, step_size=0.01):
    params = track_data.get("parameters", {})
    tile_size = float(params.get("tileSize", 1.2))
    track_width = float(params.get("trackWidth", 1.0))
    blocks = track_data.get("blocks", [])
    
    half_width = track_width / 2.0
    left_wall, right_wall, centerline = [], [], []
    
    for b in blocks:
        b_type = b.get("type", "straight")
        start_x = float(b.get("startX", 0.0))
        start_y = float(b.get("startY", 0.0))
        cur_h = int(b.get("startHeading", 0)) % 4
        fwd = DIR_VECTORS[cur_h]
        left = LEFT_VECTORS[cur_h]
        
        if b_type == "straight":
            steps = max(10, int(math.ceil(tile_size / step_size)))
            for i in range(1, steps + 1):
                t = i / steps
                px = start_x + fwd[0] * tile_size * t
                py = start_y + fwd[1] * tile_size * t
                left_wall.append((px + left[0] * half_width, py + left[1] * half_width))
                right_wall.append((px - left[0] * half_width, py - left[1] * half_width))
                centerline.append((px, py))
                
        elif b_type == "turn_left":
            r_center = tile_size / 2.0
            c_data = b.get("center", {})
            cx = float(c_data.get("x", start_x + left[0] * r_center))
            cy = float(c_data.get("y", start_y + left[1] * r_center))
            start_angle = math.atan2(start_y - cy, start_x - cx)
            arc_len = (math.pi / 2.0) * r_center
            steps = max(20, int(math.ceil(arc_len / step_size)))
            
            for i in range(1, steps + 1):
                angle = start_angle + (math.pi / 2.0) * (i / steps)
                cos_a, sin_a = math.cos(angle), math.sin(angle)
                left_wall.append((cx + (r_center - half_width) * cos_a, cy + (r_center - half_width) * sin_a))
                right_wall.append((cx + (r_center + half_width) * cos_a, cy + (r_center + half_width) * sin_a))
                centerline.append((cx + r_center * cos_a, cy + r_center * sin_a))
                
        elif b_type == "turn_right":
            r_center = tile_size / 2.0
            c_data = b.get("center", {})
            cx = float(c_data.get("x", start_x - left[0] * r_center))
            cy = float(c_data.get("y", start_y - left[1] * r_center))
            start_angle = math.atan2(start_y - cy, start_x - cx)
            arc_len = (math.pi / 2.0) * r_center
            steps = max(20, int(math.ceil(arc_len / step_size)))
            
            for i in range(1, steps + 1):
                angle = start_angle - (math.pi / 2.0) * (i / steps)
                cos_a, sin_a = math.cos(angle), math.sin(angle)
                left_wall.append((cx + (r_center + half_width) * cos_a, cy + (r_center + half_width) * sin_a))
                right_wall.append((cx + (r_center - half_width) * cos_a, cy + (r_center - half_width) * sin_a))
                centerline.append((cx + r_center * cos_a, cy + r_center * sin_a))
                
    return left_wall, right_wall, centerline

def generate_occupancy_grid(json_path, output_dir, resolution=0.02, margin=0.5):
    with open(json_path, "r", encoding="utf-8") as f:
        track_data = json.load(f)
        
    left_wall, right_wall, centerline = build_polygons(track_data)
    
    all_pts = left_wall + right_wall
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    
    min_x = math.floor((min(xs) - margin) / resolution) * resolution
    max_x = math.ceil((max(xs) + margin) / resolution) * resolution
    min_y = math.floor((min(ys) - margin) / resolution) * resolution
    max_y = math.ceil((max(ys) + margin) / resolution) * resolution
    
    width_px = int(round((max_x - min_x) / resolution))
    height_px = int(round((max_y - min_y) / resolution))
    
    print(f"[MAP] Bounding box: X=[{min_x:.2f}, {max_x:.2f}], Y=[{min_y:.2f}, {max_y:.2f}]")
    print(f"[MAP] Image dimensions: {width_px} x {height_px} px at {resolution*1000:.1f}mm/px")
    
    # In ROS Occupancy Grid standard: 254 (0xFF) = Free, 0 (0x00) = Occupied (Wall), 205 = Unknown
    img = Image.new("L", (width_px, height_px), 205)
    draw = ImageDraw.Draw(img)
    
    def world_to_px(wx, wy):
        px = int(round((wx - min_x) / resolution))
        # Note: image origin is top-left, ROS origin is bottom-left
        py = height_px - 1 - int(round((wy - min_y) / resolution))
        return (px, py)
    
    # 1. Fill corridor / free space with 254 (White = Free)
    corridor_poly = [world_to_px(p[0], p[1]) for p in left_wall] + [world_to_px(p[0], p[1]) for p in reversed(right_wall)]
    draw.polygon(corridor_poly, fill=254)
    
    # 2. Draw Walls as Occupied (Black = 0) with thickness = 18mm
    wall_px_width = max(1, int(round(0.018 / resolution)))
    for poly in [left_wall, right_wall]:
        px_coords = [world_to_px(p[0], p[1]) for p in poly]
        draw.line(px_coords, fill=0, width=wall_px_width)
        
    os.makedirs(output_dir, exist_ok=True)
    pgm_path = os.path.join(output_dir, "folkrace_track_map.pgm")
    yaml_path = os.path.join(output_dir, "folkrace_track_map.yaml")
    
    img.save(pgm_path)
    print(f"[MAP] Saved PGM map: {pgm_path}")
    
    yaml_content = f"""image: folkrace_track_map.pgm
mode: trinary
resolution: {resolution}
origin: [{min_x:.4f}, {min_y:.4f}, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    print(f"[MAP] Saved YAML metadata: {yaml_path}")

if __name__ == "__main__":
    src_json = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\nurmi\Downloads\track_layout_1787771901940.json"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else r"c:\Users\nurmi\Documents\Da Winna\maps"
    generate_occupancy_grid(src_json, out_dir, resolution=0.02)
