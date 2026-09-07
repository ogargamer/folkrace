import re
import numpy as np
import scipy.spatial as spatial
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

def parse_wbt(wbt_path):
    with open(wbt_path, "r", encoding="utf-8") as f:
        content = f.read()

    solid_blocks = content.split("Solid {")
    left_wall = None
    right_wall = None
    obstacles = []

    for block in solid_blocks[1:]:
        name_match = re.search(r'name\s+"([^"]+)"', block)
        name = name_match.group(1) if name_match else ""
        trans_match = re.search(r'translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', block)
        trans = [float(x) for x in trans_match.groups()] if trans_match else [0.0, 0.0, 0.0]
        
        if "IndexedFaceSet" in block:
            pt_match = re.search(r'point\s*\[\s*([\s\S]*?)\]', block)
            if pt_match:
                pts_str = pt_match.group(1).strip()
                pts_coords = [float(x) for x in re.split(r'[,\s]+', pts_str) if x]
                pts_3d = np.array(pts_coords).reshape(-1, 3)
                # Webots IndexedFaceSet walls have bottom ring (z=0) and top ring (z=0.12)
                # Take the z=0 points
                z0_mask = np.isclose(pts_3d[:, 2], 0.0, atol=1e-3)
                pts_2d = pts_3d[z0_mask, :2]
                
                if "left_wall" in name:
                    left_wall = pts_2d
                elif "right_wall" in name:
                    right_wall = pts_2d
        elif "Cylinder" in block:
            rad_match = re.search(r'radius\s+([-\d.]+)', block)
            rad = float(rad_match.group(1)) if rad_match else 0.05
            obstacles.append({
                "type": "circle",
                "name": name,
                "cx": trans[0],
                "cy": trans[1],
                "radius": rad
            })
        elif "Box" in block:
            size_match = re.search(r'size\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', block)
            size = [float(x) for x in size_match.groups()] if size_match else [0.1, 0.1, 0.1]
            # In Webots, box size is (sx, sy, sz), centered at translation
            obstacles.append({
                "type": "wall_rectangle",
                "name": name,
                "center_x": trans[0],
                "center_y": trans[1],
                "width": size[0],
                "extrusion": size[1],
                "size": size
            })
            
    return left_wall, right_wall, obstacles

left_wall, right_wall, obstacles = parse_wbt(r"C:\Users\nurmi\Downloads\track_world_1787780744372.wbt")
print(f"Left wall points: {len(left_wall) if left_wall is not None else 0}")
print(f"Right wall points: {len(right_wall) if right_wall is not None else 0}")
print(f"Obstacles ({len(obstacles)}):")
for o in obstacles:
    print(" ", o)
