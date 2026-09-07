import json
import math
import os

json_path = r"C:\Users\nurmi\Downloads\track_layout_1787771901940.json"
wbt_in = r"C:\Users\nurmi\Documents\Da Winna\my_project\worlds\Cyan_Champion_Track.wbt"
wbt_out_list = [
    r"C:\Users\nurmi\Documents\Da Winna\my_project\worlds\Cyan_Champion_Track_Static.wbt",
    r"C:\Users\nurmi\Documents\Da Winna\maps\cyan_champion_clean_static\Cyan_Champion_Track_Static.wbt"
]

with open(wbt_in, "r", encoding="utf-8") as f:
    text = f.read()

# Strip any moving obstacles at the end
split_idx = text.find("DEF MOVING_OBSTACLE")
if split_idx != -1:
    base_text = text[:split_idx].rstrip() + "\n"
else:
    base_text = text

with open(json_path, "r", encoding="utf-8") as jf:
    layout = json.load(jf)

obstacles = layout.get("obstacles", [])
obs_nodes = []

for i, obs in enumerate(obstacles):
    if obs.get("type") == "circle":
        cx, cy, r = obs["cx"], obs["cy"], obs["radius"]
        node_str = f"""DEF STATIC_OBSTACLE_C_{i+1} Solid {{
  translation {cx:.4f} {cy:.4f} 0.075
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.95 0.15 0.35
        roughness 0.5
        metalness 0.1
      }}
      geometry Cylinder {{
        height 0.15
        radius {r:.4f}
        subdivision 32
      }}
    }}
  ]
  name "static_pillar_{i+1}"
  contactMaterial "wall"
  boundingObject Cylinder {{
    height 0.15
    radius {r:.4f}
    subdivision 32
  }}
}}"""
        obs_nodes.append(node_str)
    elif obs.get("type") == "rect":
        x1, y1 = obs["x1"], obs["y1"]
        x2, y2 = obs["x2"], obs["y2"]
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        thickness = 0.05
        height = 0.15
        
        node_str = f"""DEF STATIC_OBSTACLE_R_{i+1} Solid {{
  translation {cx:.4f} {cy:.4f} 0.075
  rotation 0 0 1 {angle:.5f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.2 0.6 0.95
        roughness 0.5
        metalness 0.1
      }}
      geometry Box {{
        size {length:.4f} {thickness:.4f} {height:.4f}
      }}
    }}
  ]
  name "static_barrier_{i+1}"
  contactMaterial "wall"
  boundingObject Box {{
    size {length:.4f} {thickness:.4f} {height:.4f}
  }}
}}"""
        obs_nodes.append(node_str)

full_world_content = base_text + "\n" + "\n".join(obs_nodes) + "\n"

for out_path in wbt_out_list:
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(full_world_content)
    print(f"Updated {out_path} with {len(obs_nodes)} physical 100% solid static obstacles.")
