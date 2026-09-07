import re
import numpy as np

wbt_path = r"C:\Users\nurmi\Downloads\track_world_1787780744372.wbt"
with open(wbt_path, "r", encoding="utf-8") as f:
    content = f.read()

print("File size:", len(content))

# Look for walls and solids
solid_blocks = content.split("Solid {")
print(f"Total Solid blocks: {len(solid_blocks)}")

for i, block in enumerate(solid_blocks[1:], 1):
    name_match = re.search(r'name\s+"([^"]+)"', block)
    name = name_match.group(1) if name_match else "unnamed"
    trans_match = re.search(r'translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', block)
    trans = trans_match.groups() if trans_match else None
    
    # Check if IndexedFaceSet
    if "IndexedFaceSet" in block:
        # Extract point array
        pt_match = re.search(r'point\s*\[\s*([\s\S]*?)\]', block)
        if pt_match:
            pts_str = pt_match.group(1).strip()
            # Split by comma or space
            pts_coords = [float(x) for x in re.split(r'[,\s]+', pts_str) if x]
            pts_3d = np.array(pts_coords).reshape(-1, 3)
            print(f"Solid {i}: '{name}' IndexedFaceSet with {len(pts_3d)} 3D points. Bounds: X=[{pts_3d[:,0].min():.2f}, {pts_3d[:,0].max():.2f}], Y=[{pts_3d[:,1].min():.2f}, {pts_3d[:,1].max():.2f}], Z=[{pts_3d[:,2].min():.2f}, {pts_3d[:,2].max():.2f}]")
    elif "Cylinder" in block:
        rad_match = re.search(r'radius\s+([-\d.]+)', block)
        rad = float(rad_match.group(1)) if rad_match else None
        print(f"Solid {i}: '{name}' Cylinder at {trans}, radius={rad}")
    elif "Box" in block:
        size_match = re.search(r'size\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', block)
        size = size_match.groups() if size_match else None
        print(f"Solid {i}: '{name}' Box at {trans}, size={size}")
    else:
        print(f"Solid {i}: '{name}' at {trans}")
