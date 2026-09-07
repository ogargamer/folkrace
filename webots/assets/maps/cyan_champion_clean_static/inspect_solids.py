import re

with open(r'C:\Users\nurmi\Documents\Da Winna\maps\cyan_champion_clean_static\Cyan_Champion_Track_Static.wbt', 'r', encoding='utf-8') as f:
    text = f.read()

# Find all top-level Solid blocks
pattern = re.compile(r'(?:DEF\s+([A-Za-z0-9_]+)\s+)?Solid\s*\{([^{}]*(?:\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}[^{}]*)*)\}', re.MULTILINE)

matches = pattern.findall(text)
print(f"Total Solid matches: {len(matches)}")
for def_name, body in matches:
    name_m = re.search(r'name\s+"([^"]+)"', body)
    trans_m = re.search(r'translation\s+([-\d\.\s]+)', body)
    rot_m = re.search(r'rotation\s+([-\d\.\s]+)', body)
    shape_m = re.search(r'(Box|Cylinder|IndexedFaceSet)', body)
    color_m = re.search(r'baseColor\s+([-\d\.\s]+)', body)
    
    name = name_m.group(1) if name_m else "unnamed"
    trans = trans_m.group(1).strip() if trans_m else "none"
    rot = rot_m.group(1).strip() if rot_m else "none"
    shape = shape_m.group(1) if shape_m else "unknown"
    color = color_m.group(1).strip() if color_m else "none"
    
    print(f"DEF: {def_name:20s} | name: {name:20s} | trans: {trans:30s} | rot: {rot:25s} | shape: {shape:10s} | color: {color}")
