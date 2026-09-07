# ==============================================================================
# Helper to duplicate Cyan_Champion_Track.wbt without moving obstacles
# ==============================================================================
import re

src_path = r"c:\Users\nurmi\Documents\Da Winna\webots/simulation/my_project\worlds\Cyan_Champion_Track.wbt"
dst_path = r"c:\Users\nurmi\Documents\Da Winna\maps\cyan_champion_clean_static\Cyan_Champion_Track_Static.wbt"

with open(src_path, "r", encoding="utf-8") as f:
    lines = f.readlines()

# Strip moving obstacle definitions (lines 703 to 781)
clean_lines = []
for line in lines:
    if line.startswith("DEF MOVING_OBSTACLE"):
        break
    clean_lines.append(line)

with open(dst_path, "w", encoding="utf-8") as f:
    f.writelines(clean_lines)

print(f"Created clean static world without moving obstacles: {dst_path}")
