"""
================================================================================
  LAUNCHER FOR ANTI-WRONG-WAY & U-TURN TEST ENVIRONMENT (WEBOTS)
================================================================================
  Launches Webots in real-time mode with the test_wrong_way.wbt world.
  World includes:
    - Track corridors with tight hairpins at both ends
    - Front pillar (DEF FRONT_PILLAR at 2.6, 0.4, 0.06)
    - Back pillar (DEF BACK_PILLAR at -0.21, 0.39, 0.06)
    - Folkrace autonomous racing robot running champion_live_preview controller

  Interactive Webots Keyboard Controls (focus 3D window):
    [F] : Toggle FRONT PILLAR (Raise/Lower obstacle)
    [B] : Toggle BACK PILLAR  (Raise/Lower obstacle)
    [1] : Option 1 - Breadcrumb Overlap & Auto 180 Correction
    [2] : Option 2 - Kinematic Minimum Arc Radius Filter (R >= R_min)
    [3] : Option 3 - Directional Flow Costmap Stamping
    [4] : Option 4 - Lateral Corridor Separation Watchdog
    [5] : Option Combined Hybrid (All layers active)
    [0] : Disable Watchdog (Pass-through)
    [R] : Reset / Teleport Robot to Start Pose (0, 0)
    [H] : Show Help Menu
================================================================================
"""

import os
import sys
import subprocess

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
SEPTIC_DIR = os.path.dirname(CURR_DIR)
WORLD_PATH = os.path.join(SEPTIC_DIR, "worlds", "test_wrong_way.wbt")

# Standard Webots binary paths on Windows
WEBOTS_CANDIDATES = [
    r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    r"C:\Program Files\Webots\webots.exe",
    r"C:\Users\nurmi\AppData\Local\Programs\Webots\msys64\mingw64\bin\webots.exe",
]

def find_webots_bin():
    for p in WEBOTS_CANDIDATES:
        if os.path.exists(p):
            return p
    return "webots"

def main():
    webots_bin = find_webots_bin()

    print("================================================================================")
    print("  LAUNCHING ANTI-WRONG-WAY & U-TURN WEBOTS TEST ENVIRONMENT")
    print("================================================================================")
    print(f"  World: {WORLD_PATH}")
    print(f"  Binary: {webots_bin}")
    print("\n  INTERACTIVE KEYBOARD SHORTCUTS (Click inside Webots 3D view):")
    print("    [F] : Toggle FRONT PILLAR (Raise / Lower obstacle)")
    print("    [B] : Toggle BACK PILLAR  (Raise / Lower obstacle)")
    print("    [1] : Option 1 - Breadcrumb Overlap & Auto 180 Correction")
    print("    [2] : Option 2 - Kinematic Minimum Arc Radius Filter (R >= R_min)")
    print("    [3] : Option 3 - Directional Flow Costmap Stamping")
    print("    [4] : Option 4 - Lateral Corridor Separation Watchdog")
    print("    [5] : Option Combined Hybrid (All layers active)")
    print("    [0] : Disable Watchdog (Pass-through)")
    print("    [R] : Reset / Teleport Robot to Start Pose (0, 0)")
    print("    [H] : Show Help Menu")
    print("================================================================================\n")

    if not os.path.exists(WORLD_PATH):
        print(f"Error: World file not found at {WORLD_PATH}", file=sys.stderr)
        sys.exit(1)

    cmd = [webots_bin, "--mode=realtime", WORLD_PATH]
    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\n[Launcher] Closed by user.")
    except Exception as e:
        print(f"[Launcher] Error launching Webots: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()
