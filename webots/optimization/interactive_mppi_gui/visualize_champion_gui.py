"""
Launches the current all-time best MPPIGenome from winning_mppi_genome.json
inside Webots in interactive 3D GUI mode so you can visually watch it drive!
"""
import os
import sys
import json
import subprocess
from webots/simulation/my_project.controllers.folkrace_drive.mppi_folkrace_controller import MPPIGenome

WORKSPACE_ROOT = os.path.dirname(os.path.abspath(__file__))
WINNING_GENOME_FILE = os.path.join(WORKSPACE_ROOT, "winning_mppi_genome.json")
DEFAULT_WORLD = os.path.join(WORKSPACE_ROOT, "webots/simulation/my_project", "worlds", "Cyan_Champion_Track.wbt")

def find_webots_bin() -> str:
    candidates = [
        r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
        r"C:\Program Files\Webots\webots.exe",
        r"C:\Program Files\Webots\bin\webots.exe",
        os.path.join(os.environ.get("WEBOTS_HOME", ""), "msys64", "mingw64", "bin", "webots.exe"),
        os.path.join(os.environ.get("WEBOTS_HOME", ""), "webots.exe"),
        r"C:\Users\nurmi\AppData\Local\Programs\Webots\msys64\mingw64\bin\webots.exe",
        r"C:\Users\nurmi\AppData\Local\Programs\Webots\webots.exe",
    ]
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return "webots"

DEFAULT_WEBOTS_BIN = find_webots_bin()

def main():
    if not os.path.exists(WINNING_GENOME_FILE):
        print(f"Error: {WINNING_GENOME_FILE} not found. Run GA training first.")
        return

    with open(WINNING_GENOME_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    genes = data.get("genes", {})
    genome = MPPIGenome.from_dict(genes)

    print("=" * 80)
    print("  LAUNCHING CHAMPION MPPI CONTROLLER IN WEBOTS 3D GUI")
    print(f"  Distance Achieved: {data.get('distance_m', 0.0):.2f} m | Collisions: {data.get('collision_count', 0)}")
    print(f"  Horizon: H={genome.H}, Samples: K={genome.K}, dt={genome.dt:.3f}s")
    print(f"  Friction: mu={genome.mu_friction:.2f}, Inflation: {genome.r_inflation_m*100:.1f} cm")
    print("=" * 80)

    env = os.environ.copy()
    env["MPPI_GENOME_JSON"] = json.dumps(genome.to_dict())
    env["MPPI_PORT_OFFSET"] = "0"
    env["ENABLE_MOVING_OBSTACLE"] = "1"
    env["MOVING_OBS_SPEED"] = "0.50"
    env["PYTHONUNBUFFERED"] = "1"

    cmd = [DEFAULT_WEBOTS_BIN, "--mode=realtime", DEFAULT_WORLD]
    print(f"Starting Webots: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, env=env)
    proc.wait()

if __name__ == "__main__":
    main()
