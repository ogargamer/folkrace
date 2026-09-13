"""
================================================================================
  ONE-CLICK LAUNCHER & CONFIG SWITCHER FOR CHAMPION WEBOTS PREVIEW
================================================================================
  Allows selecting between all fastest champion configs from terminal,
  writing active_preview_config.json, and launching Webots in real-time mode.

  Usage:
    python launch_champion_preview.py             (Interactive menu)
    python launch_champion_preview.py --list      (List all fastest configs)
    python launch_champion_preview.py --config 1  (Launch #1 SL-MPC)
    python launch_champion_preview.py --config sl_mpc
    python launch_champion_preview.py --config septic
    python launch_champion_preview.py --config bvp
    python launch_champion_preview.py --config septic_base
================================================================================
"""

import os
import sys
import json
import argparse
import subprocess

# Determine directory paths robustly
CURRENT_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
# Check if inside webots/tools or webots/robotics/c++ septic/live_preview_and_launchers
if os.path.exists(os.path.join(CURRENT_FILE_DIR, "..", "optimization", "ga_planner_optimizer")):
    WEBOTS_DIR = os.path.abspath(os.path.join(CURRENT_FILE_DIR, ".."))
elif os.path.exists(os.path.join(CURRENT_FILE_DIR, "..", "..", "optimization", "ga_planner_optimizer")):
    WEBOTS_DIR = os.path.abspath(os.path.join(CURRENT_FILE_DIR, "..", ".."))
elif os.path.exists(os.path.join(CURRENT_FILE_DIR, "..", "..", "..", "optimization", "ga_planner_optimizer")):
    WEBOTS_DIR = os.path.abspath(os.path.join(CURRENT_FILE_DIR, "..", "..", ".."))
else:
    WEBOTS_DIR = os.path.abspath(os.path.join(CURRENT_FILE_DIR, ".."))

WORKSPACE_ROOT = os.path.dirname(WEBOTS_DIR)
GA_OPTIMIZER_DIR = os.path.join(WEBOTS_DIR, "optimization", "ga_planner_optimizer")
WORLD_PATH = os.path.join(GA_OPTIMIZER_DIR, "worlds", "Cyan_Champion_Live_Preview.wbt")
ACTIVE_CONFIG_PATH = os.path.join(GA_OPTIMIZER_DIR, "active_preview_config.json")
WEBOTS_BIN = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

CONFIGS_REGISTRY = [
    {
        "id": "sl_mpc",
        "aliases": ["1", "mpc", "sl_mpc", "option2", "opt2"],
        "title": "OPTION 2: Curvilinear SL-MPC Champion (CMA-ES Gen 194, Fitness 7170.03)",
        "file": os.path.join(GA_OPTIMIZER_DIR, "best_genome_option_2_sl_mpc.json"),
        "speed": "2.61 m/s",
        "dist": "48.87 m",
        "laps": "3 laps in 20.0s",
        "status": "Convex quadratic SL-MPC over Frenet lattice with optimized weights from ga_checkpoint.json.",
    },
    {
        "id": "septic_cpp",
        "aliases": ["2", "septic", "septic_cpp", "option3", "gen21"],
        "title": "OPTION 3: C++ Septic Spline Gen 21 Champion (Rank #2 Fastest)",
        "file": os.path.join(GA_OPTIMIZER_DIR, "best_planner_genome.json"),
        "speed": "5.35 m/s",
        "dist": "107.0 m",
        "laps": "7 laps in 20.0s",
        "status": "High-order septic polynomial with multi-threaded backward braking.",
    },
    {
        "id": "septic_base",
        "aliases": ["3", "septic_base", "septic_gen4", "septic_baseline"],
        "title": "OPTION 3: Septic Spline Baseline (Gen 4)",
        "file": os.path.join(GA_OPTIMIZER_DIR, "best_genome_option_3_septic.json"),
        "speed": "4.63 m/s",
        "dist": "92.7 m",
        "laps": "6 laps in 20.0s",
        "status": "Baseline septic spline candidate before C++ acceleration.",
    },
    {
        "id": "bvp",
        "aliases": ["4", "bvp", "lattice", "option1"],
        "title": "OPTION 1: BVP Dynamic State Lattice Champion",
        "file": os.path.join(GA_OPTIMIZER_DIR, "best_genome_option_1_bvp.json"),
        "speed": "4.51 m/s",
        "dist": "90.2 m",
        "laps": "6 laps in 20.0s",
        "status": "Boundary Value Problem polynomial lattice with dynamic obstacle cost.",
    },
]


def print_leaderboard():
    print("=" * 80)
    print("  FASTEST CHAMPION CONFIGURATIONS LEADERBOARD")
    print("=" * 80)
    for idx, cfg in enumerate(CONFIGS_REGISTRY, 1):
        print(f" [{idx}] {cfg['title']}")
        print(f"     Speed: {cfg['speed']} | Distance: {cfg['dist']} | Laps: {cfg['laps']}")
        print(f"     Status: {cfg['status']}")
        print(f"     Key CLI alias: --config {cfg['id']}")
        print("-" * 80)


def activate_config(cfg_item: dict) -> bool:
    src_file = cfg_item["file"]
    if not os.path.exists(src_file):
        print(f"Error: Genome file does not exist: {src_file}")
        return False

    try:
        with open(src_file, "r", encoding="utf-8") as f:
            data = json.load(f)

        data["config_id"] = cfg_item["id"]
        data["title"] = cfg_item["title"]

        with open(ACTIVE_CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        print(f"[Launcher] Activated configuration: {cfg_item['title']}")
        print(f"[Launcher] Written to: {ACTIVE_CONFIG_PATH}")
        return True
    except Exception as e:
        print(f"[Launcher] Failed to activate configuration: {e}")
        return False


def launch_webots(launch_gui: bool = False):
    if not os.path.exists(WEBOTS_BIN):
        print(f"Error: Webots executable not found at {WEBOTS_BIN}")
        return

    if not os.path.exists(WORLD_PATH):
        print(f"Error: World file not found at {WORLD_PATH}")
        return

    gui_proc = None
    if launch_gui:
        gui_script = os.path.join(GA_OPTIMIZER_DIR, "planners", "planner_gui.py")
        if os.path.exists(gui_script):
            print("[Launcher] Spawning Live Telemetry Map GUI (Port 9876)...", flush=True)
            gui_proc = subprocess.Popen([sys.executable, gui_script], cwd=os.path.join(GA_OPTIMIZER_DIR, "planners"))

    env = os.environ.copy()
    webots_home = r"C:\Program Files\Webots"
    webots_bin_dir = os.path.dirname(WEBOTS_BIN)
    py_dir = os.path.dirname(sys.executable)
    py_scripts = os.path.join(py_dir, "Scripts")

    env["WEBOTS_HOME"] = webots_home
    env["PATH"] = webots_bin_dir + os.pathsep + py_dir + os.pathsep + py_scripts + os.pathsep + env.get("PATH", "")
    env["PYTHONPATH"] = os.pathsep.join([
        GA_OPTIMIZER_DIR,
        os.path.join(GA_OPTIMIZER_DIR, "planners"),
        os.path.join(GA_OPTIMIZER_DIR, "controllers", "champion_live_preview"),
        os.path.join(GA_OPTIMIZER_DIR, "data"),
        WORKSPACE_ROOT,
    ])
    env["PYTHONUNBUFFERED"] = "1"

    cmd = [
        WEBOTS_BIN,
        "--mode=realtime",
        WORLD_PATH,
    ]

    print("=" * 80)
    print("  LAUNCHING REAL-TIME 3D WEBOTS CHAMPION PREVIEW")
    print(f"  World: {WORLD_PATH}")
    print("=" * 80)

    try:
        subprocess.run(cmd, env=env, cwd=GA_OPTIMIZER_DIR)
    finally:
        if gui_proc is not None and gui_proc.poll() is None:
            gui_proc.terminate()


def find_config(query: str):
    q_norm = query.strip().lower()
    for cfg in CONFIGS_REGISTRY:
        if q_norm == cfg["id"].lower() or q_norm in [a.lower() for a in cfg["aliases"]]:
            return cfg
    return None


def main():
    parser = argparse.ArgumentParser(description="Launch Webots Real-Time Preview with Selected Champion Config")
    parser.add_argument("--config", "-c", type=str, default=None, help="Config ID or number (1=SL-MPC, 2=Septic, 3=Septic Base, 4=BVP)")
    parser.add_argument("--list", "-l", action="store_true", help="List all available fastest configs")
    parser.add_argument("--gui", "-g", action="store_true", help="Launch live 2D telemetry map GUI alongside Webots")
    parser.add_argument("--no-launch", action="store_true", help="Only set active config without launching Webots")

    args = parser.parse_args()

    if args.list:
        print_leaderboard()
        return

    chosen_cfg = None

    if args.config:
        chosen_cfg = find_config(args.config)
        if not chosen_cfg:
            print(f"Error: Unknown configuration '{args.config}'.")
            print_leaderboard()
            return
    else:
        # Interactive terminal selection
        print_leaderboard()
        print("\nEnter choice (1-4) or press [Enter] for [1] SL-MPC Champion: ", end="", flush=True)
        try:
            user_input = sys.stdin.readline().strip()
        except Exception:
            user_input = ""

        if not user_input:
            user_input = "1"

        chosen_cfg = find_config(user_input)
        if not chosen_cfg:
            print(f"Invalid selection '{user_input}'. Defaulting to [1] SL-MPC Champion.")
            chosen_cfg = CONFIGS_REGISTRY[0]

    if not activate_config(chosen_cfg):
        return

    if not args.no_launch:
        launch_webots(launch_gui=args.gui)


if __name__ == "__main__":
    main()
