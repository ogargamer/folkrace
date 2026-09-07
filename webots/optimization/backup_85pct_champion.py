import os
import shutil
import json

BACKUP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "racing_results", "backup_85pct_champion")
CONTROLLER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots/simulation/my_project", "controllers", "folkrace_drive")

os.makedirs(BACKUP_DIR, exist_ok=True)

files_to_backup = [
    ("folkrace_config.json", os.path.join(CONTROLLER_DIR, "folkrace_config.json")),
    ("cyan_champion_trajectory.csv", os.path.join(CONTROLLER_DIR, "cyan_champion_trajectory.csv")),
    ("folkrace_drive.py", os.path.join(CONTROLLER_DIR, "folkrace_drive.py")),
    ("tof_pathfinders.py", os.path.join(CONTROLLER_DIR, "tof_pathfinders.py")),
]

for fname, src in files_to_backup:
    dst = os.path.join(BACKUP_DIR, fname)
    shutil.copyfile(src, dst)
    print(f"[BACKUP] Saved {fname} to {dst}")

summary = {
    "benchmark_name": "Champion 85% Pace Baseline",
    "lap_time_s": 5.87,
    "avg_speed_mps": 2.38,
    "avg_speed_kmh": 8.57,
    "max_deviation_cm": 7.5,
    "crashes": 0,
    "pace_pct": 85.0,
    "active_pathfinder": "FRENETQP",
    "track_source": "winner_margin_14cm.csv",
    "wall_time_s": 11.7
}

with open(os.path.join(BACKUP_DIR, "telemetry_summary.json"), "w") as f:
    json.dump(summary, f, indent=2)

print("\n[BACKUP COMPLETE] Successfully backed up 85% pace champion configuration!")
