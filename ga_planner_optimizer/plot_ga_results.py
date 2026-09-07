"""
================================================================================
  GENETIC ALGORITHM CONVERGENCE & PERFORMANCE PLOTTER
================================================================================
  Plots Fitness Progression, Distance Traveled, Speed, and Crash Rates
  across generations from ga_history.csv.
================================================================================
"""

import os
import sys
import csv
import argparse
from typing import Dict, List, Any
import matplotlib.pyplot as plt
import numpy as np

CURR_DIR = os.path.dirname(os.path.abspath(__file__))
HISTORY_CSV_FILE = os.path.join(CURR_DIR, "ga_history.csv")
OUTPUT_PLOT_FILE = os.path.join(CURR_DIR, "ga_convergence_summary.png")


def plot_ga_history(csv_path: str = HISTORY_CSV_FILE, out_path: str = OUTPUT_PLOT_FILE):
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found.")
        return

    data_by_planner: Dict[str, Dict[str, List[Any]]] = {}

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            planner = row.get("Planner", "OPTION_1_BVP")
            if planner not in data_by_planner:
                data_by_planner[planner] = {
                    "gen": [],
                    "best_fit": [],
                    "mean_fit": [],
                    "dist": [],
                    "speed": [],
                    "crashes": [],
                }
            data_by_planner[planner]["gen"].append(int(row["Generation"]))
            data_by_planner[planner]["best_fit"].append(float(row["Best_Fitness"]))
            data_by_planner[planner]["mean_fit"].append(float(row["Mean_Fitness"]))
            data_by_planner[planner]["dist"].append(float(row["Best_Distance_m"]))
            data_by_planner[planner]["speed"].append(float(row["Best_Speed_mps"]))
            data_by_planner[planner]["crashes"].append(int(row["Crash_Count"]))

    if not data_by_planner:
        print("No data points found in history CSV.")
        return

    colors = {
        "OPTION_1_BVP": ("#10B981", "#059669"),
        "OPTION_2_SL_MPC": ("#3B82F6", "#2563EB"),
        "OPTION_3_SEPTIC": ("#EC4899", "#DB2777"),
    }
    fallback_colors = [("#8B5CF6", "#7C3AED"), ("#F59E0B", "#D97706"), ("#06B6D4", "#0891B2")]

    fig, axs = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle("Autonomous ToF Genetic Algorithm Optimization Progress", fontsize=16, fontweight="bold")

    for i, (p_name, p_data) in enumerate(data_by_planner.items()):
        c_main, c_sub = colors.get(p_name, fallback_colors[i % len(fallback_colors)])
        gens = p_data["gen"]

        # 1. Fitness Curve
        axs[0, 0].plot(gens, p_data["best_fit"], "-o", color=c_main, label=f"{p_name} (Best)", linewidth=2)
        axs[0, 0].plot(gens, p_data["mean_fit"], "--", color=c_sub, alpha=0.5, label=f"{p_name} (Mean)")

        # 2. Distance Traveled (20s Window)
        axs[0, 1].plot(gens, p_data["dist"], "-s", color=c_main, label=f"{p_name}", linewidth=2)

        # 3. Average Forward Velocity
        axs[1, 0].plot(gens, p_data["speed"], "-^", color=c_main, label=f"{p_name}", linewidth=2)

        # 4. Crash Count
        width = 0.25
        offset = (i - (len(data_by_planner) - 1) / 2.0) * width
        axs[1, 1].bar(np.array(gens) + offset, p_data["crashes"], width=width, color=c_main, alpha=0.75, label=f"{p_name}")

    # Subplot formatting
    axs[0, 0].set_title("Fitness Score Progression (Higher is Better)", fontweight="bold")
    axs[0, 0].set_xlabel("Generation")
    axs[0, 0].set_ylabel("Fitness Score")
    axs[0, 0].grid(True, linestyle=":", alpha=0.6)
    axs[0, 0].legend()

    axs[0, 1].axhline(y=14.109, color="orange", linestyle="--", label="1 Full Lap (14.11m)")
    axs[0, 1].axhline(y=28.218, color="red", linestyle=":", label="2 Full Laps (28.22m)")
    axs[0, 1].set_title("Distance Achieved in 20.0 Seconds", fontweight="bold")
    axs[0, 1].set_xlabel("Generation")
    axs[0, 1].set_ylabel("Distance (meters)")
    axs[0, 1].grid(True, linestyle=":", alpha=0.6)
    axs[0, 1].legend()

    axs[1, 0].set_title("Average Velocity Maintained", fontweight="bold")
    axs[1, 0].set_xlabel("Generation")
    axs[1, 0].set_ylabel("Speed (m/s)")
    axs[1, 0].grid(True, linestyle=":", alpha=0.6)
    axs[1, 0].legend()

    axs[1, 1].set_title("Population Fatal Failure Count", fontweight="bold")
    axs[1, 1].set_xlabel("Generation")
    axs[1, 1].set_ylabel("Number of Failed Trials")
    axs[1, 1].grid(True, linestyle=":", alpha=0.6)
    axs[1, 1].legend()

    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    print(f"Convergence plot saved to: {out_path}")


if __name__ == "__main__":
    plot_ga_history()
