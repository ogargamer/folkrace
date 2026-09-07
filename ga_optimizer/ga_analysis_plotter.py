"""
================================================================================
  GENETIC ALGORITHM PROGRESSION & CONVERGENCE PLOTTER
================================================================================
  Reads ga_history.csv and winning_mppi_genome.json, generates telemetry plots
  and prints comparative analysis against the baseline champion genome.
================================================================================
"""

import os
import sys
import json
import csv
from typing import Dict, List, Any
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")  # Non-interactive headless backend
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HISTORY_CSV_FILE = os.path.join(WORKSPACE_ROOT, "ga_history.csv")
WINNING_GENOME_FILE = os.path.join(WORKSPACE_ROOT, "winning_mppi_genome.json")
PLOT_OUTPUT_FILE = os.path.join(WORKSPACE_ROOT, "ga_convergence_summary.png")


def plot_ga_results(csv_path: str = HISTORY_CSV_FILE, out_png: str = PLOT_OUTPUT_FILE):
    if not os.path.exists(csv_path):
        print(f"[Plotter] History file not found: {csv_path}")
        return

    records = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append(row)

    if not records:
        print("[Plotter] No records found in history CSV.")
        return

    eval_ids = [int(r["eval_id"]) for r in records]
    generations = [int(r["generation"]) for r in records]
    fitnesses = [float(r["fitness"]) for r in records]
    all_best = [float(r["all_time_best"]) for r in records]
    distances = [float(r["distance_m"]) for r in records]
    speeds = [float(r["mean_speed_mps"]) for r in records]
    collisions = [int(r["collisions"]) for r in records]

    # Find best trial by fitness
    best_idx = int(np.argmax(fitnesses))
    best_r = records[best_idx]

    # Find longest distance trial
    max_d_idx = int(np.argmax(distances))
    max_d_r = records[max_d_idx]

    print(f"\n================================================================================", flush=True)
    print(f"               GENETIC ALGORITHM OPTIMIZATION SUMMARY REPORT                    ", flush=True)
    print(f"================================================================================", flush=True)
    print(f" Total Trials Evaluated : {len(records)}")
    print(f" Total Generations      : {max(generations) + 1 if generations else 0}")
    print(f" Baseline Fitness       : {fitnesses[0]:.2f}")
    print(f" Peak Champion Fitness  : {float(best_r['fitness']):.2f} (Eval #{best_r['eval_id']}, Gen {best_r['generation']})")
    print(f"   -> Champion Distance : {float(best_r['distance_m']):.2f} m")
    print(f"   -> Champion Speed    : {float(best_r['mean_speed_mps']):.2f} m/s ({float(best_r['mean_speed_mps'])*3.6:.1f} km/h)")
    print(f"   -> Champion Collide  : {best_r['collisions']}")
    print(f" Longest Distance Run   : {float(max_d_r['distance_m']):.2f} m (at {float(max_d_r['mean_speed_mps']):.2f} m/s, Eval #{max_d_r['eval_id']})")
    print(f" Max Instant Speed Seen : {max(speeds):.2f} m/s ({max(speeds)*3.6:.1f} km/h)")

    if os.path.exists(WINNING_GENOME_FILE):
        try:
            with open(WINNING_GENOME_FILE, "r", encoding="utf-8") as gf:
                champ = json.load(gf)
            print("\nWinning Genome Parameters:")
            genes = champ.get("genes", {})
            for g_name, g_val in genes.items():
                if isinstance(g_val, float):
                    print(f"  {g_name:<24} : {g_val:.4f}")
                else:
                    print(f"  {g_name:<24} : {g_val}")
        except Exception as e:
            print(f"Error loading winning genome: {e}")

    print("================================================================================\n", flush=True)

    if not MATPLOTLIB_AVAILABLE:
        print("[Plotter] Matplotlib not available for graphical chart generation.")
        return

    # 4-Subplot Dashboard
    fig, axs = plt.subplots(2, 2, figsize=(14, 10), dpi=150)
    fig.patch.set_facecolor("#121820")

    text_color = "#E0E6ED"
    grid_color = "#2A3545"

    for ax in axs.flat:
        ax.set_facecolor("#1A2332")
        ax.tick_params(colors=text_color)
        ax.xaxis.label.set_color(text_color)
        ax.yaxis.label.set_color(text_color)
        ax.title.set_color(text_color)
        ax.grid(True, linestyle="--", alpha=0.4, color=grid_color)
        for spine in ax.spines.values():
            spine.set_color("#3A485E")

    # Plot 1: Fitness Convergence
    axs[0, 0].plot(eval_ids, fitnesses, color="#4D96FF", alpha=0.35, label="Candidate Fitness")
    axs[0, 0].plot(eval_ids, all_best, color="#00F0FF", linewidth=2.0, label="All-Time Best")
    axs[0, 0].set_title("Fitness Score Convergence", fontsize=12, fontweight="bold")
    axs[0, 0].set_xlabel("Evaluation Index")
    axs[0, 0].set_ylabel("Fitness Score")
    axs[0, 0].legend(facecolor="#1A2332", edgecolor="#3A485E", labelcolor=text_color)

    # Plot 2: Distance & Speed
    ax2_twin = axs[0, 1].twinx()
    ax2_twin.tick_params(colors="#FFB830")
    ax2_twin.yaxis.label.set_color("#FFB830")

    l1 = axs[0, 1].plot(eval_ids, distances, color="#00E676", alpha=0.7, label="Distance Traveled (m)")
    l2 = ax2_twin.plot(eval_ids, speeds, color="#FFB830", alpha=0.7, label="Mean Speed (m/s)")
    axs[0, 1].set_title("Distance (20s) & Average Speed Progression", fontsize=12, fontweight="bold")
    axs[0, 1].set_xlabel("Evaluation Index")
    axs[0, 1].set_ylabel("Distance (m)")
    ax2_twin.set_ylabel("Speed (m/s)")

    # Plot 3: Collisions vs Trials
    axs[1, 0].scatter(eval_ids, collisions, color="#FF3366", alpha=0.5, s=12, label="Collisions")
    axs[1, 0].set_title("Collision Incidents across Trials", fontsize=12, fontweight="bold")
    axs[1, 0].set_xlabel("Evaluation Index")
    axs[1, 0].set_ylabel("Collision Count")
    axs[1, 0].legend(facecolor="#1A2332", edgecolor="#3A485E", labelcolor=text_color)

    # Plot 4: Key Gene Hyperparameter Evolution
    if "mu_friction" in records[0] and "lambda_temp" in records[0]:
        mu_vals = [float(r["mu_friction"]) for r in records]
        temp_vals = [float(r["lambda_temp"]) for r in records]
        axs[1, 1].plot(eval_ids, mu_vals, color="#FF6B6B", label="mu_friction [0.60 - 0.90]")
        axs[1, 1].plot(eval_ids, temp_vals, color="#4D96FF", label="lambda_temp [0.01 - 2.0]")
        axs[1, 1].set_title("Core Gene Adaptation Trajectories", fontsize=12, fontweight="bold")
        axs[1, 1].set_xlabel("Evaluation Index")
        axs[1, 1].set_ylabel("Parameter Value")
        axs[1, 1].legend(facecolor="#1A2332", edgecolor="#3A485E", labelcolor=text_color)

    plt.tight_layout()
    plt.savefig(out_png, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()
    print(f"[Plotter] Saved convergence visualization to {out_png}")


if __name__ == "__main__":
    plot_ga_results()
