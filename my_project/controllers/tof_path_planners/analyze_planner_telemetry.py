"""
================================================================================
  TELEMETRY ANALYZER FOR TOF AUTONOMOUS PATH PLANNERS
  Parses high-rate JSONL telemetry to quantify performance, identify bottlenecks,
  and provide tuning recommendations.
================================================================================
"""

import os
import sys
import json
import numpy as np


def analyze_telemetry_file(filepath: str) -> dict:
    if not os.path.exists(filepath):
        print(f"[Warning] Telemetry file not found: {filepath}")
        return {}

    records = []
    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line_str = line.strip()
            if line_str:
                try:
                    records.append(json.loads(line_str))
                except Exception:
                    pass

    if not records:
        print(f"[Warning] No valid records in {filepath}")
        return {}

    planner = records[0].get("planner", "UNKNOWN")
    n_steps = len(records)
    total_time = records[-1]["t"] - records[0]["t"]

    speeds = [r["v_est"] for r in records]
    target_speeds = [r["v_tgt"] for r in records]
    steers = [r["steer_rad"] for r in records]
    plan_times = [r["plan_ms"] for r in records]
    map_times = [r["map_ms"] for r in records]
    min_tofs = [r["min_tof"] for r in records]
    front_tofs = [r["front_min"] for r in records]
    distances = [r["dist_m"] for r in records]
    statuses = [r["status"] for r in records]

    # Metrics
    max_dist = max(distances) if distances else 0.0
    mean_speed = float(np.mean(speeds))
    max_speed = float(np.max(speeds))
    mean_plan_ms = float(np.mean(plan_times))
    max_plan_ms = float(np.max(plan_times))
    mean_map_ms = float(np.mean(map_times))

    status_counts = {}
    for s in statuses:
        status_counts[s] = status_counts.get(s, 0) + 1

    optimal_count = sum(count for s, count in status_counts.items() if "OPTIMAL" in s)
    optimal_pct = (optimal_count / n_steps) * 100.0 if n_steps > 0 else 0.0

    # Pruning / Fallback causes
    fallback_count = n_steps - optimal_count

    # Jerk / steering variance
    steer_var = float(np.var(steers))
    steer_diffs = np.diff(steers)
    mean_steer_rate = float(np.mean(np.abs(steer_diffs))) / 0.02 if len(steer_diffs) > 0 else 0.0

    summary = {
        "planner": planner,
        "steps": n_steps,
        "duration_s": round(total_time, 2),
        "total_distance_m": round(max_dist, 3),
        "mean_speed_mps": round(mean_speed, 3),
        "max_speed_mps": round(max_speed, 3),
        "optimal_pct": round(optimal_pct, 1),
        "fallback_count": fallback_count,
        "mean_plan_ms": round(mean_plan_ms, 2),
        "max_plan_ms": round(max_plan_ms, 2),
        "mean_map_ms": round(mean_map_ms, 2),
        "steer_variance": round(steer_var, 5),
        "mean_steer_rate_rad_s": round(mean_steer_rate, 2),
        "min_wall_dist_m": round(min(min_tofs), 3) if min_tofs else 0.0,
        "status_distribution": status_counts,
    }

    print("\n" + "=" * 80)
    print(f"  TELEMETRY ANALYSIS REPORT: {planner}")
    print("=" * 80)
    print(f"  Total Distance Traveled: {summary['total_distance_m']:6.2f} m in {summary['duration_s']:5.2f} s")
    print(f"  Forward Speed          : Mean = {summary['mean_speed_mps']:4.2f} m/s | Peak = {summary['max_speed_mps']:4.2f} m/s")
    print(f"  Optimal Trajectories   : {summary['optimal_pct']:5.1f}% ({optimal_count}/{n_steps} steps)")
    print(f"  Fallback Engagements   : {fallback_count} steps")
    print(f"  Execution Latency      : Planner = {summary['mean_plan_ms']:4.2f} ms (Peak: {summary['max_plan_ms']:4.2f} ms) | Map = {summary['mean_map_ms']:4.2f} ms")
    print(f"  Steering Smoothness    : Variance = {summary['steer_variance']:.5f} | Mean Slew = {summary['mean_steer_rate_rad_s']:4.2f} rad/s")
    print(f"  Wall Clearance (Min)   : {summary['min_wall_dist_m']*100:4.1f} cm")
    print("  Status Breakdown:")
    for stat, count in status_counts.items():
        print(f"    - {stat:25s}: {count:4d} ({count*100.0/n_steps:5.1f}%)")
    print("=" * 80)
    return summary


if __name__ == "__main__":
    workspace_root = r"c:\Users\nurmi\Documents\Da Winna"
    if len(sys.argv) > 1:
        files = sys.argv[1:]
    else:
        files = [
            os.path.join(workspace_root, "telemetry_option1_bvp.jsonl"),
            os.path.join(workspace_root, "telemetry_option2_sl_mpc.jsonl"),
            os.path.join(workspace_root, "telemetry_option3_septic.jsonl"),
        ]

    for f in files:
        analyze_telemetry_file(f)
