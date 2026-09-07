"""
================================================================================
  FACT-CHECKING & BENCHMARK: C++ EXACT CONTINUOUS SMART HYBRID (15mm)
  - Evaluates on 8 ground-truth datasets (16,000+ points)
  - Checks Surface Error (cm), Containment Rate (%), Frontal Overextension (cm)
  - Measures true C++ execution time and throughput
================================================================================
"""

import os
import sys
import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
if curr_dir not in sys.path:
    sys.path.insert(0, curr_dir)

from cpp_smart_hybrid_costmap import CppSmartHybridCostmap
from benchmark_costmap_variants import evaluate_engine_on_dataset, DATASETS_DIR

test_configs = [
    ("pillar", 0.05, "5 cm Pillar (Dia 5 cm)"),
    ("cube",   0.05, "5 cm Cube (Width 5 cm)"),
    ("pillar", 0.10, "10 cm Pillar (Dia 10 cm)"),
    ("cube",   0.10, "10 cm Cube (Width 10 cm)"),
    ("pillar", 0.15, "15 cm Pillar (Dia 15 cm)"),
    ("cube",   0.15, "15 cm Cube (Width 15 cm)"),
    ("pillar", 0.20, "20 cm Pillar (Dia 20 cm)"),
    ("cube",   0.20, "20 cm Cube (Width 20 cm)"),
]

print("\n" + "=" * 95, flush=True)
print("  FACT-CHECKING C++ EXACT CONTINUOUS SMART HYBRID COSTMAP (15mm)", flush=True)
print("=" * 95, flush=True)

results = []

for shape, size, label in test_configs:
    size_cm = int(size * 100)
    csv_name = f"tof_readings_{shape}_{size_cm}cm.csv"
    if shape == "pillar" and size == 0.05:
        legacy_file = os.path.join(DATASETS_DIR, "tof_16ch_grid_readings.csv")
        csv_path = legacy_file if os.path.exists(legacy_file) else os.path.join(DATASETS_DIR, csv_name)
    else:
        csv_path = os.path.join(DATASETS_DIR, csv_name)

    if not os.path.exists(csv_path):
        continue

    # Initialize C++ engine at 15 mm resolution with bounds (-1.0, 1.0, 0.0, 2.0)
    # r_inflation_m is set to 0.05m (5cm) matching the standard validator
    engine = CppSmartHybridCostmap(
        grid_bounds=(-1.0, 1.0, 0.0, 2.0),
        resolution=0.015,
        r_inflation_m=0.05,
    )

    res = evaluate_engine_on_dataset(
        engine=engine,
        csv_file=csv_path,
        shape_type=shape,
        size_m=size,
        grid_res=0.015,
        noise_pct=0.05,
        name="C++ Smart Hybrid (15mm)",
    )
    res["label"] = label
    results.append(res)

    print(
        f"  {label:<30} | "
        f"Time: {res['mean_exec_ms']:.3f} ms ({res['throughput_hz']:>5.0f} Hz) | "
        f"Contain: {res['containment_pct']:>5.1f}% | "
        f"Mean Err: {res['mean_error_cm']:>4.2f} cm | "
        f"Close Overext: {res['close_overext_mean_cm']:>4.2f} cm | "
        f"OK Rate: {res['ok_rate_pct']:>5.1f}%",
        flush=True,
    )

print("=" * 95 + "\n", flush=True)
