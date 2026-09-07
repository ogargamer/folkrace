# Genetic Algorithm (GA) MPPI Parameter Optimizer

This system provides automated hyperparameter optimization using parallel headless Webots simulation workers.

---

## 1. Overview & Architecture

- **World**: `my_project/worlds/Cyan_Champion_Track_GA.wbt`
- **Controller**: `my_project/controllers/mppi_ga_worker/mppi_ga_worker.py`
  - Runs headless without UDP / GUI overhead for maximum simulation speed.
  - Enforces strict 20.0-second evaluation windows.
  - Aborts immediately on fatal wall collisions or stalls (< 0.15 m/s for > 1.5s) to preserve CPU compute for viable candidates.
  - Emits trial metrics in structured JSON format.

---

## 2. Tools & Scripts

| File | Purpose |
| :--- | :--- |
| **`tools/ga_mppi_optimizer.py`** | Multi-worker parallel evolution engine with elitism, blend crossover, mutation, and adaptive diversity. |
| **`tools/ga_simulation_worker.py`** | Isolated evaluation worker that spawns Webots instances and calculates the fitness score. |
| **`tools/ga_analysis_plotter.py`** | Generates convergence graphs, Pareto trade-off curves, and parameter distributions (`ga_convergence_summary.png`). |
| **`winning_mppi_genome.json`** | All-time highest fitness champion genome with full parameter sets. |
| **`ga_checkpoint.json`** | Periodic snapshot of the active population for pause and resume. |

---

## 3. How to Run Optimization

### Start Optimization (6 Parallel Workers):
```pwsh
python tools/ga_mppi_optimizer.py --workers 6 --generations 50 --population 24
```

### Resume from Checkpoint:
```pwsh
python tools/ga_mppi_optimizer.py --resume
```

### Generate Performance & Convergence Plots:
```pwsh
python tools/ga_analysis_plotter.py
```

---

## 4. Fitness Function Structure

Trial fitness is calculated from simulation metrics:
- **Clean Surviving Run** (0 collisions, no stalls):
  - Distance Traveled: `+200 pts / meter`
  - Mean Speed: `+100 pts / (m/s)`
  - Lap Completion Bonus: `+2000 pts / lap`
  - Wall Hazard Penalty: `-0.5 pts / unit`
  - Steering Jerk Penalty: `-10 pts / (rad^2 variance)`
  - Results in positive scores (`+500 to +20,000 pts`).
- **Disqualified Run** (wall impact or stall):
  - Base penalty: `-50,000 pts`
  - Progress credit: `+10 pts / meter` before failure
  - Results in negative scores (`-50,000 to -48,000 pts`).
