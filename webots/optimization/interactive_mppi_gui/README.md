# Interactive 16-Channel ToF Costmap & SVG-MPPI GUI System

This system provides real-time 3D simulation and a 2D high-fidelity PyQt5 costmap visualizer for the autonomous 4WD Folkrace racing vehicle.

---

## 1. Quick Start

Open two terminals in the workspace:

### Terminal 1: Launch 3D Webots Simulator
```pwsh
python visualize_champion_gui.py
```
- Automatically opens Webots in real-time continuous driving mode.
- Uses `webots/simulation/my_project/worlds/Cyan_Champion_Track.wbt` with the dedicated `mppi_controller`.
- **Runs indefinitely** without trial aborts or early termination.
- MPPI reverse recovery maneuvers automatically if the vehicle approaches tight obstacles.

### Terminal 2: Launch 2D Live Costmap & MPPI Rollouts Visualizer
```pwsh
python launch_costmap_gui.py
```
- Connects immediately to Webots via local UDP broadcast (Port 9876).
- Displays the fused 20mm local costmap and 16-channel conical ToF range beams.
- Overlays the **3 distinct SVG-MPPI nominal paths** alongside the chosen optimal trajectory.

---

## 2. Visualizer Layers & Color Coding

| Visual Element | Color & Style | Description |
| :--- | :--- | :--- |
| **Optimal Path** | **Bright Green** (`#00FF88`, Glowing, Solid) | The actual MPPI trajectory executed by the electronic differential. |
| **Center Seed Line** | **Bright Cyan** (`#00E5FF`, Dashed) | Pure Pursuit nominal candidate aligned with the centerline racing trajectory. |
| **Left Seed Line** | **Magenta** (`#E040FB`, Dashed) | Left-offset candidate path (+25 cm offset). |
| **Right Seed Line** | **Amber / Yellow** (`#FFD740`, Dashed) | Right-offset candidate path (-25 cm offset). |
| **Exploration Fan** | **Faint Blue** (`#3C9FF0`, Thin) | Parallel sampled candidate rollouts evaluating surrounding variations. |
| **C-Space Hazard** | **Bright Red** | Dilated lethal obstacle envelope enforcing chassis safety margins. |
| **ToF Beams** | **Green / Orange Cones** | 16-zone 11.25-degree conical optical range rays (up to 1.80m). |

---

## 3. Interactive GUI Controls (Port 9877)

The GUI right sidebar includes bi-directional simulation controls:
- **Play / Pause**: Instantly pauses or resumes the Webots physics engine and vehicle movement.
- **Step (10ms)**: Advances the simulation physics and MPPI planning by exactly one tick for frame-by-frame inspection.
- **Speed Multiplier**: Throttle playback rate dynamically:
  - `0.05x` (1/20th speed slow-motion)
  - `0.10x` (10x slow-motion)
  - `0.25x` (quarter speed)
  - `0.50x` (half speed)
  - `1.00x` (real-time 1x)
  - `2.00x` (2x fast-forward)
  - `5.00x` & `Max Speed` (uncapped simulation speed)
- **Reset Track & Vehicle**: Repositions the car to `[0.60, 0.00, 0.04]` and resets telemetry accumulators.

---

## 4. Manual Parameter Tuning

Parameters can be adjusted directly in `winning_mppi_genome.json` under `"genes"`. Key parameters include:
- `w_hazard`: Penalty for entering the C-space inflated obstacle safety zone.
- `w_prob_costmap`: Proximity penalty for approaching walls and moving obstacles.
- `w_unknown`: Penalty for traversing unobserved grid cells.
- `w_cte`: Weight keeping the vehicle glued to the racing line when clear.
- `r_inflation_m`: Inflation radius (`0.080 m to 0.130 m` provides `0.5 cm to 5.5 cm` clearance for the 15 cm wide chassis).
