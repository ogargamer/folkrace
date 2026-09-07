import csv
import math
import sys
import numpy as np
from controller import Supervisor

TIME_STEP = 10
MAX_RANGE = 1.8
X_MIN, X_MAX = -0.925, 0.925
Y_MIN, Y_MAX = 0.150, 1.800
FIXED_Z = 0.06

# Target object configurations: (Shape, Size in meters, Label)
# Supported shapes: "cylinder" (pillar) or "box" (cube)
TARGET_CONFIG = {
    "shape": "cylinder",  # "cylinder" or "box"
    "size_m": 0.10,       # 0.05 (5cm), 0.10 (10cm), 0.15 (15cm), 0.20 (20cm)
}

# Grid step is set to half-width of the target object
GRID_STEP = TARGET_CONFIG["size_m"] / 2.0

robot = Supervisor()

sensor_names = [
    "tof_fl_0", "tof_fl_1", "tof_fl_2", "tof_fl_3",
    "tof_fr_0", "tof_fr_1", "tof_fr_2", "tof_fr_3",
    "tof_bl_0", "tof_bl_1", "tof_bl_2", "tof_bl_3",
    "tof_br_0", "tof_br_1", "tof_br_2", "tof_br_3",
]

sensors = []
for name in sensor_names:
    device = robot.getDevice(name)
    if device is None:
        sys.exit(f"Error: Sensor '{name}' not found on robot.")
    device.enable(TIME_STEP)
    sensors.append(device)

obstacle = robot.getFromDef("OBSTACLE")
if obstacle is None:
    sys.exit("Error: Solid with DEF 'OBSTACLE' missing from scene tree.")
translation_field = obstacle.getField("translation")

x_points = np.arange(X_MIN, X_MAX + GRID_STEP / 2.0, GRID_STEP)
y_points = np.arange(Y_MIN, Y_MAX + GRID_STEP / 2.0, GRID_STEP)
total_positions = len(x_points) * len(y_points)
print(f"[Scanner] Target: {TARGET_CONFIG['shape']} {TARGET_CONFIG['size_m']*100:.0f}cm (Grid Step = {GRID_STEP*100:.1f}cm)")
print(f"[Scanner] Scanning {len(x_points)} x {len(y_points)} = {total_positions} positions across 16 ToF zones...")

robot.step(TIME_STEP)
dataset = []
interrupted = False

for x in x_points:
    for y in y_points:
        translation_field.setSFVec3f([float(x), float(y), FIXED_Z])
        if robot.step(TIME_STEP) == -1:
            interrupted = True
            break

        row = [round(float(x), 4), round(float(y), 4)]
        for sensor in sensors:
            range_image = sensor.getRangeImage()
            valid_ranges = [
                value for value in range_image
                if not math.isinf(value) and not math.isnan(value) and value <= MAX_RANGE
            ] if range_image else []
            row.append(round(float(min(valid_ranges) if valid_ranges else MAX_RANGE), 4))
        dataset.append(row)
    if interrupted:
        break

script_dir = os.path.dirname(os.path.abspath(__file__))
datasets_dir = os.path.join(script_dir, "data", "datasets")
os.makedirs(datasets_dir, exist_ok=True)

shape_name = "pillar" if TARGET_CONFIG["shape"] == "cylinder" else "cube"
output_filename = os.path.join(datasets_dir, f"tof_readings_{shape_name}_{int(TARGET_CONFIG['size_m']*100)}cm.csv")
with open(output_filename, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["x_m", "y_m", *sensor_names])
    writer.writerows(dataset)

print(f"[Scanner] {'Interrupted' if interrupted else 'Complete'}. {len(dataset)} rows saved to '{output_filename}'.")
if not interrupted:
    robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
