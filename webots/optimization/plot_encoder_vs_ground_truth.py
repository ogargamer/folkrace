"""
Plot Ground Truth Velocity vs Wheel Encoders vs RPM-IMU Fusion
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt

WHEEL_RADIUS = 0.0275
MOTOR_DIR = -1.0
WHEELBASE = 0.1325
TRACK_REAR = 0.1000

jsonl_path = os.path.abspath(r"webots/simulation/my_project\controllers\folkrace_drive\live_webots_slam_scans.jsonl")

records = []
with open(jsonl_path, "r", encoding="utf-8") as f:
    for line in f:
        line_str = line.strip()
        if line_str:
            try:
                records.append(json.loads(line_str))
            except Exception:
                pass

times = []
v_gt_x = []
v_gt_y = []
v_wheel = []
v_fus_x = []
v_fus_y = []
gyro_z = []
accel_x = []
accel_y = []
pos_gt_x = []
pos_gt_y = []

# Fusion simulation
vx_f = 0.0
vy_f = 0.0
traj_gt_x = []
traj_gt_y = []
traj_wheel_gyro_x = []
traj_wheel_gyro_y = []
traj_fus_x = []
traj_fus_y = []

x_wg, y_wg, yaw_wg = 0.6, 0.0, 0.0
x_f, y_f, yaw_f = 0.6, 0.0, 0.0
last_t = None

for r in records:
    t = float(r.get("time", 0.0))
    dt = t - last_t if last_t is not None else 0.01
    if dt <= 0 or dt > 0.1: dt = 0.01

    gt_x = float(r.get("v_gt_x", 0.0))
    gt_y = float(r.get("v_gt_y", 0.0))
    pos = r.get("pos", [0.0, 0.0])
    gz = float(r.get("gyro_z", r.get("gyro", [0, 0, 0])[2] if isinstance(r.get("gyro"), list) else 0.0))
    ax = float(r.get("accel_x", r.get("accel", [0, 0, 0])[0] if isinstance(r.get("accel"), list) else 0.0))
    ay = float(r.get("accel_y", r.get("accel", [0, 0, 0])[1] if isinstance(r.get("accel"), list) else 0.0))

    w_rl = float(r.get("omega_RL", r.get("w_rl", 0.0)))
    w_rr = float(r.get("omega_RR", r.get("w_rr", 0.0)))

    vw_rl = (w_rl * WHEEL_RADIUS) / MOTOR_DIR
    vw_rr = (w_rr * WHEEL_RADIUS) / MOTOR_DIR
    vw = 0.5 * (vw_rl + vw_rr)

    # Fusion filter (90% IMU + 10% wheel)
    accel_wheel = (vw - vx_f) / dt
    if abs(accel_wheel) > 12.0:
        vx_f += ax * dt
    else:
        v_proj = vx_f + ax * dt
        vx_f = (0.90 * v_proj) + (0.10 * vw)
    vx_f = max(-4.5, min(4.5, vx_f))

    slip_ay = ay - (vx_f * gz)
    vy_f += slip_ay * dt
    vy_f *= np.exp(-dt / 0.15)

    # Odometry integration
    yaw_wg += gz * dt
    x_wg += vw * np.cos(yaw_wg) * dt
    y_wg += vw * np.sin(yaw_wg) * dt

    yaw_f += gz * dt
    mid_yaw = yaw_f - (gz * dt * 0.5)
    dx_g = (vx_f * np.cos(mid_yaw)) - (vy_f * np.sin(mid_yaw))
    dy_g = (vx_f * np.sin(mid_yaw)) + (vy_f * np.cos(mid_yaw))
    x_f += dx_g * dt
    y_f += dy_g * dt

    times.append(t)
    v_gt_x.append(gt_x)
    v_gt_y.append(gt_y)
    v_wheel.append(vw)
    v_fus_x.append(vx_f)
    v_fus_y.append(vy_f)
    gyro_z.append(gz)
    accel_x.append(ax)
    accel_y.append(ay)
    pos_gt_x.append(pos[0])
    pos_gt_y.append(pos[1])

    traj_wheel_gyro_x.append(x_wg)
    traj_wheel_gyro_y.append(y_wg)
    traj_fus_x.append(x_f)
    traj_fus_y.append(y_f)

    last_t = t

times = np.array(times)
v_gt_x = np.array(v_gt_x)
v_gt_y = np.array(v_gt_y)
v_wheel = np.array(v_wheel)
v_fus_x = np.array(v_fus_x)
v_fus_y = np.array(v_fus_y)
pos_gt_x = np.array(pos_gt_x)
pos_gt_y = np.array(pos_gt_y)

# Create high-quality diagnostic figure
fig = plt.figure(figsize=(14, 10), dpi=200, facecolor='#0d1117')

# 1. Forward Speed Comparison
ax1 = fig.add_subplot(2, 2, 1, facecolor='#0d1117')
ax1.plot(times, v_gt_x, color='#58a6ff', label='Ground Truth Speed (Webots)', linewidth=2.0)
ax1.plot(times, v_wheel, color='#e3b341', label='Raw Wheel Encoders (V_wheel)', linewidth=1.2, linestyle='--')
ax1.plot(times, v_fus_x, color='#3fb950', label='RPM-IMU Fusion 90%', linewidth=1.2, linestyle=':')
ax1.set_title('Forward Velocity (Vx) vs Time', color='white', fontsize=11, fontweight='bold')
ax1.set_xlabel('Simulation Time (s)', color='#c9d1d9', fontsize=9)
ax1.set_ylabel('Speed (m/s)', color='#c9d1d9', fontsize=9)
ax1.legend(loc='upper right', facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9', fontsize=8)
ax1.grid(True, color='#21262d', linestyle='--', alpha=0.7)
ax1.tick_params(colors='#8b949e')
for s in ax1.spines.values(): s.set_color('#30363d')

# 2. Velocity Error vs Ground Truth
ax2 = fig.add_subplot(2, 2, 2, facecolor='#0d1117')
err_wheel = v_wheel - v_gt_x
err_fus = v_fus_x - v_gt_x
ax2.plot(times, err_wheel, color='#f85149', label=f'Wheel Encoder Error (RMSE={np.sqrt(np.mean(err_wheel**2)):.3f} m/s)', linewidth=1.2)
ax2.plot(times, err_fus, color='#3fb950', label=f'RPM-IMU Fusion Error (RMSE={np.sqrt(np.mean(err_fus**2)):.3f} m/s)', linewidth=1.2)
ax2.axhline(0, color='#8b949e', linestyle='--', alpha=0.6)
ax2.set_title('Longitudinal Velocity Error (Est - GT)', color='white', fontsize=11, fontweight='bold')
ax2.set_xlabel('Simulation Time (s)', color='#c9d1d9', fontsize=9)
ax2.set_ylabel('Error (m/s)', color='#c9d1d9', fontsize=9)
ax2.legend(loc='upper right', facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9', fontsize=8)
ax2.grid(True, color='#21262d', linestyle='--', alpha=0.7)
ax2.tick_params(colors='#8b949e')
for s in ax2.spines.values(): s.set_color('#30363d')

# 3. Lateral Side-Slip Velocity & Tire Slip
ax3 = fig.add_subplot(2, 2, 3, facecolor='#0d1117')
ax3.plot(times, v_gt_y, color='#f0883e', label='True Lateral Slip Speed (Vy_gt)', linewidth=1.5)
ax3.plot(times, v_fus_y, color='#d2a8ff', label='Estimated Lateral Slip (Vy_fus)', linewidth=1.2, linestyle='--')
ax3.set_title('Lateral Slip Velocity (Vy) during Cornering', color='white', fontsize=11, fontweight='bold')
ax3.set_xlabel('Simulation Time (s)', color='#c9d1d9', fontsize=9)
ax3.set_ylabel('Lateral Speed (m/s)', color='#c9d1d9', fontsize=9)
ax3.legend(loc='upper right', facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9', fontsize=8)
ax3.grid(True, color='#21262d', linestyle='--', alpha=0.7)
ax3.tick_params(colors='#8b949e')
for s in ax3.spines.values(): s.set_color('#30363d')

# 4. 2D Trajectory Drift Comparison
ax4 = fig.add_subplot(2, 2, 4, facecolor='#0d1117')
ax4.plot(pos_gt_x, pos_gt_y, color='#58a6ff', label='Ground Truth Path (Webots)', linewidth=2.5)
ax4.plot(traj_wheel_gyro_x, traj_wheel_gyro_y, color='#e3b341', label=f'Wheel + Gyro Odom (Drift={np.hypot(traj_wheel_gyro_x[-1]-pos_gt_x[-1], traj_wheel_gyro_y[-1]-pos_gt_y[-1]):.2f}m)', linewidth=1.2, linestyle='--')
ax4.plot(traj_fus_x, traj_fus_y, color='#3fb950', label=f'RPM-IMU Fusion (Drift={np.hypot(traj_fus_x[-1]-pos_gt_x[-1], traj_fus_y[-1]-pos_gt_y[-1]):.2f}m)', linewidth=1.2, linestyle=':')
ax4.set_title('Full Lap 2D Dead-Reckoning Trajectory', color='white', fontsize=11, fontweight='bold')
ax4.set_xlabel('Global X (m)', color='#c9d1d9', fontsize=9)
ax4.set_ylabel('Global Y (m)', color='#c9d1d9', fontsize=9)
ax4.legend(loc='lower left', facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9', fontsize=8)
ax4.grid(True, color='#21262d', linestyle='--', alpha=0.7)
ax4.tick_params(colors='#8b949e')
ax4.axis('equal')
for s in ax4.spines.values(): s.set_color('#30363d')

plt.suptitle('Webots Ground Truth vs Wheel Encoders & RPM-IMU Fusion Analysis', color='white', fontsize=14, fontweight='bold', y=0.98)
plt.tight_layout(rect=[0, 0, 1, 0.96])

out_plot = os.path.abspath(r"slam_results\encoder_imu_vs_ground_truth_report.png")
os.makedirs(os.path.dirname(out_plot), exist_ok=True)
plt.savefig(out_plot, dpi=200)
plt.close()
print(f"[PLOT GENERATED] {out_plot}")
