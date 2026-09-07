"""
================================================================================
  ENCODER & IMU FUSION VS WEBOTS GROUND TRUTH VELOCITY BENCHMARK
================================================================================
"""

import os
import sys
import json
import math
import subprocess
import numpy as np

WHEEL_RADIUS = 0.0275
MOTOR_DIR = -1.0
WHEELBASE = 0.1325
TRACK_REAR = 0.1000

class RPMImuFusion:
    def __init__(self, alpha=0.90, max_grip_accel=12.0, vy_decay_tau=0.15):
        self.v_x = 0.0
        self.v_y = 0.0
        self.alpha = alpha
        self.max_grip_accel = max_grip_accel
        self.vy_decay_tau = vy_decay_tau

    def update(self, w_rl, w_rr, gyro_z, accel_x, accel_y, dt):
        if dt <= 0.0:
            return self.v_x, self.v_y

        v_rl = (w_rl * WHEEL_RADIUS) / MOTOR_DIR
        v_rr = (w_rr * WHEEL_RADIUS) / MOTOR_DIR
        v_wheel = 0.5 * (v_rl + v_rr)

        accel_wheel = (v_wheel - self.v_x) / dt

        if abs(accel_wheel) > self.max_grip_accel:
            self.v_x += accel_x * dt
        else:
            v_x_imu_projected = self.v_x + accel_x * dt
            self.v_x = (self.alpha * v_x_imu_projected) + ((1.0 - self.alpha) * v_wheel)

        self.v_x = max(-4.5, min(4.5, self.v_x))

        slip_accel_y = accel_y - (self.v_x * gyro_z)
        self.v_y += slip_accel_y * dt
        self.v_y *= math.exp(-dt / self.vy_decay_tau)

        return self.v_x, self.v_y

def run_simulation(duration_s=23.0):
    world_abs = os.path.abspath(r"webots/simulation/my_project\worlds\Cyan_Champion_Track_Static.wbt")
    webots_bin = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
    
    print(f"\n[STEP 1] Running Webots simulation on {os.path.basename(world_abs)} for {duration_s:.1f}s...")
    env = os.environ.copy()
    env["WEBOTS_RUN_SECONDS"] = str(duration_s)
    env["MAX_BENCHMARK_LAPS"] = "2"
    env["ENABLE_MOVING_OBSTACLE"] = "0"
    env["WEBOTS_HOME"] = r"C:\Program Files\Webots"

    webots_cmd = [webots_bin, "--mode=fast", "--batch", "--minimize", world_abs]
    sim_proc = subprocess.Popen(webots_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
    
    for line in sim_proc.stdout:
        if "LAP COMPLETED" in line or "BENCHMARK" in line or "CRASH" in line:
            print("  " + line.strip(), flush=True)

    sim_proc.wait()
    print("  Simulation completed.\n")

def analyze_telemetry():
    jsonl_path = os.path.abspath(r"webots/simulation/my_project\controllers\folkrace_drive\live_webots_slam_scans.jsonl")
    if not os.path.exists(jsonl_path):
        print(f"[ERROR] JSONL log not found at: {jsonl_path}")
        return

    records = []
    with open(jsonl_path, "r", encoding="utf-8") as f:
        for line in f:
            line_str = line.strip()
            if not line_str:
                continue
            try:
                data = json.loads(line_str)
                records.append(data)
            except Exception:
                continue

    print(f"[STEP 2] Loaded {len(records)} telemetry steps (100 Hz). Analyzing...")
    if len(records) < 50:
        print("[ERROR] Insufficient records to analyze.")
        return

    times = []
    v_gt_x_list = []
    v_gt_y_list = []
    w_gt_z_list = []
    pos_gt_x = []
    pos_gt_y = []
    yaw_gt_list = []

    v_wheel_list = []
    v_4wd_list = []
    accel_x_list = []
    accel_y_list = []
    gyro_z_list = []

    fusion_default = RPMImuFusion(alpha=0.90, vy_decay_tau=0.15)
    v_fusion_x_list = []
    v_fusion_y_list = []

    fusion_50 = RPMImuFusion(alpha=0.50, vy_decay_tau=0.15)
    v_fusion50_x_list = []

    fusion_pure_wheel = RPMImuFusion(alpha=0.00, vy_decay_tau=0.15)
    v_fusion00_x_list = []

    # Odometry integration trajectories
    odom_gt_x = []
    odom_gt_y = []

    odom_wheel_x = 0.6
    odom_wheel_y = 0.0
    odom_wheel_yaw = 0.0
    traj_wheel_x = []
    traj_wheel_y = []

    odom_fus_x = 0.6
    odom_fus_y = 0.0
    odom_fus_yaw = 0.0
    traj_fus_x = []
    traj_fus_y = []

    odom_wheel_gyro_x = 0.6
    odom_wheel_gyro_y = 0.0
    odom_wheel_gyro_yaw = 0.0
    traj_wheel_gyro_x = []
    traj_wheel_gyro_y = []

    last_t = None

    for r in records:
        t = float(r.get("time", 0.0))
        dt = t - last_t if last_t is not None else 0.01
        if dt <= 0.0 or dt > 0.1:
            dt = 0.01

        v_gt_x = float(r.get("v_gt_x", 0.0))
        v_gt_y = float(r.get("v_gt_y", 0.0))
        w_gt_z = float(r.get("w_gt_z", 0.0))
        pos_gt = r.get("pos", [0.0, 0.0])
        yaw_gt = float(r.get("yaw", 0.0))

        w_rl = float(r.get("omega_RL", r.get("w_rl", 0.0)))
        w_rr = float(r.get("omega_RR", r.get("w_rr", 0.0)))
        w_fl = float(r.get("omega_FL", 0.0))
        w_fr = float(r.get("omega_FR", 0.0))

        gyro_z = float(r.get("gyro_z", r.get("gyro", [0, 0, 0])[2] if isinstance(r.get("gyro"), list) and len(r.get("gyro")) >= 3 else 0.0))
        accel_x = float(r.get("accel_x", r.get("accel", [0, 0, 0])[0] if isinstance(r.get("accel"), list) and len(r.get("accel")) >= 3 else 0.0))
        accel_y = float(r.get("accel_y", r.get("accel", [0, 0, 0])[1] if isinstance(r.get("accel"), list) and len(r.get("accel")) >= 3 else 0.0))

        v_rl = (w_rl * WHEEL_RADIUS) / MOTOR_DIR
        v_rr = (w_rr * WHEEL_RADIUS) / MOTOR_DIR
        v_fl = (w_fl * WHEEL_RADIUS) / MOTOR_DIR
        v_fr = (w_fr * WHEEL_RADIUS) / MOTOR_DIR

        v_wheel = 0.5 * (v_rl + v_rr)
        v_4wd = 0.25 * (v_rl + v_rr + v_fl + v_fr)

        # Run fusion filters
        fx, fy = fusion_default.update(w_rl, w_rr, gyro_z, accel_x, accel_y, dt)
        fx50, _ = fusion_50.update(w_rl, w_rr, gyro_z, accel_x, accel_y, dt)
        fx00, _ = fusion_pure_wheel.update(w_rl, w_rr, gyro_z, accel_x, accel_y, dt)

        # Trajectory integration
        # 1. Pure Wheel Dead-Reckoning (Differential Steering yaw)
        d_yaw_diff = ((v_rr - v_rl) / TRACK_REAR) * dt
        odom_wheel_yaw += d_yaw_diff
        odom_wheel_x += v_wheel * math.cos(odom_wheel_yaw) * dt
        odom_wheel_y += v_wheel * math.sin(odom_wheel_yaw) * dt

        # 2. Wheel + Gyro Dead-Reckoning
        d_yaw_gyro = gyro_z * dt
        odom_wheel_gyro_yaw += d_yaw_gyro
        odom_wheel_gyro_x += v_wheel * math.cos(odom_wheel_gyro_yaw) * dt
        odom_wheel_gyro_y += v_wheel * math.sin(odom_wheel_gyro_yaw) * dt

        # 3. Fused Odometry (Vx, Vy, Gyro yaw)
        odom_fus_yaw += d_yaw_gyro
        mid_fus_yaw = odom_fus_yaw - (d_yaw_gyro * 0.5)
        dx_g = (fx * math.cos(mid_fus_yaw)) - (fy * math.sin(mid_fus_yaw))
        dy_g = (fx * math.sin(mid_fus_yaw)) + (fy * math.cos(mid_fus_yaw))
        odom_fus_x += dx_g * dt
        odom_fus_y += dy_g * dt

        # Store
        times.append(t)
        v_gt_x_list.append(v_gt_x)
        v_gt_y_list.append(v_gt_y)
        w_gt_z_list.append(w_gt_z)
        pos_gt_x.append(pos_gt[0])
        pos_gt_y.append(pos_gt[1])
        yaw_gt_list.append(yaw_gt)

        v_wheel_list.append(v_wheel)
        v_4wd_list.append(v_4wd)
        accel_x_list.append(accel_x)
        accel_y_list.append(accel_y)
        gyro_z_list.append(gyro_z)

        v_fusion_x_list.append(fx)
        v_fusion_y_list.append(fy)
        v_fusion50_x_list.append(fx50)
        v_fusion00_x_list.append(fx00)

        traj_wheel_x.append(odom_wheel_x)
        traj_wheel_y.append(odom_wheel_y)
        traj_wheel_gyro_x.append(odom_wheel_gyro_x)
        traj_wheel_gyro_y.append(odom_wheel_gyro_y)
        traj_fus_x.append(odom_fus_x)
        traj_fus_y.append(odom_fus_y)

        last_t = t

    # Convert to numpy arrays
    times = np.array(times)
    v_gt_x = np.array(v_gt_x_list)
    v_gt_y = np.array(v_gt_y_list)
    w_gt_z = np.array(w_gt_z_list)
    v_wheel = np.array(v_wheel_list)
    v_4wd = np.array(v_4wd_list)
    v_fus_x = np.array(v_fusion_x_list)
    v_fus_y = np.array(v_fusion_y_list)
    v_fus50_x = np.array(v_fusion50_x_list)
    accel_x = np.array(accel_x_list)
    accel_y = np.array(accel_y_list)
    gyro_z = np.array(gyro_z_list)
    pos_gt_x = np.array(pos_gt_x)
    pos_gt_y = np.array(pos_gt_y)

    traj_wheel_x = np.array(traj_wheel_x)
    traj_wheel_y = np.array(traj_wheel_y)
    traj_wheel_gyro_x = np.array(traj_wheel_gyro_x)
    traj_wheel_gyro_y = np.array(traj_wheel_gyro_y)
    traj_fus_x = np.array(traj_fus_x)
    traj_fus_y = np.array(traj_fus_y)

    # 1. Statistical Error Metrics
    def calc_metrics(est, gt):
        err = est - gt
        rmse = float(np.sqrt(np.mean(err**2)))
        mae = float(np.mean(np.abs(err)))
        max_err = float(np.max(np.abs(err)))
        bias = float(np.mean(err))
        corr = float(np.corrcoef(est, gt)[0, 1]) if np.std(est) > 1e-4 and np.std(gt) > 1e-4 else 0.0
        return rmse, mae, max_err, bias, corr

    rmse_wheel_vx, mae_wheel_vx, max_wheel_vx, bias_wheel_vx, corr_wheel_vx = calc_metrics(v_wheel, v_gt_x)
    rmse_4wd_vx, mae_4wd_vx, max_4wd_vx, bias_4wd_vx, corr_4wd_vx = calc_metrics(v_4wd, v_gt_x)
    rmse_fus_vx, mae_fus_vx, max_fus_vx, bias_fus_vx, corr_fus_vx = calc_metrics(v_fus_x, v_gt_x)
    rmse_fus50_vx, mae_fus50_vx, max_fus50_vx, bias_fus50_vx, corr_fus50_vx = calc_metrics(v_fus50_x, v_gt_x)
    rmse_fus_vy, mae_fus_vy, max_fus_vy, bias_fus_vy, corr_fus_vy = calc_metrics(v_fus_y, v_gt_y)
    rmse_gyro_wz, mae_gyro_wz, max_gyro_wz, bias_gyro_wz, corr_gyro_wz = calc_metrics(gyro_z, w_gt_z)

    # 2. Wheel Slip Analysis
    # Longitudinal Slip Ratio S = (v_wheel - v_gt_x) / v_gt_x (when v_gt_x > 0.5 m/s)
    mask_moving = v_gt_x > 0.5
    if np.any(mask_moving):
        slip_ratio = (v_wheel[mask_moving] - v_gt_x[mask_moving]) / v_gt_x[mask_moving]
        mean_slip = float(np.mean(slip_ratio)) * 100.0
        max_slip = float(np.max(np.abs(slip_ratio))) * 100.0
        p95_slip = float(np.percentile(np.abs(slip_ratio), 95)) * 100.0
    else:
        mean_slip, max_slip, p95_slip = 0.0, 0.0, 0.0

    # Lateral Slip Angle Beta = atan(v_gt_y / v_gt_x)
    if np.any(mask_moving):
        beta_deg = np.degrees(np.arctan2(v_gt_y[mask_moving], v_gt_x[mask_moving]))
        mean_beta = float(np.mean(np.abs(beta_deg)))
        max_beta = float(np.max(np.abs(beta_deg)))
        p95_beta = float(np.percentile(np.abs(beta_deg), 95))
    else:
        mean_beta, max_beta, p95_beta = 0.0, 0.0, 0.0

    # 3. Trajectory Drift Errors
    # 2D Euclidean Distance Error vs Ground Truth over time
    dist_err_wheel = np.hypot(traj_wheel_x - pos_gt_x, traj_wheel_y - pos_gt_y)
    dist_err_wheel_gyro = np.hypot(traj_wheel_gyro_x - pos_gt_x, traj_wheel_gyro_y - pos_gt_y)
    dist_err_fus = np.hypot(traj_fus_x - pos_gt_x, traj_fus_y - pos_gt_y)

    total_dist_traveled = float(np.sum(np.hypot(np.diff(pos_gt_x), np.diff(pos_gt_y))))
    final_err_wheel = float(dist_err_wheel[-1])
    final_err_wheel_gyro = float(dist_err_wheel_gyro[-1])
    final_err_fus = float(dist_err_fus[-1])

    max_err_wheel = float(np.max(dist_err_wheel))
    max_err_wheel_gyro = float(np.max(dist_err_wheel_gyro))
    max_err_fus = float(np.max(dist_err_fus))

    # Centripetal Acceleration Analysis
    # Theoretical centripetal accel = v_gt_x * w_gt_z
    a_centripetal = v_gt_x * gyro_z
    diff_ay_centripetal = accel_y - a_centripetal
    rmse_ay_centripetal = float(np.sqrt(np.mean(diff_ay_centripetal**2)))

    # Output detailed report
    print("=" * 80)
    print("  GROUND TRUTH VS ENCODER & IMU FUSION BENCHMARK RESULTS")
    print("=" * 80)
    print(f" Total Simulation Steps : {len(records)} steps (100 Hz, {times[-1]:.2f}s total)")
    print(f" Total Distance Traveled: {total_dist_traveled:.2f} meters")
    print(f" Max Ground Truth Speed : {np.max(v_gt_x):.2f} m/s ({np.max(v_gt_x)*3.6:.1f} km/h)")
    print(f" Mean Ground Truth Speed: {np.mean(v_gt_x):.2f} m/s ({np.mean(v_gt_x)*3.6:.1f} km/h)")
    print("-" * 80)
    print(" [1. FORWARD VELOCITY (Vx) ACCURACY EVALUATION]")
    print(f"  * Raw Rear Encoders (V_wheel)    : RMSE = {rmse_wheel_vx:.4f} m/s | MAE = {mae_wheel_vx:.4f} m/s | Max Err = {max_wheel_vx:.4f} m/s | Bias = {bias_wheel_vx:+.4f} m/s | Corr = {corr_wheel_vx:.4f}")
    print(f"  * Raw 4WD Encoders (V_4wd)       : RMSE = {rmse_4wd_vx:.4f} m/s | MAE = {mae_4wd_vx:.4f} m/s | Max Err = {max_4wd_vx:.4f} m/s | Bias = {bias_4wd_vx:+.4f} m/s | Corr = {corr_4wd_vx:.4f}")
    print(f"  * RPM-IMU Fusion 90% (Current)   : RMSE = {rmse_fus_vx:.4f} m/s | MAE = {mae_fus_vx:.4f} m/s | Max Err = {max_fus_vx:.4f} m/s | Bias = {bias_fus_vx:+.4f} m/s | Corr = {corr_fus_vx:.4f}")
    print(f"  * RPM-IMU Fusion 50% (Balanced)  : RMSE = {rmse_fus50_vx:.4f} m/s | MAE = {mae_fus50_vx:.4f} m/s | Max Err = {max_fus50_vx:.4f} m/s | Bias = {bias_fus50_vx:+.4f} m/s | Corr = {corr_fus50_vx:.4f}")
    print("-" * 80)
    print(" [2. LATERAL VELOCITY (Vy) & YAW RATE (Wz) ACCURACY]")
    print(f"  * True Lateral Speed (Vy_gt)     : Mean = {np.mean(np.abs(v_gt_y)):.4f} m/s | Max = {np.max(np.abs(v_gt_y)):.4f} m/s | P95 = {np.percentile(np.abs(v_gt_y), 95):.4f} m/s")
    print(f"  * Fused Lateral Speed (Vy_fus)   : RMSE = {rmse_fus_vy:.4f} m/s | MAE = {mae_fus_vy:.4f} m/s | Max Err = {max_fus_vy:.4f} m/s | Corr = {corr_fus_vy:.4f}")
    print(f"  * BNO08x Gyro Yaw Rate (Wz_gyro) : RMSE = {rmse_gyro_wz:.4f} rad/s | MAE = {mae_gyro_wz:.4f} rad/s | Max Err = {max_gyro_wz:.4f} rad/s | Corr = {corr_gyro_wz:.4f}")
    print("-" * 80)
    print(" [3. PHYSICAL TIRE SLIP RATIO & SIDE-SLIP ANGLE (BETA)]")
    print(f"  * Longitudinal Slip Ratio (Sx)   : Mean = {mean_slip:+.2f}% | 95th Percentile = {p95_slip:.2f}% | Max = {max_slip:.2f}%")
    print(f"  * Vehicle Side-Slip Angle (Beta) : Mean = {mean_beta:.2f} deg | 95th Percentile = {p95_beta:.2f} deg | Max = {max_beta:.2f} deg")
    print(f"  * Ay Centripetal Cancellation Res: RMSE = {rmse_ay_centripetal:.4f} m/s^2")
    print("-" * 80)
    print(" [4. 2D TRAJECTORY DRIFT OVER FULL TRACK RUN]")
    print(f"  * Pure Wheel (Diff Heading)      : Max Drift = {max_err_wheel:.2f} m | Final Endpoint Drift = {final_err_wheel:.2f} m ({final_err_wheel/total_dist_traveled*100:.1f}%)")
    print(f"  * Wheel Encoders + Gyro Yaw      : Max Drift = {max_err_wheel_gyro:.2f} m | Final Endpoint Drift = {final_err_wheel_gyro:.2f} m ({final_err_wheel_gyro/total_dist_traveled*100:.1f}%)")
    print(f"  * RPM-IMU Fused Odometry         : Max Drift = {max_err_fus:.2f} m | Final Endpoint Drift = {final_err_fus:.2f} m ({final_err_fus/total_dist_traveled*100:.1f}%)")
    print("=" * 80)

    # Save summary dictionary for visualization / report
    summary = {
        "steps": len(records),
        "duration_s": float(times[-1]),
        "total_distance_m": total_dist_traveled,
        "max_speed_m_s": float(np.max(v_gt_x)),
        "mean_speed_m_s": float(np.mean(v_gt_x)),
        "vx_metrics": {
            "wheel_encoder": {"rmse": rmse_wheel_vx, "mae": mae_wheel_vx, "max_err": max_wheel_vx, "bias": bias_wheel_vx, "corr": corr_wheel_vx},
            "four_wheel": {"rmse": rmse_4wd_vx, "mae": mae_4wd_vx, "max_err": max_4wd_vx, "bias": bias_4wd_vx, "corr": corr_4wd_vx},
            "fusion_90": {"rmse": rmse_fus_vx, "mae": mae_fus_vx, "max_err": max_fus_vx, "bias": bias_fus_vx, "corr": corr_fus_vx},
            "fusion_50": {"rmse": rmse_fus50_vx, "mae": mae_fus50_vx, "max_err": max_fus50_vx, "bias": bias_fus50_vx, "corr": corr_fus50_vx},
        },
        "vy_metrics": {
            "mean_gt_vy": float(np.mean(np.abs(v_gt_y))),
            "max_gt_vy": float(np.max(np.abs(v_gt_y))),
            "fusion_vy_rmse": rmse_fus_vy,
            "fusion_vy_mae": mae_fus_vy,
            "fusion_vy_corr": corr_fus_vy,
        },
        "yaw_metrics": {
            "gyro_rmse": rmse_gyro_wz,
            "gyro_mae": mae_gyro_wz,
            "gyro_corr": corr_gyro_wz,
        },
        "slip_metrics": {
            "mean_slip_pct": mean_slip,
            "p95_slip_pct": p95_slip,
            "max_slip_pct": max_slip,
            "mean_beta_deg": mean_beta,
            "p95_beta_deg": p95_beta,
            "max_beta_deg": max_beta,
        },
        "drift_metrics": {
            "wheel_diff_final_m": final_err_wheel,
            "wheel_diff_max_m": max_err_wheel,
            "wheel_gyro_final_m": final_err_wheel_gyro,
            "wheel_gyro_max_m": max_err_wheel_gyro,
            "fusion_final_m": final_err_fus,
            "fusion_max_m": max_err_fus,
        }
    }

    out_json = os.path.abspath(r"webots/simulation/my_project\controllers\folkrace_drive\encoder_fusion_benchmark_summary.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSaved benchmark metrics summary to: {out_json}")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--analyze-only":
        analyze_telemetry()
    else:
        run_simulation(duration_s=23.0)
        analyze_telemetry()
