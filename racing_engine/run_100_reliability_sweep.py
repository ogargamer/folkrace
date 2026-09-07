import os, sys, math, time, json
import numpy as np

sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath('my_project/controllers/folkrace_drive'))

from tof_pathfinders import (
    FrenetTransformer, FrenetCorridorQP, WerlingLatticePlanner, MPPIPlanner, MPCCAcadosPlanner,
    NUM_TOF_ZONES, TOF_ZONE_ANGLES, TOF_MAX_RANGE_M, TOF_MIN_RANGE_M, MAX_STEERING_RAD
)

traj_path = r'my_project/controllers/folkrace_drive/cyan_champion_trajectory.csv'
data = np.genfromtxt(traj_path, delimiter=',', names=True)
s_arr, x_arr, y_arr, v_arr, yaw_arr, k_arr = data['dist_m'], data['x_m'], data['y_m'], data['v_mps'], data['yaw_rad'], data['curvature_inv_m']
frenet = FrenetTransformer(s_arr, x_arr, y_arr, yaw_arr, k_arr)

pillars = np.array([
    [2.80, 0.60],  # Pillar 2
    [2.00, 1.80],  # Pillar 3
    [0.00, 1.50],  # Pillar 4
    [-2.20, 0.40], # Pillar 5
    [-1.80, -1.20] # Pillar 6
])

def simulate_tof(rx, ry, ryaw, moving_obs_xy):
    front_x = rx + 0.09 * math.cos(ryaw)
    front_y = ry + 0.09 * math.sin(ryaw)
    tof = np.full(12, 1.50)
    for i, ang in enumerate(TOF_ZONE_ANGLES):
        ray_ang = ryaw + ang
        dx, dy = math.cos(ray_ang), math.sin(ray_ang)
        for px, py in pillars:
            p_dx, p_dy = px - front_x, py - front_y
            proj = p_dx * dx + p_dy * dy
            if 0 < proj < tof[i]:
                perp_sq = (p_dx**2 + p_dy**2) - proj**2
                if perp_sq < (0.12**2):
                    tof[i] = min(tof[i], max(0.02, proj - 0.12))
        for mx, my in moving_obs_xy:
            m_dx, m_dy = mx - front_x, my - front_y
            proj = m_dx * dx + m_dy * dy
            if 0 < proj < tof[i]:
                perp_sq = (m_dx**2 + m_dy**2) - proj**2
                if perp_sq < (0.10**2):
                    tof[i] = min(tof[i], max(0.02, proj - 0.10))
    return tof

def evaluate_run(model_type, var_id, params):
    wheelbase = 0.150
    dt = 0.01
    steps = 1500  # 15.0 seconds
    top_speed = params.get('top_speed', 2.60)
    
    if model_type == 'WERLING':
        planner = WerlingLatticePlanner(
            wheelbase_m=wheelbase,
            mu=params.get('mu', 0.90),
            top_speed=top_speed
        )
        planner.d_targets = np.array(params.get('d_targets', [-0.40, -0.20, 0.0, 0.20, 0.40]))
        planner.horizons = np.array(params.get('horizons', [0.30, 0.50, 0.70]))
        planner.dt = params.get('dt_plan', 0.04)
    elif model_type == 'MPPI':
        planner = MPPIPlanner(
            K=params.get('K', 600),
            N=params.get('N', 20),
            dt=params.get('dt_plan', 0.045),
            wheelbase_m=wheelbase,
            top_speed=top_speed
        )
        planner.sigma_steer = params.get('sigma_steer', 0.18)
        planner.sigma_accel = params.get('sigma_accel', 3.50)
        planner.temperature = params.get('temperature', 8.5)
    elif model_type == 'MPCC':
        planner = MPCCAcadosPlanner(
            N=params.get('N', 12),
            dt=params.get('dt_plan', 0.045),
            wheelbase_m=wheelbase,
            mu=params.get('mu', 0.90),
            top_speed=top_speed
        )
    elif model_type == 'FRENETQP':
        planner = FrenetCorridorQP(wheelbase_m=wheelbase)
        k_ug = params.get('k_ug', 0.060)
        k_cte = params.get('k_cte', 2.20)

    rx = float(x_arr[0])
    ry = float(y_arr[0])
    ryaw = float(yaw_arr[0])
    v_curr = 1.00
    steer_curr = 0.0
    
    total_dist = 0.0
    speeds = []
    lateral_errors = []
    min_clearances = []
    steer_rates = []
    collisions = 0
    recoveries = 0
    closest_idx = 0
    
    obs_speed = 0.50
    total_len = frenet.total_length
    obs_s = np.array([total_len * 0.20, total_len * 0.50, total_len * 0.80])
    
    for step in range(steps):
        t_sim = step * dt
        obs_curr_s = (obs_s + obs_speed * t_sim) % total_len
        moving_xy = [frenet.frenet_to_cartesian(s_val, 0.0, closest_idx) for s_val in obs_curr_s]
        
        tof = simulate_tof(rx, ry, ryaw, moving_xy)
        min_c = float(np.min(tof))
        min_clearances.append(min_c)
        if min_c < 0.04:
            collisions += 1
        elif min_c < 0.10:
            recoveries += 1
            
        curr_s, curr_d, closest_idx, t_yaw, t_k = frenet.cartesian_to_frenet(rx, ry, closest_idx, search_window=40)
        lateral_errors.append(abs(curr_d))
        
        if model_type == 'FRENETQP':
            target_d, feedforward_k, _ = planner.plan(
                rx, ry, ryaw, tof, frenet, closest_idx,
                current_steer_rad=steer_curr, v_mps=v_curr
            )
            a_y_lead = (v_curr**2) * t_k
            delta_ff = math.atan(wheelbase * t_k) + k_ug * (a_y_lead / 9.81)
            heading_err = (t_yaw - ryaw + math.pi) % (2.0 * math.pi) - math.pi
            
            min_front_zone = float(np.min(tof[4:8]))
            min_obstacle_prox = min(float(np.min(tof[0:4])), float(np.min(tof[8:12])), min_front_zone)
            if abs(target_d) > 0.06 or min_obstacle_prox < 0.20:
                k_line_weight = 0.0
            elif min_obstacle_prox >= 0.85:
                k_line_weight = 1.0
            else:
                k_line_weight = (min_obstacle_prox - 0.20) / (0.85 - 0.20)
                
            e_lat_total = curr_d - target_d
            v_soft = 0.55 + 0.22 * max(abs(v_curr), 0.50)
            stanley_steer = math.atan(k_line_weight * k_cte * (-e_lat_total) / v_soft)
            cmd_steer = delta_ff + 0.95 * heading_err + stanley_steer
            cmd_v = top_speed
        else:
            cmd_steer, cmd_v, _ = planner.plan(rx, ry, ryaw, v_curr, steer_curr, tof, frenet, closest_idx, target_v=top_speed)
            
        steer_rate = (cmd_steer - steer_curr) / dt
        steer_rate_clamped = np.clip(steer_rate, -10.47, 10.47)
        steer_rates.append(abs(steer_rate_clamped))
        steer_curr += steer_rate_clamped * dt
        steer_curr = np.clip(steer_curr, -MAX_STEERING_RAD, MAX_STEERING_RAD)
        
        accel = (cmd_v - v_curr) / dt
        accel_clamped = np.clip(accel, -14.0, 10.0)
        v_curr += accel_clamped * dt
        v_curr = np.clip(v_curr, 0.40, top_speed)
        
        beta = math.atan(0.5 * math.tan(steer_curr))
        rx += v_curr * math.cos(ryaw + beta) * dt
        ry += v_curr * math.sin(ryaw + beta) * dt
        ryaw += (v_curr / wheelbase) * math.cos(beta) * math.tan(steer_curr) * dt
        total_dist += v_curr * dt
        speeds.append(v_curr)

    # Reliability scoring (Lower is better: heavily penalizes collisions, high lateral errors, high recovery counts)
    mean_dev = float(np.mean(lateral_errors))
    mean_v = float(np.mean(speeds))
    min_c = float(np.min(min_clearances))
    
    # Stability penalty formula
    stability_penalty = (
        collisions * 1000.0 +
        recoveries * 10.0 +
        mean_dev * 100.0 +
        max(0.0, 0.15 - min_c) * 500.0 -
        mean_v * 5.0
    )

    return {
        'id': var_id,
        'model': model_type,
        'params': params,
        'distance_m': round(total_dist, 2),
        'mean_v_mps': round(mean_v, 2),
        'max_v_mps': round(float(np.max(speeds)), 2),
        'mean_dev_cm': round(mean_dev * 100, 1),
        'max_dev_cm': round(float(np.max(lateral_errors)) * 100, 1),
        'min_clearance_cm': round(min_c * 100, 1),
        'collisions': collisions,
        'recoveries': recoveries,
        'stability_score': round(stability_penalty, 1)
    }

# Build 100 distinct variations across all models
variations = []

# --- WERLING LATTICE: 30 VARIATIONS ---
werling_horizons = [
    [0.25, 0.45, 0.65],
    [0.30, 0.50, 0.70],
    [0.35, 0.60, 0.85],
    [0.20, 0.40, 0.60],
    [0.40, 0.65, 0.90]
]
werling_meshes = [
    [-0.36, -0.18, 0.0, 0.18, 0.36],
    [-0.40, -0.20, 0.0, 0.20, 0.40],
    [-0.44, -0.33, -0.22, -0.11, 0.0, 0.11, 0.22, 0.33, 0.44],
    [-0.48, -0.36, -0.24, -0.12, 0.0, 0.12, 0.24, 0.36, 0.48],
    [-0.52, -0.39, -0.26, -0.13, 0.0, 0.13, 0.26, 0.39, 0.52]
]
werling_mus = [0.75, 0.85, 0.95]
werling_speeds = [2.00, 2.40]

w_count = 0
for h in werling_horizons:
    for m in werling_meshes:
        if w_count >= 30: break
        mu = werling_mus[w_count % len(werling_mus)]
        spd = werling_speeds[w_count % len(werling_speeds)]
        w_count += 1
        variations.append(('WERLING', f'WERLING_{w_count:02d}', {
            'top_speed': spd,
            'mu': mu,
            'horizons': h,
            'd_targets': m,
            'dt_plan': 0.04
        }))

# --- MPPI: 35 VARIATIONS ---
mppi_k_list = [400, 500, 600, 700, 800]
mppi_n_list = [14, 18, 22, 26]
mppi_temp_list = [6.5, 8.0, 9.5, 11.5]
mppi_steer_noise = [0.14, 0.18, 0.22]
mppi_accel_noise = [2.6, 3.4, 4.2]
mppi_speeds = [2.00, 2.40, 2.80]

m_count = 0
for k in mppi_k_list:
    for n in mppi_n_list:
        if m_count >= 35: break
        temp = mppi_temp_list[m_count % len(mppi_temp_list)]
        s_noise = mppi_steer_noise[m_count % len(mppi_steer_noise)]
        a_noise = mppi_accel_noise[m_count % len(mppi_accel_noise)]
        spd = mppi_speeds[m_count % len(mppi_speeds)]
        m_count += 1
        variations.append(('MPPI', f'MPPI_{m_count:02d}', {
            'top_speed': spd,
            'K': k,
            'N': n,
            'temperature': temp,
            'sigma_steer': s_noise,
            'sigma_accel': a_noise,
            'dt_plan': 0.045
        }))

# --- MPCC: 25 VARIATIONS ---
mpcc_n_list = [8, 10, 12, 14, 16, 18]
mpcc_mu_list = [0.75, 0.85, 0.95, 1.05]
mpcc_speeds = [2.00, 2.30, 2.60, 2.90]

mpcc_count = 0
for n in mpcc_n_list:
    for mu in mpcc_mu_list:
        if mpcc_count >= 25: break
        spd = mpcc_speeds[mpcc_count % len(mpcc_speeds)]
        mpcc_count += 1
        variations.append(('MPCC', f'MPCC_{mpcc_count:02d}', {
            'top_speed': spd,
            'N': n,
            'mu': mu,
            'dt_plan': 0.045
        }))

# --- FRENETQP: 10 VARIATIONS ---
qp_speeds = [2.20, 2.50, 2.80, 3.10, 3.40]
qp_kugs = [0.040, 0.060]

qp_count = 0
for spd in qp_speeds:
    for kug in qp_kugs:
        if qp_count >= 10: break
        qp_count += 1
        variations.append(('FRENETQP', f'FRENETQP_{qp_count:02d}', {
            'top_speed': spd,
            'k_ug': kug,
            'k_cte': 2.20
        }))

print(f"=== GENERATED {len(variations)} TOTAL PARAMETER VARIATIONS ===")

results = []
t_start = time.perf_counter()

for idx, (m_type, v_id, p_dict) in enumerate(variations):
    res = evaluate_run(m_type, v_id, p_dict)
    results.append(res)
    if (idx + 1) % 10 == 0 or idx == 0:
        print(f"[{idx+1:3d}/100] [{res['model']:<8}] {res['id']:<14} -> Dist={res['distance_m']:5.2f}m | MeanV={res['mean_v_mps']:.2f}m/s | Dev={res['mean_dev_cm']:4.1f}cm | MinClr={res['min_clearance_cm']:4.1f}cm | Colls={res['collisions']} | Score={res['stability_score']:6.1f}")

t_total = time.perf_counter() - t_start
print(f"\n=== COMPLETED ALL 100 RUNS IN {t_total:.2f} SECONDS ===")

os.makedirs('racing_results', exist_ok=True)
with open(r'racing_results/100_reliability_sweep_results.json', 'w', encoding='utf-8') as f:
    json.dump(results, f, indent=2)

# Rank top performers per model category by stability score (lowest score is best)
best_per_model = {}
for m_type in ['WERLING', 'MPPI', 'MPCC', 'FRENETQP']:
    model_runs = [r for r in results if r['model'] == m_type]
    sorted_runs = sorted(model_runs, key=lambda x: (x['collisions'], x['stability_score']))
    best_per_model[m_type] = sorted_runs[0]
    print(f"\n>>> BEST {m_type} CONFIGURATION: {sorted_runs[0]['id']} <<<")
    print(f"    Params: {sorted_runs[0]['params']}")
    print(f"    Distance: {sorted_runs[0]['distance_m']} m | Mean Speed: {sorted_runs[0]['mean_v_mps']} m/s")
    print(f"    Mean Dev: {sorted_runs[0]['mean_dev_cm']} cm | Min Clearance: {sorted_runs[0]['min_clearance_cm']} cm")
    print(f"    Collisions: {sorted_runs[0]['collisions']} | Recoveries: {sorted_runs[0]['recoveries']}")

with open(r'racing_results/best_stability_profiles.json', 'w', encoding='utf-8') as f:
    json.dump(best_per_model, f, indent=2)

print("\n=== SAVED TOP STABILITY PROFILES TO best_stability_profiles.json ===")
