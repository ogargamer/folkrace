import os
import sys
import numpy as np
import scipy.spatial as spatial
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d

# Test refined repulsion function
def make_obstacle_aware_strict(line_pts, obs_tree, all_obs_pts, margin=0.10):
    u_1000 = np.linspace(0, 1, 1000, endpoint=False)
    res_line = line_pts.copy()
    target_push = margin + 0.015 # 11.5 cm buffer
    
    for step in range(30):
        d_obs, idx_obs = obs_tree.query(res_line)
        viol = d_obs < target_push
        if np.any(viol):
            closest_obs = all_obs_pts[idx_obs[viol]]
            v_rep = res_line[viol] - closest_obs
            dist_v = np.linalg.norm(v_rep, axis=1, keepdims=True) + 1e-6
            v_rep /= dist_v
            push_amount = (target_push - d_obs[viol])[:, np.newaxis]
            res_line[viol] += v_rep * push_amount
            
            res_line[:, 0] = gaussian_filter1d(res_line[:, 0], sigma=1.8, mode='wrap')
            res_line[:, 1] = gaussian_filter1d(res_line[:, 1], sigma=1.8, mode='wrap')
            
    tck, _ = splprep([res_line[:, 0], res_line[:, 1]], s=0.0015, per=True)
    res_fine = np.column_stack(splev(u_1000, tck))
    
    # Final check
    d_verif, _ = obs_tree.query(res_fine)
    min_c = float(np.min(d_verif)) * 100.0
    print(f"Verified minimum obstacle clearance: {min_c:.2f} cm")
    return res_fine

import scratch.test_obs_aware_offsets as t
l_fine = make_obstacle_aware_strict(t.left_line, t.obs_tree, t.all_obs_pts, 0.10)
r_fine = make_obstacle_aware_strict(t.right_line, t.obs_tree, t.all_obs_pts, 0.10)
