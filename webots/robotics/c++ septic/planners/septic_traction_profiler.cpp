#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <array>
#include <vector>

#if defined(_WIN32) || defined(__CYGWIN__)
  #define SEPTIC_API __declspec(dllexport)
#else
  #define SEPTIC_API __attribute__((visibility("default")))
#endif

// ==============================================================================
// PRECOMPUTED CONSTANTS & CAPACITY LIMITS
// ==============================================================================
static constexpr int MAX_SAMPLES = 64;
static constexpr int MAX_CANDIDATES = 512;
static constexpr int MAX_TARGETS = 8;
static constexpr int MAX_BLENDS = 8;
static constexpr int MAX_OFFSETS = 16;

// Precomputed inverse of 4x4 septic system matrix M
static constexpr float M_INV_SEPTIC[4][4] = {
    {  35.0f, -15.0f,   2.5f, -1.0f / 6.0f },
    { -84.0f,  39.0f,  -7.0f,  0.5f        },
    {  70.0f, -34.0f,   6.5f, -0.5f        },
    { -20.0f,  10.0f,  -2.0f,  1.0f / 6.0f }
};

struct SplinePoint {
    float x;
    float y;
    float dx_du;
    float dy_du;
    float d2x_du2;
    float d2y_du2;
};

struct SplineCandidate {
    float d_lat;
    float target_head;
    float blend;
    int n_pts;
    SplinePoint pts[MAX_SAMPLES];
    float kappa[MAX_SAMPLES];
    float ds[MAX_SAMPLES];
    float dkappa_ds[MAX_SAMPLES];
    float v_max_k_sq[MAX_SAMPLES];
    float v_profile_sq[MAX_SAMPLES];
    float lap_time;
    float total_cost;
    float clearance_penalty;
    float jerk_penalty;
    float align_penalty;
};

struct PlannerConfig {
    // Physical vehicle parameters
    float wheelbase;
    float vehicle_mass;
    float understeer_k_us;
    float tire_mu;
    float gravity_g;
    float max_steering_rad;
    float max_servo_rate;
    float accel_max;
    float decel_max;
    float ds_nominal;
    float top_speed_mps;
    float v_min_cruise;
    float v_max_cruise;
    float dt_loop;
    float v_entry_max;
    float v_cruise_min;
    float mu_friction;

    // Compute-heavy tuning parameters (exposed for GA)
    int probe_samples;        // Arc-length spatial resolution (16 - 64)
    int num_lat_offsets;      // Lateral candidate density (3 - 9)
    float lat_offset_max;     // Lateral search half-width (m)
    int num_head_offsets;     // Heading candidate density (3 - 7)
    float head_offset_max;    // Heading exploration half-range (rad)
    int num_blend_weights;    // Blend weight transitions (2 - 6)
    float blend_weights[MAX_BLENDS]; // Array of blend weights
    int max_extra_targets;    // Multi-gap scouting depth (0 - 4)

    // Multi-objective trajectory scoring weights (exposed for GA)
    float w_time;             // Traversal time weight
    float w_clearance;        // Obstacle clearance weight
    float w_curvature_smooth; // Curvature smoothness / jerk penalty weight
    float w_chord_align;      // Tangent alignment weight
    float w_steer_effort;     // Steering deviation effort penalty

    // Steering gains & initial dynamics
    float steer_kp;
    float pp_lookahead_gain;
    float initial_slew_scale; // Scale factor for initial turn-in jerk injection (0.0 to 1.0)
    float straight_lock_dist; // Minimum clear forward distance (m) to lock straight (default 1.70m)
    float straight_lock_speed; // Target acceleration speed (m/s) when locked straight (default 5.50m/s)
};

// Costmap probe function pointer matching C-ABI
typedef int (*CostmapProbeFn)(void* handle, const float* xs, const float* ys, int count);

// ==============================================================================
// SEPTIC POLYNOMIAL SOLVER & HORNER EVALUATION
// ==============================================================================
static inline void solve_septic_coeffs(
    float p0, float v0, float a0, float j0,
    float p1, float v1, float a1, float j1,
    float c[8]
) {
    c[0] = p0;
    c[1] = v0;
    c[2] = 0.5f * a0;
    c[3] = (1.0f / 6.0f) * j0;

    float r0 = p1 - (c[0] + c[1] + c[2] + c[3]);
    float r1 = v1 - (c[1] + 2.0f * c[2] + 3.0f * c[3]);
    float r2 = a1 - (2.0f * c[2] + 6.0f * c[3]);
    float r3 = j1 - (6.0f * c[3]);

    c[4] = M_INV_SEPTIC[0][0] * r0 + M_INV_SEPTIC[0][1] * r1 + M_INV_SEPTIC[0][2] * r2 + M_INV_SEPTIC[0][3] * r3;
    c[5] = M_INV_SEPTIC[1][0] * r0 + M_INV_SEPTIC[1][1] * r1 + M_INV_SEPTIC[1][2] * r2 + M_INV_SEPTIC[1][3] * r3;
    c[6] = M_INV_SEPTIC[2][0] * r0 + M_INV_SEPTIC[2][1] * r1 + M_INV_SEPTIC[2][2] * r2 + M_INV_SEPTIC[2][3] * r3;
    c[7] = M_INV_SEPTIC[3][0] * r0 + M_INV_SEPTIC[3][1] * r1 + M_INV_SEPTIC[3][2] * r2 + M_INV_SEPTIC[3][3] * r3;
}

static inline void eval_septic_horner(const float c[8], float u, float& p, float& v, float& a) {
    p = c[0] + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * (c[5] + u * (c[6] + u * c[7]))))));
    v = c[1] + u * (2.0f * c[2] + u * (3.0f * c[3] + u * (4.0f * c[4] + u * (5.0f * c[5] + u * (6.0f * c[6] + u * 7.0f * c[7])))));
    a = 2.0f * c[2] + u * (6.0f * c[3] + u * (12.0f * c[4] + u * (20.0f * c[5] + u * (30.0f * c[6] + u * 42.0f * c[7]))));
}

// ==============================================================================
// PLANNER ENGINE WITH SCALABLE COMPUTE & GA TUNABLE PARAMETERS
// ==============================================================================
class SepticTractionProfilerNative {
public:
    PlannerConfig cfg;
    void* costmap_handle;
    CostmapProbeFn costmap_probe_func;

    // Fixed static memory arena (zero runtime heap allocations)
    SplineCandidate candidate_pool[MAX_CANDIDATES];
    int candidate_count;

    int surviving_indices[MAX_CANDIDATES];
    int surviving_count;

    int best_spline_idx;
    float best_total_cost;
    float best_lap_time;
    float best_steer_cmd;
    float best_speed_cmd;

    // Preallocated buffers for probe checks
    float probe_xs[MAX_SAMPLES];
    float probe_ys[MAX_SAMPLES];

    SepticTractionProfilerNative(const PlannerConfig& config, void* cm_handle = nullptr, CostmapProbeFn probe_fn = nullptr)
        : cfg(config), costmap_handle(cm_handle), costmap_probe_func(probe_fn),
          candidate_count(0), surviving_count(0), best_spline_idx(-1),
          best_total_cost(1e9f), best_lap_time(1e9f), best_steer_cmd(0.0f), best_speed_cmd(0.0f)
    {}

    bool probe_candidate(const SplineCandidate& cand) {
        int n = cand.n_pts;
        for (int i = 0; i < n; ++i) {
            probe_xs[i] = cand.pts[i].x;
            probe_ys[i] = cand.pts[i].y;
        }

        if (costmap_probe_func && costmap_handle) {
            // Hierarchical coarse checkpoint pruning:
            // Check 4 coarse checkpoints first to quickly reject colliding trajectories
            if (n >= 16) {
                int chk_indices[4] = {n / 4, n / 2, (3 * n) / 4, n - 1};
                float chk_x[4], chk_y[4];
                for (int c = 0; c < 4; ++c) {
                    chk_x[c] = probe_xs[chk_indices[c]];
                    chk_y[c] = probe_ys[chk_indices[c]];
                }
                if (costmap_probe_func(costmap_handle, chk_x, chk_y, 4) != 1) {
                    return false; // Instant discard
                }
            }
            return costmap_probe_func(costmap_handle, probe_xs, probe_ys, n) == 1;
        }
        return true;
    }

    void plan(
        float xg, float yg, float theta_g, float aperture_width,
        float v_current, float steer_current, float yaw_rate_current, float steer_rate_current,
        const float* extra_targets_flat, int num_extra_targets
    ) {
        float v0 = std::max(v_current, cfg.v_min_cruise);
        float delta0 = std::clamp(steer_current, -cfg.max_steering_rad, cfg.max_steering_rad);
        float cos_d0 = std::cos(delta0);
        float cos2_d0 = std::max(cos_d0 * cos_d0, 0.05f);

        int n_samples = std::clamp(cfg.probe_samples, 16, MAX_SAMPLES);

        // 0. Fast Straight-Line Lock (Zero Compute Acceleration Bypass)
        // If the forward corridor is clear for at least cfg.straight_lock_dist (default 1.70m),
        // immediately lock onto the pure straight line (steer = 0.0, speed = top_speed_mps),
        // completely bypassing all candidate polynomial calculations and collision evaluations.
        float lock_dist = (cfg.straight_lock_dist > 0.0f) ? cfg.straight_lock_dist : 1.70f;
        bool straight_clear = false;

        if (costmap_probe_func && costmap_handle) {
            for (int s = 0; s < n_samples; ++s) {
                float u = static_cast<float>(s) / static_cast<float>(n_samples - 1);
                probe_xs[s] = 0.0f;
                probe_ys[s] = 0.05f + u * (lock_dist - 0.05f);
            }
            if (costmap_probe_func(costmap_handle, probe_xs, probe_ys, n_samples) == 1 &&
                std::abs(xg) <= 0.25f && std::abs(theta_g) <= 0.25f) {
                straight_clear = true;
            }
        } else if (yg >= lock_dist && std::abs(xg) <= 0.15f && std::abs(theta_g) <= 0.20f) {
            straight_clear = true;
        }

        if (straight_clear) {
            SplineCandidate& straight_cand = candidate_pool[0];
            straight_cand.d_lat = 0.0f;
            straight_cand.target_head = 0.0f;
            straight_cand.blend = 0.0f;
            straight_cand.n_pts = n_samples;
            float step_ds = lock_dist / static_cast<float>(n_samples - 1);
            float top_spd = (cfg.straight_lock_speed > 0.0f) ? cfg.straight_lock_speed : cfg.top_speed_mps;
            float top_spd_sq = top_spd * top_spd;

            for (int s = 0; s < n_samples; ++s) {
                float u = static_cast<float>(s) / static_cast<float>(n_samples - 1);
                straight_cand.pts[s].x = 0.0f;
                straight_cand.pts[s].y = 0.05f + u * (lock_dist - 0.05f);
                straight_cand.pts[s].dx_du = 0.0f;
                straight_cand.pts[s].dy_du = lock_dist;
                straight_cand.pts[s].d2x_du2 = 0.0f;
                straight_cand.pts[s].d2y_du2 = 0.0f;
                straight_cand.kappa[s] = 0.0f;
                straight_cand.ds[s] = step_ds;
                straight_cand.dkappa_ds[s] = 0.0f;
                straight_cand.v_max_k_sq[s] = top_spd_sq;
                straight_cand.v_profile_sq[s] = top_spd_sq;
            }
            straight_cand.lap_time = lock_dist / std::max(top_spd, 1.0f);
            straight_cand.total_cost = 0.0f;
            straight_cand.clearance_penalty = 0.0f;
            straight_cand.jerk_penalty = 0.0f;
            straight_cand.align_penalty = 0.0f;

            candidate_count = 1;
            surviving_count = 1;
            surviving_indices[0] = 0;
            best_spline_idx = 0;
            best_steer_cmd = 0.0f;
            best_speed_cmd = top_spd;
            best_lap_time = straight_cand.lap_time;
            best_total_cost = 0.0f;
            return;
        }

        // 1. Target List Setup
        struct Target {
            float x, y, aperture;
        };
        Target targets[MAX_TARGETS];
        int num_targets = 0;

        targets[num_targets++] = {xg, yg, aperture_width};

        if (extra_targets_flat && num_extra_targets > 0) {
            int extra_cnt = std::min({num_extra_targets, cfg.max_extra_targets, MAX_TARGETS - 1});
            for (int e = 0; e < extra_cnt; ++e) {
                float tx = extra_targets_flat[e * 3 + 0];
                float ty = extra_targets_flat[e * 3 + 1];
                float tap = extra_targets_flat[e * 3 + 2];
                if (std::hypot(tx - xg, ty - yg) > 0.12f && num_targets < MAX_TARGETS) {
                    targets[num_targets++] = {tx, ty, tap};
                }
            }
        }

        // 2. Dynamic Candidate Grid Setup (GA-tunable density)
        int n_lat = std::clamp(cfg.num_lat_offsets, 1, MAX_OFFSETS);
        float lat_offsets[MAX_OFFSETS];
        float lat_max = std::max(0.02f, cfg.lat_offset_max);
        for (int i = 0; i < n_lat; ++i) {
            float frac = (n_lat > 1) ? (-1.0f + 2.0f * static_cast<float>(i) / static_cast<float>(n_lat - 1)) : 0.0f;
            lat_offsets[i] = frac * lat_max;
        }

        int n_head = std::clamp(cfg.num_head_offsets, 1, MAX_OFFSETS);
        float head_offsets[MAX_OFFSETS];
        float head_max = cfg.head_offset_max;
        for (int i = 0; i < n_head; ++i) {
            float frac = (n_head > 1) ? (-1.0f + 2.0f * static_cast<float>(i) / static_cast<float>(n_head - 1)) : 0.0f;
            head_offsets[i] = frac * head_max;
        }

        int n_blends = std::clamp(cfg.num_blend_weights, 1, MAX_BLENDS);
        n_samples = std::clamp(cfg.probe_samples, 16, MAX_SAMPLES);

        // 3. Dense Candidate Spline Generation
        candidate_count = 0;

        for (int t = 0; t < num_targets; ++t) {
            float curr_tx = targets[t].x;
            float curr_ty = targets[t].y;
            float curr_aperture = targets[t].aperture;
            float curr_theta_g = std::atan2(curr_tx, std::max(0.35f, curr_ty));

            for (int l_idx = 0; l_idx < n_lat; ++l_idx) {
                float d_lat = lat_offsets[l_idx];
                if (std::abs(d_lat) > (curr_aperture * 0.42f)) {
                    continue; // Skip out-of-aperture exploration
                }
                float xt = curr_tx + d_lat;
                float yt = std::max(curr_ty, 0.35f);
                float lt = std::hypot(xt, yt);
                float theta_chord = std::atan2(xt, yt);

                for (int h_idx = 0; h_idx < n_head; ++h_idx) {
                    float d_head = head_offsets[h_idx];
                    float target_head = curr_theta_g + d_head;

                    for (int b_idx = 0; b_idx < n_blends; ++b_idx) {
                        if (candidate_count >= MAX_CANDIDATES) break;

                        float blend = cfg.blend_weights[b_idx];
                        SplineCandidate& cand = candidate_pool[candidate_count];
                        cand.d_lat = d_lat;
                        cand.target_head = target_head;
                        cand.blend = blend;
                        cand.n_pts = n_samples;

                        // Initial boundary state at u=0:
                        // True non-holonomic vehicle kinematics: tangent is aligned with chassis (+Y),
                        // with optional gentle chord bias (blend * theta_chord), while curvature kappa_0
                        // matches the exact current steering angle arc tan(delta0) / wheelbase!
                        float head_0 = blend * theta_chord;
                        float kappa_0 = std::tan(delta0) / cfg.wheelbase;

                        float vx0 = lt * std::sin(head_0);
                        float vy0 = lt * std::cos(head_0);
                        float ax0 = -(lt * lt) * kappa_0 * std::cos(head_0);
                        float ay0 =  (lt * lt) * kappa_0 * std::sin(head_0);

                        // Exact G^3 Initial Actuator Jerk Matching with Calmed Proportional Slew Injection:
                        // Normal jerk matches active steering rate dot_delta_0. If neutral, inject
                        // proportional servo slew toward the target chord to prevent O(u^4) flat starts,
                        // scaled by tunable cfg.initial_slew_scale (e.g. 0.10 - 0.15 instead of 0.40):
                        float turn_angle = -theta_chord; // Positive = turn left (-X in body frame)
                        float ref_angle = std::max(cfg.max_steering_rad * 0.75f, 0.25f);
                        float turn_ratio = std::clamp(turn_angle / ref_angle, -1.0f, 1.0f);
                        // Smooth cubic easing near neutral to suppress aggressive initial steering on gentle targets:
                        float turn_sign = (turn_ratio >= 0.0f) ? 1.0f : -1.0f;
                        float turn_mag = std::abs(turn_ratio);
                        float turn_eased = turn_sign * (turn_mag * turn_mag);

                        float slew_scale = (cfg.initial_slew_scale >= 0.0f) ? cfg.initial_slew_scale : 0.12f;
                        float effective_steer_rate = (std::abs(steer_rate_current) > 0.08f)
                                                     ? steer_rate_current
                                                     : (turn_eased * cfg.max_servo_rate * slew_scale);

                        float dkappa_ds_0 = effective_steer_rate / (cfg.wheelbase * cos2_d0 * std::max(v0, 0.5f));
                        float lt3 = lt * lt * lt;
                        float jx0 = -lt3 * dkappa_ds_0 * std::cos(head_0);
                        float jy0 =  lt3 * dkappa_ds_0 * std::sin(head_0);

                        // Target boundary state at u=1
                        float vx1 = lt * std::sin(target_head);
                        float vy1 = lt * std::cos(target_head);
                        float ax1 = 0.0f;
                        float ay1 = 0.0f;
                        float jx1 = 0.0f;
                        float jy1 = 0.0f;

                        float x_coeffs[8];
                        float y_coeffs[8];
                        solve_septic_coeffs(0.0f, vx0, ax0, jx0, xt, vx1, ax1, jx1, x_coeffs);
                        solve_septic_coeffs(0.0f, vy0, ay0, jy0, yt, vy1, ay1, jy1, y_coeffs);

                        // High-resolution Horner polynomial evaluation
                        for (int s = 0; s < n_samples; ++s) {
                            float u = static_cast<float>(s) / static_cast<float>(n_samples - 1);
                            eval_septic_horner(x_coeffs, u, cand.pts[s].x, cand.pts[s].dx_du, cand.pts[s].d2x_du2);
                            eval_septic_horner(y_coeffs, u, cand.pts[s].y, cand.pts[s].dy_du, cand.pts[s].d2y_du2);
                        }

                        candidate_count++;
                    }
                }
            }
        }

        // 4. Fast Collision Filtering
        surviving_count = 0;
        for (int i = 0; i < candidate_count; ++i) {
            if (probe_candidate(candidate_pool[i])) {
                surviving_indices[surviving_count++] = i;
            }
        }

        if (surviving_count == 0) {
            // Check 3 short emergency escape probe lines (20cm straight and +-10 degrees)
            // before falling back to blind proportional steering:
            constexpr float ESCAPE_DIST = 0.20f;
            constexpr float DEG10_RAD = 0.174533f; // 10 degrees in radians
            constexpr int ESCAPE_PTS = 10;

            struct EscapeCand {
                float steer_cmd;
                float angle_rad;
                bool clear;
            };
            EscapeCand escapes[3] = {
                { 0.0f, 0.0f, false },             // Straight (0 deg)
                { DEG10_RAD, DEG10_RAD, false },   // Left (+10 deg)
                { -DEG10_RAD, -DEG10_RAD, false }  // Right (-10 deg)
            };

            float esc_xs[ESCAPE_PTS];
            float esc_ys[ESCAPE_PTS];

            for (int e = 0; e < 3; ++e) {
                float ang = escapes[e].angle_rad;
                float sin_a = std::sin(ang);
                float cos_a = std::cos(ang);

                for (int p = 0; p < ESCAPE_PTS; ++p) {
                    float s = 0.02f + (static_cast<float>(p) / static_cast<float>(ESCAPE_PTS - 1)) * (ESCAPE_DIST - 0.02f);
                    esc_xs[p] = -s * sin_a; // -X is left, +X is right
                    esc_ys[p] =  s * cos_a; // +Y is forward
                }

                if (costmap_probe_func && costmap_handle) {
                    escapes[e].clear = (costmap_probe_func(costmap_handle, esc_xs, esc_ys, ESCAPE_PTS) == 1);
                } else {
                    escapes[e].clear = true;
                }
            }

            // Selection order based on target offset (xg):
            int chosen_idx = -1;
            if (xg < -0.05f) {
                if (escapes[1].clear) chosen_idx = 1;
                else if (escapes[0].clear) chosen_idx = 0;
                else if (escapes[2].clear) chosen_idx = 2;
            } else if (xg > 0.05f) {
                if (escapes[2].clear) chosen_idx = 2;
                else if (escapes[0].clear) chosen_idx = 0;
                else if (escapes[1].clear) chosen_idx = 1;
            } else {
                if (escapes[0].clear) chosen_idx = 0;
                else if (escapes[1].clear) chosen_idx = 1;
                else if (escapes[2].clear) chosen_idx = 2;
            }

            if (chosen_idx >= 0) {
                SplineCandidate& esc_cand = candidate_pool[0];
                esc_cand.d_lat = 0.0f;
                esc_cand.target_head = escapes[chosen_idx].angle_rad;
                esc_cand.blend = 0.0f;
                esc_cand.n_pts = n_samples;
                float ang = escapes[chosen_idx].angle_rad;
                float sin_a = std::sin(ang);
                float cos_a = std::cos(ang);
                float step_ds = ESCAPE_DIST / static_cast<float>(n_samples - 1);
                float v_cruise = (cfg.v_cruise_min > 0.0f) ? cfg.v_cruise_min : cfg.v_min_cruise;
                float v_sq = v_cruise * v_cruise;

                for (int s = 0; s < n_samples; ++s) {
                    float dist_s = (static_cast<float>(s) / static_cast<float>(n_samples - 1)) * ESCAPE_DIST;
                    esc_cand.pts[s].x = -dist_s * sin_a;
                    esc_cand.pts[s].y =  dist_s * cos_a;
                    esc_cand.pts[s].dx_du = -ESCAPE_DIST * sin_a;
                    esc_cand.pts[s].dy_du =  ESCAPE_DIST * cos_a;
                    esc_cand.pts[s].d2x_du2 = 0.0f;
                    esc_cand.pts[s].d2y_du2 = 0.0f;
                    esc_cand.kappa[s] = 0.0f;
                    esc_cand.ds[s] = step_ds;
                    esc_cand.dkappa_ds[s] = 0.0f;
                    esc_cand.v_max_k_sq[s] = v_sq;
                    esc_cand.v_profile_sq[s] = v_sq;
                }
                esc_cand.lap_time = ESCAPE_DIST / v_cruise;
                esc_cand.total_cost = 0.0f;

                candidate_count = 3;
                surviving_count = 1;
                surviving_indices[0] = 0;
                best_spline_idx = 0;
                best_steer_cmd = escapes[chosen_idx].steer_cmd;
                best_speed_cmd = v_cruise;
                best_lap_time = esc_cand.lap_time;
                best_total_cost = 0.0f;
                return;
            }

            // Safe fallback recovery if all 3 short probe lines also fail:
            best_spline_idx = -1;
            best_steer_cmd = std::clamp(-std::atan2(xg, std::max(0.40f, yg)), -cfg.max_steering_rad, cfg.max_steering_rad);

            // Side Flank Obstacle Guard in fallback: prevent steering into adjacent obstacles
            if (costmap_probe_func && costmap_handle) {
                float left_flank_xs[4]  = {-0.09f, -0.09f, -0.09f, -0.09f};
                float left_flank_ys[4]  = { 0.02f,  0.07f,  0.12f,  0.18f};
                float right_flank_xs[4] = { 0.09f,  0.09f,  0.09f,  0.09f};
                float right_flank_ys[4] = { 0.02f,  0.07f,  0.12f,  0.18f};

                bool left_blocked  = (costmap_probe_func(costmap_handle, left_flank_xs, left_flank_ys, 4) != 1);
                bool right_blocked = (costmap_probe_func(costmap_handle, right_flank_xs, right_flank_ys, 4) != 1);

                if (left_blocked && right_blocked) {
                    best_steer_cmd = 0.0f;
                } else if (left_blocked && best_steer_cmd > 0.0f) {
                    best_steer_cmd = 0.0f;
                } else if (right_blocked && best_steer_cmd < 0.0f) {
                    best_steer_cmd = 0.0f;
                }
            }

            best_speed_cmd = cfg.v_min_cruise;
            best_lap_time = 1e9f;
            best_total_cost = 1e9f;
            return;
        }

        // 5. Bidirectional Dynamic Traction Profiler & Multi-Objective Cost Optimization
        best_total_cost = 1e9f;
        best_lap_time = 1e9f;
        best_spline_idx = surviving_indices[0];
        best_speed_cmd = cfg.v_min_cruise;
        best_steer_cmd = 0.0f;

        const float mu_g = cfg.mu_friction * cfg.gravity_g;
        const float a_accel = std::min(cfg.accel_max, 0.90f * mu_g);

        for (int s_idx = 0; s_idx < surviving_count; ++s_idx) {
            int c_idx = surviving_indices[s_idx];
            SplineCandidate& cand = candidate_pool[c_idx];
            int n_pts = cand.n_pts;

            // Step 1: Exact Curvature & Local Segment Arc Lengths ds_k
            for (int i = 0; i < n_pts; ++i) {
                float dx = cand.pts[i].dx_du;
                float dy = cand.pts[i].dy_du;
                float d2x = cand.pts[i].d2x_du2;
                float d2y = cand.pts[i].d2y_du2;

                float speed_sq = dx * dx + dy * dy;
                float denom = std::max(std::pow(speed_sq, 1.5f), 1e-6f);
                cand.kappa[i] = (dx * d2y - dy * d2x) / denom;
            }

            for (int i = 0; i < n_pts - 1; ++i) {
                float seg_dx = cand.pts[i + 1].x - cand.pts[i].x;
                float seg_dy = cand.pts[i + 1].y - cand.pts[i].y;
                cand.ds[i] = std::max(std::hypot(seg_dx, seg_dy), 1e-4f);
            }

            // Central differentiation for curvature rate dkappa/ds
            cand.dkappa_ds[0] = (cand.kappa[1] - cand.kappa[0]) / cand.ds[0];
            cand.dkappa_ds[n_pts - 1] = (cand.kappa[n_pts - 1] - cand.kappa[n_pts - 2]) / cand.ds[n_pts - 2];
            for (int i = 1; i < n_pts - 1; ++i) {
                cand.dkappa_ds[i] = (cand.kappa[i + 1] - cand.kappa[i - 1]) / (cand.ds[i] + cand.ds[i - 1]);
            }

            // Step 2: Understeer Ceiling & Local Slew Ceiling in v^2 Domain with 15% Grip Reserve
            const float mu_g_eff = mu_g * 0.85f; // Reserve 15% grip for front steering authority
            for (int i = 0; i < n_pts; ++i) {
                float k_abs = std::abs(cand.kappa[i]);
                float v_corner_sq = mu_g_eff / (k_abs + cfg.understeer_k_us * mu_g + 1e-4f);

                // Local steering slew limit: cos^2(delta_k) = 1 / (1 + (L*kappa_k)^2)
                float l_kappa = cfg.wheelbase * cand.kappa[i];
                float inv_cos2_delta_k = 1.0f + l_kappa * l_kappa;
                float dk_abs = std::max(std::abs(cand.dkappa_ds[i]), 1e-3f);
                float v_slew = (cfg.max_servo_rate * inv_cos2_delta_k) / (cfg.wheelbase * dk_abs);
                float v_slew_sq = v_slew * v_slew;

                float v_entry_sq = cfg.v_entry_max * cfg.v_entry_max;
                cand.v_max_k_sq[i] = std::min({v_entry_sq, v_corner_sq, v_slew_sq});
            }

            // Step 3: Backward Trail-Braking Pass in v^2 Domain (Kamm Friction Circle)
            cand.v_profile_sq[n_pts - 1] = cand.v_max_k_sq[n_pts - 1];
            for (int i = n_pts - 2; i >= 0; --i) {
                float v_next_sq = cand.v_profile_sq[i + 1];
                float lat_accel = v_next_sq * std::abs(cand.kappa[i + 1]);
                float grip_ratio = std::min(1.0f, lat_accel / mu_g_eff);
                float grip_rem_sq = std::max(0.0f, 1.0f - grip_ratio * grip_ratio);
                float available_braking = mu_g_eff * std::sqrt(grip_rem_sq);
                float a_brake = std::min(cfg.decel_max, available_braking);

                float v_cand_sq = v_next_sq + 2.0f * a_brake * cand.ds[i];
                cand.v_profile_sq[i] = std::min(cand.v_max_k_sq[i], v_cand_sq);
            }

            // Step 4: Forward Traction Pass in v^2 Domain (Kamm Friction Circle Protection)
            float v0_entry = std::max(v0, cfg.v_cruise_min);
            cand.v_profile_sq[0] = std::min(cand.v_profile_sq[0], v0_entry * v0_entry);
            for (int i = 0; i < n_pts - 1; ++i) {
                float v_curr_sq = cand.v_profile_sq[i];
                float lat_accel = v_curr_sq * std::abs(cand.kappa[i]);
                float grip_ratio = std::min(1.0f, lat_accel / mu_g_eff);
                float grip_rem_sq = std::max(0.0f, 1.0f - grip_ratio * grip_ratio);
                float available_accel = mu_g_eff * std::sqrt(grip_rem_sq);
                float a_accel_i = std::min(a_accel, available_accel);

                float v_next_accel_sq = v_curr_sq + 2.0f * a_accel_i * cand.ds[i];
                cand.v_profile_sq[i + 1] = std::min(cand.v_profile_sq[i + 1], v_next_accel_sq);
            }

            // Traversal Time with Exact Local Segments: T = sum(ds_k / v_seg_k)
            float lap_time = 0.0f;
            float jerk_sum = 0.0f;
            for (int i = 0; i < n_pts - 1; ++i) {
                float v_k = std::sqrt(cand.v_profile_sq[i]);
                float v_k1 = std::sqrt(cand.v_profile_sq[i + 1]);
                float v_seg = 0.5f * (v_k + v_k1);
                lap_time += cand.ds[i] / std::max(v_seg, 0.20f);
                jerk_sum += cand.dkappa_ds[i] * cand.dkappa_ds[i] * cand.ds[i];
            }
            cand.lap_time = lap_time;
            cand.jerk_penalty = jerk_sum;

            // Target chord alignment penalty
            float final_xt = cand.pts[n_pts - 1].x;
            float final_yt = cand.pts[n_pts - 1].y;
            float target_chord = std::atan2(final_xt, std::max(0.35f, final_yt));
            cand.align_penalty = std::abs(cand.target_head - target_chord);

            // Lateral offset clearance margin preference (central bias inside aperture)
            cand.clearance_penalty = std::abs(cand.d_lat);

            // Candidate Speed Command & Steering at Dynamic Lookahead Anchor
            float v_cand_cmd = std::clamp(std::sqrt(cand.v_profile_sq[1]), cfg.v_min_cruise, cfg.v_max_cruise);
            float lookahead_target_m = std::clamp(v_cand_cmd * 0.30f, 0.35f, 0.80f);
            int lk_idx = 2;
            float best_diff = 1e9f;
            for (int i = 2; i < n_pts; ++i) {
                float dist = std::hypot(cand.pts[i].x, cand.pts[i].y);
                float diff = std::abs(dist - lookahead_target_m);
                if (diff < best_diff) {
                    best_diff = diff;
                    lk_idx = i;
                }
            }

            float lx = cand.pts[lk_idx].x;
            float ly = cand.pts[lk_idx].y;
            float l_dist_sq = lx * lx + ly * ly;

            // Combined Stanley / Pure-Pursuit: Spline tangent angle + cross-track curvature scaled by steer_kp
            float theta_tangent = -std::atan2(cand.pts[lk_idx].dx_du, cand.pts[lk_idx].dy_du);
            float pure_pursuit_angle = std::atan2(2.0f * cfg.wheelbase * (-lx), l_dist_sq);
            float req_delta = theta_tangent + cfg.steer_kp * pure_pursuit_angle +
                              cfg.understeer_k_us * (v_cand_cmd * v_cand_cmd) * (2.0f * (-lx) / l_dist_sq);

            float dt_step = (cfg.dt_loop > 0.001f) ? cfg.dt_loop : 0.020f;
            float max_slew_step = cfg.max_servo_rate * dt_step;
            float delta_err = req_delta - delta0;
            float cmd_delta = delta0 + std::clamp(delta_err, -max_slew_step, max_slew_step);
            float steer_cmd = std::clamp(cmd_delta, -cfg.max_steering_rad, cfg.max_steering_rad);

            // Friction circle speed reduction
            float kappa_cmd = std::tan(steer_cmd) / std::max(cfg.wheelbase, 0.01f);
            float ay_needed = (v_cand_cmd * v_cand_cmd) * std::abs(kappa_cmd);
            if (ay_needed > mu_g * 0.80f) {
                float v_lat_max = std::sqrt(mu_g * 0.80f / std::max(std::abs(kappa_cmd), 0.01f));
                v_cand_cmd = std::clamp(v_cand_cmd, cfg.v_min_cruise, v_lat_max);
            }

            float steer_effort = std::abs(steer_cmd - delta0);

            // Multi-Objective Trajectory Cost Formulation
            float total_cost = cfg.w_time * lap_time +
                               cfg.w_curvature_smooth * cand.jerk_penalty +
                               cfg.w_chord_align * cand.align_penalty +
                               cfg.w_clearance * cand.clearance_penalty +
                               cfg.w_steer_effort * steer_effort;
            cand.total_cost = total_cost;

            if (total_cost < best_total_cost) {
                best_total_cost = total_cost;
                best_lap_time = lap_time;
                best_spline_idx = c_idx;
                best_speed_cmd = v_cand_cmd;
                best_steer_cmd = steer_cmd;
            }
        }

        // 6. Side Flank Obstacle Guard: Prevent steering into obstacles touching the vehicle flanks
        if (costmap_probe_func && costmap_handle) {
            float left_flank_xs[4]  = {-0.09f, -0.09f, -0.09f, -0.09f};
            float left_flank_ys[4]  = { 0.02f,  0.07f,  0.12f,  0.18f};
            float right_flank_xs[4] = { 0.09f,  0.09f,  0.09f,  0.09f};
            float right_flank_ys[4] = { 0.02f,  0.07f,  0.12f,  0.18f};

            bool left_blocked  = (costmap_probe_func(costmap_handle, left_flank_xs, left_flank_ys, 4) != 1);
            bool right_blocked = (costmap_probe_func(costmap_handle, right_flank_xs, right_flank_ys, 4) != 1);

            if (left_blocked && right_blocked) {
                // Pinched on both sides: lock straight
                best_steer_cmd = 0.0f;
            } else if (left_blocked && best_steer_cmd > 0.0f) {
                // Left flank touching obstacle: prohibit left turning, steer straight
                best_steer_cmd = 0.0f;
            } else if (right_blocked && best_steer_cmd < 0.0f) {
                // Right flank touching obstacle: prohibit right turning, steer straight
                best_steer_cmd = 0.0f;
            }
        }
    }
};

// ==============================================================================
// C-ABI EXPORTED INTERFACES
// ==============================================================================
extern "C" {

SEPTIC_API void* septic_profiler_create_v2(
    float wheelbase,
    float vehicle_mass,
    float understeer_k_us,
    float tire_mu,
    float gravity_g,
    float max_steering_rad,
    float max_servo_rate,
    float accel_max,
    float decel_max,
    float ds_nominal,
    float top_speed_mps,
    float v_min_cruise,
    float v_max_cruise,
    float dt_loop,
    float v_entry_max,
    float v_cruise_min,
    float mu_friction,
    // Scalable Compute Parameters (GA)
    int probe_samples,
    int num_lat_offsets,
    float lat_offset_max,
    int num_head_offsets,
    float head_offset_max,
    int num_blend_weights,
    const float* blend_weights,
    int max_extra_targets,
    // Multi-objective cost weights (GA)
    float w_time,
    float w_clearance,
    float w_curvature_smooth,
    float w_chord_align,
    float w_steer_effort,
    void* costmap_handle,
    CostmapProbeFn probe_fn
) {
    PlannerConfig cfg;
    cfg.wheelbase = wheelbase;
    cfg.vehicle_mass = vehicle_mass;
    cfg.understeer_k_us = understeer_k_us;
    cfg.tire_mu = tire_mu;
    cfg.gravity_g = gravity_g;
    cfg.max_steering_rad = max_steering_rad;
    cfg.max_servo_rate = max_servo_rate;
    cfg.accel_max = accel_max;
    cfg.decel_max = decel_max;
    cfg.ds_nominal = ds_nominal;
    cfg.top_speed_mps = top_speed_mps;
    cfg.v_min_cruise = v_min_cruise;
    cfg.v_max_cruise = v_max_cruise;
    cfg.dt_loop = dt_loop;
    cfg.v_entry_max = v_entry_max;
    cfg.v_cruise_min = v_cruise_min;
    cfg.mu_friction = mu_friction;

    cfg.probe_samples = probe_samples;
    cfg.num_lat_offsets = num_lat_offsets;
    cfg.lat_offset_max = lat_offset_max;
    cfg.num_head_offsets = num_head_offsets;
    cfg.head_offset_max = head_offset_max;
    cfg.num_blend_weights = num_blend_weights;

    int n_blends = std::clamp(num_blend_weights, 1, MAX_BLENDS);
    if (blend_weights) {
        for (int i = 0; i < n_blends; ++i) {
            cfg.blend_weights[i] = blend_weights[i];
        }
    } else {
        cfg.blend_weights[0] = 0.15f;
        cfg.blend_weights[1] = 0.45f;
        cfg.blend_weights[2] = 0.85f;
        cfg.num_blend_weights = 3;
    }

    cfg.max_extra_targets = max_extra_targets;
    cfg.w_time = w_time;
    cfg.w_clearance = w_clearance;
    cfg.w_curvature_smooth = w_curvature_smooth;
    cfg.w_chord_align = w_chord_align;
    cfg.w_steer_effort = w_steer_effort;
    cfg.steer_kp = 1.35f;
    cfg.pp_lookahead_gain = 0.45f;
    cfg.initial_slew_scale = 0.12f;
    cfg.straight_lock_dist = 1.70f;
    cfg.straight_lock_speed = top_speed_mps;

    return new SepticTractionProfilerNative(cfg, costmap_handle, probe_fn);
}

// V3 Interface with explicit steer_kp and pp_lookahead_gain from GA
SEPTIC_API void* septic_profiler_create_v3(
    float wheelbase,
    float vehicle_mass,
    float understeer_k_us,
    float tire_mu,
    float gravity_g,
    float max_steering_rad,
    float max_servo_rate,
    float accel_max,
    float decel_max,
    float ds_nominal,
    float top_speed_mps,
    float v_min_cruise,
    float v_max_cruise,
    float dt_loop,
    float v_entry_max,
    float v_cruise_min,
    float mu_friction,
    int probe_samples,
    int num_lat_offsets,
    float lat_offset_max,
    int num_head_offsets,
    float head_offset_max,
    int num_blend_weights,
    const float* blend_weights,
    int max_extra_targets,
    float w_time,
    float w_clearance,
    float w_curvature_smooth,
    float w_chord_align,
    float w_steer_effort,
    float steer_kp,
    float pp_lookahead_gain,
    void* costmap_handle,
    CostmapProbeFn probe_fn
) {
    PlannerConfig cfg;
    cfg.wheelbase = wheelbase;
    cfg.vehicle_mass = vehicle_mass;
    cfg.understeer_k_us = understeer_k_us;
    cfg.tire_mu = tire_mu;
    cfg.gravity_g = gravity_g;
    cfg.max_steering_rad = max_steering_rad;
    cfg.max_servo_rate = max_servo_rate;
    cfg.accel_max = accel_max;
    cfg.decel_max = decel_max;
    cfg.ds_nominal = ds_nominal;
    cfg.top_speed_mps = top_speed_mps;
    cfg.v_min_cruise = v_min_cruise;
    cfg.v_max_cruise = v_max_cruise;
    cfg.dt_loop = dt_loop;
    cfg.v_entry_max = v_entry_max;
    cfg.v_cruise_min = v_cruise_min;
    cfg.mu_friction = mu_friction;

    cfg.probe_samples = probe_samples;
    cfg.num_lat_offsets = num_lat_offsets;
    cfg.lat_offset_max = lat_offset_max;
    cfg.num_head_offsets = num_head_offsets;
    cfg.head_offset_max = head_offset_max;
    cfg.num_blend_weights = num_blend_weights;

    int n_blends = std::clamp(num_blend_weights, 1, MAX_BLENDS);
    if (blend_weights) {
        for (int i = 0; i < n_blends; ++i) {
            cfg.blend_weights[i] = blend_weights[i];
        }
    } else {
        cfg.blend_weights[0] = 0.15f;
        cfg.blend_weights[1] = 0.45f;
        cfg.blend_weights[2] = 0.85f;
        cfg.num_blend_weights = 3;
    }

    cfg.max_extra_targets = max_extra_targets;
    cfg.w_time = w_time;
    cfg.w_clearance = w_clearance;
    cfg.w_curvature_smooth = w_curvature_smooth;
    cfg.w_chord_align = w_chord_align;
    cfg.w_steer_effort = w_steer_effort;
    cfg.steer_kp = steer_kp;
    cfg.pp_lookahead_gain = pp_lookahead_gain;
    cfg.initial_slew_scale = 0.12f;
    cfg.straight_lock_dist = 1.70f;
    cfg.straight_lock_speed = top_speed_mps;

    return new SepticTractionProfilerNative(cfg, costmap_handle, probe_fn);
}

// Backwards-compatible legacy create interface
SEPTIC_API void* septic_profiler_create(
    float wheelbase,
    float vehicle_mass,
    float understeer_k_us,
    float tire_mu,
    float gravity_g,
    float max_steering_rad,
    float max_servo_rate,
    float accel_max,
    float decel_max,
    float ds_nominal,
    float top_speed_mps,
    float v_min_cruise,
    float v_max_cruise,
    int probe_samples,
    float dt_loop,
    float blend_w0,
    float blend_w1,
    float blend_w2,
    float v_entry_max,
    float v_cruise_min,
    float mu_friction,
    void* costmap_handle,
    CostmapProbeFn probe_fn
) {
    float default_blends[3] = {blend_w0, blend_w1, blend_w2};
    return septic_profiler_create_v2(
        wheelbase, vehicle_mass, understeer_k_us, tire_mu, gravity_g,
        max_steering_rad, max_servo_rate, accel_max, decel_max, ds_nominal,
        top_speed_mps, v_min_cruise, v_max_cruise, dt_loop,
        v_entry_max, v_cruise_min, mu_friction,
        probe_samples, 3, 0.08f, 3, 0.20f, 3, default_blends, 2,
        1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        costmap_handle, probe_fn
    );
}

SEPTIC_API void septic_profiler_destroy(void* handle) {
    if (handle) {
        delete static_cast<SepticTractionProfilerNative*>(handle);
    }
}

SEPTIC_API void septic_profiler_set_costmap(void* handle, void* costmap_handle, CostmapProbeFn probe_fn) {
    if (handle) {
        auto* prof = static_cast<SepticTractionProfilerNative*>(handle);
        prof->costmap_handle = costmap_handle;
        prof->costmap_probe_func = probe_fn;
    }
}

SEPTIC_API void septic_profiler_set_initial_slew_scale(void* handle, float scale) {
    if (handle) {
        auto* prof = static_cast<SepticTractionProfilerNative*>(handle);
        prof->cfg.initial_slew_scale = scale;
    }
}

SEPTIC_API void septic_profiler_set_straight_lock(void* handle, float dist_m, float speed_mps) {
    if (handle) {
        auto* prof = static_cast<SepticTractionProfilerNative*>(handle);
        prof->cfg.straight_lock_dist = dist_m;
        prof->cfg.straight_lock_speed = speed_mps;
    }
}

SEPTIC_API int septic_profiler_plan(
    void* handle,
    float xg, float yg, float theta_g, float aperture_width,
    float v_current, float steer_current, float yaw_rate_current, float steer_rate_current,
    const float* extra_targets_flat, int num_extra_targets,
    float* out_cmd_steer_rad,
    float* out_cmd_speed_mps,
    float* out_best_lap_time,
    int* out_surviving_count,
    int* out_candidate_count
) {
    if (!handle) return 0;
    auto* prof = static_cast<SepticTractionProfilerNative*>(handle);

    prof->plan(
        xg, yg, theta_g, aperture_width,
        v_current, steer_current, yaw_rate_current, steer_rate_current,
        extra_targets_flat, num_extra_targets
    );

    if (out_cmd_steer_rad) *out_cmd_steer_rad = prof->best_steer_cmd;
    if (out_cmd_speed_mps) *out_cmd_speed_mps = prof->best_speed_cmd;
    if (out_best_lap_time) *out_best_lap_time = prof->best_lap_time;
    if (out_surviving_count) *out_surviving_count = prof->surviving_count;
    if (out_candidate_count) *out_candidate_count = prof->candidate_count;

    return (prof->best_spline_idx >= 0) ? 1 : 0;
}

SEPTIC_API int septic_profiler_get_best_line(
    void* handle,
    float* out_xs,
    float* out_ys,
    int max_points
) {
    if (!handle) return 0;
    auto* prof = static_cast<SepticTractionProfilerNative*>(handle);
    if (prof->best_spline_idx < 0) return 0;

    const SplineCandidate& best = prof->candidate_pool[prof->best_spline_idx];
    int count = std::min(max_points, best.n_pts);
    for (int i = 0; i < count; ++i) {
        out_xs[i] = best.pts[i].x;
        out_ys[i] = best.pts[i].y;
    }
    return count;
}

SEPTIC_API int septic_profiler_get_candidate_lines(
    void* handle,
    float* out_all_xs,
    float* out_all_ys,
    int max_candidates,
    int max_points_per_candidate
) {
    if (!handle) return 0;
    auto* prof = static_cast<SepticTractionProfilerNative*>(handle);

    int cand_count = std::min(max_candidates, prof->candidate_count);
    for (int c = 0; c < cand_count; ++c) {
        const SplineCandidate& cand = prof->candidate_pool[c];
        int count = std::min(max_points_per_candidate, cand.n_pts);
        for (int p = 0; p < count; ++p) {
            int offset = c * max_points_per_candidate + p;
            out_all_xs[offset] = cand.pts[p].x;
            out_all_ys[offset] = cand.pts[p].y;
        }
    }
    return cand_count;
}

}
