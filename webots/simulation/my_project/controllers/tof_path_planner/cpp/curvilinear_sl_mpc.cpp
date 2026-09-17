#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <vector>

#include "osqp.h"
#include "workspace.h"
#include "mpc_qp_tables.hpp"

#if defined(_WIN32) || defined(__CYGWIN__)
  #define SL_MPC_API __declspec(dllexport)
#else
  #define SL_MPC_API __attribute__((visibility("default")))
#endif

// ==============================================================================
// VEHICLE & SOLVER CONSTANTS (Matching planner_config.py)
// ==============================================================================
static constexpr double PI_CONST = 3.14159265358979323846;
static constexpr double WHEELBASE_CONST = 0.1325;
static constexpr double VEHICLE_MASS_CONST = 0.600;
static constexpr double IZ_CONST = 0.0025;
static constexpr double LF_CONST = 0.06625;
static constexpr double LR_CONST = 0.06625;
static constexpr double C_AF_CONST = 34.0;
static constexpr double C_AR_CONST = 38.0;
static constexpr double TIRE_MU_CONST = 0.85;
static constexpr double GRAVITY_G_CONST = 9.81;
static constexpr double MAX_STEERING_RAD_CONST = 0.650;
static constexpr double MAX_SERVO_RATE_RAD_S_CONST = 15.4;
static constexpr double OPT2_WEIGHT_SLACK_EPS_CONST = 500.0;
static constexpr double OPT2_FRICTION_OCTAGON_C1_CONST = 1.2;
static constexpr double OPT2_TARGET_SPEED_MPS_CONST = 4.50;
static constexpr double V_MIN_CRUISE_CONST = 0.60;
static constexpr double V_MAX_CRUISE_CONST = 5.50;
static constexpr double DT_PLANNER_CONST = 1.0 / 60.0;
static constexpr double STRAIGHT_LOCK_DIST_M_CONST = 1.70;
static constexpr double STRAIGHT_LOCK_SPEED_MPS_CONST = 5.50;

// Maximum trajectory points capacity
static constexpr int MAX_TRAJ_POINTS = 64;

// Costmap Probing Function Pointer Signature
typedef int (*CostmapProbeFn)(void* handle, const float* xs, const float* ys, int count);

struct HermiteState {
    double gx;
    double gy;
    double nx;
    double ny;
};

// ==============================================================================
// OPTION 2: CURVILINEAR SL-MPC PLANNER CORE
// ==============================================================================
class CurvilinearSLMPCNative {
public:
    void* costmap_handle;
    CostmapProbeFn costmap_probe_func;

    int N;
    double dt;
    double q_progress;
    double q_cte;
    double q_heading;
    double r_steer;
    double r_accel;
    double r_steer_rate;
    double mu_friction;
    double v_corner_factor;
    double pure_pursuit_blend_gain;
    double lookahead_blend_m;
    double v_cruise_max;
    double v_cruise_min;
    double straight_lock_dist_m;
    double straight_lock_speed_mps;
    double dt_actuator;

    int nx;
    int nu;

    // Preallocated buffers for OSQP data update
    double q_vec[MPC_N_VARS];
    double l_arr[MPC_N_CONSTR];
    double u_arr[MPC_N_CONSTR];
    double P_data[MPC_NNZ_P];
    double A_data[MPC_NNZ_A];

    // Trajectory output buffers
    int best_line_count;
    double best_line_x[MAX_TRAJ_POINTS];
    double best_line_y[MAX_TRAJ_POINTS];

    int guide_line_count;
    double guide_line_x[MAX_TRAJ_POINTS];
    double guide_line_y[MAX_TRAJ_POINTS];

    int corridor_count;
    double corridor_left_x[MAX_TRAJ_POINTS];
    double corridor_left_y[MAX_TRAJ_POINTS];
    double corridor_right_x[MAX_TRAJ_POINTS];
    double corridor_right_y[MAX_TRAJ_POINTS];

    // Independent primal & dual warm-start cache per solver instance (prevents cross-contamination in dual-solve)
    bool has_solution;
    double saved_x[MPC_N_VARS];
    double saved_y[MPC_N_CONSTR];

    CurvilinearSLMPCNative(
        void* cm_handle,
        CostmapProbeFn probe_fn,
        int horizon_N = 30,
        double dt_val = 0.020,
        double q_prog = 8.0,
        double q_lat = 18.0,
        double q_head = 8.0,
        double r_st = 5.0,
        double r_acc = 2.0,
        double r_str_rate = 10.0,
        double mu = TIRE_MU_CONST,
        double v_corner = 0.82,
        double pp_blend = 0.35,
        double lookahead_blend = 0.45,
        double v_max = V_MAX_CRUISE_CONST,
        double v_min = V_MIN_CRUISE_CONST,
        double lock_dist = STRAIGHT_LOCK_DIST_M_CONST,
        double lock_spd = STRAIGHT_LOCK_SPEED_MPS_CONST
    ) : costmap_handle(cm_handle),
        costmap_probe_func(probe_fn),
        N(horizon_N),
        dt(dt_val),
        q_progress(q_prog),
        q_cte(q_lat),
        q_heading(q_head),
        r_steer(r_st),
        r_accel(r_acc),
        r_steer_rate(r_str_rate),
        mu_friction(mu),
        v_corner_factor(v_corner),
        pure_pursuit_blend_gain(pp_blend),
        lookahead_blend_m(lookahead_blend),
        v_cruise_max(v_max),
        v_cruise_min(v_min),
        straight_lock_dist_m(lock_dist),
        straight_lock_speed_mps(lock_spd),
        dt_actuator(DT_PLANNER_CONST),
        nx(7),
        nu(2),
        best_line_count(0),
        guide_line_count(0),
        corridor_count(0),
        has_solution(false)
    {
        // Copy constant structure of A_data
        std::memset(A_data, 0, sizeof(A_data));
        for (int k = 0; k < NUM_CONST_ENTRIES; ++k) {
            A_data[A_CONST_ENTRIES[k].p] = A_CONST_ENTRIES[k].val;
        }
        // Base P_data
        for (int k = 0; k < MPC_NNZ_P; ++k) {
            P_data[k] = P_base_data[k];
        }

        // Balanced high-precision tolerances: matched to actuator resolution (polishing strictly disabled per user request)
        if (solver.settings) {
            solver.settings->eps_abs = 0.0015;
            solver.settings->eps_rel = 0.0015;
            solver.settings->max_iter = 60;
            solver.settings->check_termination = 10;
            solver.settings->polishing = 0;
        }
    }

    // --------------------------------------------------------------------------
    // Probes Costmap laterally along normal vectors [d_min, d_max]
    // --------------------------------------------------------------------------
    void find_corridor_bounds(
        double gx, double gy,
        double nx_left, double ny_left,
        double pref_side,
        double& out_d_min, double& out_d_max
    ) {
        // High-density lateral probing: d_vals from -0.65 to 0.65 step 0.010 (131 samples, 10 mm resolution)
        constexpr int N_PROBE = 131;
        float pxs[N_PROBE];
        float pys[N_PROBE];
        float d_vals[N_PROBE];

        for (int i = 0; i < N_PROBE; ++i) {
            float d = -0.65f + i * 0.010f;
            d_vals[i] = d;
            pxs[i] = static_cast<float>(gx + d * nx_left);
            pys[i] = static_cast<float>(gy + d * ny_left);
        }

        bool free_mask[N_PROBE];
        bool any_free = false;

        if (costmap_probe_func && costmap_handle) {
            for (int i = 0; i < N_PROBE; ++i) {
                free_mask[i] = (costmap_probe_func(costmap_handle, &pxs[i], &pys[i], 1) == 1);
                if (free_mask[i]) any_free = true;
            }
        } else {
            // Default free when no costmap is attached
            for (int i = 0; i < N_PROBE; ++i) free_mask[i] = true;
            any_free = true;
        }

        if (!any_free) {
            out_d_min = -0.05;
            out_d_max = 0.05;
            return;
        }

        // Identify contiguous free segments
        struct Segment {
            int s;
            int e;
        };
        Segment segments[N_PROBE];
        int n_segs = 0;

        bool in_seg = false;
        int start_idx = 0;
        for (int i = 0; i < N_PROBE; ++i) {
            if (free_mask[i] && !in_seg) {
                in_seg = true;
                start_idx = i;
            } else if (!free_mask[i] && in_seg) {
                in_seg = false;
                segments[n_segs++] = {start_idx, i - 1};
            }
        }
        if (in_seg) {
            segments[n_segs++] = {start_idx, N_PROBE - 1};
        }

        if (n_segs == 0) {
            out_d_min = -0.05;
            out_d_max = 0.05;
            return;
        }

        struct ValidSeg {
            double d_min;
            double d_max;
        };
        ValidSeg valid_segs[N_PROBE];
        int n_valid = 0;

        for (int i = 0; i < n_segs; ++i) {
            double w = d_vals[segments[i].e] - d_vals[segments[i].s];
            if (w >= 0.12) {
                valid_segs[n_valid++] = {d_vals[segments[i].s], d_vals[segments[i].e]};
            }
        }

        if (n_valid == 0) {
            // Pick widest
            int best_idx = 0;
            int max_span = segments[0].e - segments[0].s;
            for (int i = 1; i < n_segs; ++i) {
                int span = segments[i].e - segments[i].s;
                if (span > max_span) {
                    max_span = span;
                    best_idx = i;
                }
            }
            out_d_min = d_vals[segments[best_idx].s];
            out_d_max = d_vals[segments[best_idx].e];
            return;
        }

        // Check if d=0 is contained in any valid segment
        for (int i = 0; i < n_valid; ++i) {
            if (valid_segs[i].d_min <= 0.0 && 0.0 <= valid_segs[i].d_max) {
                out_d_min = valid_segs[i].d_min;
                out_d_max = valid_segs[i].d_max;
                return;
            }
        }

        // Choose segment closest to preferred bypass side
        double target_center = pref_side * 0.30;
        int best_idx = 0;
        double min_dist = std::abs(0.5 * (valid_segs[0].d_min + valid_segs[0].d_max) - target_center);
        for (int i = 1; i < n_valid; ++i) {
            double d = std::abs(0.5 * (valid_segs[i].d_min + valid_segs[i].d_max) - target_center);
            if (d < min_dist) {
                min_dist = d;
                best_idx = i;
            }
        }
        out_d_min = valid_segs[best_idx].d_min;
        out_d_max = valid_segs[best_idx].d_max;
    }

    // --------------------------------------------------------------------------
    // Solves 7x7 Linear System (I - 0.5*dt*Ac) * [Ad, Bd] = [I + 0.5*dt*Ac, dt*Bc]
    // --------------------------------------------------------------------------
    void discretize_dynamics(
        double v_bar, double dt_val,
        double Ad[7][7], double Bd[7][2]
    ) {
        double Ac[7][7] = {0};
        Ac[0][3] = 1.0;
        Ac[1][2] = v_bar;
        Ac[1][4] = 1.0;
        Ac[2][5] = 1.0;

        double denom_v = std::max(v_bar, 0.20);
        Ac[4][4] = -(C_AF_CONST + C_AR_CONST) / (VEHICLE_MASS_CONST * denom_v);
        Ac[4][5] = -(LF_CONST * C_AF_CONST - LR_CONST * C_AR_CONST) / (VEHICLE_MASS_CONST * denom_v) - v_bar;
        Ac[4][6] = C_AF_CONST / VEHICLE_MASS_CONST;

        Ac[5][4] = -(LF_CONST * C_AF_CONST - LR_CONST * C_AR_CONST) / (IZ_CONST * denom_v);
        Ac[5][5] = -(LF_CONST * LF_CONST * C_AF_CONST + LR_CONST * LR_CONST * C_AR_CONST) / (IZ_CONST * denom_v);
        Ac[5][6] = (LF_CONST * C_AF_CONST) / IZ_CONST;

        double Bc[7][2] = {0};
        Bc[3][0] = 1.0 / VEHICLE_MASS_CONST;
        Bc[6][1] = 1.0;

        // Form M_left = I - 0.5 * dt * Ac
        // Form M_right_Ad = I + 0.5 * dt * Ac
        // Form M_right_Bd = dt * Bc
        // Augment: [M_left | M_right_Ad | M_right_Bd] of size 7 x (7 + 7 + 2) = 7 x 16
        double M[7][16] = {0};
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                double delta = (i == j) ? 1.0 : 0.0;
                M[i][j] = delta - 0.5 * dt_val * Ac[i][j];
                M[i][7 + j] = delta + 0.5 * dt_val * Ac[i][j];
            }
            M[i][14] = dt_val * Bc[i][0];
            M[i][15] = dt_val * Bc[i][1];
        }

        // Gauss-Jordan elimination with partial pivoting
        for (int i = 0; i < 7; ++i) {
            int pivot = i;
            double max_val = std::abs(M[i][i]);
            for (int r = i + 1; r < 7; ++r) {
                double val = std::abs(M[r][i]);
                if (val > max_val) {
                    max_val = val;
                    pivot = r;
                }
            }
            if (pivot != i) {
                for (int c = 0; c < 16; ++c) {
                    std::swap(M[i][c], M[pivot][c]);
                }
            }

            double diag = M[i][i];
            double inv_diag = 1.0 / diag;
            for (int c = 0; c < 16; ++c) {
                M[i][c] *= inv_diag;
            }

            for (int r = 0; r < 7; ++r) {
                if (r != i) {
                    double factor = M[r][i];
                    for (int c = 0; c < 16; ++c) {
                        M[r][c] -= factor * M[i][c];
                    }
                }
            }
        }

        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                Ad[i][j] = M[i][7 + j];
            }
            Bd[i][0] = M[i][14];
            Bd[i][1] = M[i][15];
        }
    }

    // --------------------------------------------------------------------------
    // Evaluates Hermite Spline Reference Curve: position and left unit normal
    // --------------------------------------------------------------------------
    static inline HermiteState eval_hermite(
        double u, double T0_x, double T0_y, double xg, double yg, double T1_x, double T1_y
    ) {
        u = std::max(0.0, std::min(1.0, u));
        double u2 = u * u;
        double u3 = u2 * u;

        double h10 = u3 - 2.0 * u2 + u;
        double h01 = -2.0 * u3 + 3.0 * u2;
        double h11 = u3 - u2;

        double gx = h10 * T0_x + h01 * xg + h11 * T1_x;
        double gy = h10 * T0_y + h01 * yg + h11 * T1_y;

        double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
        double dh01 = -6.0 * u2 + 6.0 * u;
        double dh11 = 3.0 * u2 - 2.0 * u;

        double dx = dh10 * T0_x + dh01 * xg + dh11 * T1_x;
        double dy = dh10 * T0_y + dh01 * yg + dh11 * T1_y;

        double t_norm = std::hypot(dx, dy);
        double nx, ny;
        if (t_norm < 1e-4) {
            nx = -1.0;
            ny = 0.0;
        } else {
            nx = -dy / t_norm;
            ny = dx / t_norm;
        }
        return {gx, gy, nx, ny};
    }

    // --------------------------------------------------------------------------
    // Evaluates analytical curvature kappa(u) of Hermite Spline Reference Curve
    // --------------------------------------------------------------------------
    static inline double eval_hermite_curvature(
        double u, double T0_x, double T0_y, double xg, double yg, double T1_x, double T1_y
    ) {
        u = std::max(0.0, std::min(1.0, u));
        double u2 = u * u;

        double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
        double dh01 = -6.0 * u2 + 6.0 * u;
        double dh11 = 3.0 * u2 - 2.0 * u;

        double dx = dh10 * T0_x + dh01 * xg + dh11 * T1_x;
        double dy = dh10 * T0_y + dh01 * yg + dh11 * T1_y;

        double d2h10 = 6.0 * u - 4.0;
        double d2h01 = -12.0 * u + 6.0;
        double d2h11 = 6.0 * u - 2.0;

        double ddx = d2h10 * T0_x + d2h01 * xg + d2h11 * T1_x;
        double ddy = d2h10 * T0_y + d2h01 * yg + d2h11 * T1_y;

        double speed_sq = dx * dx + dy * dy;
        double denom = std::pow(speed_sq, 1.5);
        if (denom < 1e-6) return 0.0;
        return std::abs(dx * ddy - dy * ddx) / denom;
    }

    // --------------------------------------------------------------------------
    // MAIN OPTION 2 SL-MPC PLAN FUNCTION
    // --------------------------------------------------------------------------
    int plan(
        double xg, double yg, double theta_g, double /*aperture_width*/,
        double v_current, double steer_current, double yaw_rate_current,
        double* out_cmd_steer_rad,
        double* out_cmd_speed_mps,
        double* out_best_cost,
        int* out_osqp_iters,
        int* out_status_code
    ) {
        double v_bar = std::max(v_current, V_MIN_CRUISE_CONST);
        double delta_0 = std::max(-MAX_STEERING_RAD_CONST, std::min(MAX_STEERING_RAD_CONST, steer_current));
        double r_0 = yaw_rate_current;

        // 0. Fast Straight-Line Lock (Zero Compute Bypass when Corridor is Open Ahead >= 1.70m)
        if (yg >= straight_lock_dist_m && std::abs(xg) <= 0.20 && std::abs(theta_g) <= 0.22) {
            *out_cmd_steer_rad = 0.0;
            *out_cmd_speed_mps = straight_lock_speed_mps;
            *out_best_cost = 0.0;
            *out_osqp_iters = 0;
            *out_status_code = 1; // STRAIGHT_LOCKED

            best_line_count = N + 1;
            for (int k = 0; k <= N; ++k) {
                double u = static_cast<double>(k) / N;
                best_line_x[k] = 0.0;
                best_line_y[k] = 0.05 + u * (straight_lock_dist_m - 0.05);
            }
            guide_line_count = 0;
            corridor_count = 0;
            return 1;
        }

        // 1. Establish Curvilinear Reference Spline towards gap centroid
        double L_guide = std::hypot(xg, std::max(0.35, yg));
        double v_nom = std::max(v_bar, 1.20);
        double dt_val = this->dt;

        // Tangent vector at start:
        double T0_x = -L_guide * std::sin(delta_0);
        double T0_y = L_guide * std::cos(delta_0);

        // Tangent vector at target gap opening:
        double T1_x = xg;
        double T1_y = std::max(0.35, yg);

        // Corner braking ceiling from target gap curvature and local spline curvature
        double dist_gap_sq = xg * xg + yg * yg;
        double kappa_turn = 2.0 * std::abs(xg) / std::max(dist_gap_sq, 0.04);
        double lat_limit = mu_friction * GRAVITY_G_CONST * v_corner_factor;
        double a_decel = 1.00 * mu_friction * GRAVITY_G_CONST;

        // Compute local curvature-limited speed ceiling at each stage k
        double v_ref_k[N + 1];
        for (int k = 0; k <= N; ++k) {
            double sk = std::min(L_guide, k * v_nom * dt_val);
            double uk = sk / std::max(L_guide, 0.01);
            double kappa_k = std::max(kappa_turn, eval_hermite_curvature(uk, T0_x, T0_y, xg, yg, T1_x, T1_y));
            double v_corner_k = std::sqrt(lat_limit / std::max(kappa_k, 0.08));
            v_ref_k[k] = std::min(v_cruise_max, std::max(v_cruise_min, v_corner_k));
        }

        // Backward dynamic braking pass (kinematic reachability)
        for (int k = N - 1; k >= 0; --k) {
            double ds_step = v_nom * dt_val;
            double v_brake_limit = std::sqrt(v_ref_k[k + 1] * v_ref_k[k + 1] + 2.0 * a_decel * ds_step);
            v_ref_k[k] = std::min(v_ref_k[k], v_brake_limit);
        }
        double v_ref = v_ref_k[0];

        // Preferred bypass side
        double pref_side = (xg < 0.0) ? 1.0 : ((xg > 0.0) ? -1.0 : 0.0);



        // Sample corridor bounds for each MPC horizon step k with continuous sub-stepping:
        double d_min_bounds[N + 1];
        double d_max_bounds[N + 1];
        HermiteState step_states[N + 1];

        // Front-bumper clearance margin based on initial yaw error estimate
        double e_psi_0 = std::atan2(xg, std::max(0.35, yg));
        double yaw_clearance_margin = LF_CONST * std::abs(std::sin(e_psi_0));

        for (int k = 0; k <= N; ++k) {
            double sk = std::min(L_guide, k * v_nom * dt_val);
            double uk = sk / std::max(L_guide, 0.01);
            step_states[k] = eval_hermite(uk, T0_x, T0_y, xg, yg, T1_x, T1_y);

            double d_min_k, d_max_k;
            find_corridor_bounds(step_states[k].gx, step_states[k].gy, step_states[k].nx, step_states[k].ny, pref_side, d_min_k, d_max_k);

            // Sub-step continuous collision envelope: focused on immediate near-field horizon (k <= 6)
            if (k >= 1 && k <= 6) {
                double sk_prev = std::min(L_guide, (k - 1) * v_nom * dt_val);
                double u_mid = (0.5 * (sk_prev + sk)) / std::max(L_guide, 0.01);
                HermiteState s_mid = eval_hermite(u_mid, T0_x, T0_y, xg, yg, T1_x, T1_y);
                double d_min_mid, d_max_mid;
                find_corridor_bounds(s_mid.gx, s_mid.gy, s_mid.nx, s_mid.ny, pref_side, d_min_mid, d_max_mid);
                d_min_k = std::max(d_min_k, d_min_mid);
                d_max_k = std::min(d_max_k, d_max_mid);
            }

            // Apply vehicle yaw overhang margin
            d_min_k += yaw_clearance_margin;
            d_max_k -= yaw_clearance_margin;

            d_min_bounds[k] = std::max(-0.60, std::min(-0.04, d_min_k));
            d_max_bounds[k] = std::max(0.04, std::min(0.60, d_max_k));
            if (d_min_bounds[k] > d_max_bounds[k]) {
                d_min_bounds[k] = -0.10;
                d_max_bounds[k] = 0.10;
            }
        }

        // Zero-cost corridor & guide line populated directly from MPC horizon samples
        corridor_count = N + 1;
        guide_line_count = N + 1;
        for (int k = 0; k <= N; ++k) {
            guide_line_x[k] = step_states[k].gx;
            guide_line_y[k] = step_states[k].gy;
            corridor_left_x[k] = step_states[k].gx + d_max_bounds[k] * step_states[k].nx;
            corridor_left_y[k] = step_states[k].gy + d_max_bounds[k] * step_states[k].ny;
            corridor_right_x[k] = step_states[k].gx + d_min_bounds[k] * step_states[k].nx;
            corridor_right_y[k] = step_states[k].gy + d_min_bounds[k] * step_states[k].ny;
        }

        // 3. Discretize dynamics
        double Ad[7][7];
        double Bd[7][2];
        discretize_dynamics(v_bar, dt_val, Ad, Bd);

        // Update A_data with Ad and Bd entries
        for (int idx = 0; idx < NUM_AD_ENTRIES; ++idx) {
            const auto& e = A_AD_ENTRIES[idx];
            A_data[e.p] = -Ad[e.i][e.j];
        }
        for (int idx = 0; idx < NUM_BD_ENTRIES; ++idx) {
            const auto& e = A_BD_ENTRIES[idx];
            A_data[e.p] = -Bd[e.i][e.j];
        }

        // Friction envelope limits
        double lat_accel_max = 0.95 * mu_friction * GRAVITY_G_CONST;
        double max_Fx = mu_friction * VEHICLE_MASS_CONST * GRAVITY_G_CONST;
        double c1 = OPT2_FRICTION_OCTAGON_C1_CONST;
        double s_fx = 1.0 / std::max(max_Fx, 1e-3);

        // Update Diamond entries in A_data
        for (int idx = 0; idx < NUM_DIAMOND_FX_ENTRIES; ++idx) {
            const auto& e = A_DIAMOND_FX_ENTRIES[idx];
            A_data[e.p] = e.sx * s_fx;
        }
        for (int idx = 0; idx < NUM_DIAMOND_DELTA_ENTRIES; ++idx) {
            const auto& e = A_DIAMOND_DELTA_ENTRIES[idx];
            double vk_nom = std::max(v_ref_k[e.k], v_bar - e.k * a_decel * dt_val);
            double s_fy_k = c1 * (vk_nom * vk_nom) / std::max(0.10, lat_accel_max * WHEELBASE_CONST);
            A_data[e.p] = e.sy * s_fy_k;
        }

        // Setup linear cost q_vec
        std::memset(q_vec, 0, sizeof(q_vec));
        double r_acc = r_accel;
        constexpr int VX_P_INDICES[31] = {2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 50, 54, 58, 62, 66, 70, 74, 78, 82, 86, 90, 94, 98, 102, 106, 110, 114, 118, 122};

        for (int k = 0; k <= N; ++k) {
            int ix = k * nx;
            double vk_target = v_ref_k[k];
            double w_v_k = (vk_target < 2.2) ? (r_acc * 3.0) : r_acc;
            q_vec[ix + 3] += -w_v_k * vk_target;
            P_data[VX_P_INDICES[k]] = w_v_k;
            int ie_c = (N + 1) * nx + N * nu + k;
            q_vec[ie_c] += OPT2_WEIGHT_SLACK_EPS_CONST;
        }

        double w_progress = (v_ref < 2.0) ? (q_progress * 0.35) : q_progress;
        q_vec[N * nx] += -w_progress;

        double w_slip_slack = 150.0;
        for (int k = 1; k <= N; ++k) {
            int ie_s = (N + 1) * nx + N * nu + (N + 1) + (k - 1);
            q_vec[ie_s] += w_slip_slack;
        }

        // 4. Construct Bounds vectors l_arr and u_arr
        std::memset(l_arr, 0, sizeof(l_arr));
        std::memset(u_arr, 0, sizeof(u_arr));

        // Initial state
        double x_init[7] = {0.0, 0.0, e_psi_0, v_bar, 0.0, r_0, delta_0};
        for (int i = 0; i < 7; ++i) {
            l_arr[i] = x_init[i];
            u_arr[i] = x_init[i];
        }

        // Dynamics: l=0, u=0 (already 0)

        // Slack Corridor
        int row_corr = 7 + N * nx;
        for (int k = 0; k <= N; ++k) {
            l_arr[row_corr + 2 * k] = -10.0;
            u_arr[row_corr + 2 * k] = d_max_bounds[k];
            l_arr[row_corr + 2 * k + 1] = d_min_bounds[k];
            u_arr[row_corr + 2 * k + 1] = 10.0;
        }

        // Slew limits
        int row_slew = row_corr + 2 * (N + 1);
        for (int k = 0; k < N; ++k) {
            l_arr[row_slew + k] = -MAX_SERVO_RATE_RAD_S_CONST;
            u_arr[row_slew + k] = MAX_SERVO_RATE_RAD_S_CONST;
        }

        // Steer limits
        int row_steer = row_slew + N;
        for (int k = 0; k < N; ++k) {
            l_arr[row_steer + k] = -MAX_STEERING_RAD_CONST;
            u_arr[row_steer + k] = MAX_STEERING_RAD_CONST;
        }

        // Physical Tire Traction limit
        int row_trac = row_steer + N;
        for (int k = 1; k <= N; ++k) {
            int idx = k - 1;
            double vk_nom = std::max(v_ref, v_bar - k * a_decel * dt_val);
            double delta_k_max = std::max(
                8.0 * PI_CONST / 180.0,
                std::min(MAX_STEERING_RAD_CONST, std::atan((lat_accel_max * WHEELBASE_CONST) / std::max(0.20, vk_nom * vk_nom)))
            );
            l_arr[row_trac + 2 * idx] = -10.0;
            u_arr[row_trac + 2 * idx] = delta_k_max;
            l_arr[row_trac + 2 * idx + 1] = -delta_k_max;
            u_arr[row_trac + 2 * idx + 1] = 10.0;
        }

        // Slack non-negativity
        int row_slack_corr = row_trac + 2 * N;
        for (int k = 0; k <= N; ++k) {
            l_arr[row_slack_corr + k] = 0.0;
            u_arr[row_slack_corr + k] = 1.0;
        }
        int row_slack_slip = row_slack_corr + (N + 1);
        for (int k = 0; k < N; ++k) {
            l_arr[row_slack_slip + k] = 0.0;
            u_arr[row_slack_slip + k] = 1.0;
        }

        // Motor Force bounds
        int row_motor = row_slack_slip + N;
        for (int k = 0; k < N; ++k) {
            l_arr[row_motor + k] = -max_Fx;
            u_arr[row_motor + k] = max_Fx;
        }

        // Friction diamond bounds
        int row_diamond = row_motor + N;
        for (int k = 0; k < 4 * (N - 1); ++k) {
            l_arr[row_diamond + k] = -10.0;
            u_arr[row_diamond + k] = 1.25;
        }

        // 5. Update OSQP problem data and solve
        osqp_update_data_mat(&solver, P_data, OSQP_NULL, MPC_NNZ_P, A_data, OSQP_NULL, MPC_NNZ_A);
        osqp_update_data_vec(&solver, q_vec, l_arr, u_arr);

        // Load instance-specific warm start to prevent dual-solve cross-contamination
        if (has_solution) {
            osqp_warm_start(&solver, saved_x, saved_y);
        } else {
            osqp_cold_start(&solver);
        }

        osqp_solve(&solver);

        int status_val = solver.info->status_val;



        *out_osqp_iters = solver.info->iter;
        *out_best_cost = solver.info->obj_val;

        if (status_val == 1 || status_val == 2 || status_val == 7) {
            *out_status_code = 0; // OPTIMAL_OSQP_MPC

            // Cache solution for this instance's next warm start
            for (int i = 0; i < MPC_N_VARS; ++i) {
                saved_x[i] = solver.solution->x[i];
            }
            if (solver.solution->y) {
                for (int i = 0; i < MPC_N_CONSTR; ++i) {
                    saved_y[i] = solver.solution->y[i];
                }
            }
            has_solution = true;

            // Extract best predicted line
            best_line_count = N + 1;
            double last_u = 0.0;
            for (int k = 0; k <= N; ++k) {
                double pk_s = solver.solution->x[k * nx + 0];
                double pk_d = solver.solution->x[k * nx + 1];
                double uk = std::max(0.0, std::min(1.0, pk_s / std::max(L_guide, 0.01)));
                HermiteState s = eval_hermite(uk, T0_x, T0_y, xg, yg, T1_x, T1_y);
                best_line_x[k] = s.gx + pk_d * s.nx;
                best_line_y[k] = s.gy + pk_d * s.ny;
                last_u = uk;
            }

            // Reference curve extension into gap aperture
            if (last_u < 0.96) {
                int n_ext = std::max(6, static_cast<int>(std::round((1.0 - last_u) * 20.0)));
                double p_d_last = solver.solution->x[N * nx + 1];
                for (int i = 1; i < n_ext && best_line_count < MAX_TRAJ_POINTS; ++i) {
                    double u = last_u + (1.0 - last_u) * (static_cast<double>(i) / (n_ext - 1));
                    HermiteState s = eval_hermite(u, T0_x, T0_y, xg, yg, T1_x, T1_y);
                    double decay = 1.0 - static_cast<double>(i) / (n_ext - 1);
                    best_line_x[best_line_count] = s.gx + (p_d_last * decay) * s.nx;
                    best_line_y[best_line_count] = s.gy + (p_d_last * decay) * s.ny;
                    best_line_count++;
                }
            }

            // Optimal steering output
            double v_delta_0 = solver.solution->x[(N + 1) * nx + 1];
            double delta_opt = delta_0 + v_delta_0 * dt_actuator;
            double cmd_steer = std::max(-MAX_STEERING_RAD_CONST, std::min(MAX_STEERING_RAD_CONST, delta_opt));

            // Pure pursuit blend
            if (pure_pursuit_blend_gain > 0.01) {
                double pp_u = std::min(1.0, lookahead_blend_m / std::max(0.10, L_guide));
                HermiteState pp_s = eval_hermite(pp_u, T0_x, T0_y, xg, yg, T1_x, T1_y);
                double pp_dist_sq = pp_s.gx * pp_s.gx + pp_s.gy * pp_s.gy;
                double pp_steer = std::max(-MAX_STEERING_RAD_CONST, std::min(MAX_STEERING_RAD_CONST, 2.0 * (-pp_s.gx) * WHEELBASE_CONST / std::max(0.04, pp_dist_sq)));
                cmd_steer = std::max(-MAX_STEERING_RAD_CONST, std::min(MAX_STEERING_RAD_CONST, (1.0 - pure_pursuit_blend_gain) * cmd_steer + pure_pursuit_blend_gain * pp_steer));
            }
            *out_cmd_steer_rad = cmd_steer;

            // Optimal forward speed command
            double fx_0 = solver.solution->x[(N + 1) * nx + 0];
            double cmd_speed;
            if (fx_0 < -0.15) {
                cmd_speed = std::max(v_cruise_min, std::min(v_cruise_max, v_bar + (fx_0 / VEHICLE_MASS_CONST) * dt_actuator));
            } else {
                double max_accel = 0.85 * mu_friction * GRAVITY_G_CONST;
                cmd_speed = std::max(v_cruise_min, std::min(v_cruise_max, std::min(v_ref, v_bar + max_accel * dt_actuator * 2.0)));
            }
            *out_cmd_speed_mps = cmd_speed;
            return 1;
        }

        // Fallback on solver error
        *out_status_code = -1; // FALLBACK_SL_MPC
        *out_cmd_steer_rad = std::max(-MAX_STEERING_RAD_CONST, std::min(MAX_STEERING_RAD_CONST, -std::atan2(xg, std::max(0.40, yg))));
        *out_cmd_speed_mps = v_cruise_min;

        best_line_count = 16;
        for (int i = 0; i < 16; ++i) {
            double u = static_cast<double>(i) / 15.0;
            HermiteState s = eval_hermite(u, T0_x, T0_y, xg, yg, T1_x, T1_y);
            best_line_x[i] = s.gx;
            best_line_y[i] = s.gy;
        }
        return 0;
    }
};

// ==============================================================================
// C-ABI EXPORTS FOR PYTHON CTYPES
// ==============================================================================
extern "C" {

SL_MPC_API void* sl_mpc_create(
    void* costmap_handle,
    CostmapProbeFn probe_fn,
    int horizon_N,
    double dt,
    double q_progress,
    double q_cte,
    double q_heading,
    double r_steer,
    double r_accel,
    double r_steer_rate,
    double mu_friction,
    double v_corner_factor,
    double pure_pursuit_blend_gain,
    double lookahead_blend_m,
    double v_cruise_max,
    double v_cruise_min,
    double straight_lock_dist_m,
    double straight_lock_speed_mps
) {
    return new CurvilinearSLMPCNative(
        costmap_handle, probe_fn, horizon_N, dt, q_progress, q_cte, q_heading,
        r_steer, r_accel, r_steer_rate, mu_friction, v_corner_factor,
        pure_pursuit_blend_gain, lookahead_blend_m, v_cruise_max, v_cruise_min,
        straight_lock_dist_m, straight_lock_speed_mps
    );
}

SL_MPC_API void sl_mpc_destroy(void* handle) {
    if (handle) {
        delete static_cast<CurvilinearSLMPCNative*>(handle);
    }
}

SL_MPC_API void sl_mpc_set_costmap(void* handle, void* cm_handle, CostmapProbeFn probe_fn) {
    if (handle) {
        auto* planner = static_cast<CurvilinearSLMPCNative*>(handle);
        planner->costmap_handle = cm_handle;
        planner->costmap_probe_func = probe_fn;
    }
}

SL_MPC_API int sl_mpc_plan(
    void* handle,
    double xg, double yg, double theta_g, double aperture_width,
    double v_current, double steer_current, double yaw_rate_current,
    double* out_cmd_steer_rad,
    double* out_cmd_speed_mps,
    double* out_best_cost,
    int* out_osqp_iters,
    int* out_status_code
) {
    if (!handle) return 0;
    auto* planner = static_cast<CurvilinearSLMPCNative*>(handle);
    return planner->plan(
        xg, yg, theta_g, aperture_width,
        v_current, steer_current, yaw_rate_current,
        out_cmd_steer_rad, out_cmd_speed_mps,
        out_best_cost, out_osqp_iters, out_status_code
    );
}

SL_MPC_API int sl_mpc_get_best_line(
    void* handle,
    double* out_xs,
    double* out_ys,
    int max_points
) {
    if (!handle) return 0;
    auto* planner = static_cast<CurvilinearSLMPCNative*>(handle);
    int count = std::min(max_points, planner->best_line_count);
    for (int i = 0; i < count; ++i) {
        out_xs[i] = planner->best_line_x[i];
        out_ys[i] = planner->best_line_y[i];
    }
    return count;
}

SL_MPC_API int sl_mpc_get_guide_line(
    void* handle,
    double* out_xs,
    double* out_ys,
    int max_points
) {
    if (!handle) return 0;
    auto* planner = static_cast<CurvilinearSLMPCNative*>(handle);
    int count = std::min(max_points, planner->guide_line_count);
    for (int i = 0; i < count; ++i) {
        out_xs[i] = planner->guide_line_x[i];
        out_ys[i] = planner->guide_line_y[i];
    }
    return count;
}

SL_MPC_API int sl_mpc_get_corridor(
    void* handle,
    double* out_left_xs, double* out_left_ys,
    double* out_right_xs, double* out_right_ys,
    int max_points
) {
    if (!handle) return 0;
    auto* planner = static_cast<CurvilinearSLMPCNative*>(handle);
    int count = std::min(max_points, planner->corridor_count);
    for (int i = 0; i < count; ++i) {
        out_left_xs[i] = planner->corridor_left_x[i];
        out_left_ys[i] = planner->corridor_left_y[i];
        out_right_xs[i] = planner->corridor_right_x[i];
        out_right_ys[i] = planner->corridor_right_y[i];
    }
    return count;
}

} // extern "C"
