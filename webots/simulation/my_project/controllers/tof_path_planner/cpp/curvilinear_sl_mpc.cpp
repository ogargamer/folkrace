#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "mpc_qp_tables.hpp"
#include "osqp.h"
#include "workspace.h"

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
static constexpr double MAX_STEERING_RAD_CONST =
    0.562285; // ~32.22 deg 45% Ackermann center limit (yields 35.0 deg
              // inner, 30.0 deg outer)
static constexpr double MAX_SERVO_RATE_RAD_S_CONST =
    11.16; // Half-speed 1st test (50% of 22.31 rad/s PTK 9497 tire rate)
static constexpr double MPC_WEIGHT_SLACK_EPS_CONST = 500.0;
static constexpr double MPC_FRICTION_OCTAGON_C1_CONST = 1.2;
static constexpr double MPC_TARGET_SPEED_MPS_CONST = 4.50;
static constexpr double V_MIN_CRUISE_CONST = 0.60;
static constexpr double V_MAX_CRUISE_CONST = 5.50;
static constexpr double DT_PLANNER_CONST = 1.0 / 60.0;
static constexpr double STRAIGHT_LOCK_DIST_M_CONST = 1.70;
static constexpr double STRAIGHT_LOCK_SPEED_MPS_CONST = 5.50;

// Maximum trajectory points capacity
static constexpr int MAX_TRAJ_POINTS = 64;

// Costmap Probing Function Pointer Signature
typedef int (*CostmapProbeFn)(void *handle, const float *xs, const float *ys,
                              int count);

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
  void *costmap_handle;
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
  double v_cruise_max;
  double v_cruise_min;
  double straight_lock_dist_m;
  double straight_lock_speed_mps;
  double dt_actuator;

  int nx;
  int nu;
  int max_iterations;

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

  // Emergency Shield trajectory recording (2 branches from Step 2, 2 branches
  // from Step 3)
  bool shield_active;
  int shield_chosen_id; // -1 = None, 0 = k2_left, 1 = k2_right, 2 = k1_left, 3
                        // = k1_right

  int shield_k1_l_cnt;
  double shield_k1_l_x[MAX_TRAJ_POINTS];
  double shield_k1_l_y[MAX_TRAJ_POINTS];
  bool shield_k1_l_safe;

  int shield_k1_r_cnt;
  double shield_k1_r_x[MAX_TRAJ_POINTS];
  double shield_k1_r_y[MAX_TRAJ_POINTS];
  bool shield_k1_r_safe;

  int shield_k2_l_cnt;
  double shield_k2_l_x[MAX_TRAJ_POINTS];
  double shield_k2_l_y[MAX_TRAJ_POINTS];
  bool shield_k2_l_safe;

  int shield_k2_r_cnt;
  double shield_k2_r_x[MAX_TRAJ_POINTS];
  double shield_k2_r_y[MAX_TRAJ_POINTS];
  bool shield_k2_r_safe;

  // Straight-ahead max-decel probe (reference line for cascade gating)
  int shield_straight_cnt;
  double shield_straight_x[MAX_TRAJ_POINTS];
  double shield_straight_y[MAX_TRAJ_POINTS];
  bool shield_straight_safe;

  // Pure zero-steering straight rollout probe
  int shield_straight_zero_cnt;
  double shield_straight_zero_x[MAX_TRAJ_POINTS];
  double shield_straight_zero_y[MAX_TRAJ_POINTS];
  bool shield_straight_zero_safe;

  // Independent primal & dual warm-start cache per solver instance (prevents
  // cross-contamination in dual-solve)
  bool has_solution;
  double saved_x[MPC_N_VARS];
  double saved_y[MPC_N_CONSTR];

  CurvilinearSLMPCNative(void *cm_handle, CostmapProbeFn probe_fn,
                         int horizon_N = 30, double dt_val = 0.020,
                         double q_prog = 8.0, double q_lat = 18.0,
                         double q_head = 8.0, double r_st = 5.0,
                         double r_acc = 2.0, double r_str_rate = 10.0,
                         double mu = TIRE_MU_CONST, double v_corner = 0.82,
                         double v_max = V_MAX_CRUISE_CONST,
                         double v_min = V_MIN_CRUISE_CONST,
                         double lock_dist = STRAIGHT_LOCK_DIST_M_CONST,
                         double lock_spd = STRAIGHT_LOCK_SPEED_MPS_CONST)
      : costmap_handle(cm_handle), costmap_probe_func(probe_fn), N(horizon_N),
        dt(dt_val), q_progress(q_prog), q_cte(q_lat), q_heading(q_head),
        r_steer(r_st), r_accel(r_acc), r_steer_rate(r_str_rate),
        mu_friction(mu), v_corner_factor(v_corner), v_cruise_max(v_max),
        v_cruise_min(v_min), straight_lock_dist_m(lock_dist),
        straight_lock_speed_mps(lock_spd), dt_actuator(DT_PLANNER_CONST), nx(7),
        nu(2), max_iterations(35), best_line_count(0), guide_line_count(0),
        corridor_count(0), shield_active(false), shield_chosen_id(-1),
        shield_k1_l_cnt(0), shield_k1_l_safe(false), shield_k1_r_cnt(0),
        shield_k1_r_safe(false), shield_k2_l_cnt(0), shield_k2_l_safe(false),
        shield_k2_r_cnt(0), shield_k2_r_safe(false), shield_straight_cnt(0),
        shield_straight_safe(false), shield_straight_zero_cnt(0),
        shield_straight_zero_safe(false), has_solution(false) {
    // Copy constant structure of A_data
    std::memset(A_data, 0, sizeof(A_data));
    for (int k = 0; k < NUM_CONST_ENTRIES; ++k) {
      A_data[A_CONST_ENTRIES[k].p] = A_CONST_ENTRIES[k].val;
    }
    // Base P_data
    for (int k = 0; k < MPC_NNZ_P; ++k) {
      P_data[k] = P_base_data[k];
    }

    // P is diagonal. Layout (N = 30): per stage k = 0..N the state weights
    // [d, e_psi, vx, delta] at 4k..4k+3 (vx is overwritten every plan()),
    // then per stage k = 0..N-1 the input weights [Fx, v_delta] at
    // 4(N+1) + 2k..2k+1, then slacks. P_base_data was baked for the reference
    // weights below, so scale relative to them: reference weights reproduce
    // the original behaviour exactly.
    static_assert(MPC_NNZ_P >= 4 * 31 + 2 * 30, "unexpected P layout");
    constexpr double REF_Q_CTE = 18.0;
    constexpr double REF_Q_HEADING = 8.0;
    constexpr double REF_R_STEER = 5.0;
    constexpr double REF_R_STEER_RATE = 10.0;
    const double s_cte = std::max(q_cte, 0.0) / REF_Q_CTE;
    const double s_head = std::max(q_heading, 0.0) / REF_Q_HEADING;
    const double s_steer = std::max(r_steer, 0.0) / REF_R_STEER;
    const double s_rate = std::max(r_steer_rate, 0.0) / REF_R_STEER_RATE;
    for (int k = 0; k <= 30; ++k) {
      P_data[4 * k + 0] *= s_cte;
      P_data[4 * k + 1] *= s_head;
      P_data[4 * k + 3] *= s_steer;
    }
    for (int k = 0; k < 30; ++k) {
      P_data[4 * 31 + 2 * k + 1] *= s_rate;
    }

    // Balanced high-precision tolerances: matched to actuator resolution
    // (polishing strictly disabled per user request)
    if (solver.settings) {
      solver.settings->eps_abs = 0.003;
      solver.settings->eps_rel = 0.003;
      solver.settings->max_iter = 35;
      solver.settings->check_termination = 5;
      solver.settings->polishing = 0;
    }
  }

  // --------------------------------------------------------------------------
  // Probes Costmap laterally along normal vectors [d_min, d_max]
  // --------------------------------------------------------------------------
  void find_corridor_bounds(double gx, double gy, double nx_left,
                            double ny_left, double pref_side, double &out_d_min,
                            double &out_d_max) {
    // High-density lateral probing: d_vals from -0.65 to 0.655 step 0.015 (88
    // samples, 15 mm resolution)
    constexpr int N_PROBE = 88;
    float pxs[N_PROBE];
    float pys[N_PROBE];
    float d_vals[N_PROBE];

    for (int i = 0; i < N_PROBE; ++i) {
      float d = -0.65f + i * 0.015f;
      d_vals[i] = d;
      pxs[i] = static_cast<float>(gx + d * nx_left);
      pys[i] = static_cast<float>(gy + d * ny_left);
    }

    bool free_mask[N_PROBE];
    bool any_free = false;

    if (costmap_probe_func && costmap_handle) {
      for (int i = 0; i < N_PROBE; ++i) {
        free_mask[i] =
            (costmap_probe_func(costmap_handle, &pxs[i], &pys[i], 1) == 1);
        if (free_mask[i])
          any_free = true;
      }
    } else {
      // Default free when no costmap is attached
      for (int i = 0; i < N_PROBE; ++i)
        free_mask[i] = true;
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
    double min_dist = std::abs(
        0.5 * (valid_segs[0].d_min + valid_segs[0].d_max) - target_center);
    for (int i = 1; i < n_valid; ++i) {
      double d = std::abs(0.5 * (valid_segs[i].d_min + valid_segs[i].d_max) -
                          target_center);
      if (d < min_dist) {
        min_dist = d;
        best_idx = i;
      }
    }
    out_d_min = valid_segs[best_idx].d_min;
    out_d_max = valid_segs[best_idx].d_max;
  }

  // --------------------------------------------------------------------------
  // Solves 7x7 Linear System (I - 0.5*dt*Ac) * [Ad, Bd] = [I + 0.5*dt*Ac,
  // dt*Bc]
  // --------------------------------------------------------------------------
  void discretize_dynamics(double v_bar, double dt_val, double Ad[7][7],
                           double Bd[7][2]) {
    double Ac[7][7] = {0};
    Ac[0][3] = 1.0;
    Ac[1][2] = v_bar;
    Ac[1][4] = 1.0;
    Ac[2][5] = 1.0;

    double denom_v = std::max(v_bar, 0.20);
    Ac[4][4] = -(C_AF_CONST + C_AR_CONST) / (VEHICLE_MASS_CONST * denom_v);
    Ac[4][5] = -(LF_CONST * C_AF_CONST - LR_CONST * C_AR_CONST) /
                   (VEHICLE_MASS_CONST * denom_v) -
               v_bar;
    Ac[4][6] = C_AF_CONST / VEHICLE_MASS_CONST;

    Ac[5][4] =
        -(LF_CONST * C_AF_CONST - LR_CONST * C_AR_CONST) / (IZ_CONST * denom_v);
    Ac[5][5] =
        -(LF_CONST * LF_CONST * C_AF_CONST + LR_CONST * LR_CONST * C_AR_CONST) /
        (IZ_CONST * denom_v);
    Ac[5][6] = (LF_CONST * C_AF_CONST) / IZ_CONST;

    double Bc[7][2] = {0};
    Bc[3][0] = 1.0 / VEHICLE_MASS_CONST;
    Bc[6][1] = 1.0;

    // Form M_left = I - 0.5 * dt * Ac
    // Form M_right_Ad = I + 0.5 * dt * Ac
    // Form M_right_Bd = dt * Bc
    // Augment: [M_left | M_right_Ad | M_right_Bd] of size 7 x (7 + 7 + 2) = 7 x
    // 16
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
  static inline HermiteState eval_hermite(double u, double x0, double y0,
                                          double T0_x, double T0_y, double xg,
                                          double yg, double T1_x, double T1_y) {
    u = std::max(0.0, std::min(1.0, u));
    double u2 = u * u;
    double u3 = u2 * u;

    double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    double h10 = u3 - 2.0 * u2 + u;
    double h01 = -2.0 * u3 + 3.0 * u2;
    double h11 = u3 - u2;

    double gx = h00 * x0 + h10 * T0_x + h01 * xg + h11 * T1_x;
    double gy = h00 * y0 + h10 * T0_y + h01 * yg + h11 * T1_y;

    double dh00 = 6.0 * u2 - 6.0 * u;
    double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
    double dh01 = -6.0 * u2 + 6.0 * u;
    double dh11 = 3.0 * u2 - 2.0 * u;

    double dx = dh00 * x0 + dh10 * T0_x + dh01 * xg + dh11 * T1_x;
    double dy = dh00 * y0 + dh10 * T0_y + dh01 * yg + dh11 * T1_y;

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
  static inline double eval_hermite_curvature(double u, double x0, double y0,
                                              double T0_x, double T0_y,
                                              double xg, double yg, double T1_x,
                                              double T1_y) {
    u = std::max(0.0, std::min(1.0, u));
    double u2 = u * u;

    double dh00 = 6.0 * u2 - 6.0 * u;
    double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
    double dh01 = -6.0 * u2 + 6.0 * u;
    double dh11 = 3.0 * u2 - 2.0 * u;

    double dx = dh00 * x0 + dh10 * T0_x + dh01 * xg + dh11 * T1_x;
    double dy = dh00 * y0 + dh10 * T0_y + dh01 * yg + dh11 * T1_y;

    double d2h00 = 12.0 * u - 6.0;
    double d2h10 = 6.0 * u - 4.0;
    double d2h01 = -12.0 * u + 6.0;
    double d2h11 = 6.0 * u - 2.0;

    double ddx = d2h00 * x0 + d2h10 * T0_x + d2h01 * xg + d2h11 * T1_x;
    double ddy = d2h00 * y0 + d2h10 * T0_y + d2h01 * yg + d2h11 * T1_y;

    double speed_sq = dx * dx + dy * dy;
    double denom = std::pow(speed_sq, 1.5);
    if (denom < 1e-6)
      return 0.0;
    return std::abs(dx * ddy - dy * ddx) / denom;
  }

  // --------------------------------------------------------------------------
  // Emergency Shield: Evaluates evasion primitives against the costmap
  // --------------------------------------------------------------------------
  bool simulate_and_probe_branch(double x0, double y0, double v0, double psi0,
                                 double delta0, double target_steer,
                                 double v_cmd, double lookahead_time,
                                 int lookahead_steps, double *out_xs = nullptr,
                                 double *out_ys = nullptr,
                                 int *out_count = nullptr) {
    double dt_sim = lookahead_time / static_cast<double>(lookahead_steps);
    double x = x0;
    double y = y0;
    double psi = psi0;
    double v = std::max(0.0, v0);
    double delta = delta0;

    if (out_count)
      *out_count = 0;
    if (out_xs && out_ys && out_count) {
      out_xs[0] = x;
      out_ys[0] = y;
      *out_count = 1;
    }

    bool is_safe = true;
    for (int i = 0; i < lookahead_steps; ++i) {
      double max_delta_change = MAX_SERVO_RATE_RAD_S_CONST * dt_sim;
      if (target_steer > delta + max_delta_change)
        delta += max_delta_change;
      else if (target_steer < delta - max_delta_change)
        delta -= max_delta_change;
      else
        delta = target_steer;

      double lat_accel = (v * v) * std::abs(std::tan(delta)) / WHEELBASE_CONST;
      double lat_accel_max = mu_friction * GRAVITY_G_CONST;
      double avail_long_accel_sq =
          lat_accel_max * lat_accel_max - lat_accel * lat_accel;
      double max_ax = (avail_long_accel_sq > 0.0)
                          ? std::sqrt(avail_long_accel_sq)
                          : (0.45 * lat_accel_max);

      if (v_cmd < v) {
        v = std::max(v_cmd, v - max_ax * dt_sim);
      } else {
        v = std::min(v_cmd, v + max_ax * dt_sim);
      }

      // In robot frame: +Y forward, +X right, delta > 0 turns left (towards -X)
      double max_yaw_rate = (mu_friction * GRAVITY_G_CONST) / std::max(0.35, v);
      double d_psi = (v / WHEELBASE_CONST) * std::tan(delta);
      d_psi = std::max(-max_yaw_rate, std::min(max_yaw_rate, d_psi));
      double psi_mid = psi + 0.5 * d_psi * dt_sim;
      psi += d_psi * dt_sim;
      x -= v * std::sin(psi_mid) *
           dt_sim; // Leftward lateral displacement for psi > 0
      y += v * std::cos(psi_mid) * dt_sim; // Forward longitudinal displacement

      if (out_xs && out_ys && out_count && *out_count < MAX_TRAJ_POINTS) {
        out_xs[*out_count] = x;
        out_ys[*out_count] = y;
        (*out_count)++;
      }

      float px = static_cast<float>(x);
      float py = static_cast<float>(y);
      float px_f = static_cast<float>(x - 0.06 * std::sin(psi));
      float py_f = static_cast<float>(y + 0.06 * std::cos(psi));
      if (is_safe && costmap_probe_func && costmap_handle) {
        if (costmap_probe_func(costmap_handle, &px, &py, 1) == 0 ||
            costmap_probe_func(costmap_handle, &px_f, &py_f, 1) == 0) {
          is_safe = false; // Collision
        }
      }
    }
    return is_safe; // Safe
  }

  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------
  // Physical Trajectory & 4-Branch Escape Rollout (Step 1 Left/Right, Step 2
  // Left/Right)
  // --------------------------------------------------------------------------
  void run_shield_rollout_and_cascade(
      double &cmd_steer, double &cmd_speed, double xg, double v_bar,
      double delta_0, bool use_mpc_step2 = false, double mpc_x_2 = 0.0,
      double mpc_y_2 = 0.0, double mpc_psi_2 = 0.0, double mpc_v_2 = 0.0,
      double mpc_delta_2 = 0.0, double x0 = 0.0, double y0 = 0.0,
      double psi0 = 0.0) {
    double dt_sim_step = dt_actuator > 0.005 ? dt_actuator : (1.0 / 60.0);
    double max_delta_rate_step = MAX_SERVO_RATE_RAD_S_CONST * dt_sim_step;
    double max_a_acc = 0.85 * mu_friction * GRAVITY_G_CONST;
    double max_a_dec = 1.00 * mu_friction * GRAVITY_G_CONST;

    // Actual State at t = 0 (vehicle frame origin or projected anchor: x0, y0,
    // psi0)
    double act_x = x0;
    double act_y = y0;
    double act_psi = psi0;
    double act_v = std::max(0.0, v_bar);
    double act_delta = delta_0;

    // State at Step 2 (t = 2 * dt). On a successful MPC solve it comes straight
    // from the solution; otherwise propagate two steps of kinematics below.
    double act_x_2 = mpc_x_2;
    double act_y_2 = mpc_y_2;
    double act_psi_2 = mpc_psi_2;
    double act_v_2 = mpc_v_2;
    double act_delta_2 = mpc_delta_2;

    if (!use_mpc_step2) {
      // Advance to Step 1 (t = 1 * dt)
      double d_delta_0 =
          std::max(-max_delta_rate_step,
                   std::min(max_delta_rate_step, cmd_steer - act_delta));
      double act_delta_1 = act_delta + d_delta_0;
      double delta_mid_0 = act_delta + 0.5 * d_delta_0;

      double act_v_1 = act_v;
      if (cmd_speed < act_v)
        act_v_1 = std::max(cmd_speed, act_v - max_a_dec * dt_sim_step);
      else
        act_v_1 = std::min(cmd_speed, act_v + max_a_acc * dt_sim_step);
      double v_mid_0 = 0.5 * (act_v + act_v_1);

      double max_yaw_rate_0 =
          (mu_friction * GRAVITY_G_CONST) / std::max(0.35, v_mid_0);
      double d_psi_0 =
          (v_mid_0 / WHEELBASE_CONST) * std::tan(delta_mid_0) * dt_sim_step;
      d_psi_0 = std::max(-max_yaw_rate_0 * dt_sim_step,
                         std::min(max_yaw_rate_0 * dt_sim_step, d_psi_0));
      double psi_mid_0 = act_psi + 0.5 * d_psi_0;
      double act_psi_1 = act_psi + d_psi_0;

      // In vehicle local frame: +Y is forward, +X is right, delta > 0 turns
      // left (towards -X)
      double act_x_1 = act_x - v_mid_0 * std::sin(psi_mid_0) * dt_sim_step;
      double act_y_1 = act_y + v_mid_0 * std::cos(psi_mid_0) * dt_sim_step;

      // Advance to Step 2 (t = 2 * dt)
      double d_delta_1 =
          std::max(-max_delta_rate_step,
                   std::min(max_delta_rate_step, cmd_steer - act_delta_1));
      act_delta_2 = act_delta_1 + d_delta_1;
      double delta_mid_1 = act_delta_1 + 0.5 * d_delta_1;

      act_v_2 = act_v_1;
      if (cmd_speed < act_v_1)
        act_v_2 = std::max(cmd_speed, act_v_1 - max_a_dec * dt_sim_step);
      else
        act_v_2 = std::min(cmd_speed, act_v_1 + max_a_acc * dt_sim_step);
      double v_mid_1 = 0.5 * (act_v_1 + act_v_2);

      double max_yaw_rate_1 =
          (mu_friction * GRAVITY_G_CONST) / std::max(0.35, v_mid_1);
      double d_psi_1 =
          (v_mid_1 / WHEELBASE_CONST) * std::tan(delta_mid_1) * dt_sim_step;
      d_psi_1 = std::max(-max_yaw_rate_1 * dt_sim_step,
                         std::min(max_yaw_rate_1 * dt_sim_step, d_psi_1));
      double psi_mid_1 = act_psi_1 + 0.5 * d_psi_1;
      act_psi_2 = act_psi_1 + d_psi_1;

      act_x_2 = act_x_1 - v_mid_1 * std::sin(psi_mid_1) * dt_sim_step;
      act_y_2 = act_y_1 + v_mid_1 * std::cos(psi_mid_1) * dt_sim_step;
    }

    shield_active = false;
    shield_chosen_id = -1;
    shield_k1_l_cnt = 0;
    shield_k1_r_cnt = 0;
    shield_k1_l_safe = false;
    shield_k1_r_safe = false;

    shield_k2_l_cnt = 0;
    shield_k2_r_cnt = 0;
    shield_k2_l_safe = false;
    shield_k2_r_safe = false;
    shield_straight_cnt = 0;
    shield_straight_safe = false;
    shield_straight_zero_cnt = 0;
    shield_straight_zero_safe = false;

    double lookahead_time = 10.0 / 60.0; // 10 steps * 16.67ms = 166.7ms
    int lookahead_steps = 10;
    double straight_lookahead_time = 12.0 / 60.0; // 12 steps (200.0ms)
    double escape_speed = 1.223;                  // Safe escape velocity

    // 0A. Pure Zero-Steering Straight Line:
    // Starts with 0.0 steering, heading 0.0, at vehicle origin (0, 0).
    // An actually straight line projecting directly forward along the vehicle
    // longitudinal axis (+Y).
    shield_straight_zero_safe = simulate_and_probe_branch(
        x0, y0, act_v, psi0, delta_0, // x0, y0, psi0, delta0
        0.0, // target_steer = 0.0 (stays with 0 steer!)
        std::max(v_cruise_min,
                 act_v - 0.70 * max_a_dec * straight_lookahead_time),
        straight_lookahead_time, 12, shield_straight_zero_x,
        shield_straight_zero_y, &shield_straight_zero_cnt);

    // 0B. MPC Course Follow Probe (target_steer = cmd_steer):
    // Actively steers along planned MPC course toward cmd_steer, evaluating
    // forward path safety.
    shield_straight_safe = simulate_and_probe_branch(
        act_x_2, act_y_2, act_v_2, act_psi_2, act_delta_2,
        cmd_steer, // actively steer along planned course toward cmd_steer
        std::max(v_cruise_min,
                 act_v_2 - 0.70 * max_a_dec * straight_lookahead_time),
        straight_lookahead_time, 12, shield_straight_x, shield_straight_y,
        &shield_straight_cnt);

    // Near-horizon obstacle check along MPC trajectory points (imminent
    // collision check only)
    if (use_mpc_step2 && shield_straight_safe && best_line_count > 2) {
      int probe_steps = std::min(6, best_line_count - 1 - 2);
      for (int i = 0; i <= probe_steps; ++i) {
        float px = static_cast<float>(best_line_x[2 + i]);
        float py = static_cast<float>(best_line_y[2 + i]);
        if (costmap_probe_func && costmap_handle) {
          if (costmap_probe_func(costmap_handle, &px, &py, 1) == 0) {
            shield_straight_safe = false;
            break;
          }
        }
      }
    }

    // 1. Step 0 (k=0) Evasion Micro-Rollouts from Current State (immediate)
    shield_k1_l_safe = simulate_and_probe_branch(
        act_x, act_y, act_v, act_psi, act_delta, +MAX_STEERING_RAD_CONST,
        escape_speed, lookahead_time, lookahead_steps, shield_k1_l_x,
        shield_k1_l_y, &shield_k1_l_cnt);
    shield_k1_r_safe = simulate_and_probe_branch(
        act_x, act_y, act_v, act_psi, act_delta, -MAX_STEERING_RAD_CONST,
        escape_speed, lookahead_time, lookahead_steps, shield_k1_r_x,
        shield_k1_r_y, &shield_k1_r_cnt);

    // 2. Step 2 (k=2) Evasion Micro-Rollouts from Actual State 2
    shield_k2_l_safe = simulate_and_probe_branch(
        act_x_2, act_y_2, act_v_2, act_psi_2, act_delta_2,
        +MAX_STEERING_RAD_CONST, escape_speed, lookahead_time, lookahead_steps,
        shield_k2_l_x, shield_k2_l_y, &shield_k2_l_cnt);
    shield_k2_r_safe = simulate_and_probe_branch(
        act_x_2, act_y_2, act_v_2, act_psi_2, act_delta_2,
        -MAX_STEERING_RAD_CONST, escape_speed, lookahead_time, lookahead_steps,
        shield_k2_r_x, shield_k2_r_y, &shield_k2_r_cnt);

    // 3. Safety Shield Cascade:
    // Shield must only trigger when BOTH a side AND the straight line are
    // blocked! If straight is clear -> hold course (MPC path is safe). If both
    // sides are clear -> do not override MPC (open space on both sides).
    double gentle_speed = std::max(v_cruise_min, act_v * 0.85);
    double emergency_speed = std::max(v_cruise_min, act_v * 0.70);

    if (!shield_straight_safe) {
      if (shield_k2_l_safe && !shield_k2_r_safe) {
        // Step 2 Left clear, Right blocked AND straight blocked -> swerve Left
        shield_active = true;
        cmd_steer = +MAX_STEERING_RAD_CONST;
        shield_chosen_id = 0; // k2_left
        cmd_speed = gentle_speed;
      } else if (!shield_k2_l_safe && shield_k2_r_safe) {
        // Step 2 Right clear, Left blocked AND straight blocked -> swerve Right
        shield_active = true;
        cmd_steer = -MAX_STEERING_RAD_CONST;
        shield_chosen_id = 1; // k2_right
        cmd_speed = gentle_speed;
      } else if (!shield_k2_l_safe && !shield_k2_r_safe) {
        // Both Step 2 swerves blocked AND straight blocked -> fall back to Step
        // 0 immediate branch
        if (shield_k1_l_safe && !shield_k1_r_safe) {
          shield_active = true;
          cmd_steer = +MAX_STEERING_RAD_CONST;
          shield_chosen_id = 2; // k1_left
          cmd_speed = emergency_speed;
        } else if (!shield_k1_l_safe && shield_k1_r_safe) {
          shield_active = true;
          cmd_steer = -MAX_STEERING_RAD_CONST;
          shield_chosen_id = 3; // k1_right
          cmd_speed = emergency_speed;
        } else if (shield_k1_l_safe && shield_k1_r_safe) {
          shield_active = true;
          cmd_steer = ((xg - x0) < 0.0) ? +MAX_STEERING_RAD_CONST
                                        : -MAX_STEERING_RAD_CONST;
          shield_chosen_id = ((xg - x0) < 0.0) ? 2 : 3; // k1_left or k1_right
          cmd_speed = emergency_speed;
        } else {
          // All forward evasion paths exhausted -> recommend reverse
          shield_active = true;
          shield_chosen_id = 4; // ALL_BLOCKED_REVERSE
          cmd_speed = 0.0;
        }
      }
      // If shield_k2_l_safe && shield_k2_r_safe:
      // Both sides are open! Shield does NOT trigger - MPC retains control.
    }
  }

  // --------------------------------------------------------------------------
  // MAIN OPTION 2 SL-MPC PLAN FUNCTION
  // --------------------------------------------------------------------------
  int plan(double xg, double yg, double theta_g, double /*aperture_width*/,
           double v_current, double steer_current, double yaw_rate_current,
           double *out_cmd_steer_rad, double *out_cmd_speed_mps,
           double *out_best_cost, int *out_osqp_iters, int *out_status_code,
           double x0 = 0.0, double y0 = 0.0, double psi0 = 0.0) {
    double v_bar = std::max(0.0, v_current);
    double delta_0 = steer_current;
    double r_0 = yaw_rate_current;

    // 0. Fast Straight-Line Lock (Zero Compute Bypass when Corridor is Open
    // Ahead >= 1.70m)
    if ((yg - y0) >= straight_lock_dist_m && std::abs(xg - x0) <= 0.20 &&
        std::abs(theta_g - psi0) <= 0.22) {
      // Verify front blindspot (0-40cm) is clear of obstacles before engaging
      // straight lock
      bool straight_clear = true;
      float cos_psi = -static_cast<float>(std::sin(psi0));
      float sin_psi = static_cast<float>(std::cos(psi0));
      if (costmap_probe_func && costmap_handle) {
        for (double py_check = 0.05; py_check <= 0.40; py_check += 0.05) {
          float px = static_cast<float>(x0 + py_check * cos_psi);
          float py = static_cast<float>(y0 + py_check * sin_psi);
          if (costmap_probe_func(costmap_handle, &px, &py, 1) == 0) {
            straight_clear = false;
            break;
          }
        }
      }

      if (straight_clear) {
        double cmd_steer = 0.0;
        double cmd_speed = straight_lock_speed_mps;
        run_shield_rollout_and_cascade(cmd_steer, cmd_speed, xg, v_bar, delta_0,
                                       false, 0, 0, 0, 0, 0, x0, y0, psi0);

        *out_cmd_steer_rad = cmd_steer;
        *out_cmd_speed_mps = cmd_speed;
        *out_best_cost = 0.0;
        *out_osqp_iters = 0;
        *out_status_code = 1; // STRAIGHT_LOCKED

        best_line_count = N + 1;
        for (int k = 0; k <= N; ++k) {
          double u = static_cast<double>(k) / N;
          double d_proj = 0.05 + u * (straight_lock_dist_m - 0.05);
          best_line_x[k] = x0 + d_proj * cos_psi;
          best_line_y[k] = y0 + d_proj * sin_psi;
        }
        guide_line_count = 0;
        corridor_count = 0;
        return 1;
      }
    }

    // 1. Establish Curvilinear Reference Spline towards gap centroid
    double L_guide = std::hypot(xg - x0, std::max(0.35, yg - y0));
    double v_nom = std::max(v_bar, 1.20);
    double dt_val = this->dt;

    // Tangent vector at start:
    // Aligned with the vehicle's projected heading (psi0):
    double T0_x = -L_guide * std::sin(psi0);
    double T0_y = L_guide * std::cos(psi0);

    // Tangent vector at target gap opening:
    double T1_x = xg - x0;
    double T1_y = std::max(0.35, yg - y0);

    // Corner braking ceiling from target gap curvature and local spline
    // curvature
    double dist_gap_sq = (xg - x0) * (xg - x0) + (yg - y0) * (yg - y0);
    double kappa_turn = 2.0 * std::abs(xg - x0) / std::max(dist_gap_sq, 0.04);
    double lat_limit = mu_friction * GRAVITY_G_CONST * v_corner_factor;
    double a_decel = 1.00 * mu_friction * GRAVITY_G_CONST;

    // Preferred bypass side
    double pref_side =
        ((xg - x0) < 0.0) ? 1.0 : (((xg - x0) > 0.0) ? -1.0 : 0.0);

    // Sample corridor bounds for each MPC horizon step k with continuous
    // sub-stepping
    double d_min_bounds[N + 1];
    double d_max_bounds[N + 1];
    HermiteState step_states[N + 1];

    // Front-bumper clearance margin based on gap angle
    double angle_to_gap = std::atan2(xg - x0, std::max(0.35, yg - y0));
    double yaw_clearance_margin =
        LF_CONST * std::abs(std::sin(angle_to_gap - psi0));

    for (int k = 0; k <= N; ++k) {
      double sk = std::min(L_guide, k * v_nom * dt_val);
      double uk = sk / std::max(L_guide, 0.01);
      step_states[k] = eval_hermite(uk, x0, y0, T0_x, T0_y, xg, yg, T1_x, T1_y);

      double d_min_k, d_max_k;
      find_corridor_bounds(step_states[k].gx, step_states[k].gy,
                           step_states[k].nx, step_states[k].ny, pref_side,
                           d_min_k, d_max_k);

      // Sub-step continuous collision envelope: focused on immediate near-field
      // horizon (k <= 6)
      if (k >= 1 && k <= 6) {
        double sk_prev = std::min(L_guide, (k - 1) * v_nom * dt_val);
        double u_mid = (0.5 * (sk_prev + sk)) / std::max(L_guide, 0.01);
        HermiteState s_mid =
            eval_hermite(u_mid, x0, y0, T0_x, T0_y, xg, yg, T1_x, T1_y);
        double d_min_mid, d_max_mid;
        find_corridor_bounds(s_mid.gx, s_mid.gy, s_mid.nx, s_mid.ny, pref_side,
                             d_min_mid, d_max_mid);
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

    // Compute local speed ceiling at each stage k:
    // Any dot falling inside an obstacle / C-space is assigned v_cruise_min
    double v_ref_k[N + 1];
    bool blocked_ahead = false;
    for (int k = 0; k <= N; ++k) {
      double sk = std::min(L_guide, k * v_nom * dt_val);
      double uk = sk / std::max(L_guide, 0.01);

      // Probe whether step dot k is inside obstacle / C-space
      if (!blocked_ahead && costmap_probe_func && costmap_handle) {
        float px = static_cast<float>(step_states[k].gx);
        float py = static_cast<float>(step_states[k].gy);
        if (costmap_probe_func(costmap_handle, &px, &py, 1) == 0) {
          blocked_ahead = true;
        }
      }

      double kappa_k =
          std::max(kappa_turn, eval_hermite_curvature(uk, x0, y0, T0_x, T0_y,
                                                      xg, yg, T1_x, T1_y));
      double v_corner_k = std::sqrt(lat_limit / std::max(kappa_k, 0.08));

      if (blocked_ahead) {
        v_ref_k[k] = v_cruise_min;
      } else {
        v_ref_k[k] = std::min(v_cruise_max, std::max(v_cruise_min, v_corner_k));
      }
    }

    // Backward dynamic braking pass (kinematic reachability)
    for (int k = N - 1; k >= 0; --k) {
      double ds_step = v_nom * dt_val;
      double v_brake_limit =
          std::sqrt(v_ref_k[k + 1] * v_ref_k[k + 1] + 2.0 * a_decel * ds_step);
      v_ref_k[k] = std::min(v_ref_k[k], v_brake_limit);
      v_ref_k[k] = std::max(
          v_ref_k[k], v_cruise_min); // Never go below minimum cruise speed
    }
    double v_ref = v_ref_k[0];

    // Zero-cost corridor & guide line populated directly from MPC horizon
    // samples
    corridor_count = N + 1;
    guide_line_count = N + 1;
    for (int k = 0; k <= N; ++k) {
      guide_line_x[k] = step_states[k].gx;
      guide_line_y[k] = step_states[k].gy;
      corridor_left_x[k] =
          step_states[k].gx + d_max_bounds[k] * step_states[k].nx;
      corridor_left_y[k] =
          step_states[k].gy + d_max_bounds[k] * step_states[k].ny;
      corridor_right_x[k] =
          step_states[k].gx + d_min_bounds[k] * step_states[k].nx;
      corridor_right_y[k] =
          step_states[k].gy + d_min_bounds[k] * step_states[k].ny;
    }

    // 3. Discretize dynamics
    double Ad[7][7];
    double Bd[7][2];
    discretize_dynamics(v_bar, dt_val, Ad, Bd);

    // Update A_data with Ad and Bd entries
    for (int idx = 0; idx < NUM_AD_ENTRIES; ++idx) {
      const auto &e = A_AD_ENTRIES[idx];
      A_data[e.p] = -Ad[e.i][e.j];
    }
    for (int idx = 0; idx < NUM_BD_ENTRIES; ++idx) {
      const auto &e = A_BD_ENTRIES[idx];
      A_data[e.p] = -Bd[e.i][e.j];
    }

    // Friction envelope limits
    double lat_accel_max = 0.95 * mu_friction * GRAVITY_G_CONST;
    double max_Fx = mu_friction * VEHICLE_MASS_CONST * GRAVITY_G_CONST;
    double c1 = MPC_FRICTION_OCTAGON_C1_CONST;
    double s_fx = 1.0 / std::max(max_Fx, 1e-3);

    // Update Diamond entries in A_data
    for (int idx = 0; idx < NUM_DIAMOND_FX_ENTRIES; ++idx) {
      const auto &e = A_DIAMOND_FX_ENTRIES[idx];
      A_data[e.p] = e.sx * s_fx;
    }
    for (int idx = 0; idx < NUM_DIAMOND_DELTA_ENTRIES; ++idx) {
      const auto &e = A_DIAMOND_DELTA_ENTRIES[idx];
      double vk_nom = std::max(v_ref_k[e.k], v_bar - e.k * a_decel * dt_val);
      double s_fy_k = c1 * (vk_nom * vk_nom) /
                      std::max(0.10, lat_accel_max * WHEELBASE_CONST);
      A_data[e.p] = e.sy * s_fy_k;
    }

    // Setup linear cost q_vec
    std::memset(q_vec, 0, sizeof(q_vec));
    double r_acc = r_accel;
    constexpr int VX_P_INDICES[31] = {
        2,  6,  10, 14, 18, 22, 26, 30, 34, 38,  42,  46,  50,  54,  58, 62,
        66, 70, 74, 78, 82, 86, 90, 94, 98, 102, 106, 110, 114, 118, 122};

    for (int k = 0; k <= N; ++k) {
      int ix = k * nx;
      double vk_target = v_ref_k[k];
      double w_v_k = (vk_target < 2.2) ? (r_acc * 3.0) : r_acc;
      q_vec[ix + 3] += -w_v_k * vk_target;
      P_data[VX_P_INDICES[k]] = w_v_k;
      int ie_c = (N + 1) * nx + N * nu + k;
      q_vec[ie_c] += MPC_WEIGHT_SLACK_EPS_CONST;
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

    // Initial state: heading error towards gap target drives the QP to steer
    // into the turn
    double e_psi_0 = angle_to_gap - psi0;
    while (e_psi_0 > PI_CONST)
      e_psi_0 -= 2.0 * PI_CONST;
    while (e_psi_0 < -PI_CONST)
      e_psi_0 += 2.0 * PI_CONST;
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
      double delta_k_max =
          std::max(8.0 * PI_CONST / 180.0,
                   std::min(MAX_STEERING_RAD_CONST,
                            std::atan((lat_accel_max * WHEELBASE_CONST) /
                                      std::max(0.20, vk_nom * vk_nom))));
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
    osqp_update_data_mat(&solver, P_data, OSQP_NULL, MPC_NNZ_P, A_data,
                         OSQP_NULL, MPC_NNZ_A);
    osqp_update_data_vec(&solver, q_vec, l_arr, u_arr);

    // Load instance-specific warm start to prevent dual-solve
    // cross-contamination
    if (has_solution) {
      osqp_warm_start(&solver, saved_x, saved_y);
    } else {
      osqp_cold_start(&solver);
    }

    if (solver.settings) {
      solver.settings->max_iter = max_iterations;
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

      // Extract best predicted line via forward kinematic bicycle rollout of
      // optimal solution
      best_line_count = N + 1;
      best_line_x[0] = x0;
      best_line_y[0] = y0;
      double cur_x = x0;
      double cur_y = y0;
      double cur_psi = psi0;
      double mpc_psi_2 = psi0;

      for (int k = 0; k < N; ++k) {
        double vk = std::max(0.10, solver.solution->x[k * nx + 3]);
        double deltak = solver.solution->x[k * nx + 6];
        double d_psi = (vk / WHEELBASE_CONST) * std::tan(deltak);
        double max_yaw_rate =
            (mu_friction * GRAVITY_G_CONST) / std::max(0.35, vk);
        d_psi = std::max(-max_yaw_rate, std::min(max_yaw_rate, d_psi));

        double psi_mid = cur_psi + 0.5 * d_psi * dt_val;
        cur_psi += d_psi * dt_val;
        cur_x -= vk * std::sin(psi_mid) * dt_val;
        cur_y += vk * std::cos(psi_mid) * dt_val;

        best_line_x[k + 1] = cur_x;
        best_line_y[k + 1] = cur_y;

        if (k == 1) {
          // Exactly at Step 2 (end of step 1 rollout):
          // Record true heading of the vehicle along best_line
          mpc_psi_2 = cur_psi;
        }
      }

      // Optimal steering output
      double v_delta_0 = solver.solution->x[(N + 1) * nx + 1];
      double delta_opt = delta_0 + v_delta_0 * dt_actuator;
      double cmd_steer = std::max(-MAX_STEERING_RAD_CONST,
                                  std::min(MAX_STEERING_RAD_CONST, delta_opt));
      *out_cmd_steer_rad = cmd_steer;

      // Optimal forward speed command
      double fx_0 = solver.solution->x[(N + 1) * nx + 0];
      double cmd_speed;
      if (fx_0 < -0.15) {
        cmd_speed = std::max(
            v_cruise_min,
            std::min(v_cruise_max,
                     v_bar + (fx_0 / VEHICLE_MASS_CONST) * dt_actuator));
      } else {
        double max_accel = 0.85 * mu_friction * GRAVITY_G_CONST;
        cmd_speed = std::max(
            v_cruise_min,
            std::min(v_cruise_max,
                     std::min(v_ref, v_bar + max_accel * dt_actuator * 2.0)));
      }

      double mpc_x_2 = best_line_x[2];
      double mpc_y_2 = best_line_y[2];
      double mpc_v_2 = solver.solution->x[2 * nx + 3];
      double mpc_delta_2 = solver.solution->x[2 * nx + 6];

      run_shield_rollout_and_cascade(cmd_steer, cmd_speed, xg, v_bar, delta_0,
                                     true, mpc_x_2, mpc_y_2, mpc_psi_2, mpc_v_2,
                                     mpc_delta_2, x0, y0, psi0);

      *out_cmd_steer_rad = cmd_steer;
      *out_cmd_speed_mps = cmd_speed;
      return 1;
    }

    // Fallback on solver error
    *out_status_code = -1; // FALLBACK_SL_MPC
    double fb_steer = std::max(
        -MAX_STEERING_RAD_CONST,
        std::min(MAX_STEERING_RAD_CONST,
                 -std::atan2(xg - x0, std::max(0.40, yg - y0)) + psi0));
    double fb_speed = v_cruise_min;
    run_shield_rollout_and_cascade(fb_steer, fb_speed, xg, v_bar, delta_0,
                                   false, 0, 0, 0, 0, 0, x0, y0, psi0);

    *out_cmd_steer_rad = fb_steer;
    *out_cmd_speed_mps = fb_speed;

    best_line_count = 16;
    for (int i = 0; i < 16; ++i) {
      double u = static_cast<double>(i) / 15.0;
      HermiteState s = eval_hermite(u, x0, y0, T0_x, T0_y, xg, yg, T1_x, T1_y);
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

SL_MPC_API void *sl_mpc_create(void *costmap_handle, CostmapProbeFn probe_fn,
                               int horizon_N, double dt, double q_progress,
                               double q_cte, double q_heading, double r_steer,
                               double r_accel, double r_steer_rate,
                               double mu_friction, double v_corner_factor,
                               double v_cruise_max, double v_cruise_min,
                               double straight_lock_dist_m,
                               double straight_lock_speed_mps) {
  return new CurvilinearSLMPCNative(
      costmap_handle, probe_fn, horizon_N, dt, q_progress, q_cte, q_heading,
      r_steer, r_accel, r_steer_rate, mu_friction, v_corner_factor,
      v_cruise_max, v_cruise_min, straight_lock_dist_m,
      straight_lock_speed_mps);
}

SL_MPC_API void sl_mpc_destroy(void *handle) {
  if (handle) {
    delete static_cast<CurvilinearSLMPCNative *>(handle);
  }
}

SL_MPC_API void sl_mpc_set_costmap(void *handle, void *cm_handle,
                                   CostmapProbeFn probe_fn) {
  if (handle) {
    auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
    planner->costmap_handle = cm_handle;
    planner->costmap_probe_func = probe_fn;
  }
}

SL_MPC_API void sl_mpc_set_mu(void *handle, double mu) {
  if (handle) {
    auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
    planner->mu_friction = mu;
  }
}

SL_MPC_API void sl_mpc_reset(void *handle) {
  if (handle) {
    auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
    planner->has_solution = false;
  }
}

SL_MPC_API void sl_mpc_set_max_iter(void *handle, int max_iter) {
  if (handle) {
    auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
    planner->max_iterations = max_iter;
  }
}

SL_MPC_API int sl_mpc_plan(void *handle, double xg, double yg, double theta_g,
                           double aperture_width, double v_current,
                           double steer_current, double yaw_rate_current,
                           double *out_cmd_steer_rad, double *out_cmd_speed_mps,
                           double *out_best_cost, int *out_osqp_iters,
                           int *out_status_code) {
  if (!handle)
    return 0;
  auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
  return planner->plan(xg, yg, theta_g, aperture_width, v_current,
                       steer_current, yaw_rate_current, out_cmd_steer_rad,
                       out_cmd_speed_mps, out_best_cost, out_osqp_iters,
                       out_status_code, 0.0, 0.0, 0.0);
}

SL_MPC_API int
sl_mpc_plan_projected(void *handle, double xg, double yg, double theta_g,
                      double aperture_width, double v_current,
                      double steer_current, double yaw_rate_current, double x0,
                      double y0, double psi0, double *out_cmd_steer_rad,
                      double *out_cmd_speed_mps, double *out_best_cost,
                      int *out_osqp_iters, int *out_status_code) {
  if (!handle)
    return 0;
  auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
  return planner->plan(xg, yg, theta_g, aperture_width, v_current,
                       steer_current, yaw_rate_current, out_cmd_steer_rad,
                       out_cmd_speed_mps, out_best_cost, out_osqp_iters,
                       out_status_code, x0, y0, psi0);
}

SL_MPC_API int sl_mpc_get_best_line(void *handle, double *out_xs,
                                    double *out_ys, int max_points) {
  if (!handle)
    return 0;
  auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
  int count = std::min(max_points, planner->best_line_count);
  for (int i = 0; i < count; ++i) {
    out_xs[i] = planner->best_line_x[i];
    out_ys[i] = planner->best_line_y[i];
  }
  return count;
}

SL_MPC_API int sl_mpc_get_guide_line(void *handle, double *out_xs,
                                     double *out_ys, int max_points) {
  if (!handle)
    return 0;
  auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
  int count = std::min(max_points, planner->guide_line_count);
  for (int i = 0; i < count; ++i) {
    out_xs[i] = planner->guide_line_x[i];
    out_ys[i] = planner->guide_line_y[i];
  }
  return count;
}

SL_MPC_API int sl_mpc_get_corridor(void *handle, double *out_left_xs,
                                   double *out_left_ys, double *out_right_xs,
                                   double *out_right_ys, int max_points) {
  if (!handle)
    return 0;
  auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);
  int count = std::min(max_points, planner->corridor_count);
  for (int i = 0; i < count; ++i) {
    out_left_xs[i] = planner->corridor_left_x[i];
    out_left_ys[i] = planner->corridor_left_y[i];
    out_right_xs[i] = planner->corridor_right_x[i];
    out_right_ys[i] = planner->corridor_right_y[i];
  }
  return count;
}

SL_MPC_API int sl_mpc_get_shield_branches(
    void *handle, double *out_k1_l_xs, double *out_k1_l_ys, int *out_k1_l_cnt,
    int *out_k1_l_safe, double *out_k1_r_xs, double *out_k1_r_ys,
    int *out_k1_r_cnt, int *out_k1_r_safe, double *out_k2_l_xs,
    double *out_k2_l_ys, int *out_k2_l_cnt, int *out_k2_l_safe,
    double *out_k2_r_xs, double *out_k2_r_ys, int *out_k2_r_cnt,
    int *out_k2_r_safe, double *out_str_xs, double *out_str_ys,
    int *out_str_cnt, int *out_str_safe, double *out_str_zero_xs,
    double *out_str_zero_ys, int *out_str_zero_cnt, int *out_str_zero_safe,
    int *out_active, int *out_chosen_id) {
  if (!handle)
    return 0;
  auto *planner = static_cast<CurvilinearSLMPCNative *>(handle);

  // K1 Left
  if (out_k1_l_xs && out_k1_l_ys && out_k1_l_cnt) {
    *out_k1_l_cnt = std::min(MAX_TRAJ_POINTS, planner->shield_k1_l_cnt);
    for (int i = 0; i < *out_k1_l_cnt; ++i) {
      out_k1_l_xs[i] = planner->shield_k1_l_x[i];
      out_k1_l_ys[i] = planner->shield_k1_l_y[i];
    }
  }
  if (out_k1_l_safe)
    *out_k1_l_safe = planner->shield_k1_l_safe ? 1 : 0;

  // K1 Right
  if (out_k1_r_xs && out_k1_r_ys && out_k1_r_cnt) {
    *out_k1_r_cnt = std::min(MAX_TRAJ_POINTS, planner->shield_k1_r_cnt);
    for (int i = 0; i < *out_k1_r_cnt; ++i) {
      out_k1_r_xs[i] = planner->shield_k1_r_x[i];
      out_k1_r_ys[i] = planner->shield_k1_r_y[i];
    }
  }
  if (out_k1_r_safe)
    *out_k1_r_safe = planner->shield_k1_r_safe ? 1 : 0;

  // K2 Left
  if (out_k2_l_xs && out_k2_l_ys && out_k2_l_cnt) {
    *out_k2_l_cnt = std::min(MAX_TRAJ_POINTS, planner->shield_k2_l_cnt);
    for (int i = 0; i < *out_k2_l_cnt; ++i) {
      out_k2_l_xs[i] = planner->shield_k2_l_x[i];
      out_k2_l_ys[i] = planner->shield_k2_l_y[i];
    }
  }
  if (out_k2_l_safe)
    *out_k2_l_safe = planner->shield_k2_l_safe ? 1 : 0;

  // K2 Right
  if (out_k2_r_xs && out_k2_r_ys && out_k2_r_cnt) {
    *out_k2_r_cnt = std::min(MAX_TRAJ_POINTS, planner->shield_k2_r_cnt);
    for (int i = 0; i < *out_k2_r_cnt; ++i) {
      out_k2_r_xs[i] = planner->shield_k2_r_x[i];
      out_k2_r_ys[i] = planner->shield_k2_r_y[i];
    }
  }
  if (out_k2_r_safe)
    *out_k2_r_safe = planner->shield_k2_r_safe ? 1 : 0;

  // Straight (MPC course probe)
  if (out_str_xs && out_str_ys && out_str_cnt) {
    *out_str_cnt = std::min(MAX_TRAJ_POINTS, planner->shield_straight_cnt);
    for (int i = 0; i < *out_str_cnt; ++i) {
      out_str_xs[i] = planner->shield_straight_x[i];
      out_str_ys[i] = planner->shield_straight_y[i];
    }
  }
  if (out_str_safe)
    *out_str_safe = planner->shield_straight_safe ? 1 : 0;

  // Straight Zero (0-steering straight rollout)
  if (out_str_zero_xs && out_str_zero_ys && out_str_zero_cnt) {
    *out_str_zero_cnt =
        std::min(MAX_TRAJ_POINTS, planner->shield_straight_zero_cnt);
    for (int i = 0; i < *out_str_zero_cnt; ++i) {
      out_str_zero_xs[i] = planner->shield_straight_zero_x[i];
      out_str_zero_ys[i] = planner->shield_straight_zero_y[i];
    }
  }
  if (out_str_zero_safe)
    *out_str_zero_safe = planner->shield_straight_zero_safe ? 1 : 0;

  if (out_active)
    *out_active = planner->shield_active ? 1 : 0;
  if (out_chosen_id)
    *out_chosen_id = planner->shield_chosen_id;

  return planner->shield_active ? 1 : 0;
}

} // extern "C"
