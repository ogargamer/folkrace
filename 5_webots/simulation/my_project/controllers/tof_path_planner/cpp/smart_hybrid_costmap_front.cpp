#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>


#if defined(_WIN32) || defined(__CYGWIN__)
#define COSTMAP_API __declspec(dllexport)
#else
#define COSTMAP_API __attribute__((visibility("default")))
#endif

// ==============================================================================
// 16-CHANNEL FRONT-ONLY TOF SENSOR RIG DEFINITION (from only_front_tofs.proto)
// Robot Body Frame: +Y is Forward, +X is Right, -X is Left, +Z is Up
// ==============================================================================
struct ToFSensor {
  const char *name;
  float x0;
  float y0;
  float heading;
  float fov_rad;
  float min_range;
  float max_range;
};

static const ToFSensor SENSOR_RIG[16] = {
    // Front-Left Array (cross-pointing Right, rotated 3 deg outwards)
    // Leftmost ToF (pointing straight and right, zones 1.1 deg to 34.9 deg)
    {"tof_front_left_4.1deg", -0.058f, 0.092f, 1.5512f, 0.1963f, 0.001f, 2.40f},
    {"tof_front_left_15.4deg", -0.058f, 0.092f, 1.3548f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_left_26.6deg", -0.058f, 0.092f, 1.1585f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_left_37.9deg", -0.058f, 0.092f, 0.9621f, 0.1963f, 0.001f,
     2.40f},
    // Left ToF (pointing right, zones 46.1 deg to 79.9 deg)
    {"tof_front_left_49.1deg", -0.057f, 0.090f, 0.7658f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_left_60.4deg", -0.057f, 0.090f, 0.5694f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_left_71.6deg", -0.057f, 0.090f, 0.3731f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_left_82.9deg", -0.057f, 0.090f, 0.1767f, 0.1963f, 0.001f,
     2.40f},

    // Front-Right Array (cross-pointing Left, mirrored along Y axis, rotated 3
    // deg outwards)
    // Rightmost ToF (pointing straight and left, zones 1.1 deg to 34.9 deg)
    {"tof_front_right_4.1deg", 0.058f, 0.092f, 1.5904f, 0.1963f, 0.001f, 2.40f},
    {"tof_front_right_15.4deg", 0.058f, 0.092f, 1.7868f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_right_26.6deg", 0.058f, 0.092f, 1.9831f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_right_37.9deg", 0.058f, 0.092f, 2.1795f, 0.1963f, 0.001f,
     2.40f},
    // Right ToF (pointing left, zones 46.1 deg to 79.9 deg)
    {"tof_front_right_49.1deg", 0.057f, 0.090f, 2.3758f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_right_60.4deg", 0.057f, 0.090f, 2.5722f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_right_71.6deg", 0.057f, 0.090f, 2.7685f, 0.1963f, 0.001f,
     2.40f},
    {"tof_front_right_82.9deg", 0.057f, 0.090f, 2.9649f, 0.1963f, 0.001f,
     2.40f},
};

static inline float wrap_angle(float a) {
  constexpr float TWO_PI = 2.0f * 3.14159265358979323846f;
  constexpr float PI = 3.14159265358979323846f;
  a = std::fmod(a + PI, TWO_PI);
  if (a < 0.0f)
    a += TWO_PI;
  return a - PI;
}

// Precomputed static polar cell within a sensor's cone
struct ConeCell {
  uint32_t cell_idx;
  float r;            // Geometric distance to sensor origin
  float hit_log_odds; // Precomputed continuous Gaussian angular decay weight
};

// ==============================================================================
// EXACT CONTINUOUS SMART HYBRID COSTMAP ENGINE (FRONT-ONLY RIG)
// ==============================================================================
class SmartHybridCostmapFront {
public:
  float x_min, x_max, y_min, y_max;
  float res;
  float inv_res;
  float scout_step_ds;
  int width, height, total_cells;
  float r_inflation_m;
  int inflation_cells;
  int persistence_frames = 0;
  int fix_yaw_sign = 1;
  int use_true_aperture = 1;
  float w_gap_center = 0.832f;

  // Flat preallocated buffers (zero runtime heap allocations)
  std::vector<float> log_odds;
  std::vector<float> prob_map;
  std::vector<uint8_t> raw_hazard;
  std::vector<uint8_t> hazard_mask;
  std::vector<uint8_t> unknown_mask;
  std::vector<uint8_t> persistence_count;
  std::vector<int> kernel_dx;
  std::vector<int> kernel_dy;
  std::vector<int> span_dx_max;
  std::vector<float> sigmoid_lut;

  // Internal performance timers (in ms)
  float t_step1_ms = 0.0f;
  float t_step2a_ms = 0.0f;
  float t_step2b_ms = 0.0f;
  float t_step3_ms = 0.0f;
  float t_step4_ms = 0.0f;
  float t_total_ms = 0.0f;

  // Static cone cells for each of the 16 sensors (precomputed once, sorted by
  // r)
  std::vector<ConeCell> sensor_cones[16];

  // Precomputed ray LUTs for fast gap scouting (zero runtime trig / div)
  float sensor_cos[16];
  float sensor_sin[16];
  float sensor_col0[16];
  float sensor_row0[16];
  float sensor_dcol[16];
  float sensor_drow[16];

  struct PrecomputedRay {
    float ox, oy, cos_az, sin_az, col0, row0, dcol, drow;
  };
  PrecomputedRay pod_rays[180];

  SmartHybridCostmapFront(float x_min_ = -1.25f, float x_max_ = 1.25f,
                          float y_min_ = 0.0f, float y_max_ = 2.60f,
                          float res_ = 0.015f, float r_inflation_m_ = 0.075f)
      : x_min(x_min_), x_max(x_max_), y_min(y_min_), y_max(y_max_), res(res_),
        inv_res(1.0f / res_), scout_step_ds(std::max(res_ * 0.8f, 0.012f)),
        r_inflation_m(r_inflation_m_) {

    width = static_cast<int>(std::round((x_max - x_min) / res)) + 1;
    height = static_cast<int>(std::round((y_max - y_min) / res)) + 1;
    total_cells = width * height;

    log_odds.resize(total_cells, 0.0f);
    prob_map.resize(total_cells, 0.50f);
    raw_hazard.resize(total_cells, 0);
    hazard_mask.resize(total_cells, 0);
    unknown_mask.resize(total_cells, 1);
    persistence_count.resize(total_cells, 0);

    // Precompute fast Sigmoid LUT (2048 entries across [-10.0f, +10.0f])
    sigmoid_lut.resize(2048);
    for (int i = 0; i < 2048; ++i) {
      float lo = -10.0f + static_cast<float>(i) * (20.0f / 2047.0f);
      sigmoid_lut[i] = 1.0f / (1.0f + std::exp(-lo));
    }

    // Precompute circular dilation structuring element and horizontal spans
    inflation_cells = static_cast<int>(std::round(r_inflation_m / res));
    kernel_dx.clear();
    kernel_dy.clear();
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        if (dx * dx + dy * dy <= inflation_cells * inflation_cells) {
          kernel_dx.push_back(dx);
          kernel_dy.push_back(dy);
        }
      }
    }

    span_dx_max.resize(2 * inflation_cells + 1, 0);
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      int max_dx = static_cast<int>(std::floor(
          std::sqrt(std::max(0, inflation_cells * inflation_cells - dy * dy))));
      span_dx_max[dy + inflation_cells] = max_dx;
    }

    // Precompute static cone cells for each sensor (done ONCE at
    // initialization)
    _precompute_sensor_cones();

    // Precompute 16 ToF beam ray LUTs and 180 corner pod ray LUTs
    for (int i = 0; i < 16; ++i) {
      float ch = std::cos(SENSOR_RIG[i].heading);
      float sh = std::sin(SENSOR_RIG[i].heading);
      sensor_cos[i] = ch;
      sensor_sin[i] = sh;
      sensor_col0[i] = (SENSOR_RIG[i].x0 - x_min) * inv_res;
      sensor_row0[i] = (SENSOR_RIG[i].y0 - y_min) * inv_res;
      sensor_dcol[i] = scout_step_ds * ch * inv_res;
      sensor_drow[i] = scout_step_ds * sh * inv_res;
    }

    constexpr int NUM_CORNER_RAYS = 90;
    constexpr float PI_F = 3.14159265358979323846f;
    constexpr float DEG2RAD_F = PI_F / 180.0f;
    struct PodCfg {
      float ox, oy, start_deg, end_deg;
    };
    const PodCfg pods[2] = {{-0.0575f, 0.091f, -4.5f, 85.5f},
                            {0.0575f, 0.091f, 4.5f, -85.5f}};
    int ray_idx = 0;
    for (int p = 0; p < 2; ++p) {
      float start_rel = pods[p].start_deg * DEG2RAD_F;
      float end_rel = pods[p].end_deg * DEG2RAD_F;
      float step_rel =
          (end_rel - start_rel) / static_cast<float>(NUM_CORNER_RAYS - 1);
      for (int r_i = 0; r_i < NUM_CORNER_RAYS; ++r_i) {
        float rel_a = start_rel + r_i * step_rel;
        float az = (PI_F * 0.5f) - rel_a;
        float caz = std::cos(az);
        float saz = std::sin(az);
        pod_rays[ray_idx++] = {pods[p].ox,
                               pods[p].oy,
                               caz,
                               saz,
                               (pods[p].ox - x_min) * inv_res,
                               (pods[p].oy - y_min) * inv_res,
                               scout_step_ds * caz * inv_res,
                               scout_step_ds * saz * inv_res};
      }
    }
  }

  void _precompute_sensor_cones() {
    constexpr float L_OCC_CENTER = 5.5f;
    // Winning sweet spot flank overlap weight: 2.9 (locks adjacent beams
    // without bloat)
    constexpr float L_OCC_EDGE = 2.9f;

    for (int s_idx = 0; s_idx < 16; ++s_idx) {
      const auto &s = SENSOR_RIG[s_idx];
      sensor_cones[s_idx].clear();

      float half_fov = s.fov_rad * 0.5f; // exact physical beam edge
      // Optimal Gaussian standard deviation factor: 2.10
      float sigma_theta = half_fov / 2.10f;
      float two_sigma_theta_sq = 2.0f * sigma_theta * sigma_theta;

      for (int r = 0; r < height; ++r) {
        float cell_y = y_min + r * res + res * 0.5f;
        float dy = cell_y - s.y0;
        int row_off = r * width;

        for (int c = 0; c < width; ++c) {
          float cell_x = x_min + c * res + res * 0.5f;
          float dx = cell_x - s.x0;

          float dist = std::hypot(dx, dy);
          if (dist < s.min_range || dist > s.max_range)
            continue;

          float phi = std::atan2(dy, dx);
          float dth = std::abs(wrap_angle(phi - s.heading));
          if (dth > half_fov)
            continue;

          // Continuous Gaussian angular decay weight
          float ang_factor = std::exp(-(dth * dth) / two_sigma_theta_sq);
          float hit_val = L_OCC_EDGE + (L_OCC_CENTER - L_OCC_EDGE) * ang_factor;

          uint32_t idx = static_cast<uint32_t>(row_off + c);
          sensor_cones[s_idx].push_back({idx, dist, hit_val});
        }
      }

      // Sort cells by radial distance for fast sequential evaluation
      std::sort(sensor_cones[s_idx].begin(), sensor_cones[s_idx].end(),
                [](const ConeCell &a, const ConeCell &b) { return a.r < b.r; });
    }
  }

  // MAIN PROCESS FUNCTION: EXACT CONTINUOUS MATH, ZERO RUNTIME ATAN2
  void process_frame(const float *ranges_16,
                     bool compute_prob_and_unknown = true) {
    auto t_start = std::chrono::high_resolution_clock::now();

    // Step 1: Flat buffer reset (< 1 microsecond)
    std::fill(log_odds.begin(), log_odds.end(), 0.0f);

    constexpr float L_FREE = -2.5f;

    // Origin Launch Clearance: Robot chassis footprint behind sensors is safe
    for (int r = 0; r < height; ++r) {
      float cell_y = y_min + r * res + res * 0.5f;
      if (cell_y > 0.095f)
        break;
      int row_off = r * width;
      for (int c = 0; c < width; ++c) {
        float cell_x = x_min + c * res + res * 0.5f;
        if (std::abs(cell_x) <= 0.080f && cell_y >= 0.0f) {
          log_odds[row_off + c] = std::min(log_odds[row_off + c], L_FREE);
        }
      }
    }
    auto t_step1 = std::chrono::high_resolution_clock::now();

    // Step 2: Exact Continuous Float Evaluation via Precomputed Cone Cells
    // Pass 2A: Accumulate Obstacle Hit Evidence for active returns
    for (int s_idx = 0; s_idx < 16; ++s_idx) {
      float raw_z = ranges_16[s_idx];
      if (raw_z <= 0.0f || std::isnan(raw_z))
        continue; // Offline/missing sensor -> leave as unknown space
      float z = std::min(SENSOR_RIG[s_idx].max_range,
                         std::max(SENSOR_RIG[s_idx].min_range, raw_z));
      bool is_hit = (z < SENSOR_RIG[s_idx].max_range - 0.01f);
      if (!is_hit)
        continue;

      float sigma_r = std::max(0.030f * z, res);
      float r_hit_min = z - 0.12f * sigma_r;
      float r_hit_max = z + std::max(0.075f, 1.25f * sigma_r);

      const auto &cells = sensor_cones[s_idx];
      size_t n = cells.size();
      for (size_t i = 0; i < n; ++i) {
        float r = cells[i].r;
        if (r < r_hit_min)
          continue;
        if (r <= r_hit_max) {
          log_odds[cells[i].cell_idx] += cells[i].hit_log_odds;
        } else {
          break;
        }
      }
    }
    auto t_step2a = std::chrono::high_resolution_clock::now();

    // Pass 2B: Strict Free-Space & Negative Cone Space-Carving
    // Any beam that is negative (or travels through open air before a hit
    // further along) physically guarantees NO obstacle exists inside that
    // free-flight cone. It decisively carves away phantom arcs, lateral bloat,
    // and front overextensions. Hits confirmed by active returns are protected
    // and never overwritten by open-air rays.
    for (int s_idx = 0; s_idx < 16; ++s_idx) {
      float raw_z = ranges_16[s_idx];
      if (raw_z <= 0.0f || std::isnan(raw_z))
        continue; // Offline/missing sensor -> leave as unknown space
      float z = std::min(SENSOR_RIG[s_idx].max_range,
                         std::max(SENSOR_RIG[s_idx].min_range, raw_z));
      bool is_hit = (z < SENSOR_RIG[s_idx].max_range - 0.01f);

      float r_carve_max;
      if (!is_hit) {
        // Entire cone guaranteed 100% empty up to max range
        r_carve_max = SENSOR_RIG[s_idx].max_range;
      } else {
        // Guaranteed empty before hit distance; safety margin prevents clipping
        // the surface
        float sigma_r = std::max(0.030f * z, res);
        r_carve_max = z - std::max(0.025f, 1.0f * sigma_r);
      }

      if (r_carve_max > SENSOR_RIG[s_idx].min_range) {
        const auto &cells = sensor_cones[s_idx];
        size_t n = cells.size();
        for (size_t i = 0; i < n; ++i) {
          if (cells[i].r < r_carve_max) {
            uint32_t c_idx = cells[i].cell_idx;
            // Space-carve: reset/override any phantom hit evidence to confirmed
            // free
            log_odds[c_idx] = std::min(log_odds[c_idx], L_FREE);
          } else {
            break;
          }
        }
      }
    }

    // Origin Launch Clearance: Robot chassis footprint behind sensors is safe
    for (int r = 0; r < height; ++r) {
      float cell_y = y_min + r * res + res * 0.5f;
      if (cell_y > 0.095f)
        break;
      int row_off = r * width;
      for (int c = 0; c < width; ++c) {
        float cell_x = x_min + c * res + res * 0.5f;
        if (std::abs(cell_x) <= 0.080f && cell_y >= 0.0f) {
          log_odds[row_off + c] = std::min(log_odds[row_off + c], L_FREE);
        }
      }
    }
    auto t_step2b = std::chrono::high_resolution_clock::now();

    // Step 3: Fast Sigmoid Probability Map, Confirmed Obstacle Threshold &
    // Unknown Space
    if (compute_prob_and_unknown) {
      constexpr float LUT_SCALE = 2047.0f / 20.0f;
      for (int i = 0; i < total_cells; ++i) {
        float lo = std::max(-10.0f, std::min(10.0f, log_odds[i]));
        int lut_idx = static_cast<int>((lo + 10.0f) * LUT_SCALE);
        lut_idx = std::max(0, std::min(2047, lut_idx));
        float p = sigmoid_lut[lut_idx];
        prob_map[i] = p;

        // Confirmed obstacle hits only (prob >= 0.65, log_odds >= 0.62)
        uint8_t raw = (lo >= 0.62f) ? 1 : 0;

        if (raw == 1) {
          persistence_count[i] = static_cast<uint8_t>(std::max(
              0,
              std::min(10,
                       persistence_frames))); // Configurable persistence frames
        } else if (lo <= -2.0f) {
          // Actively confirmed free by direct sensor cone: cancel persistence
          // immediately
          persistence_count[i] = 0;
        } else if (persistence_count[i] > 0) {
          persistence_count[i]--;
          raw = 1; // Retain confirmed hit during transient dropout
        }
        raw_hazard[i] = raw;

        // Unobserved space: untouched / uncertain cells (std::abs(lo) < 0.5f,
        // prob ~ 0.50)
        unknown_mask[i] = (std::abs(lo) < 0.5f && raw == 0) ? 1 : 0;
      }
    } else if (persistence_frames <= 0) {
      for (int i = 0; i < total_cells; ++i) {
        float lo = log_odds[i];
        uint8_t raw = (lo >= 0.62f) ? 1 : 0;
        raw_hazard[i] = raw;
        unknown_mask[i] = (std::abs(lo) < 0.5f && raw == 0) ? 1 : 0;
      }
    } else {
      const uint8_t p_frames =
          static_cast<uint8_t>(std::max(0, std::min(10, persistence_frames)));
      for (int i = 0; i < total_cells; ++i) {
        float lo = log_odds[i];
        uint8_t raw = (lo >= 0.62f) ? 1 : 0;
        if (raw == 1) {
          persistence_count[i] = p_frames;
        } else if (lo <= -2.0f) {
          persistence_count[i] = 0;
        } else if (persistence_count[i] > 0) {
          persistence_count[i]--;
          raw = 1;
        }
        raw_hazard[i] = raw;
        unknown_mask[i] = (std::abs(lo) < 0.5f && raw == 0) ? 1 : 0;
      }
    }
    auto t_step3 = std::chrono::high_resolution_clock::now();

    // Step 4: Clean Circular C-Space Morphological Dilation on Confirmed
    // Obstacle Boundaries
    std::fill(hazard_mask.begin(), hazard_mask.end(), 0);
    if (inflation_cells > 0) {
      for (int r = 0; r < height; ++r) {
        int row_off = r * width;
        for (int c = 0; c < width; ++c) {
          if (raw_hazard[row_off + c] == 1) {
            // Skip dilation for interior obstacle cells (already surrounded on
            // 4 sides)
            if (r > 0 && r < height - 1 && c > 0 && c < width - 1 &&
                raw_hazard[(r - 1) * width + c] == 1 &&
                raw_hazard[(r + 1) * width + c] == 1 &&
                raw_hazard[row_off + (c - 1)] == 1 &&
                raw_hazard[row_off + (c + 1)] == 1) {
              hazard_mask[row_off + c] = 1;
              continue; // Interior obstacle: perimeter cells dilate outward
                        // past this cell
            }

            // Boundary obstacle: Dilate using fast contiguous horizontal spans
            for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
              int nr = r + dy;
              if (nr < 0 || nr >= height)
                continue;
              int dx = span_dx_max[dy + inflation_cells];
              int nc_start = std::max(0, c - dx);
              int nc_end = std::min(width - 1, c + dx);
              int span_len = nc_end - nc_start + 1;
              if (span_len > 0) {
                std::memset(&hazard_mask[nr * width + nc_start], 1, span_len);
              }
            }
          }
        }
      }
    } else {
      hazard_mask = raw_hazard;
    }

    // Confirmed obstacle boundaries take precedence over unknown space
    if (compute_prob_and_unknown) {
      for (int i = 0; i < total_cells; ++i) {
        if (hazard_mask[i] == 1) {
          unknown_mask[i] = 0;
        }
      }
    }
    auto t_end = std::chrono::high_resolution_clock::now();

    t_step1_ms =
        std::chrono::duration<float, std::milli>(t_step1 - t_start).count();
    t_step2a_ms =
        std::chrono::duration<float, std::milli>(t_step2a - t_step1).count();
    t_step2b_ms =
        std::chrono::duration<float, std::milli>(t_step2b - t_step2a).count();
    t_step3_ms =
        std::chrono::duration<float, std::milli>(t_step3 - t_step2b).count();
    t_step4_ms =
        std::chrono::duration<float, std::milli>(t_end - t_step3).count();
    t_total_ms =
        std::chrono::duration<float, std::milli>(t_end - t_start).count();
  }

  // Step 6: O(1) Instant Trajectory Probe (Zero Allocation)
  bool probe_path(const float *xs, const float *ys, int count,
                  const uint8_t *custom_hazard_mask = nullptr) const {
    const uint8_t *active_haz =
        custom_hazard_mask ? custom_hazard_mask : hazard_mask.data();
    for (int i = 0; i < count; ++i) {
      float x = xs[i];
      float y = ys[i];
      if (x < x_min || x > x_max || y < y_min || y > y_max) {
        return false; // Out of bounds
      }
      int c = static_cast<int>(std::round((x - x_min) / res));
      int r = static_cast<int>(std::round((y - y_min) / res));
      if (c >= 0 && c < width && r >= 0 && r < height) {
        if (active_haz[r * width + c] == 1) {
          return false; // Collision detected
        }
      }
    }
    return true; // Collision-free
  }
  // ==========================================================================
  // C++ HIGH-PERFORMANCE GAP SCOUT & TARGET DISCOVERY CORE
  // Marches 180 dense radial rays through the updated hazard_mask, evaluates
  // the 16 physical ToF beam sightlines, and returns primary & challenger
  // targets
  // ==========================================================================
  struct TargetCandidate {
    float tx, ty, dist, aperture, steer_equiv, rel_angle;
    bool is_clear;
  };

  int scout_gaps(const float *ranges_16, float max_lookahead_m,
                 float steer_current, float w_gap_distance, float w_gap_width,
                 float w_align_steer, float w_align_straight,
                 float w_side_clearance, float gap_dist_exp_k,
                 float gap_dist_thresh_m, float alt_min_separation_m,
                 float alt_aperture_bias, float scout_obs_width_m,
                 float scout_block_dist_m, float scout_straight_pen_k,
                 float scout_straight_exp_k, float *out_targets_x,
                 float *out_targets_y, float *out_targets_aperture,
                 int max_targets, const uint8_t *custom_hazard_mask = nullptr,
                 float *out_targets_clear = nullptr, float anchor_x = 0.0f,
                 float anchor_y = 0.0f, float anchor_yaw = 0.0f) {
    const uint8_t *active_haz =
        custom_hazard_mask ? custom_hazard_mask : hazard_mask.data();
    constexpr float GAP_MIN_LOOKAHEAD_M = 0.20f;
    constexpr float GAP_MAX_LOOKAHEAD_M = 2.30f;
    constexpr float GAP_MIN_WIDTH_M = 0.0f;

    float max_r = std::min(max_lookahead_m, GAP_MAX_LOOKAHEAD_M);
    float step_ds = std::max(res * 0.8f, 0.012f);
    int n_steps = static_cast<int>(max_r / step_ds);

    // Preallocated fixed buffer for candidates
    constexpr int MAX_CAND = 256;
    TargetCandidate candidates[MAX_CAND];
    int n_cand = 0;

    // 1. Evaluate 16 physical ToF sensor sightlines strictly through costmap
    // hazard mask
    if (ranges_16) {
      for (int i = 0; i < 16; ++i) {
        if (n_cand >= MAX_CAND)
          break;
        float sx = SENSOR_RIG[i].x0;
        float sy = SENSOR_RIG[i].y0;
        float max_range = SENSOR_RIG[i].max_range;
        float z = std::min(max_range, std::max(0.10f, ranges_16[i]));

        float cos_h = sensor_cos[i];
        float sin_h = sensor_sin[i];
        float clear_dist = z;
        bool hit_obs = false;

        int n_b_steps = static_cast<int>(z / step_ds);
        float dcol = sensor_dcol[i];
        float drow = sensor_drow[i];
        float cf = sensor_col0[i] + dcol;
        float rf = sensor_row0[i] + drow;
        for (int s = 1; s <= n_b_steps; ++s, cf += dcol, rf += drow) {
          if (cf < -0.5f || rf < -0.5f) {
            clear_dist = std::max(0.0f, s * step_ds - step_ds);
            break;
          }
          int col = static_cast<int>(cf + 0.5f);
          int row = static_cast<int>(rf + 0.5f);
          if (col >= width || row >= height) {
            clear_dist = std::max(0.0f, s * step_ds - step_ds);
            break;
          }
          if (active_haz[row * width + col] > 0) {
            clear_dist = std::max(0.0f, s * step_ds - step_ds);
            hit_obs = true;
            break;
          }
        }

        float eff_dist =
            hit_obs ? std::max(0.05f, clear_dist - 0.075f) : clear_dist;
        float tx = sx + eff_dist * cos_h;
        float ty = std::max(0.08f, sy + eff_dist * sin_h);

        float d_anchor_x = tx - anchor_x;
        float d_anchor_y = std::max(0.05f, ty - anchor_y);
        float total_dist = std::hypot(d_anchor_x, d_anchor_y);
        float yaw_adj = fix_yaw_sign ? anchor_yaw : -anchor_yaw;
        float rel_angle =
            wrap_angle(std::atan2(d_anchor_x, d_anchor_y) + yaw_adj);
        float steer_equiv = -rel_angle;
        float aperture_m =
            std::max(GAP_MIN_WIDTH_M, std::min(1.20f, total_dist * 0.35f));
        bool is_clear = (!hit_obs) && (ranges_16[i] >= (max_range - 0.01f) ||
                                       total_dist >= (max_r - 0.05f));

        candidates[n_cand++] = {tx,          ty,        total_dist, aperture_m,
                                steer_equiv, rel_angle, is_clear};
      }
    }

    // 2. Evaluate 180 dense radial exploration rays (90 Left Pod, 90 Right Pod)
    constexpr float PI = 3.14159265358979323846f;
    constexpr float DEG2RAD = PI / 180.0f;

    for (int r_idx = 0; r_idx < 180; ++r_idx) {
      if (n_cand >= MAX_CAND)
        break;
      const auto &ray = pod_rays[r_idx];
      float clear_dist = max_r;
      bool hit_obs = false;

      float cf = ray.col0 + ray.dcol;
      float rf = ray.row0 + ray.drow;
      for (int s = 1; s <= n_steps; ++s, cf += ray.dcol, rf += ray.drow) {
        if (cf < -0.5f || rf < -0.5f) {
          clear_dist = std::max(0.0f, s * step_ds - step_ds);
          break;
        }
        int col = static_cast<int>(cf + 0.5f);
        int row = static_cast<int>(rf + 0.5f);
        if (col >= width || row >= height) {
          clear_dist = std::max(0.0f, s * step_ds - step_ds);
          break;
        }
        if (active_haz[row * width + col] > 0) {
          clear_dist = std::max(0.0f, s * step_ds - step_ds);
          hit_obs = true;
          break;
        }
      }

      float eff_dist =
          hit_obs ? std::max(0.05f, clear_dist - 0.06f) : clear_dist;
      float tx = ray.ox + eff_dist * ray.cos_az;
      float ty = std::max(0.08f, ray.oy + eff_dist * ray.sin_az);

      float d_anchor_x = tx - anchor_x;
      float d_anchor_y = std::max(0.05f, ty - anchor_y);
      float total_dist = std::hypot(d_anchor_x, d_anchor_y);
      float yaw_adj = fix_yaw_sign ? anchor_yaw : -anchor_yaw;
      float rel_angle =
          wrap_angle(std::atan2(d_anchor_x, d_anchor_y) + yaw_adj);
      float steer_equiv = -rel_angle;
      float aperture_m =
          std::max(GAP_MIN_WIDTH_M, std::min(1.20f, total_dist * 0.30f));
      bool is_clear = (!hit_obs);

      candidates[n_cand++] = {tx,          ty,        total_dist, aperture_m,
                              steer_equiv, rel_angle, is_clear};
    }

    if (n_cand == 0) {
      if (out_targets_x)
        out_targets_x[0] = 0.0f;
      if (out_targets_y)
        out_targets_y[0] = max_lookahead_m;
      if (out_targets_aperture)
        out_targets_aperture[0] = 0.40f;
      return 1;
    }

    // 3. Score Candidate Targets
    float cand_edge_margin[MAX_CAND] = {0.0f};
    if (use_true_aperture != 0 || w_gap_center > 0.0f) {
      float ray_dist_deg[181];
      for (int b = 0; b <= 180; ++b)
        ray_dist_deg[b] = 0.0f;
      for (int i = 0; i < n_cand; ++i) {
        int bin =
            std::max(0, std::min(180, static_cast<int>(std::round(
                                          candidates[i].rel_angle / DEG2RAD)) +
                                          90));
        if (candidates[i].dist > ray_dist_deg[bin]) {
          ray_dist_deg[bin] = candidates[i].dist;
        }
      }
      for (int b = 6; b <= 174; ++b) {
        if (ray_dist_deg[b] <= 0.0f && ray_dist_deg[b - 1] > 0.0f &&
            ray_dist_deg[b + 1] > 0.0f) {
          ray_dist_deg[b] = 0.5f * (ray_dist_deg[b - 1] + ray_dist_deg[b + 1]);
        }
      }
      for (int i = 0; i < n_cand; ++i) {
        int bin =
            std::max(0, std::min(180, static_cast<int>(std::round(
                                          candidates[i].rel_angle / DEG2RAD)) +
                                          90));
        float d_ref = std::max(0.20f, candidates[i].dist * 0.85f);
        int dl = 1;
        while (dl <= 30 && (bin - dl) >= 5 && ray_dist_deg[bin - dl] >= d_ref) {
          dl++;
        }
        int dr = 1;
        while (dr <= 30 && (bin + dr) <= 175 &&
               ray_dist_deg[bin + dr] >= d_ref) {
          dr++;
        }
        float theta_l = static_cast<float>(dl - 1) * DEG2RAD;
        float theta_r = static_cast<float>(dr - 1) * DEG2RAD;
        float true_w =
            std::min(1.20f, candidates[i].dist * (theta_l + theta_r));
        cand_edge_margin[i] =
            std::min(0.50f, candidates[i].dist * std::min(theta_l, theta_r));
        if (use_true_aperture != 0) {
          candidates[i].aperture = true_w;
        }
      }
    }

    float max_clear_dist = 0.10f;
    for (int i = 0; i < n_cand; ++i) {
      if (candidates[i].dist > max_clear_dist)
        max_clear_dist = candidates[i].dist;
    }

    // Single direct raycast straight ahead from anchor on inflated costmap
    float d_center = max_r;
    int n_center_steps = static_cast<int>(max_r / step_ds);
    float cos_fwd = -std::sin(anchor_yaw);
    float sin_fwd = std::cos(anchor_yaw);
    float dcol_fwd = step_ds * cos_fwd * inv_res;
    float drow_fwd = step_ds * sin_fwd * inv_res;
    float cf_fwd = (anchor_x - x_min) * inv_res + dcol_fwd;
    float rf_fwd = (anchor_y + 0.095f - y_min) * inv_res + drow_fwd;
    for (int s = 1; s <= n_center_steps;
         ++s, cf_fwd += dcol_fwd, rf_fwd += drow_fwd) {
      if (cf_fwd < -0.5f || rf_fwd < -0.5f) {
        d_center = std::max(0.0f, s * step_ds - step_ds);
        break;
      }
      int col = static_cast<int>(cf_fwd + 0.5f);
      int row = static_cast<int>(rf_fwd + 0.5f);
      if (col >= width || row >= height || active_haz[row * width + col] > 0) {
        d_center = std::max(0.0f, s * step_ds - step_ds);
        break;
      }
    }

    float side_diff = 0.0f;
    if (ranges_16) {
      // Group 8 sensors per side:
      // Left-side openness (indices 0 [left front ~4.1 deg] + 9..15 [right pod
      // cross-pointing left 15.4 to 82.9 deg]) Right-side openness (indices 8
      // [right front ~4.1 deg] + 1..7 [left pod cross-pointing right 15.4
      // to 82.9 deg])
      float l_sum = ranges_16[0];
      for (int k = 9; k <= 15; ++k) {
        l_sum += ranges_16[k];
      }
      float l_total = l_sum / 8.0f;

      float r_sum = ranges_16[8];
      for (int k = 1; k <= 7; ++k) {
        r_sum += ranges_16[k];
      }
      float r_total = r_sum / 8.0f;

      float denom = std::max(0.25f, 0.5f * (l_total + r_total));
      side_diff = (l_total - r_total) / denom;
    }

    float w_straight = w_align_straight;
    float straight_block_penalty = 0.0f;
    float theta_shadow = 0.0f;
    if (d_center < scout_block_dist_m) {
      // Distance-scaled angular shadow half-width (scales naturally as obstacle
      // nears bumper)
      theta_shadow =
          std::atan2(scout_obs_width_m * 0.5f, std::max(d_center, 0.05f));
      straight_block_penalty =
          scout_straight_pen_k *
          std::max(0.0f, (scout_block_dist_m - d_center) / scout_block_dist_m);
    }
    float w_steer = (std::abs(steer_current) > 0.08f && d_center >= 1.0f)
                        ? w_align_steer
                        : 0.20f;
    float min_allowed_dist =
        (max_clear_dist > 0.85f)
            ? std::max(GAP_MIN_LOOKAHEAD_M, max_clear_dist * 0.45f)
            : GAP_MIN_LOOKAHEAD_M;

    struct ScoredCandidate {
      float score;
      int cand_idx;
    };
    ScoredCandidate scored[MAX_CAND];
    int n_scored = 0;
    float cand_dist_factor[MAX_CAND] = {0.0f};
    float cand_center_penalty[MAX_CAND] = {0.0f};

    for (int i = 0; i < n_cand; ++i) {
      const auto &c = candidates[i];
      if (c.dist < min_allowed_dist)
        continue;

      float d_cand = c.dist;
      float d_thresh = gap_dist_thresh_m;
      float dist_base = d_cand / std::max(max_clear_dist, 0.10f);
      float dist_factor =
          (d_cand > d_thresh)
              ? dist_base * std::exp(gap_dist_exp_k * (d_cand - d_thresh))
              : dist_base;
      cand_dist_factor[i] = dist_factor;

      float width_norm = c.aperture / 1.00f;
      float steer_diff = std::abs(c.steer_equiv - steer_current);
      float straight_diff = std::abs(c.rel_angle);
      float side_bonus =
          w_side_clearance * side_diff * (-std::sin(c.rel_angle));
      float center_penalty = 0.0f;
      if (straight_block_penalty > 0.0f && theta_shadow > 0.001f &&
          std::abs(c.rel_angle) < theta_shadow) {
        float angle_ratio = 1.0f - (std::abs(c.rel_angle) / theta_shadow);
        center_penalty = straight_block_penalty *
                         std::exp(scout_straight_exp_k * angle_ratio);
      }
      cand_center_penalty[i] = center_penalty;

      // Neighboring ToF Zone Width Bonus (0.3x lowest cost / best neighboring
      // zone clearance)
      float neighbor_bonus = 0.0f;
      if (ranges_16) {
        // Find matching physical zone index for candidate angle
        // Zones 0..7 (Left, 4.1 deg to 82.9 deg) and 8..15 (Right, -4.1 deg to
        // -82.9 deg)
        int z_idx = 0;
        if (c.rel_angle >= 0.0f) {
          z_idx = std::min(
              7, std::max(0, static_cast<int>(std::round(
                                 (c.rel_angle / DEG2RAD - 4.1f) / 11.25f))));
        } else {
          z_idx = std::min(
              15,
              std::max(8, 8 + static_cast<int>(std::round(
                                  (-c.rel_angle / DEG2RAD - 4.1f) / 11.25f))));
        }
        float left_neighbor_dist =
            (z_idx > 0 && z_idx != 8) ? ranges_16[z_idx - 1] : 0.0f;
        float right_neighbor_dist =
            (z_idx < 15 && z_idx != 7) ? ranges_16[z_idx + 1] : 0.0f;
        float min_neighbor_dist =
            std::max(left_neighbor_dist, right_neighbor_dist);
        float neighbor_norm = std::min(2.40f, min_neighbor_dist) / 2.40f;
        neighbor_bonus = 0.30f * w_gap_width * neighbor_norm;
      }

      float clear_bonus = c.is_clear ? 3.0f : 0.0f;
      float center_bonus = (w_gap_center > 0.0f)
                               ? (w_gap_center * (cand_edge_margin[i] / 0.50f))
                               : 0.0f;
      float sc = w_gap_distance * dist_factor + w_gap_width * width_norm +
                 neighbor_bonus + side_bonus + clear_bonus + center_bonus -
                 center_penalty - w_steer * steer_diff -
                 w_straight * straight_diff;

      scored[n_scored++] = {sc, i};
    }

    if (n_scored == 0) {
      if (out_targets_x)
        out_targets_x[0] = 0.0f;
      if (out_targets_y)
        out_targets_y[0] = max_lookahead_m;
      if (out_targets_aperture)
        out_targets_aperture[0] = 0.40f;
      if (out_targets_clear)
        out_targets_clear[0] = 1.0f;
      return 1;
    }

    std::sort(scored, scored + n_scored,
              [](const ScoredCandidate &a, const ScoredCandidate &b) {
                return a.score > b.score;
              });

    // 4. Smart Corridor Selection (Primary + Challenger)
    // Candidate 0 is the strictly highest-scoring opening instantaneous
    int c0_idx = scored[0].cand_idx;
    const auto &c0 = candidates[c0_idx];

    int num_found = 0;
    if (out_targets_x && out_targets_y && out_targets_aperture) {
      out_targets_x[0] = c0.tx;
      out_targets_y[0] = c0.ty;
      out_targets_aperture[0] = c0.aperture;
      if (out_targets_clear)
        out_targets_clear[0] = c0.is_clear ? 1.0f : 0.0f;
      num_found = 1;

      if (max_targets >= 2) {
        // Challenger Line: best opening separated by alt_min_separation_m or
        // opposite side, and NOT from adjacent ToF zones (minimum angular
        // separation of >= 24.0 deg / 2+ zones)
        int best_alt_idx = -1;
        float best_alt_score = -1e9f;
        constexpr float MIN_ALT_ANGULAR_SEP_RAD = 24.0f * DEG2RAD;

        for (int k = 0; k < n_scored; ++k) {
          const auto &c = candidates[scored[k].cand_idx];
          float lat_sep = std::abs(c.tx - c0.tx);
          float ang_sep = std::abs(c.rel_angle - c0.rel_angle);
          bool is_opposite_side = (c0.tx < -0.04f && c.tx > 0.04f) ||
                                  (c0.tx > 0.04f && c.tx < -0.04f);

          // Must be separated by minimum distance AND not be an immediately
          // adjacent zone
          if ((lat_sep >= alt_min_separation_m || is_opposite_side) &&
              (ang_sep >= MIN_ALT_ANGULAR_SEP_RAD || is_opposite_side)) {
            int c_idx = scored[k].cand_idx;
            float dist_factor = cand_dist_factor[c_idx];
            float width_norm = c.aperture / 1.00f;
            float steer_diff = std::abs(c.steer_equiv - steer_current);
            float straight_diff = std::abs(c.rel_angle);
            float side_bonus_alt = (w_side_clearance * 0.45f) * side_diff *
                                   (-std::sin(c.rel_angle));
            float clear_bonus_alt =
                c.is_clear ? (3.0f * alt_aperture_bias) : 0.0f;
            float center_penalty = cand_center_penalty[c_idx];

            float center_bonus_alt =
                (w_gap_center > 0.0f)
                    ? (w_gap_center * (cand_edge_margin[c_idx] / 0.50f))
                    : 0.0f;
            float score_alt = w_gap_distance * dist_factor +
                              (w_gap_width * alt_aperture_bias) * width_norm +
                              side_bonus_alt + clear_bonus_alt +
                              center_bonus_alt - center_penalty -
                              w_steer * steer_diff - w_straight * straight_diff;

            if (score_alt > best_alt_score) {
              best_alt_score = score_alt;
              best_alt_idx = scored[k].cand_idx;
            }
          }
        }

        if (best_alt_idx >= 0) {
          const auto &calt = candidates[best_alt_idx];
          out_targets_x[1] = calt.tx;
          out_targets_y[1] = calt.ty;
          out_targets_aperture[1] = calt.aperture;
          if (out_targets_clear)
            out_targets_clear[1] = calt.is_clear ? 1.0f : 0.0f;
          num_found = 2;
        }
      }
    }

    return num_found;
  }
};

// ==============================================================================
// C-ABI EXPORTS FOR PYTHON CTYPES AND CONTROLLERS
// ==============================================================================
extern "C" {

COSTMAP_API void *costmap_create(float x_min, float x_max, float y_min,
                                 float y_max, float res, float r_inflation_m) {
  return new SmartHybridCostmapFront(x_min, x_max, y_min, y_max, res,
                                     r_inflation_m);
}

COSTMAP_API void costmap_destroy(void *handle) {
  if (handle) {
    delete static_cast<SmartHybridCostmapFront *>(handle);
  }
}

COSTMAP_API void costmap_clear(void *handle) {
  if (!handle)
    return;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  std::fill(cm->log_odds.begin(), cm->log_odds.end(), 0.0f);
  std::fill(cm->prob_map.begin(), cm->prob_map.end(), 0.50f);
  std::fill(cm->raw_hazard.begin(), cm->raw_hazard.end(), 0);
  std::fill(cm->hazard_mask.begin(), cm->hazard_mask.end(), 0);
  std::fill(cm->unknown_mask.begin(), cm->unknown_mask.end(), 1);
  std::fill(cm->persistence_count.begin(), cm->persistence_count.end(), 0);
}

COSTMAP_API void costmap_process_frame(void *handle, const float *ranges_16,
                                       float *out_prob_map,
                                       uint8_t *out_hazard_mask) {
  if (!handle)
    return;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  cm->process_frame(ranges_16, out_prob_map != nullptr);

  if (out_prob_map) {
    std::memcpy(out_prob_map, cm->prob_map.data(),
                cm->total_cells * sizeof(float));
  }
  if (out_hazard_mask) {
    std::memcpy(out_hazard_mask, cm->hazard_mask.data(),
                cm->total_cells * sizeof(uint8_t));
  }
}

COSTMAP_API void costmap_get_unknown_mask(void *handle,
                                          uint8_t *out_unknown_mask) {
  if (!handle || !out_unknown_mask)
    return;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  std::memcpy(out_unknown_mask, cm->unknown_mask.data(),
              cm->total_cells * sizeof(uint8_t));
}

COSTMAP_API void costmap_get_perf(void *handle, float *out_timers_6) {
  if (!handle || !out_timers_6)
    return;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  out_timers_6[0] = cm->t_step1_ms;
  out_timers_6[1] = cm->t_step2a_ms;
  out_timers_6[2] = cm->t_step2b_ms;
  out_timers_6[3] = cm->t_step3_ms;
  out_timers_6[4] = cm->t_step4_ms;
  out_timers_6[5] = cm->t_total_ms;
}

COSTMAP_API int costmap_scout_gaps(
    void *handle, const float *ranges_16, float max_lookahead_m,
    float steer_current, float w_gap_distance, float w_gap_width,
    float w_align_steer, float w_align_straight, float w_side_clearance,
    float gap_dist_exp_k, float gap_dist_thresh_m, float alt_min_separation_m,
    float alt_aperture_bias, float scout_obs_width_m, float scout_block_dist_m,
    float scout_straight_pen_k, float scout_straight_exp_k,
    float *out_targets_x, float *out_targets_y, float *out_targets_aperture,
    int max_targets, const uint8_t *custom_hazard_mask = nullptr,
    float *out_targets_clear = nullptr) {
  if (!handle)
    return 0;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  return cm->scout_gaps(
      ranges_16, max_lookahead_m, steer_current, w_gap_distance, w_gap_width,
      w_align_steer, w_align_straight, w_side_clearance, gap_dist_exp_k,
      gap_dist_thresh_m, alt_min_separation_m, alt_aperture_bias,
      scout_obs_width_m, scout_block_dist_m, scout_straight_pen_k,
      scout_straight_exp_k, out_targets_x, out_targets_y, out_targets_aperture,
      max_targets, custom_hazard_mask, out_targets_clear, 0.0f, 0.0f, 0.0f);
}

COSTMAP_API int costmap_scout_gaps_projected(
    void *handle, const float *ranges_16, float max_lookahead_m,
    float steer_current, float w_gap_distance, float w_gap_width,
    float w_align_steer, float w_align_straight, float w_side_clearance,
    float gap_dist_exp_k, float gap_dist_thresh_m, float alt_min_separation_m,
    float alt_aperture_bias, float scout_obs_width_m, float scout_block_dist_m,
    float scout_straight_pen_k, float scout_straight_exp_k,
    float *out_targets_x, float *out_targets_y, float *out_targets_aperture,
    int max_targets, const uint8_t *custom_hazard_mask,
    float *out_targets_clear, float anchor_x, float anchor_y,
    float anchor_yaw) {
  if (!handle)
    return 0;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  return cm->scout_gaps(
      ranges_16, max_lookahead_m, steer_current, w_gap_distance, w_gap_width,
      w_align_steer, w_align_straight, w_side_clearance, gap_dist_exp_k,
      gap_dist_thresh_m, alt_min_separation_m, alt_aperture_bias,
      scout_obs_width_m, scout_block_dist_m, scout_straight_pen_k,
      scout_straight_exp_k, out_targets_x, out_targets_y, out_targets_aperture,
      max_targets, custom_hazard_mask, out_targets_clear, anchor_x, anchor_y,
      anchor_yaw);
}

COSTMAP_API int costmap_probe_path(void *handle, const float *xs,
                                   const float *ys, int count) {
  if (!handle)
    return 0;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  return cm->probe_path(xs, ys, count) ? 1 : 0;
}

COSTMAP_API int costmap_probe_path_custom(void *handle, const float *xs,
                                          const float *ys, int count,
                                          const uint8_t *custom_hazard_mask) {
  if (!handle)
    return 0;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  return cm->probe_path(xs, ys, count, custom_hazard_mask) ? 1 : 0;
}

COSTMAP_API int costmap_get_width(void *handle) {
  if (!handle)
    return 0;
  return static_cast<SmartHybridCostmapFront *>(handle)->width;
}

COSTMAP_API int costmap_get_height(void *handle) {
  if (!handle)
    return 0;
  return static_cast<SmartHybridCostmapFront *>(handle)->height;
}

COSTMAP_API void costmap_set_advanced_params(void *handle,
                                             int persistence_frames,
                                             int fix_yaw_sign,
                                             int use_true_aperture,
                                             float w_gap_center) {
  if (!handle)
    return;
  auto *cm = static_cast<SmartHybridCostmapFront *>(handle);
  cm->persistence_frames = std::max(0, std::min(10, persistence_frames));
  cm->fix_yaw_sign = (fix_yaw_sign != 0) ? 1 : 0;
  cm->use_true_aperture = (use_true_aperture != 0) ? 1 : 0;
  cm->w_gap_center = std::max(0.0f, std::min(20.0f, w_gap_center));
}
}
