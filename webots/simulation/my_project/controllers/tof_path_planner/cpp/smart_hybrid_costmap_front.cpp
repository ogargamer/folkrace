#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cstring>

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
    const char* name;
    float x0;
    float y0;
    float heading;
    float fov_rad;
    float min_range;
    float max_range;
};

static const ToFSensor SENSOR_RIG[16] = {
    // Front-Left Array (X = -0.070 m, Y = +0.095 m, cross-pointing Right)
    {"tof_front_left_4.1deg",  -0.070f, 0.095f, 1.4987f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_left_15.4deg", -0.070f, 0.095f, 1.3024f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_left_26.6deg", -0.070f, 0.095f, 1.1060f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_left_37.9deg", -0.070f, 0.095f, 0.9097f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_left_49.1deg", -0.070f, 0.095f, 0.7134f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_left_60.4deg", -0.070f, 0.095f, 0.5170f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_left_71.6deg", -0.070f, 0.095f, 0.3207f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_left_82.9deg", -0.070f, 0.095f, 0.1243f, 0.1963f, 0.001f, 1.80f},

    // Front-Right Array (X = +0.070 m, Y = +0.095 m, cross-pointing Left)
    {"tof_front_right_4.1deg",   0.070f, 0.095f, 1.6428f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_right_15.4deg",  0.070f, 0.095f, 1.8391f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_right_26.6deg",  0.070f, 0.095f, 2.0356f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_right_37.9deg",  0.070f, 0.095f, 2.2318f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_right_49.1deg",  0.070f, 0.095f, 2.4282f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_right_60.4deg",  0.070f, 0.095f, 2.6245f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_right_71.6deg",  0.070f, 0.095f, 2.8208f, 0.1963f, 0.001f, 1.80f},
    {"tof_front_right_82.9deg",  0.070f, 0.095f, 3.0172f, 0.1963f, 0.001f, 1.80f},
};

static inline float wrap_angle(float a) {
    constexpr float TWO_PI = 2.0f * 3.14159265358979323846f;
    constexpr float PI = 3.14159265358979323846f;
    a = std::fmod(a + PI, TWO_PI);
    if (a < 0.0f) a += TWO_PI;
    return a - PI;
}

// Precomputed static polar cell within a sensor's cone
struct ConeCell {
    uint32_t cell_idx;
    float r;               // Geometric distance to sensor origin
    float hit_log_odds;    // Precomputed continuous Gaussian angular decay weight
};

// ==============================================================================
// EXACT CONTINUOUS SMART HYBRID COSTMAP ENGINE (FRONT-ONLY RIG)
// ==============================================================================
class SmartHybridCostmapFront {
public:
    float x_min, x_max, y_min, y_max;
    float res;
    int width, height, total_cells;
    float r_inflation_m;
    int inflation_cells;

    // Flat preallocated buffers (zero runtime heap allocations)
    std::vector<float> log_odds;
    std::vector<float> prob_map;
    std::vector<uint8_t> raw_hazard;
    std::vector<uint8_t> hazard_mask;
    std::vector<uint8_t> unknown_mask;
    std::vector<uint8_t> persistence_count;
    std::vector<int> kernel_dx;
    std::vector<int> kernel_dy;

    // Static cone cells for each of the 16 sensors (precomputed once, sorted by r)
    std::vector<ConeCell> sensor_cones[16];

    SmartHybridCostmapFront(
        float x_min_ = -1.25f, float x_max_ = 1.25f,
        float y_min_ = 0.0f, float y_max_ = 1.90f,
        float res_ = 0.010f, float r_inflation_m_ = 0.075f
    ) : x_min(x_min_), x_max(x_max_), y_min(y_min_), y_max(y_max_),
        res(res_), r_inflation_m(r_inflation_m_) {

        width = static_cast<int>(std::round((x_max - x_min) / res)) + 1;
        height = static_cast<int>(std::round((y_max - y_min) / res)) + 1;
        total_cells = width * height;

        log_odds.resize(total_cells, 0.0f);
        prob_map.resize(total_cells, 0.50f);
        raw_hazard.resize(total_cells, 0);
        hazard_mask.resize(total_cells, 0);
        unknown_mask.resize(total_cells, 1);
        persistence_count.resize(total_cells, 0);

        // Precompute circular dilation structuring element
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

        // Precompute static cone cells for each sensor (done ONCE at initialization)
        _precompute_sensor_cones();
    }

    void _precompute_sensor_cones() {
        constexpr float L_OCC_CENTER = 5.5f;
        // Winning sweet spot flank overlap weight: 2.9 (locks adjacent beams without bloat)
        constexpr float L_OCC_EDGE = 2.9f;

        for (int s_idx = 0; s_idx < 16; ++s_idx) {
            const auto& s = SENSOR_RIG[s_idx];
            sensor_cones[s_idx].clear();

            float half_fov = s.fov_rad * 0.5f;  // exact physical beam edge
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
                    if (dist < s.min_range || dist > s.max_range) continue;

                    float phi = std::atan2(dy, dx);
                    float dth = std::abs(wrap_angle(phi - s.heading));
                    if (dth > half_fov) continue;

                    // Continuous Gaussian angular decay weight
                    float ang_factor = std::exp(-(dth * dth) / two_sigma_theta_sq);
                    float hit_val = L_OCC_EDGE + (L_OCC_CENTER - L_OCC_EDGE) * ang_factor;

                    uint32_t idx = static_cast<uint32_t>(row_off + c);
                    sensor_cones[s_idx].push_back({idx, dist, hit_val});
                }
            }

            // Sort cells by radial distance for fast sequential evaluation
            std::sort(sensor_cones[s_idx].begin(), sensor_cones[s_idx].end(),
                      [](const ConeCell& a, const ConeCell& b) {
                          return a.r < b.r;
                      });
        }
    }

    // MAIN PROCESS FUNCTION: EXACT CONTINUOUS MATH, ZERO RUNTIME ATAN2
    void process_frame(const float* ranges_16) {
        // Step 1: Flat buffer reset (< 1 microsecond)
        std::fill(log_odds.begin(), log_odds.end(), 0.0f);

        constexpr float L_FREE = -2.5f;

        // Origin Launch Clearance: Robot chassis footprint is safe
        for (int r = 0; r < height; ++r) {
            float cell_y = y_min + r * res + res * 0.5f;
            if (cell_y > 0.18f) break;
            int row_off = r * width;
            for (int c = 0; c < width; ++c) {
                float cell_x = x_min + c * res + res * 0.5f;
                if (std::abs(cell_x) <= (0.10f + r_inflation_m) && cell_y >= 0.0f) {
                    log_odds[row_off + c] = std::min(log_odds[row_off + c], L_FREE);
                }
            }
        }

        // Step 2: Exact Continuous Float Evaluation via Precomputed Cone Cells
        for (int s_idx = 0; s_idx < 16; ++s_idx) {
            float raw_z = ranges_16[s_idx];
            float z = std::min(SENSOR_RIG[s_idx].max_range, std::max(SENSOR_RIG[s_idx].min_range, raw_z));
            bool is_hit = (z < SENSOR_RIG[s_idx].max_range - 0.01f);

            // Halved depth expansion into unobserved space behind detected obstacle (~7.5 cm)
            float sigma_r = std::max(0.030f * z, res);
            float r_hit_min = z - 0.12f * sigma_r;
            float r_hit_max = z + std::max(0.075f, 1.25f * sigma_r);

            const auto& cells = sensor_cones[s_idx];
            size_t n = cells.size();

            if (is_hit) {
                for (size_t i = 0; i < n; ++i) {
                    float r = cells[i].r;
                    if (r < r_hit_min) {
                        log_odds[cells[i].cell_idx] += L_FREE;
                    } else if (r <= r_hit_max) {
                        log_odds[cells[i].cell_idx] += cells[i].hit_log_odds;
                    } else {
                        // Beyond obstacle hit depth: stop scanning shadow
                        break;
                    }
                }
            } else {
                // Entire cone is clear to max range
                for (size_t i = 0; i < n; ++i) {
                    log_odds[cells[i].cell_idx] += L_FREE;
                }
            }
        }

        // Step 3: Sigmoid Probability Map, Confirmed Obstacle Threshold & Unknown Space
        for (int i = 0; i < total_cells; ++i) {
            float lo = std::max(-10.0f, std::min(10.0f, log_odds[i]));
            float p = 1.0f / (1.0f + std::exp(-lo));
            prob_map[i] = p;

            // Confirmed obstacle hits only (prob >= 0.65, log_odds >= 0.62)
            uint8_t raw = (lo >= 0.62f) ? 1 : 0;

            if (raw == 1) {
                persistence_count[i] = 3; // Hold confirmed hit for up to 3 frames (50 ms)
            } else if (lo <= -2.0f) {
                // Actively confirmed free by direct sensor cone: cancel persistence immediately
                persistence_count[i] = 0;
            } else if (persistence_count[i] > 0) {
                persistence_count[i]--;
                raw = 1; // Retain confirmed hit during transient dropout
            }
            raw_hazard[i] = raw;

            // Unobserved space: untouched / uncertain cells (std::abs(lo) < 0.5f, prob ~ 0.50)
            unknown_mask[i] = (std::abs(lo) < 0.5f && raw == 0) ? 1 : 0;
        }

        // Step 4: Clean Circular C-Space Morphological Dilation on Confirmed Obstacle Boundaries
        std::fill(hazard_mask.begin(), hazard_mask.end(), 0);
        if (inflation_cells > 0) {
            for (int r = 0; r < height; ++r) {
                int row_off = r * width;
                for (int c = 0; c < width; ++c) {
                    if (raw_hazard[row_off + c] == 1) {
                        for (size_t k = 0; k < kernel_dx.size(); ++k) {
                            int nr = r + kernel_dy[k];
                            int nc = c + kernel_dx[k];
                            if (nr >= 0 && nr < height && nc >= 0 && nc < width) {
                                hazard_mask[nr * width + nc] = 1;
                            }
                        }
                    }
                }
            }
        } else {
            hazard_mask = raw_hazard;
        }

        // Confirmed obstacle boundaries take precedence over unknown space
        for (int i = 0; i < total_cells; ++i) {
            if (hazard_mask[i] == 1) {
                unknown_mask[i] = 0;
            }
        }
    }

    // Step 6: O(1) Instant Trajectory Probe (Zero Allocation)
    bool probe_path(const float* xs, const float* ys, int count, const uint8_t* custom_hazard_mask = nullptr) const {
        const uint8_t* active_haz = custom_hazard_mask ? custom_hazard_mask : hazard_mask.data();
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
    // the 16 physical ToF beam sightlines, and returns primary & challenger targets
    // ==========================================================================
    struct TargetCandidate {
        float tx, ty, dist, aperture, steer_equiv, rel_angle;
    };

    int scout_gaps(
        const float* ranges_16,
        float max_lookahead_m,
        float steer_current,
        float w_gap_distance,
        float w_gap_width,
        float w_align_steer,
        float w_align_straight,
        float w_side_clearance,
        float gap_dist_exp_k,
        float gap_progress_margin_m,
        float alt_min_separation_m,
        float alt_aperture_bias,
        float scout_obs_width_m,
        float scout_block_dist_m,
        float scout_straight_pen_k,
        float scout_straight_exp_k,
        float* out_targets_x,
        float* out_targets_y,
        float* out_targets_aperture,
        int max_targets,
        const uint8_t* custom_hazard_mask = nullptr
    ) {
        const uint8_t* active_haz = custom_hazard_mask ? custom_hazard_mask : hazard_mask.data();
        constexpr float GAP_MIN_LOOKAHEAD_M = 0.30f;
        constexpr float GAP_MAX_LOOKAHEAD_M = 1.70f;
        constexpr float GAP_MIN_WIDTH_M = 0.0f;
        constexpr float TRACK_CORRIDOR_WIDTH_M = 1.00f;

        float max_r = std::min(max_lookahead_m, GAP_MAX_LOOKAHEAD_M);
        float step_ds = std::max(res * 0.8f, 0.012f);
        int n_steps = static_cast<int>(max_r / step_ds);

        // Preallocated fixed buffer for candidates
        constexpr int MAX_CAND = 256;
        TargetCandidate candidates[MAX_CAND];
        int n_cand = 0;

        // 1. Evaluate 16 physical ToF sensor sightlines strictly through costmap hazard mask
        if (ranges_16) {
            for (int i = 0; i < 16; ++i) {
                if (n_cand >= MAX_CAND) break;
                float sx = SENSOR_RIG[i].x0;
                float sy = SENSOR_RIG[i].y0;
                float heading = SENSOR_RIG[i].heading;
                float max_range = SENSOR_RIG[i].max_range;
                float z = std::min(max_range, std::max(0.10f, ranges_16[i]));

                float cos_h = std::cos(heading);
                float sin_h = std::sin(heading);
                float clear_dist = z;

                int n_b_steps = static_cast<int>(z / step_ds);
                for (int s = 1; s <= n_b_steps; ++s) {
                    float d = s * step_ds;
                    float px = sx + d * cos_h;
                    float py = sy + d * sin_h;
                    int col = static_cast<int>(std::round((px - x_min) / res));
                    int row = static_cast<int>(std::round((py - y_min) / res));
                    if (col < 0 || col >= width || row < 0 || row >= height) {
                        clear_dist = std::max(0.0f, d - step_ds);
                        break;
                    }
                    if (active_haz[row * width + col] > 0) {
                        clear_dist = std::max(0.0f, d - step_ds);
                        break;
                    }
                }

                float eff_dist = std::max(0.05f, clear_dist - 0.075f);
                float tx = sx + eff_dist * cos_h;
                float ty = std::max(0.08f, sy + eff_dist * sin_h);
                float total_dist = std::hypot(tx, ty);
                float rel_angle = std::atan2(tx, ty);
                float steer_equiv = -rel_angle;
                float aperture_m = std::max(GAP_MIN_WIDTH_M, std::min(1.20f, total_dist * 0.35f));

                candidates[n_cand++] = {tx, ty, total_dist, aperture_m, steer_equiv, rel_angle};
            }
        }

        // 2. Evaluate 180 dense radial exploration rays (90 Left Pod, 90 Right Pod)
        constexpr int NUM_CORNER_RAYS = 90;
        constexpr float PI = 3.14159265358979323846f;
        constexpr float DEG2RAD = PI / 180.0f;

        // Left Pod (-0.07m, 0.095m): -1.5 deg to +88.5 deg
        // Right Pod (+0.07m, 0.095m): +1.5 deg to -88.5 deg
        struct PodConfig {
            float ox, oy;
            float start_rel_deg, end_rel_deg;
        };
        const PodConfig pods[2] = {
            {-0.070f, 0.095f, -1.5f, 88.5f},
            {0.070f, 0.095f, 1.5f, -88.5f}
        };

        for (int p = 0; p < 2; ++p) {
            float ox = pods[p].ox;
            float oy = pods[p].oy;
            float start_rel = pods[p].start_rel_deg * DEG2RAD;
            float end_rel = pods[p].end_rel_deg * DEG2RAD;
            float step_rel = (end_rel - start_rel) / static_cast<float>(NUM_CORNER_RAYS - 1);

            for (int r_idx = 0; r_idx < NUM_CORNER_RAYS; ++r_idx) {
                if (n_cand >= MAX_CAND) break;
                float rel_a = start_rel + r_idx * step_rel;
                float az = (PI * 0.5f) - rel_a;
                float cos_az = std::cos(az);
                float sin_az = std::sin(az);
                float clear_dist = max_r;

                for (int s = 1; s <= n_steps; ++s) {
                    float d = s * step_ds;
                    float px = ox + d * cos_az;
                    float py = oy + d * sin_az;
                    int col = static_cast<int>(std::round((px - x_min) / res));
                    int row = static_cast<int>(std::round((py - y_min) / res));
                    if (col < 0 || col >= width || row < 0 || row >= height) {
                        clear_dist = std::max(0.0f, d - step_ds);
                        break;
                    }
                    if (active_haz[row * width + col] > 0) {
                        clear_dist = std::max(0.0f, d - step_ds);
                        break;
                    }
                }

                float eff_dist = std::max(0.05f, clear_dist - 0.06f);
                float tx = ox + eff_dist * cos_az;
                float ty = std::max(0.08f, oy + eff_dist * sin_az);
                float total_dist = std::hypot(tx, ty);
                float rel_angle = std::atan2(tx, ty);
                float steer_equiv = -rel_angle;
                float aperture_m = std::max(GAP_MIN_WIDTH_M, std::min(1.20f, total_dist * 0.30f));

                candidates[n_cand++] = {tx, ty, total_dist, aperture_m, steer_equiv, rel_angle};
            }
        }

        if (n_cand == 0) {
            if (out_targets_x) out_targets_x[0] = 0.0f;
            if (out_targets_y) out_targets_y[0] = max_lookahead_m;
            if (out_targets_aperture) out_targets_aperture[0] = 0.40f;
            return 1;
        }

        // 3. Score Candidate Targets
        float max_clear_dist = 0.10f;
        for (int i = 0; i < n_cand; ++i) {
            if (candidates[i].dist > max_clear_dist) max_clear_dist = candidates[i].dist;
        }

        // Single direct raycast straight ahead (x = 0, +Y axis) on inflated costmap
        float d_center = max_r;
        int n_center_steps = static_cast<int>(max_r / step_ds);
        for (int s = 1; s <= n_center_steps; ++s) {
            float d = s * step_ds;
            float py = 0.095f + d;
            int col = static_cast<int>(std::round((0.0f - x_min) / res));
            int row = static_cast<int>(std::round((py - y_min) / res));
            if (col < 0 || col >= width || row < 0 || row >= height || active_haz[row * width + col] > 0) {
                d_center = std::max(0.0f, d - step_ds);
                break;
            }
        }

        float side_diff = 0.0f;
        if (ranges_16) {
            // Right pod sensors (indices 13, 14, 15) point toward LEFT flank (60.4, 71.6, 82.9 deg left)
            float l_sum = ranges_16[15] + ranges_16[14] + ranges_16[13];
            // Left pod sensors (indices 5, 6, 7) point toward RIGHT flank (60.4, 71.6, 82.9 deg right)
            float r_sum = ranges_16[7] + ranges_16[6] + ranges_16[5];
            float l_mean = l_sum / 3.0f;
            float r_mean = r_sum / 3.0f;
            float denom = std::max(0.25f, 0.5f * (l_mean + r_mean));
            side_diff = (l_mean - r_mean) / denom;
        }

        float w_straight = w_align_straight;
        float straight_block_penalty = 0.0f;
        float theta_shadow = 0.0f;
        if (d_center < scout_block_dist_m) {
            // Distance-scaled angular shadow half-width (scales naturally as obstacle nears bumper)
            theta_shadow = std::atan2(scout_obs_width_m * 0.5f, std::max(d_center, 0.05f));
            straight_block_penalty = scout_straight_pen_k * std::max(0.0f, (scout_block_dist_m - d_center) / scout_block_dist_m);
        }
        float w_steer = (std::abs(steer_current) > 0.08f && d_center >= 1.0f) ? w_align_steer : 0.20f;
        float min_allowed_dist = (max_clear_dist > 0.85f) ? std::max(GAP_MIN_LOOKAHEAD_M, max_clear_dist * 0.45f) : GAP_MIN_LOOKAHEAD_M;

        struct ScoredCandidate {
            float score;
            int cand_idx;
        };
        ScoredCandidate scored[MAX_CAND];
        int n_scored = 0;

        for (int i = 0; i < n_cand; ++i) {
            const auto& c = candidates[i];
            if (c.dist < min_allowed_dist) continue;

            float d_cand = c.dist;
            float d_thresh = TRACK_CORRIDOR_WIDTH_M + gap_progress_margin_m;
            float dist_base = d_cand / std::max(max_clear_dist, 0.10f);
            float dist_factor = (d_cand > d_thresh) ? dist_base * std::exp(gap_dist_exp_k * (d_cand - d_thresh)) : dist_base;

            float width_norm = c.aperture / 1.00f;
            float steer_diff = std::abs(c.steer_equiv - steer_current);
            float straight_diff = std::abs(c.rel_angle);
            float side_bonus = w_side_clearance * side_diff * (-std::sin(c.rel_angle));
            float center_penalty = 0.0f;
            if (straight_block_penalty > 0.0f && theta_shadow > 0.001f && std::abs(c.rel_angle) < theta_shadow) {
                float angle_ratio = 1.0f - (std::abs(c.rel_angle) / theta_shadow);
                center_penalty = straight_block_penalty * std::exp(scout_straight_exp_k * angle_ratio);
            }

            // Neighboring ToF Zone Width Bonus (0.3x lowest cost / best neighboring zone clearance)
            float neighbor_bonus = 0.0f;
            if (ranges_16) {
                // Find matching physical zone index for candidate angle
                // Zones 0..7 (Left, 4.1 deg to 82.9 deg) and 8..15 (Right, -4.1 deg to -82.9 deg)
                int z_idx = 0;
                if (c.rel_angle >= 0.0f) {
                    z_idx = std::min(7, std::max(0, static_cast<int>(std::round((c.rel_angle / DEG2RAD - 4.1f) / 11.25f))));
                } else {
                    z_idx = std::min(15, std::max(8, 8 + static_cast<int>(std::round((-c.rel_angle / DEG2RAD - 4.1f) / 11.25f))));
                }
                float left_neighbor_dist = (z_idx > 0 && z_idx != 8) ? ranges_16[z_idx - 1] : 0.0f;
                float right_neighbor_dist = (z_idx < 15 && z_idx != 7) ? ranges_16[z_idx + 1] : 0.0f;
                float min_neighbor_dist = std::max(left_neighbor_dist, right_neighbor_dist);
                float neighbor_norm = std::min(1.80f, min_neighbor_dist) / 1.80f;
                neighbor_bonus = 0.30f * w_gap_width * neighbor_norm;
            }

            float sc = w_gap_distance * dist_factor
                     + w_gap_width * width_norm
                     + neighbor_bonus
                     + side_bonus
                     - center_penalty
                     - w_steer * steer_diff
                     - w_straight * straight_diff;

            scored[n_scored++] = {sc, i};
        }

        if (n_scored == 0) {
            if (out_targets_x) out_targets_x[0] = 0.0f;
            if (out_targets_y) out_targets_y[0] = max_lookahead_m;
            if (out_targets_aperture) out_targets_aperture[0] = 0.40f;
            return 1;
        }

        std::sort(scored, scored + n_scored, [](const ScoredCandidate& a, const ScoredCandidate& b) {
            return a.score > b.score;
        });

        // 4. Smart Corridor Selection (Primary + Challenger)
        // Candidate 0 is the strictly highest-scoring opening instantaneous
        int c0_idx = scored[0].cand_idx;
        const auto& c0 = candidates[c0_idx];

        int num_found = 0;
        if (out_targets_x && out_targets_y && out_targets_aperture) {
            out_targets_x[0] = c0.tx;
            out_targets_y[0] = c0.ty;
            out_targets_aperture[0] = c0.aperture;
            num_found = 1;

            if (max_targets >= 2) {
                // Challenger Line: best opening separated by alt_min_separation_m or opposite side,
                // and NOT from adjacent ToF zones (minimum angular separation of >= 24.0 deg / 2+ zones)
                int best_alt_idx = -1;
                float best_alt_score = -1e9f;
                constexpr float MIN_ALT_ANGULAR_SEP_RAD = 24.0f * DEG2RAD;

                for (int k = 0; k < n_scored; ++k) {
                    const auto& c = candidates[scored[k].cand_idx];
                    float lat_sep = std::abs(c.tx - c0.tx);
                    float ang_sep = std::abs(c.rel_angle - c0.rel_angle);
                    bool is_opposite_side = (c0.tx < -0.04f && c.tx > 0.04f) || (c0.tx > 0.04f && c.tx < -0.04f);

                    // Must be separated by minimum distance AND not be an immediately adjacent zone
                    if ((lat_sep >= alt_min_separation_m || is_opposite_side) && (ang_sep >= MIN_ALT_ANGULAR_SEP_RAD || is_opposite_side)) {
                        float d_cand = c.dist;
                        float d_thresh = TRACK_CORRIDOR_WIDTH_M + gap_progress_margin_m;
                        float dist_base = d_cand / std::max(max_clear_dist, 0.10f);
                        float dist_factor = (d_cand > d_thresh) ? dist_base * std::exp(gap_dist_exp_k * (d_cand - d_thresh)) : dist_base;
                        float width_norm = c.aperture / 1.00f;
                        float steer_diff = std::abs(c.steer_equiv - steer_current);
                        float straight_diff = std::abs(c.rel_angle);
                        float side_bonus_alt = (w_side_clearance * 0.45f) * side_diff * (-std::sin(c.rel_angle));
                        float center_penalty = 0.0f;
                        if (straight_block_penalty > 0.0f && theta_shadow > 0.001f && std::abs(c.rel_angle) < theta_shadow) {
                            float angle_ratio = 1.0f - (std::abs(c.rel_angle) / theta_shadow);
                            center_penalty = straight_block_penalty * std::exp(scout_straight_exp_k * angle_ratio);
                        }

                        float score_alt = w_gap_distance * dist_factor
                                        + (w_gap_width * alt_aperture_bias) * width_norm
                                        + side_bonus_alt
                                        - center_penalty
                                        - w_steer * steer_diff
                                        - w_straight * straight_diff;

                        if (score_alt > best_alt_score) {
                            best_alt_score = score_alt;
                            best_alt_idx = scored[k].cand_idx;
                        }
                    }
                }

                if (best_alt_idx >= 0) {
                    const auto& calt = candidates[best_alt_idx];
                    out_targets_x[1] = calt.tx;
                    out_targets_y[1] = calt.ty;
                    out_targets_aperture[1] = calt.aperture;
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

COSTMAP_API void* costmap_create(
    float x_min, float x_max,
    float y_min, float y_max,
    float res, float r_inflation_m
) {
    return new SmartHybridCostmapFront(x_min, x_max, y_min, y_max, res, r_inflation_m);
}

COSTMAP_API void costmap_destroy(void* handle) {
    if (handle) {
        delete static_cast<SmartHybridCostmapFront*>(handle);
    }
}

COSTMAP_API void costmap_clear(void* handle) {
    if (!handle) return;
    auto* cm = static_cast<SmartHybridCostmapFront*>(handle);
    std::fill(cm->log_odds.begin(), cm->log_odds.end(), 0.0f);
    std::fill(cm->prob_map.begin(), cm->prob_map.end(), 0.50f);
    std::fill(cm->raw_hazard.begin(), cm->raw_hazard.end(), 0);
    std::fill(cm->hazard_mask.begin(), cm->hazard_mask.end(), 0);
    std::fill(cm->unknown_mask.begin(), cm->unknown_mask.end(), 1);
    std::fill(cm->persistence_count.begin(), cm->persistence_count.end(), 0);
}

COSTMAP_API void costmap_process_frame(
    void* handle,
    const float* ranges_16,
    float* out_prob_map,
    uint8_t* out_hazard_mask
) {
    if (!handle) return;
    auto* cm = static_cast<SmartHybridCostmapFront*>(handle);
    cm->process_frame(ranges_16);

    if (out_prob_map) {
        std::memcpy(out_prob_map, cm->prob_map.data(), cm->total_cells * sizeof(float));
    }
    if (out_hazard_mask) {
        std::memcpy(out_hazard_mask, cm->hazard_mask.data(), cm->total_cells * sizeof(uint8_t));
    }
}

COSTMAP_API void costmap_get_unknown_mask(void* handle, uint8_t* out_unknown_mask) {
    if (!handle || !out_unknown_mask) return;
    auto* cm = static_cast<SmartHybridCostmapFront*>(handle);
    std::memcpy(out_unknown_mask, cm->unknown_mask.data(), cm->total_cells * sizeof(uint8_t));
}

COSTMAP_API int costmap_scout_gaps(
    void* handle,
    const float* ranges_16,
    float max_lookahead_m,
    float steer_current,
    float w_gap_distance,
    float w_gap_width,
    float w_align_steer,
    float w_align_straight,
    float w_side_clearance,
    float gap_dist_exp_k,
    float gap_progress_margin_m,
    float alt_min_separation_m,
    float alt_aperture_bias,
    float scout_obs_width_m,
    float scout_block_dist_m,
    float scout_straight_pen_k,
    float scout_straight_exp_k,
    float* out_targets_x,
    float* out_targets_y,
    float* out_targets_aperture,
    int max_targets,
    const uint8_t* custom_hazard_mask
) {
    if (!handle) return 0;
    auto* cm = static_cast<SmartHybridCostmapFront*>(handle);
    return cm->scout_gaps(
        ranges_16, max_lookahead_m, steer_current,
        w_gap_distance, w_gap_width, w_align_steer, w_align_straight, w_side_clearance,
        gap_dist_exp_k, gap_progress_margin_m, alt_min_separation_m, alt_aperture_bias,
        scout_obs_width_m, scout_block_dist_m, scout_straight_pen_k, scout_straight_exp_k,
        out_targets_x, out_targets_y, out_targets_aperture,
        max_targets,
        custom_hazard_mask
    );
}

COSTMAP_API int costmap_probe_path(
    void* handle,
    const float* xs,
    const float* ys,
    int count
) {
    if (!handle) return 0;
    auto* cm = static_cast<SmartHybridCostmapFront*>(handle);
    return cm->probe_path(xs, ys, count) ? 1 : 0;
}

COSTMAP_API int costmap_probe_path_custom(
    void* handle,
    const float* xs,
    const float* ys,
    int count,
    const uint8_t* custom_hazard_mask
) {
    if (!handle) return 0;
    auto* cm = static_cast<SmartHybridCostmapFront*>(handle);
    return cm->probe_path(xs, ys, count, custom_hazard_mask) ? 1 : 0;
}

COSTMAP_API int costmap_get_width(void* handle) {
    if (!handle) return 0;
    return static_cast<SmartHybridCostmapFront*>(handle)->width;
}

COSTMAP_API int costmap_get_height(void* handle) {
    if (!handle) return 0;
    return static_cast<SmartHybridCostmapFront*>(handle)->height;
}

}
