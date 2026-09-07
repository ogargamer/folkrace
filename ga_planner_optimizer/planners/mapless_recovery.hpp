#pragma once
#include <cmath>
#include <algorithm>
#include <cstdint>

enum class RecoveryState : uint8_t {
    NORMAL_TRACKING = 0,
    SPIN_OR_IMPACT_BRAKE = 1,
    REALIGN_REVERSE_ARC = 2,
    REALIGN_FORWARD_ALIGN = 3,
    SETTLE = 4
};

struct MaplessRecoveryConfig {
    float impact_g_thresh       = 3.2f;    // Gs: Collision impact threshold (3.2 * 9.81 m/s^2)
    float spin_rate_thresh      = 6.0f;    // rad/s (~343 deg/s): Uncontrolled rotation threshold
    float max_controlled_yaw    = 3.5f;    // rad/s: Maximum possible yaw rate under normal grip
    float inverted_angle_thresh = 1.85f;   // rad (~106 deg): Threshold indicating car is backward
    float realign_tol_rad       = 0.40f;   // rad (~23 deg): Acceptable heading to resume
    float pivot_speed_mps       = 0.55f;   // m/s: Crawl speed while turning back around
    float reverse_speed_mps     = -1.00f;  // m/s: Fast reverse speed near tire traction limit
    float max_steer_rad         = 0.6981f; // rad: Full steering lock (+/- 40 deg)
    float settle_time_s         = 0.08f;   // Post-recovery stabilization period
    float brake_timeout_s       = 0.30f;   // Max time to wait in brake state
    float max_recovery_time_s   = 3.0f;    // Global recovery safety timeout
};

class MaplessRecoveryController {
public:
    RecoveryState state = RecoveryState::NORMAL_TRACKING;
    MaplessRecoveryConfig cfg;

    float locked_target_yaw = 0.0f;
    bool initialized_anchor = false;
    float settle_timer = 0.0f;
    float state_timer = 0.0f;
    float total_recovery_timer = 0.0f;

    explicit MaplessRecoveryController(MaplessRecoveryConfig config = MaplessRecoveryConfig())
        : cfg(config) {}

    static inline float wrap_angle(float angle) {
        constexpr float PI = 3.14159265358979323846f;
        constexpr float TWO_PI = 2.0f * PI;
        while (angle >  PI) angle -= TWO_PI;
        while (angle < -PI) angle += TWO_PI;
        return angle;
    }

    void reset(float initial_yaw = 0.0f) {
        state = RecoveryState::NORMAL_TRACKING;
        settle_timer = 0.0f;
        state_timer = 0.0f;
        total_recovery_timer = 0.0f;
        locked_target_yaw = initial_yaw;
        initialized_anchor = true;
    }

    /**
     * @brief Self-contained mapless recovery loop at 50 Hz.
     * @param current_yaw       Current world yaw from IMU (rad).
     * @param current_yaw_rate  Z-gyro rate (rad/s).
     * @param accel_x_g         Surge/lateral acceleration in Gs.
     * @param accel_y_g         Forward acceleration in Gs.
     * @param current_speed     Estimated forward ground speed (m/s).
     * @param front_clearance_m Minimum clearance from front ToFs (meters).
     * @param dt                Time delta (0.020s for 50Hz).
     * @param[out] out_steer    Overridden steer command if recovering.
     * @param[out] out_speed    Overridden speed command if recovering.
     * @return true if recovery is active (override planner), false to let gap planner run.
     */
    bool update(
        float current_yaw,
        float current_yaw_rate,
        float accel_x_g,
        float accel_y_g,
        float current_speed,
        float front_clearance_m,
        float dt,
        float& out_steer,
        float& out_speed
    ) {
        if (!initialized_anchor) {
            locked_target_yaw = current_yaw;
            initialized_anchor = true;
        }

        float total_accel_g = std::hypot(accel_x_g, accel_y_g);
        float abs_yaw_rate = std::abs(current_yaw_rate);

        if (state != RecoveryState::NORMAL_TRACKING) {
            total_recovery_timer += dt;
            state_timer += dt;

            // Global safety timeout guard
            if (total_recovery_timer >= cfg.max_recovery_time_s) {
                reset(current_yaw);
                return false;
            }
        }

        switch (state) {
            case RecoveryState::NORMAL_TRACKING: {
                bool is_spinout = abs_yaw_rate > cfg.spin_rate_thresh;
                bool is_impact = total_accel_g > cfg.impact_g_thresh;
                float heading_err = wrap_angle(locked_target_yaw - current_yaw);
                bool is_wrong_way = std::abs(heading_err) > 1.5708f;

                if (is_spinout || (is_impact && is_wrong_way)) {
                    state = RecoveryState::SPIN_OR_IMPACT_BRAKE;
                    state_timer = 0.0f;
                    total_recovery_timer = 0.0f;
                    out_speed = 0.0f;
                    out_steer = 0.0f;
                    return true;
                }

                // Update anchor smoothly during healthy forward motion
                if (current_speed > 0.20f && abs_yaw_rate < cfg.max_controlled_yaw) {
                    locked_target_yaw = current_yaw;
                }
                return false;
            }

            case RecoveryState::SPIN_OR_IMPACT_BRAKE: {
                out_speed = 0.0f;
                out_steer = 0.0f;

                // Proceed when rotation settles OR on timeout
                if (abs_yaw_rate < 1.2f || state_timer >= cfg.brake_timeout_s) {
                    float heading_err = wrap_angle(locked_target_yaw - current_yaw);

                    if (std::abs(heading_err) > cfg.inverted_angle_thresh) {
                        // Inverted: reverse arc to rotate chassis back toward anchor
                        state = RecoveryState::REALIGN_REVERSE_ARC;
                        state_timer = 0.0f;
                    } else {
                        // Facing correct direction: return immediately to normal/backup tracking
                        reset(current_yaw);
                        return false;
                    }
                }
                return true;
            }

            case RecoveryState::REALIGN_REVERSE_ARC: {
                float heading_err = wrap_angle(locked_target_yaw - current_yaw);

                // In reverse, steering Left (+steer) swings nose Right (CW).
                // If heading_err > 0, we need nose to rotate Left (CCW) -> steer Right (-steer).
                float turn_dir = (heading_err > 0.0f) ? -1.0f : 1.0f;
                out_steer = turn_dir * cfg.max_steer_rad;
                out_speed = cfg.reverse_speed_mps;

                if (front_clearance_m >= 0.25f || std::abs(heading_err) < 1.20f || state_timer >= 1.50f) {
                    state = RecoveryState::REALIGN_FORWARD_ALIGN;
                    state_timer = 0.0f;
                }
                return true;
            }

            case RecoveryState::REALIGN_FORWARD_ALIGN: {
                float heading_err = wrap_angle(locked_target_yaw - current_yaw);

                // Driving forward: positive steer turns Left (CCW).
                float turn_dir = (heading_err > 0.0f) ? 1.0f : -1.0f;
                out_steer = turn_dir * cfg.max_steer_rad;
                out_speed = cfg.pivot_speed_mps;

                if (front_clearance_m < 0.09f && state_timer > 0.20f) {
                    state = RecoveryState::REALIGN_REVERSE_ARC;
                    state_timer = 0.0f;
                    out_steer = -turn_dir * cfg.max_steer_rad;
                    out_speed = cfg.reverse_speed_mps;
                    return true;
                }

                if (std::abs(heading_err) < cfg.realign_tol_rad) {
                    state = RecoveryState::SETTLE;
                    settle_timer = 0.0f;
                    state_timer = 0.0f;
                }
                return true;
            }

            case RecoveryState::SETTLE: {
                settle_timer += dt;
                float heading_err = wrap_angle(locked_target_yaw - current_yaw);

                out_steer = std::clamp(heading_err * 0.8f, -cfg.max_steer_rad, cfg.max_steer_rad);
                out_speed = cfg.pivot_speed_mps * 1.2f;

                if (settle_timer >= cfg.settle_time_s) {
                    reset(current_yaw);
                    return false;
                }
                return true;
            }
        }

        return false;
    }
};
