#include "../include/webots/optimization/racing_engine.hpp"

extern "C" {

#ifdef _WIN32
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API __attribute__((visibility("default")))
#endif

EXPORT_API void* create_webots/optimization/racing_engine(
    double weight_curvature,
    double weight_path_smoothness,
    double weight_jerk,
    double robot_safety_margin,
    int num_stages,
    int max_sub_iters,
    double friction_coeff,
    double max_speed,
    double max_accel,
    double max_brake,
    int num_waypoints
) {
    OptimizerConfigCPP cfg;
    cfg.weight_curvature = weight_curvature;
    cfg.weight_path_smoothness = weight_path_smoothness;
    cfg.weight_jerk = weight_jerk;
    cfg.robot_safety_margin = robot_safety_margin;
    cfg.num_progression_stages = num_stages;
    cfg.max_solver_sub_iters = max_sub_iters;
    cfg.friction_coeff = friction_coeff;
    cfg.max_speed = max_speed;
    cfg.max_accel = max_accel;
    cfg.max_brake = max_brake;
    cfg.num_waypoints = static_cast<size_t>(num_waypoints);

    return new TUM_RacingEngine_CPP(cfg);
}

EXPORT_API void destroy_webots/optimization/racing_engine(void* enginePtr) {
    if (enginePtr) {
        delete static_cast<TUM_RacingEngine_CPP*>(enginePtr);
    }
}

EXPORT_API void optimize_track_cpp(
    void* enginePtr,
    const double* leftWallX, const double* leftWallY, int leftWallCount,
    const double* rightWallX, const double* rightWallY, int rightWallCount,
    const double* inCenterlineX, const double* inCenterlineY, int inCenterlineCount,
    const int* obsTypes, const double* obsParams, int obsCount, // obsParams: [cx, cy, r_or_w, h, angle] (5 per obs)
    double* outRacingLineX, double* outRacingLineY,
    double* outCenterlineX, double* outCenterlineY,
    double* outVelocities,
    double* outMetrics // [minApexSpeed, topSpeed, lapTime, minClearanceCm, solveTimeMs, solveTimeUs]
) {
    if (!enginePtr) return;
    auto* engine = static_cast<TUM_RacingEngine_CPP*>(enginePtr);

    TrackDataCPP track;
    track.leftWall.reserve(leftWallCount);
    for (int i = 0; i < leftWallCount; ++i) {
        track.leftWall.emplace_back(leftWallX[i], leftWallY[i]);
    }

    track.rightWall.reserve(rightWallCount);
    for (int i = 0; i < rightWallCount; ++i) {
        track.rightWall.emplace_back(rightWallX[i], rightWallY[i]);
    }

    if (inCenterlineCount > 0 && inCenterlineX && inCenterlineY) {
        track.inputCenterline.reserve(inCenterlineCount);
        for (int i = 0; i < inCenterlineCount; ++i) {
            track.inputCenterline.emplace_back(inCenterlineX[i], inCenterlineY[i]);
        }
    }

    track.obstacles.reserve(obsCount);
    for (int i = 0; i < obsCount; ++i) {
        ObstacleCPP obs;
        obs.type = (obsTypes[i] == 0) ? ObstacleType::CIRCLE : ObstacleType::WALL_RECTANGLE;
        obs.cx = obsParams[i * 5 + 0];
        obs.cy = obsParams[i * 5 + 1];
        if (obs.type == ObstacleType::CIRCLE) {
            obs.radius = obsParams[i * 5 + 2];
            obs.width = 0; obs.height = 0; obs.angleRad = 0;
        } else {
            obs.radius = 0;
            obs.width = obsParams[i * 5 + 2];
            obs.height = obsParams[i * 5 + 3];
            obs.angleRad = obsParams[i * 5 + 4];
        }
        track.obstacles.push_back(obs);
    }

    RacingLineResultCPP res = engine->optimize(track);

    size_t N = res.racingLine.size();
    for (size_t i = 0; i < N; ++i) {
        outRacingLineX[i] = res.racingLine[i].x;
        outRacingLineY[i] = res.racingLine[i].y;
        outCenterlineX[i] = res.centerline[i].x;
        outCenterlineY[i] = res.centerline[i].y;
        outVelocities[i] = res.velocities[i];
    }

    outMetrics[0] = res.minApexSpeed;
    outMetrics[1] = res.topSpeed;
    outMetrics[2] = res.lapTime;
    outMetrics[3] = res.minPhysicalClearanceCm;
    outMetrics[4] = res.solveTimeMs;
    outMetrics[5] = res.solveTimeUs;
}

}
