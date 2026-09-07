#pragma once
#include <vector>
#include <string>
#include <memory>
#include "kdtree2d.hpp"
#include "cubic_spline2d.hpp"

enum class ObstacleType {
    CIRCLE,
    WALL_RECTANGLE
};

struct ObstacleCPP {
    ObstacleType type;
    std::string name;
    double cx, cy;
    double radius;              // For circle
    double width, height;       // For rectangle
    double angleRad;            // For rectangle rotation
};

struct TrackDataCPP {
    std::vector<Point2D> leftWall;
    std::vector<Point2D> rightWall;
    std::vector<Point2D> inputCenterline;
    std::vector<ObstacleCPP> obstacles;
};

struct OptimizerConfigCPP {
    double weight_curvature = 3.20;
    double weight_path_smoothness = 0.055;
    double weight_jerk = 0.38;
    double robot_safety_margin = 0.10; // 100mm
    int num_progression_stages = 7;
    int max_solver_sub_iters = 300;
    double friction_coeff = 0.90;      // mu
    double max_speed = 4.00;           // m/s
    double max_accel = 2.00;           // m/s^2
    double max_brake = 3.00;           // m/s^2
    size_t num_waypoints = 360;
};

struct RacingLineResultCPP {
    std::vector<Point2D> racingLine;
    std::vector<Point2D> centerline;
    std::vector<double> alpha;
    std::vector<double> velocities;
    std::vector<double> minBounds;
    std::vector<double> maxBounds;
    double minApexSpeed = 0.0;
    double topSpeed = 0.0;
    double lapTime = 0.0;
    double minPhysicalClearanceCm = 0.0;
    double solveTimeMs = 0.0;
    double solveTimeUs = 0.0;
};

class TUM_RacingEngine_CPP {
private:
    OptimizerConfigCPP config;

    void sampleObstaclePoints(const std::vector<ObstacleCPP>& obs, std::vector<Point2D>& outPts, double spacing) const;
    void generateAllBoundaryPoints(const TrackDataCPP& track, std::vector<Point2D>& outPts, double spacing) const;
    void computeCorridorBounds(
        const std::vector<Point2D>& centerline,
        const std::vector<Point2D>& normals,
        const KDTree2D& barrierTree,
        std::vector<double>& outMinBounds,
        std::vector<double>& outMaxBounds
    ) const;

public:
    TUM_RacingEngine_CPP(const OptimizerConfigCPP& cfg = OptimizerConfigCPP());
    void setConfig(const OptimizerConfigCPP& cfg) { config = cfg; }
    const OptimizerConfigCPP& getConfig() const { return config; }

    RacingLineResultCPP optimize(const TrackDataCPP& track) const;
};
