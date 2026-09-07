#include "../include/webots/optimization/racing_engine.hpp"
#include <chrono>
#include <iostream>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <LBFGSB.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

TUM_RacingEngine_CPP::TUM_RacingEngine_CPP(const OptimizerConfigCPP& cfg) : config(cfg) {}

void TUM_RacingEngine_CPP::sampleObstaclePoints(const std::vector<ObstacleCPP>& obsList, std::vector<Point2D>& outPts, double spacing) const {
    for (const auto& obs : obsList) {
        if (obs.type == ObstacleType::CIRCLE) {
            int steps = std::max(16, static_cast<int>(2.0 * M_PI * obs.radius / spacing));
            for (int i = 0; i < steps; ++i) {
                double theta = (2.0 * M_PI * i) / steps;
                outPts.emplace_back(obs.cx + obs.radius * std::cos(theta), obs.cy + obs.radius * std::sin(theta));
            }
        } else if (obs.type == ObstacleType::WALL_RECTANGLE) {
            double hw = obs.width / 2.0;
            double hh = obs.height / 2.0;
            double c = std::cos(obs.angleRad);
            double s = std::sin(obs.angleRad);

            auto rotatePoint = [&](double lx, double ly) {
                return Point2D(obs.cx + lx * c - ly * s, obs.cy + lx * s + ly * c);
            };

            int stepsW = std::max(4, static_cast<int>(obs.width / spacing));
            int stepsH = std::max(4, static_cast<int>(obs.height / spacing));

            for (int i = 0; i <= stepsW; ++i) {
                double t = static_cast<double>(i) / stepsW;
                double lx = -hw + t * obs.width;
                outPts.push_back(rotatePoint(lx, -hh));
                outPts.push_back(rotatePoint(lx, hh));
            }
            for (int i = 0; i <= stepsH; ++i) {
                double t = static_cast<double>(i) / stepsH;
                double ly = -hh + t * obs.height;
                outPts.push_back(rotatePoint(-hw, ly));
                outPts.push_back(rotatePoint(hw, ly));
            }
        }
    }
}

void TUM_RacingEngine_CPP::generateAllBoundaryPoints(const TrackDataCPP& track, std::vector<Point2D>& outPts, double spacing) const {
    outPts.clear();
    auto sampleLine = [&](const std::vector<Point2D>& wall) {
        size_t N = wall.size();
        for (size_t i = 0; i < N; ++i) {
            size_t iNext = (i + 1) % N;
            double dist = std::hypot(wall[iNext].x - wall[i].x, wall[iNext].y - wall[i].y);
            int steps = std::max(1, static_cast<int>(dist / spacing));
            for (int s = 0; s < steps; ++s) {
                double t = static_cast<double>(s) / steps;
                outPts.emplace_back(
                    (1.0 - t) * wall[i].x + t * wall[iNext].x,
                    (1.0 - t) * wall[i].y + t * wall[iNext].y
                );
            }
        }
    };

    sampleLine(track.leftWall);
    sampleLine(track.rightWall);
    sampleObstaclePoints(track.obstacles, outPts, spacing);
}

void TUM_RacingEngine_CPP::computeCorridorBounds(
    const std::vector<Point2D>& centerline,
    const std::vector<Point2D>& normals,
    const KDTree2D& barrierTree,
    std::vector<double>& outMinBounds,
    std::vector<double>& outMaxBounds
) const {
    size_t N = centerline.size();
    outMinBounds.resize(N);
    outMaxBounds.resize(N);

    double effectiveMargin = config.robot_safety_margin + 0.015; // 11.5 cm
    const int numAlphas = 121;
    std::vector<double> testAlphas(numAlphas);
    for (int k = 0; k < numAlphas; ++k) {
        testAlphas[k] = -0.60 + (1.20 * k) / (numAlphas - 1);
    }

    for (size_t i = 0; i < N; ++i) {
        const Point2D& pt = centerline[i];
        const Point2D& norm = normals[i];

        std::vector<int> safeIndices;
        safeIndices.reserve(numAlphas);
        double maxDist = -1.0;
        int bestIdx = 0;

        for (int k = 0; k < numAlphas; ++k) {
            Point2D queryPt(pt.x + norm.x * testAlphas[k], pt.y + norm.y * testAlphas[k]);
            double d = barrierTree.queryNearest(queryPt);
            if (d >= effectiveMargin) {
                safeIndices.push_back(k);
            }
            if (d > maxDist) {
                maxDist = d;
                bestIdx = k;
            }
        }

        if (safeIndices.empty()) {
            double bestA = testAlphas[bestIdx];
            outMinBounds[i] = bestA - 0.02;
            outMaxBounds[i] = bestA + 0.02;
        } else {
            // Find contiguous safe segments
            std::vector<std::pair<double, double>> segments;
            int segStart = safeIndices[0];
            int prevIdx = safeIndices[0];

            for (size_t s = 1; s < safeIndices.size(); ++s) {
                if (safeIndices[s] > prevIdx + 1) {
                    segments.emplace_back(testAlphas[segStart], testAlphas[prevIdx]);
                    segStart = safeIndices[s];
                }
                prevIdx = safeIndices[s];
            }
            segments.emplace_back(testAlphas[segStart], testAlphas[prevIdx]);

            // Select segment containing 0.0 or closest to 0.0
            std::pair<double, double> bestSeg = segments[0];
            double minZeroDist = 999.0;

            for (const auto& seg : segments) {
                if (seg.first <= 0.0 && 0.0 <= seg.second) {
                    bestSeg = seg;
                    break;
                }
                double zd = std::min(std::abs(seg.first), std::abs(seg.second));
                if (zd < minZeroDist) {
                    minZeroDist = zd;
                    bestSeg = seg;
                }
            }

            outMinBounds[i] = bestSeg.first;
            outMaxBounds[i] = bestSeg.second;
            if (outMinBounds[i] >= outMaxBounds[i]) {
                double mid = 0.5 * (outMinBounds[i] + outMaxBounds[i]);
                outMinBounds[i] = mid - 0.02;
                outMaxBounds[i] = mid + 0.02;
            }
        }
    }
}

// Objective function for LBFGSpp
class TUM_QP_Functor {
private:
    const Eigen::MatrixXd& H;
    const Eigen::VectorXd& f;
public:
    TUM_QP_Functor(const Eigen::MatrixXd& _H, const Eigen::VectorXd& _f) : H(_H), f(_f) {}

    double operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad) {
        grad = H * x + f;
        return 0.5 * x.dot(H * x) + f.dot(x);
    }
};

RacingLineResultCPP TUM_RacingEngine_CPP::optimize(const TrackDataCPP& track) const {
    auto tStart = std::chrono::high_resolution_clock::now();

    size_t N = config.num_waypoints;

    // 1. Build Spatial Index of Boundaries and Obstacles
    std::vector<Point2D> allBarriers;
    generateAllBoundaryPoints(track, allBarriers, 0.02);
    KDTree2D barrierTree;
    barrierTree.build(allBarriers);

    // 2. Generate Initial Medial Centerline
    std::vector<Point2D> centerline;
    if (!track.inputCenterline.empty()) {
        centerline = PeriodicCubicSpline2D::resampleUniform(track.inputCenterline, N);
    } else {
        size_t rawSize = std::min(track.leftWall.size(), track.rightWall.size());
        std::vector<Point2D> rawMid(rawSize);
        for (size_t i = 0; i < rawSize; ++i) {
            rawMid[i] = Point2D(
                0.5 * (track.leftWall[i].x + track.rightWall[i].x),
                0.5 * (track.leftWall[i].y + track.rightWall[i].y)
            );
        }
        centerline = PeriodicCubicSpline2D::resampleUniform(rawMid, N);
    }

    // 3. Compute Differential Properties of Medial Centerline
    SplineProperties2D centerProps = PeriodicCubicSpline2D::computeProperties(centerline);

    // 4. Continuous Interval Corridor Raycasting
    std::vector<double> minBounds, maxBounds;
    computeCorridorBounds(centerline, centerProps.normals, barrierTree, minBounds, maxBounds);

    // 5. Build Differential Matrices M1, M2, M3 in Eigen
    std::vector<double> ds = centerProps.ds;
    std::vector<double> kappa_c = centerProps.curvature;

    // Macro-curvature Gaussian filter
    std::vector<double> kc_macro(N, 0.0);
    const int filterHalf = 16;
    for (size_t i = 0; i < N; ++i) {
        double sum = 0.0, wSum = 0.0;
        for (int k = -filterHalf; k <= filterHalf; ++k) {
            size_t idx = (i + k + N) % N;
            double w = std::exp(-0.5 * (k * k) / (4.0 * 4.0));
            sum += kappa_c[idx] * w;
            wSum += w;
        }
        kc_macro[i] = sum / wSum;
    }

    Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(N, N);
    Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(N, N);
    Eigen::MatrixXd M3 = Eigen::MatrixXd::Zero(N, N);
    Eigen::MatrixXd W_ds = Eigen::MatrixXd::Zero(N, N);

    for (size_t i = 0; i < N; ++i) {
        size_t iPrev = (i + N - 1) % N;
        size_t iNext = (i + 1) % N;
        double ds_p = ds[iPrev];
        double ds_c = ds[i];
        double ds_avg = 0.5 * (ds_p + ds_c);

        M2(i, iPrev) = 1.0 / (ds_p * ds_avg);
        M2(i, i)     = -(1.0 / ds_p + 1.0 / ds_c) / ds_avg;
        M2(i, iNext) = 1.0 / (ds_c * ds_avg);

        M1(i, i) = -1.0;
        M1(i, iNext) = 1.0;

        M3(i, (i + N - 2) % N) = -1.0;
        M3(i, iPrev) = 2.0;
        M3(i, iNext) = -2.0;
        M3(i, (i + 2) % N) = 1.0;

        W_ds(i, i) = ds[i];
    }

    // Hessian H = Wk * M2^T Wds M2 + Wp * M1^T M1 + Wj * M3^T M3 + 1e-5 * I
    Eigen::MatrixXd H = config.weight_curvature * (M2.transpose() * W_ds * M2)
                      + config.weight_path_smoothness * (M1.transpose() * M1)
                      + config.weight_jerk * (M3.transpose() * M3)
                      + 1e-5 * Eigen::MatrixXd::Identity(N, N);

    Eigen::VectorXd kc_vec(N);
    for (size_t i = 0; i < N; ++i) kc_vec(i) = kc_macro[i];

    Eigen::VectorXd f_base = config.weight_curvature * (M2.transpose() * W_ds * kc_vec);

    Eigen::VectorXd lb(N);
    Eigen::VectorXd ub(N);
    for (size_t i = 0; i < N; ++i) {
        lb(i) = minBounds[i];
        ub(i) = maxBounds[i];
    }

    // 6. Multi-Stage Continuation with Native L-BFGS-B
    Eigen::VectorXd cur_alpha = Eigen::VectorXd::Zero(N);
    int numStages = config.num_progression_stages;

    LBFGSpp::LBFGSBParam<double> param;
    param.max_iterations = config.max_solver_sub_iters;
    param.epsilon = 1e-8;
    param.m = 8;

    for (int stage = 1; stage <= numStages; ++stage) {
        double w_prog = (numStages == 1) ? 1.0 : (0.10 + 0.90 * (stage - 1) / (numStages - 1));
        Eigen::VectorXd f_step = f_base * w_prog;

        LBFGSpp::LBFGSBSolver<double> solver(param);
        TUM_QP_Functor functor(H, f_step);

        double fval = 0.0;
        try {
            solver.minimize(functor, cur_alpha, fval, lb, ub);
        } catch (const std::exception& e) {
            // Continuation robust fallback
        }
    }

    std::vector<double> alpha(N);
    for (size_t i = 0; i < N; ++i) alpha[i] = cur_alpha(i);

    // 7. Reconstruct 2D Racing Line Points
    std::vector<Point2D> rawRacingLine(N);
    for (size_t i = 0; i < N; ++i) {
        rawRacingLine[i] = Point2D(
            centerline[i].x + centerProps.normals[i].x * alpha[i],
            centerline[i].y + centerProps.normals[i].y * alpha[i]
        );
    }
    std::vector<Point2D> finalRacingLine = PeriodicCubicSpline2D::resampleUniform(rawRacingLine, N);
    SplineProperties2D finalProps = PeriodicCubicSpline2D::computeProperties(finalRacingLine);

    // 8. Dynamic Forward/Backward Velocity Profiler
    std::vector<double> v_profile(N, config.max_speed);
    double g = 9.81;

    // Cornering limit: v = sqrt(mu * g / kappa)
    for (size_t i = 0; i < N; ++i) {
        double k = std::max(std::abs(finalProps.curvature[i]), 1e-4);
        double v_corner = std::sqrt((config.friction_coeff * g) / k);
        v_profile[i] = std::min(config.max_speed, v_corner);
    }

    // Backward pass (Braking limit)
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
            size_t iNext = (i + 1) % N;
            double v_allowed = std::sqrt(v_profile[iNext] * v_profile[iNext] + 2.0 * config.max_brake * finalProps.ds[i]);
            v_profile[i] = std::min(v_profile[i], v_allowed);
        }
    }

    // Forward pass (Acceleration limit)
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t i = 0; i < N; ++i) {
            size_t iPrev = (i + N - 1) % N;
            double v_allowed = std::sqrt(v_profile[iPrev] * v_profile[iPrev] + 2.0 * config.max_accel * finalProps.ds[iPrev]);
            v_profile[i] = std::min(v_profile[i], v_allowed);
        }
    }

    // 9. Extract Metrics
    double minSpd = config.max_speed;
    double maxSpd = 0.0;
    double lapT = 0.0;
    for (size_t i = 0; i < N; ++i) {
        minSpd = std::min(minSpd, v_profile[i]);
        maxSpd = std::max(maxSpd, v_profile[i]);
        lapT += finalProps.ds[i] / std::max(v_profile[i], 0.5);
    }

    double minClearance = 999.0;
    for (size_t i = 0; i < N; ++i) {
        double d = barrierTree.queryNearest(finalRacingLine[i]);
        if (d < minClearance) minClearance = d;
    }

    auto tEnd = std::chrono::high_resolution_clock::now();
    double solveUs = std::chrono::duration<double, std::micro>(tEnd - tStart).count();
    double solveMs = solveUs / 1000.0;

    RacingLineResultCPP result;
    result.racingLine = finalRacingLine;
    result.centerline = centerline;
    result.alpha = alpha;
    result.velocities = v_profile;
    result.minBounds = minBounds;
    result.maxBounds = maxBounds;
    result.minApexSpeed = minSpd;
    result.topSpeed = maxSpd;
    result.lapTime = lapT;
    result.minPhysicalClearanceCm = minClearance * 100.0;
    result.solveTimeMs = solveMs;
    result.solveTimeUs = solveUs;

    return result;
}
