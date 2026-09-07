#pragma once
#include <vector>
#include <cmath>
#include "kdtree2d.hpp"

struct SplineProperties2D {
    std::vector<Point2D> points;
    std::vector<Point2D> tangents;
    std::vector<Point2D> normals;
    std::vector<double> curvature;
    std::vector<double> ds;
    double totalLength = 0.0;
};

class PeriodicCubicSpline2D {
public:
    static SplineProperties2D computeProperties(const std::vector<Point2D>& pts) {
        size_t N = pts.size();
        SplineProperties2D props;
        props.points = pts;
        props.tangents.resize(N);
        props.normals.resize(N);
        props.curvature.resize(N);
        props.ds.resize(N);

        props.totalLength = 0.0;
        for (size_t i = 0; i < N; ++i) {
            size_t iNext = (i + 1) % N;
            double dx = pts[iNext].x - pts[i].x;
            double dy = pts[iNext].y - pts[i].y;
            double d = std::hypot(dx, dy);
            props.ds[i] = (d > 1e-6) ? d : 1e-6;
            props.totalLength += props.ds[i];
        }

        // Tangents and Normals via central differences
        for (size_t i = 0; i < N; ++i) {
            size_t iPrev = (i + N - 1) % N;
            size_t iNext = (i + 1) % N;

            double dx = pts[iNext].x - pts[iPrev].x;
            double dy = pts[iNext].y - pts[iPrev].y;
            double len = std::hypot(dx, dy);
            if (len < 1e-6) len = 1e-6;

            double tx = dx / len;
            double ty = dy / len;
            props.tangents[i] = Point2D(tx, ty);
            props.normals[i] = Point2D(-ty, tx); // Leftward normal
        }

        // Curvature kappa = (x' y'' - y' x'') / (x'^2 + y'^2)^(1.5)
        for (size_t i = 0; i < N; ++i) {
            size_t iPrev = (i + N - 1) % N;
            size_t iNext = (i + 1) % N;

            double ds_p = props.ds[iPrev];
            double ds_c = props.ds[i];
            double ds_avg = 0.5 * (ds_p + ds_c);

            double ddx = (pts[iNext].x - 2.0 * pts[i].x + pts[iPrev].x) / (ds_avg * ds_avg);
            double ddy = (pts[iNext].y - 2.0 * pts[i].y + pts[iPrev].y) / (ds_avg * ds_avg);

            double dx = (pts[iNext].x - pts[iPrev].x) / (2.0 * ds_avg);
            double dy = (pts[iNext].y - pts[iPrev].y) / (2.0 * ds_avg);

            double speedSq = dx * dx + dy * dy;
            double speed = std::sqrt(speedSq);
            if (speed < 1e-4) speed = 1e-4;

            double kappa = (dx * ddy - dy * ddx) / (speed * speed * speed);
            props.curvature[i] = kappa;
        }

        return props;
    }

    static std::vector<Point2D> resampleUniform(const std::vector<Point2D>& pts, size_t numOut) {
        size_t N = pts.size();
        if (N < 3) return pts;

        std::vector<double> cumDist(N + 1, 0.0);
        for (size_t i = 0; i < N; ++i) {
            size_t iNext = (i + 1) % N;
            cumDist[i + 1] = cumDist[i] + std::hypot(pts[iNext].x - pts[i].x, pts[iNext].y - pts[i].y);
        }
        double totalLen = cumDist.back();

        std::vector<Point2D> out(numOut);
        for (size_t j = 0; j < numOut; ++j) {
            double targetDist = (static_cast<double>(j) / numOut) * totalLen;

            // Find segment
            size_t seg = 0;
            while (seg < N && cumDist[seg + 1] < targetDist) {
                seg++;
            }
            if (seg >= N) seg = N - 1;

            double segLen = cumDist[seg + 1] - cumDist[seg];
            double t = (segLen > 1e-6) ? (targetDist - cumDist[seg]) / segLen : 0.0;
            t = std::clamp(t, 0.0, 1.0);

            size_t i0 = seg;
            size_t i1 = (seg + 1) % N;
            out[j] = Point2D(
                (1.0 - t) * pts[i0].x + t * pts[i1].x,
                (1.0 - t) * pts[i0].y + t * pts[i1].y
            );
        }
        return out;
    }
};
