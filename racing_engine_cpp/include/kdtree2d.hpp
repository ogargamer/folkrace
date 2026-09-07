#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

struct Point2D {
    double x;
    double y;
    Point2D() : x(0), y(0) {}
    Point2D(double _x, double _y) : x(_x), y(_y) {}
};

class KDTree2D {
private:
    struct Node {
        Point2D pt;
        int axis;
        Node* left;
        Node* right;
        Node(const Point2D& p, int a) : pt(p), axis(a), left(nullptr), right(nullptr) {}
    };

    Node* root = nullptr;
    std::vector<Point2D> points;

    Node* buildTree(std::vector<Point2D>& pts, int depth) {
        if (pts.empty()) return nullptr;
        int axis = depth % 2;
        size_t medianIdx = pts.size() / 2;

        std::nth_element(pts.begin(), pts.begin() + medianIdx, pts.end(),
            [axis](const Point2D& a, const Point2D& b) {
                return (axis == 0) ? (a.x < b.x) : (a.y < b.y);
            });

        Node* node = new Node(pts[medianIdx], axis);
        std::vector<Point2D> leftPts(pts.begin(), pts.begin() + medianIdx);
        std::vector<Point2D> rightPts(pts.begin() + medianIdx + 1, pts.end());

        node->left = buildTree(leftPts, depth + 1);
        node->right = buildTree(rightPts, depth + 1);
        return node;
    }

    void nearestQuery(Node* node, const Point2D& target, double& bestDistSq, Point2D& bestPt) const {
        if (!node) return;

        double dSq = (node->pt.x - target.x) * (node->pt.x - target.x) + 
                     (node->pt.y - target.y) * (node->pt.y - target.y);
        if (dSq < bestDistSq) {
            bestDistSq = dSq;
            bestPt = node->pt;
        }

        int axis = node->axis;
        double diff = (axis == 0) ? (target.x - node->pt.x) : (target.y - node->pt.y);

        Node* first = (diff <= 0) ? node->left : node->right;
        Node* second = (diff <= 0) ? node->right : node->left;

        nearestQuery(first, target, bestDistSq, bestPt);

        if (diff * diff < bestDistSq) {
            nearestQuery(second, target, bestDistSq, bestPt);
        }
    }

    void clearTree(Node* node) {
        if (!node) return;
        clearTree(node->left);
        clearTree(node->right);
        delete node;
    }

public:
    KDTree2D() = default;
    ~KDTree2D() { clearTree(root); }

    void build(const std::vector<Point2D>& pts) {
        clearTree(root);
        points = pts;
        root = buildTree(points, 0);
    }

    double queryNearest(const Point2D& target, Point2D* outNearest = nullptr) const {
        if (!root) return std::numeric_limits<double>::infinity();
        double bestDistSq = std::numeric_limits<double>::infinity();
        Point2D bestPt;
        nearestQuery(root, target, bestDistSq, bestPt);
        if (outNearest) *outNearest = bestPt;
        return std::sqrt(bestDistSq);
    }
};
