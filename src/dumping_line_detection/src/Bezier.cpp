#include <iostream>
#include "dumping_line_detection/Bezier.h"

using namespace std;


Bezier::Bezier(vector<Point> _p) : p(_p) {
    if (_p.size() < 2) {
        cout << "Error: At least two control points are required!" << endl;
        exit(0);
    }
}

Bezier::~Bezier() {
    p.clear();
    pTrack.clear();
}

double Bezier::binomialCoefficient(int n, int k) {
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;
    double res = 1;
    for (int i = 0; i < k; i++) {
        res *= (n - i);
        res /= (i + 1);
    }
    return res;
}

Point Bezier::interpolate(double u) {
    int n = p.size() - 1;
    Point result = {0.0, 0.0};
    for (int i = 0; i <= n; i++) {
        double binom = binomialCoefficient(n, i);
        double weight = binom * pow(1 - u, n - i) * pow(u, i);
        result.x += weight * p[i].x;
        result.y += weight * p[i].y;
    }
    return result;
}

double Bezier::calculateDistance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void Bezier::createBezierCurve() {
    // Calculate total distance
    total_dist_ = 0.0;
    Point prev_point = p[0];
    for (size_t i = 1; i < p.size(); i++) {
        total_dist_ += calculateDistance(prev_point, p[i]);
        prev_point = p[i];
    }

    // Calculate delta_u
    double number_of_segments = total_dist_ / resolution_;
    delta_u = 1.0 / number_of_segments;

    // Generate curve points
    for (double u = 0; u <= 1.0; u += delta_u) {
        if (u > 1.0) u = 1.0;  // Ensure u doesn't exceed 1.0
        Point bezier_point = interpolate(u);
        pTrack.push_back(bezier_point);
    }
}
