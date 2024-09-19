#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <vector>
#include <string>

struct Point {
    double x = 0.0, y = 0.0 , z = 0.0;
};

class TSPSolver {
public:
    TSPSolver();
    void setPoints(const std::vector<Point>& newPoints);
    std::pair<double, std::vector<Point>> solveGreedyTSPWithStartPoint(const Point& start_point, const std::vector<Point>& points);

private:
    std::vector<Point> points;
    // 距离矩阵
    std::vector<std::vector<double>> distance_matrix;
    double distance(const Point& p1, const Point& p2);
    std::pair<double, std::vector<Point>> greedyOpenTSP(const Point& start_point);
    // 从缓存的距离矩阵中获取两点间的距离
    double getDistanceFromMatrix(int i, int j);
};

#endif // TSP_SOLVER_H
