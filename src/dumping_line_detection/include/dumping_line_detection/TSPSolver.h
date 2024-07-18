#ifndef TSP_SOLVER_H
#define TSP_SOLVER_H

#include <vector>
#include <string>

struct Point {
    double x, y;
};

class TSPSolver {
public:
    TSPSolver();
    void setPoints(const std::vector<Point>& points);
    std::vector<Point> readPointsFromFile(const std::string& filename);
    std::pair<double, std::vector<int>> findBestGreedyStart();

private:
    std::vector<Point> points;
    double distance(const Point& p1, const Point& p2);
    std::pair<double, std::vector<int>> greedyOpenTSP(int start_index);
};

#endif // TSP_SOLVER_H
