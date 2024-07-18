#include "dumping_line_detection/TSPSolver.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <iostream>

TSPSolver::TSPSolver() {}

void TSPSolver::setPoints(const std::vector<Point>& newPoints) {
    points = newPoints;
}

std::vector<Point> TSPSolver::readPointsFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    std::vector<Point> loadedPoints;
    if (file.is_open()) {
        while (getline(file, line)) {
            std::istringstream iss(line);
            std::string part;
            std::vector<std::string> parts;
            while (iss >> part) {
                parts.push_back(part);
            }
            if (parts.size() >= 3) {
                try {
                    double x = std::stod(parts[1]);
                    double y = std::stod(parts[2]);
                    loadedPoints.push_back({x, y});
                } catch (std::exception& e) {
                    std::cout << "转换坐标到浮点数时出错，在这一行：" << line << std::endl;
                }
            }
        }
        file.close();
    } else {
        std::cout << "无法打开文件：" << filename << std::endl;
    }
    points = loadedPoints; // Update the member variable
    return points;
}

double TSPSolver::distance(const Point& p1, const Point& p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

std::pair<double, std::vector<int>> TSPSolver::greedyOpenTSP(int start_index) {
    int n = points.size();
    std::vector<bool> visited(n, false);
    std::vector<int> path;
    path.push_back(start_index);
    visited[start_index] = true;
    int current_point = start_index;
    double total_distance = 0.0;

    while (path.size() < n) {
        int next_point = -1;
        double min_distance = std::numeric_limits<double>::infinity();
        for (int i = 0; i < n; ++i) {
            if (!visited[i]) {
                double dist = distance(points[current_point], points[i]);
                if (dist < min_distance) {
                    min_distance = dist;
                    next_point = i;
                }
            }
        }
        visited[next_point] = true;
        path.push_back(next_point);
        total_distance += min_distance;
        current_point = next_point;
    }

    return {total_distance, path};
}

std::pair<double, std::vector<int>> TSPSolver::findBestGreedyStart() {
    int n = points.size();
    double best_distance = std::numeric_limits<double>::infinity();
    std::vector<int> best_path;

    for (int start_index = 0; start_index < n; ++start_index) {
        auto result = greedyOpenTSP(start_index);
        if (result.first < best_distance) {
            best_distance = result.first;
            best_path = result.second;
        }
    }

    return {best_distance, best_path};
}
