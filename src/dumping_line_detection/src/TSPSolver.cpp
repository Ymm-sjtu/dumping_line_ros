#include "dumping_line_detection/TSPSolver.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <iostream>
#include <vector>

TSPSolver::TSPSolver() {}

void TSPSolver::setPoints(const std::vector<Point>& newPoints) {
    points = newPoints;
    // 初始化距离矩阵
    int n = points.size();
    distance_matrix.resize(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            distance_matrix[i][j] = distance(points[i], points[j]);
            distance_matrix[j][i] = distance_matrix[i][j];  // 对称距离矩阵
        }
    }
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
    setPoints(loadedPoints);  // 设置点并计算距离矩阵
    return points;
}

double TSPSolver::distance(const Point& p1, const Point& p2) {
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

double TSPSolver::getDistanceFromMatrix(int i, int j) {
    return distance_matrix[i][j];
}

std::pair<double, std::vector<Point>> TSPSolver::greedyOpenTSP(const Point& start_point) {
    int n = points.size();
    std::vector<bool> visited(n, false);
    std::vector<Point> path;

    // 开始贪婪算法，第一步从start_point开始
    int current_index = -1;
    double total_distance = 0.0;

    // 找到距离 start_point 最近的点，作为路径的第一个点
    double min_distance_to_start = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        double dist = distance(points[i], start_point);
        if (dist < min_distance_to_start) {
            min_distance_to_start = dist;
            current_index = i;
        }
    }

    // 加入路径并标记访问过的点
    path.push_back({points[current_index].x, points[current_index].y});
    visited[current_index] = true;
    total_distance += min_distance_to_start;  // 加上 start_point 到第一个点的距离

    // 开始从当前点寻找下一个最近的点
    while (path.size() < n) {
        int next_index = -1;
        double min_distance = std::numeric_limits<double>::infinity();
        for (int i = 0; i < n; ++i) {
            if (!visited[i]) {
                double dist = getDistanceFromMatrix(current_index, i);  // 使用缓存的距离
                if (dist < min_distance) {
                    min_distance = dist;
                    next_index = i;
                }
            }
        }

        visited[next_index] = true;
        path.push_back({points[next_index].x, points[next_index].y});
        total_distance += min_distance;
        current_index = next_index;
    }

    return {total_distance, path};
}


std::pair<double, std::vector<Point>> TSPSolver::solveGreedyTSPWithStartPoint(const Point& start_point, const std::vector<Point>& points) {
    setPoints(points);  // 设置点集并计算距离矩阵

    // 直接调用贪婪算法，以指定的起点开始计算
    auto result = greedyOpenTSP(start_point);

    return result;  // 返回总距离和路径
}
