#include <vector>
#include <cmath>
#include "dumping_line_detection/TSPSolver.h"

using namespace std;

class Bezier {
public:
    Bezier(vector<Point> _p);
    ~Bezier();
    void createBezierCurve();
    vector<Point> getTrackPoints() const { return pTrack; }

private:
    vector<Point> p;        // 控制点
    vector<Point> pTrack;   // 曲线上的点
    double resolution_ = 0.2;     // 采样分辨率
    double total_dist_;     // 总距离
    double delta_u;         // 每个参数增量

    double binomialCoefficient(int n, int k);
    Point interpolate(double u);
    double calculateDistance(Point a, Point b);
};
