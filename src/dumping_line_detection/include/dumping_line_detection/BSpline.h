#pragma once
#include <cmath>
#include <ctime>
#include <vector>

#include "dumping_line_detection/TSPSolver.h"

// #include "geometry_msgs/Point.h"

using namespace std;

constexpr auto Swidth = 1000;
constexpr auto Sheight = 1200;
constexpr auto deltaTIME = 50;

enum Type//B样条类型
{
	uniform,//均匀
	quniform//准均匀
};

// class Point//点
// {
// public:
// 	double x;
// 	double y;
// };

class Bspline //B样条曲线
{
public:
	Bspline(int _k, int _type, vector<Point> _p);
	~Bspline();
	double BsplineBfunc(int i, int k, double uu);//计算每个u和每个i对应的B样条
	void creatBspline();//计算整个的B样条


public:
	int k;//阶数
	int n;//控制点数-1
	int type; //B样条类型
	vector<double> u;//自变量
	double delta_u = 0.05;//自变量间隔
	double uBegin;
	double uEnd;
	vector<Point> p;//控制点
	vector<Point> pTrack;//轨迹点
	bool bDelayShow = true;//是否显示曲线生成的过程
	double total_dist_ = 0; //控制点总距离
	double resolution_ = 0.2; //0.2m一个点

private:
	double adaptiveCalDeltaU(double x_now, double y_now, double x_last, double y_last, double num);
};
