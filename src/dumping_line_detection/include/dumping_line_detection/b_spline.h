#pragma once
#include "ros/ros.h"
#include <iostream>
#include <cmath>
#include <ctime>
#include <vector>

#include "dumping_line_detection/TSPSolver.h"
#include "std_msgs/Int32MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
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
	void delay(int time); //延时函数，单位ms
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

class bSplinePlanning
{
	public:
    	std::vector<Point> trackPoints;
		std::vector<geometry_msgs::Point> trackPointsVec;
		bSplinePlanning(const int _scale, const int _type, const nav_msgs::OccupancyGrid _grid, int _low_limit, int _up_limit);
		// bSplinePlanning(const int _scale, const int _type, const std::vector<Point> _points, int _low_limit, int _up_limit);
		~bSplinePlanning(){};

		visualization_msgs::Marker marker_line;
		nav_msgs::OccupancyGrid gridForVisualization;
		
	private:
		int scale = 0;
		int type = 0;
		int low_limit = 0;
		int up_limit = 0;
		int simple_sample_scale = 3;
		nav_msgs::OccupancyGrid origiGrid;
		std::vector<Point> origiPoints;//初始点集

		ros::NodeHandle nh;
		// std::vector<Point> sampledPoints;//稀疏采样后的点集
	
		std::vector<Point> sortPointsByOrder(const std::vector<Point>& points, const std::vector<int>& order);
		void getOrigiPointsFromGridPoints(const nav_msgs::OccupancyGrid , const int , const int );

};// end class