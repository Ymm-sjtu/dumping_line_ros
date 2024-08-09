#pragma once
#include <cmath>
#include <ctime>
#include <vector>

#include "dumping_line_detection/TSPSolver.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/node_handle.h"
#include "visualization_msgs/Marker.h"

class PointProcess
{
	public:
    	std::vector<Point> trackPoints;
		std::vector<geometry_msgs::Point> trackPointsVec;
		PointProcess(const int _scale, const int _type, const nav_msgs::OccupancyGrid _grid, const int _low_limit, const int _up_limit, const int _min_left_index, const int _min_right_index);
		// bSplinePlanning(const int _scale, const int _type, const std::vector<Point> _points, int _low_limit, int _up_limit);
		~PointProcess(){};

		visualization_msgs::Marker marker_line;
		nav_msgs::OccupancyGrid gridAfterDfs;
		
	private:
		int scale = 0;
		int type = 0;
		int low_limit = 0;
		int up_limit = 0;
		int simple_sample_scale = 1;
		nav_msgs::OccupancyGrid origiGrid;
		std::vector<Point> origiPoints;//初始点集

		ros::NodeHandle nh;
		// std::vector<Point> sampledPoints;//稀疏采样后的点集
	
		nav_msgs::OccupancyGrid maxAreaOccupancyGrid(const nav_msgs::OccupancyGrid& grid);
		int dfs(std::vector<int8_t>& gridData, int x, int y, int width, int height, int index, std::vector<std::pair<int, int>>& islandIndices);
		std::vector<Point> sortPointsByOrder(const std::vector<Point>& points, const std::vector<int>& order);
		void getOrigiPointsFromGridPoints(const nav_msgs::OccupancyGrid _gridPoints, const int _upper_limit, const int _lower_limit, const int _min_left_index, const int _min_right_index);
		double euclideanDistance(const Point& p1, const Point& p2);
		size_t findClosestPointIndex(const std::vector<Point>& points, const Point& givenPoint);
		void cutGridPoints(const nav_msgs::OccupancyGrid _gridPoints, const int min_left_index, const int min_right_index);



};// end class