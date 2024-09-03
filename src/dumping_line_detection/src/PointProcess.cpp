#include "dumping_line_detection/PointProcess.h"
// #include "dumping_line_detection/BSpline.h"
#include "dumping_line_detection/Bezier.h"
#include <iostream>
#include <jsoncpp/json/reader.h>
#include <fstream>

PointProcess::PointProcess(const int _scale, const int _type, const nav_msgs::OccupancyGrid _gridPoints, const int _lower_limit, const int _upper_limit, const int _first_index, const int _second_index)
{
	scale = _scale;
	type = _type;
	origiGrid = _gridPoints;
	low_limit = _lower_limit;
	up_limit = _upper_limit;
	// ros::param::get("simple_sample_scale", simple_sample_scale);

	//Visualization Initializaion
	marker_line.points.clear();
	marker_line.header.frame_id = "map";
	marker_line.header.stamp = ros::Time::now();
	marker_line.ns = "bspline";
	marker_line.id = 4;
	marker_line.type = visualization_msgs::Marker::LINE_STRIP;
	marker_line.action = visualization_msgs::Marker::ADD;
	marker_line.scale.x = 0.1;
	// marker_lines.scale.y = 10;
	// marker_lines.scale.z = 10;
	marker_line.color.r = 1.0f;
	marker_line.color.g = 0.4f;
	marker_line.color.b = 0.22f;
	marker_line.color.a = 1;

	ros::Rate rsleep(6);

	getOrigiPointsFromGridPoints(_gridPoints, _upper_limit, _lower_limit, _first_index, _second_index);
	if(origiPoints.size() >= 3 )
	// {	
	// 	Bspline bSpliner(_scale, _type, origiPoints);
	// 	bSpliner.creatBspline();
	// 	trackPoints = bSpliner.pTrack;
	// }
	{	
		Bezier Bezier(origiPoints);
		Bezier.createBezierCurve();
		trackPoints = Bezier.getTrackPoints();
	}
	geometry_msgs::Point temp_point;
	trackPointsVec.clear();

	//todo：后续优化
	for(int j = 10; j < trackPoints.size()-10; j++)
	{
		temp_point.x = trackPoints.at(j).x;
		temp_point.y = trackPoints.at(j).y;
		temp_point.z = 0;
		trackPointsVec.push_back(temp_point);
		marker_line.points.push_back(temp_point);
	}
	// ROS_WARN("Sent marker b spline.");
	// pub_marker_lines.publish(marker_lines);
	// rsleep.sleep();
}

// 计算两点之间的欧几里得距离
double PointProcess::euclideanDistance(const Point& p1, const Point& p2) {
	return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 找到距离给定点最近的点的索引
size_t PointProcess::findClosestPointIndex(const std::vector<Point>& points, const Point& givenPoint) {
    size_t closestIndex = 0;
    double minDistance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < points.size(); ++i) {
        double distance = euclideanDistance(points[i], givenPoint);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }

    return closestIndex;
}
void PointProcess::cutGridPoints(const nav_msgs::OccupancyGrid _gridPoints, const int min_left_index, const int min_right_index)
{
	int _width = _gridPoints.info.width;
	float _resolution = _gridPoints.info.resolution;
	float _x_0 = _gridPoints.info.origin.position.x;
	float _y_0 = _gridPoints.info.origin.position.y;

	float left_x = (min_left_index % _width) * _resolution + 0.5 * _resolution + _x_0;
    float left_y = static_cast<int>(min_left_index / _width) * _resolution+ 0.5 * _resolution + _y_0;
    float right_x = (min_right_index % _width) * _resolution + 0.5 * _resolution + _x_0;
    float right_y = static_cast<int>(min_right_index / _width) * _resolution+ 0.5 * _resolution + _y_0;

	Point left_point = {left_x, left_y};
	Point right_point = {right_x, right_y};
	size_t left_index = findClosestPointIndex(origiPoints, left_point);
	size_t right_index = findClosestPointIndex(origiPoints, right_point);

	std::cout << "first_index = " << left_index << "; second_index = " << right_index << std::endl;

	//获得origiPoints中min_left和min_right之间的点，还是按照顺序，并且赋给origiPoints
	std::vector<Point> tempPoints;
	if(left_index > right_index)
	{
		for(size_t i = right_index; i <= left_index; i++)
		{
			tempPoints.push_back(origiPoints.at(i));
		}
	}
	else
	{
		for(size_t i = left_index; i <= right_index; i++)
		{
			tempPoints.push_back(origiPoints.at(i));
		}
	}
	origiPoints = tempPoints;
}

 nav_msgs::OccupancyGrid PointProcess::maxAreaOccupancyGrid(const nav_msgs::OccupancyGrid& grid) {
	if (grid.data.empty()) return grid;

	int width = grid.info.width;
	int height = grid.info.height;
	int maxArea = 0;
	vector<pair<int, int>> maxIslandIndices;

	// Create a copy of the data to manipulate
	 vector<int8_t> gridData = grid.data;
	int islandIndex = 2; // Start indexing islands from 2

	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			if (gridData[y * width + x] == 100) {
				vector<pair<int, int>> currentIslandIndices;
				int area = dfs(gridData, x, y, width, height, islandIndex, currentIslandIndices);
				if (area > maxArea) {
					maxArea = area;
					maxIslandIndices = currentIslandIndices;
				}
				++islandIndex; // Increment the index for the next island
			}
		}
	}

	// Create the result OccupancyGrid with only the maximum island
	nav_msgs::OccupancyGrid resultGrid = grid;
	resultGrid.data.assign(grid.data.size(), 0); // Initialize all cells to 0

	for (const auto& index : maxIslandIndices) {
		resultGrid.data[index.second * width + index.first] = 100;
	}

	return resultGrid;
}

int PointProcess::dfs(vector<int8_t>& gridData, int x, int y, int width, int height, int index, vector<pair<int, int>>& islandIndices) {
	if (x < 0 || x >= width || y < 0 || y >= height || gridData[y * width + x] != 100)
		return 0;

	gridData[y * width + x] = index; // Mark the cell with the current island index
	islandIndices.push_back({x, y}); // Record the index of the current cell
	int area = 1;

	// Explore all 8 directions
	const vector<int> directions = {-1, 0, 1};
	for (int dx : directions) {
		for (int dy : directions) {
			if (dx == 0 && dy == 0) continue; // Skip the cell itself
			area += dfs(gridData, x + dx, y + dy, width, height, index, islandIndices);
		}
	}

	return area;
}

double PointProcess::getDistanceToLine(const Point& p, const Point& linePoint, double direction) 
{
    double dx = std::cos(direction);
    double dy = std::sin(direction);

    return std::abs(dy * (p.x - linePoint.x) - dx * (p.y - linePoint.y)) / std::sqrt(dx * dx + dy * dy);
}

// 判断点在直线的哪一侧
int PointProcess::getSideOfLine(const Point& p, const Point& linePoint, double direction) {
    double dx = std::cos(direction);
    double dy = std::sin(direction);

    // 使用直线方程符号判断侧边
    double value = dy * (p.x - linePoint.x) - dx * (p.y - linePoint.y);
    
    if (value > 0) return 1;   // 点在直线一侧
    if (value < 0) return -1;  // 点在直线另一侧
    return 0;  // 点在直线上
}


bool PointProcess::findClosestPointAndClassify(const std::vector<Point>& _points)
{	
	if (_points.empty()) return false;

	double minDistance = std::numeric_limits<double>::max();
    bool found = false;

	// 读取 JSON 文件
	std::ifstream json_file("/home/ymm/dumping_line_ros/config.json");
    Json::Reader reader;
    Json::Value root;

    if (!reader.parse(json_file, root, false)) {
        ROS_ERROR("Failed to parse the JSON file.");
        return false;
    }
	
	// 直线的起点（由 Pose 中的 position 决定）
    Point posePosition;
    posePosition.x = root["pose"]["position"]["x"].asDouble();
    posePosition.y = root["pose"]["position"]["y"].asDouble();

	// 直线的方向（由 Pose 中的 orientation 决定）
	geometry_msgs::Pose initial_pose;
    initial_pose.orientation.z = root["pose"]["orientation"]["z"].asDouble();
    initial_pose.orientation.w = root["pose"]["orientation"]["w"].asDouble();
	double yaw = atan2(initial_pose.orientation.z, initial_pose.orientation.w) * 2;

    // 遍历所有点，进行最近点判断以及分类
    for (const auto& point : _points) {
        // 计算到直线的距离
        double distance = getDistanceToLine(point, posePosition, yaw);
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = point;
            found = true;
        }

        // 判断点位于哪一侧
        int side = getSideOfLine(point, posePosition, yaw);
        if (side > 0) {
            sideA.push_back(point);  // 一侧
        } else if (side < 0) {
            sideB.push_back(point);  // 另一侧
        }
        // 如果 side == 0，则点在直线上，忽略
    }
	std::cout << "Closest Point: (" << closestPoint.x << ", " << closestPoint.y << ")" << std::endl;
	return found;
}

std::pair<double, std::vector<Point>> PointProcess::combineResults(const std::pair<double, std::vector<Point>>& resultA, const std::pair<double, std::vector<Point>>& resultB) {
    // 复制 resultA 和 resultB 的路径
    std::vector<Point> pathA = resultA.second;
    std::vector<Point> pathB = resultB.second;

    // 反转 pathA
    std::reverse(pathA.begin(), pathA.end());

    // 组合两个路径
    pathA.insert(pathA.end(), pathB.begin(), pathB.end());

    // 计算总距离，这里假设你需要计算新的总距离
    double total_distance = resultA.first + resultB.first;  // 根据实际情况调整计算方式

    return {total_distance, pathA};
}

void PointProcess::getOrigiPointsFromGridPoints(const nav_msgs::OccupancyGrid _gridPoints, const int _upper_limit, const int _lower_limit, const int _first_index, const int _second_index)
{
	ROS_ASSERT(_upper_limit <=100 && _lower_limit >= 0);
	origiPoints.clear();
	Point _point;
	int _data;
	int _width = _gridPoints.info.width;
	int _height = _gridPoints.info.height;
	float _resolution = _gridPoints.info.resolution;
	float _x_0 = _gridPoints.info.origin.position.x;
	float _y_0 = _gridPoints.info.origin.position.y;

	int loop = 0;
	
	gridAfterDfs = maxAreaOccupancyGrid(_gridPoints);
	// gridForVisualization.header = _gridPoints.header;
	// gridForVisualization.info = _gridPoints.info;
	// gridForVisualization.data.clear();
	// gridForVisualization.data.resize(_gridPoints.data.size(), 0);
	int num = 0;
	for(int i = 0; i < _gridPoints.data.size(); i++)
	{
		_data = gridAfterDfs.data.at(i);
		if(_data > _lower_limit && _data <= _upper_limit )
		{
			// gridForVisualization.data.at(i) = -100;
			loop++;
			if(loop % simple_sample_scale == 0)
			{
				_point.x = (i % _width)*_resolution+0.5*_resolution + _x_0;
				_point.y = static_cast<int>(i/_width)*_resolution+0.5*_resolution + _y_0;
				origiPoints.push_back(_point);
				loop = 0;
				// std::cout << num << " " << _point.x << " " << _point.y << std::endl;
				num++;
			}
		}
	}
	findClosestPointAndClassify(origiPoints);
	
	TSPSolver solver;
	auto resultA = solver.solveGreedyTSPWithStartPoint(closestPoint, sideA);
	auto resultB = solver.solveGreedyTSPWithStartPoint(closestPoint, sideB);
	
	origiPoints = std::move(combineResults(resultA, resultB).second);

	// num = 0;
	// for (const auto& p : origiPoints) {
    //     std::cout << num << " " << p.x << " " << p.y << std::endl;
	// 	num++;
    // }
	// std::cout << "========================" << std::endl;
	// std::cout << "min_left_index " << _min_left_index << " min_right_index " << _min_right_index << std::endl;
	cutGridPoints(_gridPoints, _first_index, _second_index);
	
}