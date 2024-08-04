#include "dumping_line_detection/PointProcess.h"
#include "dumping_line_detection/BSpline.h"
#include "dumping_line_detection/Bezier.h"

PointProcess::PointProcess(const int _scale, const int _type, const nav_msgs::OccupancyGrid _gridPoints, const int _lower_limit, const int _upper_limit, const int _min_left_index, const int _min_right_index)
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

	getOrigiPointsFromGridPoints(_gridPoints, _upper_limit, _lower_limit, _min_left_index, _min_right_index);
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
	for(int j = 0; j < trackPoints.size(); j++)
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

std::vector<Point> PointProcess::sortPointsByOrder(const std::vector<Point>& points, const std::vector<int>& order) {
    std::vector<Point> sortedPoints(order.size());
    for (size_t i = 0; i < order.size(); ++i) {
        sortedPoints[i] = points[order[i]];  // 根据给定顺序重新排列点
    }
    return sortedPoints;
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

	std::cout << "======================" << std::endl;
	std::cout << "left_index " << left_index << " right_index " << right_index << std::endl;

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

void PointProcess::getOrigiPointsFromGridPoints(const nav_msgs::OccupancyGrid _gridPoints, const int _upper_limit, const int _lower_limit, const int _min_left_index, const int _min_right_index)
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
	
	gridForVisualization.header = _gridPoints.header;
	gridForVisualization.info = _gridPoints.info;
	gridForVisualization.data.clear();
	gridForVisualization.data.resize(_gridPoints.data.size(), 0);
	int num = 0;
	for(int i = 0; i < _gridPoints.data.size(); i++)
	{
		_data = _gridPoints.data.at(i);
		if(_data > _lower_limit && _data <= _upper_limit )
		{
			gridForVisualization.data.at(i) = -100;
			loop++;
			if(loop % simple_sample_scale == 0)
			{
				_point.x = (i % _width)*_resolution+0.5*_resolution + _x_0;
				_point.y = static_cast<int>(i/_width)*_resolution+0.5*_resolution + _y_0;
				origiPoints.push_back(_point);
				loop = 0;
				std::cout << num << " " << _point.x << " " << _point.y << std::endl;
				num++;
			}
		}
	}
	TSPSolver solver;
	solver.setPoints(origiPoints);
    auto result = solver.findBestGreedyStart();
	origiPoints = sortPointsByOrder(origiPoints, result.second);

	num = 0;
	for (const auto& p : origiPoints) {
        std::cout << num << " " << p.x << " " << p.y << std::endl;
		num++;
    }
	// std::cout << "========================" << std::endl;
	// std::cout << "min_left_index " << _min_left_index << " min_right_index " << _min_right_index << std::endl;
	cutGridPoints(_gridPoints, _min_left_index, _min_right_index);
	
}