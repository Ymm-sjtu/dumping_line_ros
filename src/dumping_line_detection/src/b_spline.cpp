#include "dumping_line_detection/b_spline.h"

Bspline::Bspline(int _k, int _type, vector<Point> _p)
{
	k = _k;
	n = _p.size() - 1;
	if (k > n + 1 || _p.empty())//k必需<=n+1， 不能一个控制点都没有
	{
		cout << "error!" << endl;
		system("pause");
		exit(0);
	}

	type = _type;
	p = _p;

	double u_tmp = 0.0;
	u.push_back(u_tmp);//先把0.0存入

	if (type == uniform)//均匀
	{
		double dis_u = 1.0 / (k + n);
		for (int i = 1; i < n + k + 1; i++) //n + k + 1个分段
		{
			u_tmp += dis_u;
			u.push_back(u_tmp);
		}
	}
	else if (type == quniform)//准均匀
	{
		int j = 3;//重复度
		double dis_u = 1.0 / (k + n - (j - 1) * 2);
		for (int i = 1; i < j; i++)
		{
			u.push_back(u_tmp);
		}
		for (int i = j; i < n + k - j + 2; i++)
		{
			u_tmp += dis_u;
			u.push_back(u_tmp);
		}
		for (int i = n + k - j + 2; i < n + k + 1; i++)//n + k + 1个分段
		{
			u.push_back(u_tmp);
		}
	}

	// cout << "阶数：" << k << ", 控制点数：" << n + 1 << endl;
	// cout << "delta_u= " << delta_u << ", u的序列为:";
	// for (long unsigned int i = 0; i < u.size(); i++)
	// {
	// 	cout << u[i] << ", ";
	// }
	// cout << endl;

	uBegin = u[k - 1];
	uEnd = u[n + 1];//计算u的区间
	// cout << "uBegin= " << uBegin << ", uEnd= " << uEnd << endl;
}

Bspline::~Bspline()
{
	p.clear();
	u.clear();
	pTrack.clear();
}

void Bspline::delay(int time) //延时函数，单位ms
{
	clock_t  now = clock();
	while (clock() - now < time)
	{

	}
}

double Bspline::BsplineBfunc(int i, int k, double uu)//计算每个u和每个i对应的B样条
{
	//cout << "****************i= " << i << endl;
	/*if (i == n + 1)
	{
		return 0.0;//防止越界
	}*/

	double Bfunc = 0.0;

	if (k == 1)//递归退出的条件
	{
		if (u[i] <= uu && uu < u[i + 1])
		{
			Bfunc = 1.0;
		}
		else
		{
			Bfunc = 0.0;
		}
	}
	else if (k >= 2)
	{
		double A = 0.0;
		double B = 0.0;

		if (u[i + k - 1] - u[i] == 0.0)
		{
			//cout << "A = 0.0; u[i+k-1]= " << u[i + k - 1] << ", u[i]= " << u[i] << endl;
			A = 0.0;//约定分母为0时，整个除式为0
		}
		else
		{
			A = (uu - u[i]) / (u[i + k - 1] - u[i]);
				
			/*if (A <= 0.0)
			{
				cout << "A < 0.0; A= " << A << ", uu= " << uu << ", u[i]= " << u[i] << ", u[i + k - 1]= " << u[i + k - 1] << ", i= " << i << ", k= " << k << endl;
			}*/
		}

		if (u[i + k] - u[i + 1] == 0.0)
		{
			//cout << "B = 0.0; u[i + k]= " << u[i + k] << ", u[i + 1] " << u[i + 1] << endl;
			B = 0.0;//约定分母为0时，整个除式为0
		}
		else
		{
			B = (u[i + k] - uu) / (u[i + k] - u[i + 1]);

			/*if (B <= 0.0)
			{
				cout << "B < 0.0; B= " << B << ", uu= " << uu << ", u[i]= " << u[i] << ", u[i + k]= " << u[i + k] << ", u[i + 1]= " << u[i + 1] << ", i= " << i << ", k= " << k << endl;
			}*/
		}

		Bfunc = A * BsplineBfunc(i, k - 1, uu) + B * BsplineBfunc(i + 1, k - 1, uu);//递归
	}

	//cout << "Bfunc= " << Bfunc << endl;
	return Bfunc;
}

void Bspline::creatBspline()//计算整个的B样条
{
	for (long unsigned int i = 1; i < p.size(); i++)
		total_dist_ += sqrt(pow(p.at(i).x-p.at(i-1).x,2) + pow(p.at(i).y-p.at(i-1).y,2));
	double number = total_dist_/resolution_;
	delta_u = 1/number;
	// std::cout << "total_dist_ " << total_dist_ <<std::endl;
	// std::cout << "number " << number <<std::endl;
	for (double uu = uBegin; uu <= uEnd; uu += delta_u)//u的循环放外层，对应每个u，去遍历所有控制点
	{
		Point Pu = { 0.0, 0.0 };//每轮循环初始化
		for (int i = 0; i < n + 1; i++)//i从0到n，每个控制点
		{
			double xtmp = p[i].x;
			double ytmp = p[i].y;
			//  if (i < p.size()-1)
			//  	delta_u = adaptiveCalDeltaU(xtmp, ytmp, p.at(i+1).x, p.at(i+1).y, number);
			double BfuncTmp = BsplineBfunc(i, k, uu);
			Pu.x += xtmp * BfuncTmp;
			Pu.y += ytmp * BfuncTmp;//累加
			// std::cout << "delta_u " << delta_u <<std::endl;
		}
		pTrack.push_back(Pu);//轨迹点
	}

	// cout << "track point: " << endl;
	// for (auto it = pTrack.begin(); it != pTrack.end(); it++)
	// {
	// 	cout << "(" << it->x << ", " << it->y << ") ";
	// }
	// cout << endl;
}

double Bspline::adaptiveCalDeltaU(double x_now, double y_now, double x_next, double y_next, double num){
	double dist = sqrt(pow(x_now-x_next,2) + pow(y_now-y_next,2));
	double this_seg_num = dist/total_dist_* num;
	return 1/this_seg_num;
}

bSplinePlanning::bSplinePlanning(const int _scale, const int _type, const nav_msgs::OccupancyGrid _gridPoints, const int _lower_limit, const int _upper_limit)
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

	getOrigiPointsFromGridPoints(_gridPoints, _upper_limit, _lower_limit);
	if(origiPoints.size() >= 3 )
	{	
		Bspline bSpliner(_scale, _type, origiPoints);
		bSpliner.creatBspline();
		trackPoints = bSpliner.pTrack;
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

std::vector<Point> bSplinePlanning::sortPointsByOrder(const std::vector<Point>& points, const std::vector<int>& order) {
    std::vector<Point> sortedPoints(order.size());
    for (size_t i = 0; i < order.size(); ++i) {
        sortedPoints[i] = points[order[i]];  // 根据给定顺序重新排列点
    }
    return sortedPoints;
}

void bSplinePlanning::getOrigiPointsFromGridPoints(const nav_msgs::OccupancyGrid _gridPoints, int _upper_limit, int _lower_limit)
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
	
}





