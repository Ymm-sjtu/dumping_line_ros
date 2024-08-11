#include <jsoncpp/json/reader.h>
#include <ros/ros.h>
#include "dumping_line_detection/convolution_core.h"
#include "dumping_line_detection/PointProcess.h"
#include "dumping_line_detection/BSpline.h"
#include <fstream>


namespace convolution_ns{
    
convolution::convolution()
{

    //Visualization Initialization
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "convolution_ns";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = 2;
    arrow.scale.y = 0.2;
    arrow.scale.z = 0.2;
    arrow.color.r = 255;
    arrow.color.g = 165;
    arrow.color.b = 0; 
    arrow.color.a = 0.9;
    arrow.lifetime = ros::Duration();
    fitted_points.header.frame_id = "map";
    fitted_points.header.stamp = ros::Time::now();
    fitted_points.ns = "convolution_points_ns";
    fitted_points.type = visualization_msgs::Marker::POINTS;
    fitted_points.action = visualization_msgs::Marker::ADD;
    fitted_points.scale.x = 0.2;
    fitted_points.scale.y = 0.2;
    fitted_points.scale.z = 0.2;
    fitted_points.color.r  = 255;
    fitted_points.color.g = 100;
    fitted_points.color.b = 60; 
    fitted_points.color.a = 0.9;
    fitted_points.lifetime = ros::Duration();

    //地图初始赋名
    grid.header.frame_id = "null";

    //sub and pub function
    sub_map = nh.subscribe<nav_msgs::OccupancyGrid>("/map_open", 1, &convolution::callbackSubGrid, this);

    pub_grid_after_cut = nh.advertise<nav_msgs::OccupancyGrid>("/map_cut", 1);
    pub_grid_after_conv = nh.advertise<nav_msgs::OccupancyGrid>("/map_convolution", 1);
    pub_fitted_points = nh.advertise<visualization_msgs::Marker>("/fitted_points", 1);
    pub_marker_b_spline = nh.advertise<visualization_msgs::Marker>("/marker_b_spline", 1);
    pub_grid_after_dfs = nh.advertise<nav_msgs::OccupancyGrid>("/map_dfs", 1);

    //test
    pub_grid_after_found = nh.advertise<nav_msgs::OccupancyGrid>("/map_found", 1);
}

// Function to read and parse the JSON file
bool convolution::parseJsonFile(const string& filename) {
    std::ifstream json_file(filename);
    Json::Reader reader;
    Json::Value root;

    if (!reader.parse(json_file, root, false)) {
        ROS_ERROR("Failed to parse the JSON file.");
        return false;
    }

    // Extract data from the JSON
    recommend_pose.position.x = root["pose"]["position"]["x"].asDouble();
    recommend_pose.position.y = root["pose"]["position"]["y"].asDouble();
    recommend_pose.position.z = root["pose"]["position"]["z"].asDouble();
    recommend_pose.orientation.x = root["pose"]["orientation"]["x"].asDouble();
    recommend_pose.orientation.y = root["pose"]["orientation"]["y"].asDouble();
    recommend_pose.orientation.z = root["pose"]["orientation"]["z"].asDouble();
    recommend_pose.orientation.w = root["pose"]["orientation"]["w"].asDouble();
    
    angle_right = root["angles"]["angle_right"].asDouble();
    angle_left = root["angles"]["angle_left"].asDouble();
    return true;
}


void convolution::main_loop()
{
    ros::Rate sleep(1);
    core.scale = 2;
    double convolutionStartTime, convolutionCostTime;
    while(ros::ok())
    {
        ros::spinOnce();//此处执行回调函数
        bool ifReadJson = parseJsonFile("/home/ymm/dumping_line_ws/config.json");
        if(grid.header.frame_id != "null" && ifSubGrid && ifReadJson)
        {
            recommend_pose.position.x -= grid.info.origin.position.x;
            recommend_pose.position.y -= grid.info.origin.position.y;
            ROS_INFO("Receive ALL : Convolution Process Start.");

            //cut the grid map
            ROS_ASSERT_MSG(cutGridMap() == true,"Please select a recommended posture again");
            pub_grid_after_cut.publish(grid_after_cut);

            vehicle_posi_index = getIndex(vehicle_position);

            grid = grid_after_cut;
        
            bool ifCleaned = getOrientedAreaInGrid(0.02);//函数前后只有grid发生了变化

            //卷积算法
            convolutionStartTime = ros::Time::now().toSec();
            calcConvolution2x2();
            convolutionCostTime = ros::Time::now().toSec() - convolutionStartTime;
            ROS_INFO("Time cost fot map convolution = %f ms", convolutionCostTime*1000);
            pub_grid_after_conv.publish(grid_after_conv);//保留挡墙内边缘

            PointProcess process(5, quniform, grid_after_conv, 0, 100 , min_left_index, min_right_index);
            pub_grid_after_dfs.publish(process.gridAfterDfs);
            pub_marker_b_spline.publish(process.marker_line);
            //排土线

            ROS_INFO("marker_line success");

            normalFitSequence fitter(process.trackPointsVec, vehicle_position, 60);//计算出拟合于线上的点集，存于数据成员poses中

            // 拟合点集可视化
            fitted_points.points.clear();
            for(int i =0; i < fitter.poses.size(); i++)
            {
                fitted_points.points.push_back(fitter.poses.at(i).position);
            }

            std::string file_name = "/home/ymm/dumping_line_ws/points.csv";  // 定义要保存的文件名
            fitter.savePointsToFile(fitted_points, file_name);  // 调用函数，保存点到文件

            ROS_INFO("Succeed in Making Sure GOUND PARK POSITION.");
                
            //程序复位
            reset();
        }
        sleep.sleep();
    }
}

/*
    ###################################
    ###
    ###               Main Process Function
    ###
    ###################################
*/

/**
 * @brief Calculates the new position based on the given pose and distance.
 * 
 * This function takes a pose and a distance as input and calculates the new position
 * by moving the pose in the direction of its orientation vector by the specified distance.
 * 
 * @param pose The input pose from which the new position will be calculated.
 * @param distance The distance by which the pose will be moved.
 * @return The new position as a geometry_msgs::Point.
 */
geometry_msgs::Point convolution::getNewPosition(const geometry_msgs::Pose& pose, double distance) 
{
    // 获取四元数
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    // 计算旋转后的方向向量
    tf2::Vector3 direction(1.0, 0.0, 0.0);  // 初始方向向量，假设沿 x 轴正方向
    direction = tf2::quatRotate(q, direction);

    // 归一化方向向量
    direction.normalize();

    // 计算新位置
    geometry_msgs::Point newPos;
    newPos.x = pose.position.x + direction.x() * distance;
    newPos.y = pose.position.y + direction.y() * distance;
    newPos.z = pose.position.z + direction.z() * distance;

    ROS_INFO("New Position: (%f, %f, %f)", newPos.x, newPos.y, newPos.z);
    return newPos;
}

// 计算旋转后的直线方程的斜率 k 和截距 b
bool convolution::computeRotatedLineEquation(const geometry_msgs::Pose& pose, double rotation_angle, double& k, double& b) {
    // 提取四元数
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    // 计算初始旋转角度 theta
    double initial_theta = 2 * atan2(q.z(), q.w());

    // 计算旋转后的角度
    double rotated_theta = initial_theta + rotation_angle;

    // 计算旋转后的方向向量
    double direction_x = cos(rotated_theta);
    double direction_y = sin(rotated_theta);

    // 检查 direction_x 是否为零
    if (std::abs(direction_x) < 1e-6) {
        // 垂直直线的情况，返回 false
        return false;
    } else {
        // 非垂直直线的情况
        k = direction_y / direction_x;
        b = pose.position.y - k * pose.position.x;
        return true;
    }
}

bool convolution::cutGridMap()
{
    
    geometry_msgs::Pose initial_pose = recommend_pose;
    geometry_msgs::Point newPos = getNewPosition(initial_pose, 5);

    //计算旋转后的直线方程
    double rotation_angle_right = angle_right * M_PI / 180.0;  // 转换为弧度
    double rotation_angle_left = angle_left * M_PI / 180.0;  // 转换为弧度

    double k_p, k_n, b_p, b_n;
    double min_left_abs = 100.0, min_right_abs = 100.0;
    double left_abs = 0.0, right_abs = 0.0;
    
    bool is_not_vertical_p = computeRotatedLineEquation(recommend_pose, rotation_angle_right, k_p, b_p); //逆时针为正
    bool is_not_vertical_n = computeRotatedLineEquation(recommend_pose, rotation_angle_left, k_n, b_n);//顺时针为负
    
    //计算位姿方向上的点，带入直线方程后的正负
    bool flag_p, flag_n;
    if (is_not_vertical_p) {
        std::cout << "逆时针旋转后的直线方程: y = " << k_p << " * x + " << b_p << std::endl;
        flag_p = ((newPos.y - k_p*newPos.x - b_p) > 0)? 1:0;
    } else {
        std::cout << "逆时针旋转后的直线方程: x = " << recommend_pose.position.x << std::endl;
        return false;
    }

    if (is_not_vertical_n) {
        std::cout << "顺时针旋转后的直线方程: y = " << k_n << " * x + " << b_n << std::endl;
        flag_n = ((newPos.y - k_n*newPos.x - b_n) > 0)? 1:0;
    } else {
        std::cout << "顺时针旋转后的直线方程: x = " << recommend_pose.position.x << std::endl;
        return false;
    }

    int width = grid_after_cut.info.width;
    int height = grid_after_cut.info.height;

    bool flag_grid_map_p, flag_grid_map_n;
    // 遍历栅格地图的每个栅格
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            // 计算栅格在数据数组中的索引
            int index = i * width + j;
            flag_grid_map_p = ((i - k_p*j - b_p) > 0)? 1:0;
            flag_grid_map_n = ((i - k_n*j - b_n) > 0)? 1:0;

            // 检查栅格的值是否满足条件
            if (flag_grid_map_p == flag_p && flag_grid_map_n == flag_n) {
                // 符合条件，保留值不变
                grid_after_cut.data[index] = grid.data[index];
            } else {
                // 不符合条件，置为0
                grid_after_cut.data[index] = 0;
            }
        }
    }

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            // 计算栅格在数据数组中的索引
            int index = i * width + j;
            //找到grid_after_cut中带入直线方程后绝对值最小的栅格点
            if(grid_after_cut.data[index] == 100)
            {
                left_abs = std::abs(i - k_n*j - b_n);
                right_abs = std::abs(i - k_p*j - b_p);
                if(left_abs < min_left_abs)
                {
                    min_left_abs = left_abs;
                    min_left_index = index;
                }
                if(right_abs < min_right_abs)
                {
                    min_right_abs = right_abs;
                    min_right_index = index;
                }
            }
        }
    }
    // int min_left_x = min_left_index % width;
    // int min_left_y = min_left_index / width;
    // int min_right_x = min_right_index % width;
    // int min_right_y = min_right_index / width;
    // std::cout << " =========================== "  << std::endl;
    // std::cout << "min_left_x = " << min_left_x << " min_left_y = " << min_left_y << std::endl;
    // std::cout << "min_right_x = " << min_right_x << " min_right_y = " << min_right_y << std::endl;

    return true;
}

/**
 * 根据oriented_pose的朝向，找到grid中第一个黑色栅格，并寻找与其相邻的栅格块。
 * 
 * 该函数从oriented_pose指定的朝向开始，以指定的步长在grid中搜索，直到找到第一个黑色栅格，
 * 然后继续搜索以找到与该黑色栅格相邻的所有栅格，形成一个栅格块。
 * 
 * 输入参数：
 *   - step_dist：搜索步长。
 * 
 * 处理的成员变量：
 *   - grid：被搜索的栅格地图。
 * 
 * 输出：
 *   - 返回一个包含找到的栅格块的grid。
 *   - 返回一个布尔值，表示是否找到了目标区域块。
 */
bool convolution::getOrientedAreaInGrid(const float step_dist)
{
    geometry_msgs::Point temp_point = recommend_pose.position;

    std::vector<int> active_vec;
    std::vector<int> nearby_indexes;
    std::vector<int> final_indexes;

    int temp_index = 0;
    float yaw = tf::getYaw(recommend_pose.orientation);
    // ROS_INFO("yaw = %f", yaw);
    bool findPoint = false;
    bool overflow = false;
    int curIndex = temp_index;
    do{
        temp_point.x += step_dist * cos(yaw);
        temp_point.y += step_dist * sin(yaw);
        temp_index = getIndex(temp_point);

        if(temp_index == curIndex){continue;}
        else{curIndex = temp_index;}
        // ROS_INFO("x = %f, y = %f, index = %d", temp_point.x, temp_point.y, temp_index);
        if(temp_index >= grid.data.size() || temp_point.x >= grid.info.resolution*grid.info.width
            || temp_point.y >=  grid.info.resolution * grid.info.height
            || temp_point.x < 0|| temp_point.y < 0)
        {
            overflow = true;
        }

        ROS_ASSERT(temp_index < grid.data.size() && temp_index >= 0);
        if(grid.data.at(temp_index) == 100)
        {
            findPoint = true;
        }
    }while(!overflow && !findPoint);//若未溢出且未找到符合条件栅格点则继续循环

    //若溢出
    if(overflow){return false;}

    //若未溢出且找到了栅格，则从该栅格开始全部相邻栅格置为100，其他不相邻的置0
    if(!overflow)
    {
        // ROS_INFO("curIndex = %d and grid.at(curIndex) = %d",temp_index, grid.data.at(temp_index));
        ROS_ASSERT_MSG(temp_index < grid.data.size(), "ERROR : Not The Suitable Grid But System Thinks It Suitable.");

        //声明一个用于标记“栅格是否已经被用过”的vector
        std::vector<bool> states;
        states.resize(grid.data.size(), false);//false表示全部grid的栅格都未用

        //在active_vec里填入第一个元素
        active_vec.push_back(temp_index);//在活跃列表内填入第一个符合要求的indexes
        ROS_ASSERT(temp_index >= 0 && temp_index < states.size());
        states.at(temp_index) = true;//state随之更新

        while(active_vec.size() != 0)
        {
            getNeighborIndexes(active_vec.back(), nearby_indexes, states, grid.info.width, grid.info.height);
            final_indexes.push_back(active_vec.back());//末尾一位必然是符合要求的
            active_vec.pop_back();//删除已经用过的最后一位元素
            
            int _tmp = 0;
            for(int i = 0; i < nearby_indexes.size(); i++)
            {
                _tmp = nearby_indexes.at(i);
                ROS_ASSERT(_tmp < grid.data.size() && _tmp >= 0);
                if(grid.data.at(_tmp) == 100)
                {
                    active_vec.push_back(_tmp);//符合要求的才能被填入active_vec
                }
            }
        }
        grid.data.clear();
        grid.data.resize(grid.info.width*grid.info.height, 0);
        for(int i = 0; i < final_indexes.size(); i++)
        {
            ROS_ASSERT(final_indexes.at(i) < grid.data.size() && final_indexes.at(i) >= 0);
            grid.data.at(final_indexes.at(i)) = 100;
        }

    }
    //grid_after_found：以推荐位姿方向找到的栅格点为种子点，得到的连续栅格地图
    grid_after_found = grid;
    pub_grid_after_found.publish(grid_after_found);
    //end
    return true;    
}

/**
 * 计算给定点在栅格地图中的索引。
 * 
 * 该函数根据点的x和y坐标、栅格的分辨率（每个栅格单元的大小）以及栅格的宽度（每行的栅格单元数），
 * 计算出该点在栅格地图中的索引。栅格地图通过一维数组表示，此函数计算出的索引即为点在该一维数组中的位置。
 * 
 * @param _point 一个geometry_msgs::Point类型的参数，表示需要计算索引的三维空间中的点。
 *               由于栅格地图是二维的，因此只使用该点的x和y坐标。
 * @return 返回一个int类型的值，表示点在栅格地图一维数组表示中的索引。
 */
int convolution::getIndex(const geometry_msgs::Point _point)
{
    return static_cast<int>((_point.y )/grid.info.resolution) * grid.info.width + static_cast<int>((_point.x )/grid.info.resolution);
}


int convolution::getRelativeLocation(const int _index, const int _width, const int _height)
{
    ROS_ASSERT(_index >= 0 && _index < _width*_height);
    int flag_up_down = -1;
    int flag_right_left = -1;
    if(_index % _width == 0)//最左列
    {
        flag_right_left = LEFT_EDGE;
    }
    else if(_index % _width == _width - 1)//最右列
    {
        flag_right_left = RIGHT_EDGE;
    }

    if(static_cast<int>(_index/_width) == 0)//最下行
    {
        flag_up_down = DOWN_EDGE;
    }
    else if(static_cast<int>(_index/_width) == _height - 1)//最上行
    {
        flag_up_down = UP_EDGE;
    }

    if(flag_right_left == -1 && flag_up_down == UP_EDGE){return UP_EDGE;}
    else if(flag_right_left == -1 && flag_up_down == DOWN_EDGE){return DOWN_EDGE;}
    else if(flag_right_left == RIGHT_EDGE && flag_up_down == -1){return RIGHT_EDGE;}
    else if(flag_right_left == LEFT_EDGE && flag_up_down == -1){return LEFT_EDGE;}
    else if(flag_right_left == RIGHT_EDGE && flag_up_down == UP_EDGE){return RIGHTUP_EDGE;}
    else if(flag_right_left == RIGHT_EDGE && flag_up_down == DOWN_EDGE){return RIGHTDOWN_EDGE;}
    else if(flag_right_left == LEFT_EDGE && flag_up_down == UP_EDGE){return LEFTUP_EDGE;}
    else if(flag_right_left == LEFT_EDGE && flag_up_down == DOWN_EDGE){return LEFTDOWN_EDGE;}
    else return FREE;
}
/*
    查找_index附近的所有的index，并存入_vec中
    输入：
        _index：当前栅格索引
        _width：栅格地图宽度
        _height：栅格地图高度
        _vec：存储周围栅格index的vector
    输出：
        _vec：存储周围栅格index的vector
*/
void convolution::getNeighborIndexes(const int _index, std::vector<int> &_vec, std::vector<bool> &_states,const int _width, const int _height)
{
    _vec.clear();
    int RL = getRelativeLocation(_index, _width, _height);

    //需要存上方栅格
    if(RL != UP_EDGE && RL != LEFTUP_EDGE && RL != RIGHTUP_EDGE)
    {
        ROS_ASSERT(_index + _width >= 0 && _index + _width < _states.size());
        if(_states.at(_index + _width) == false)//上方栅格未用
        {
            _vec.push_back(_index + _width);//存入上方栅格
            _states.at(_index + _width) = true;//上方栅格状态改为已用
        }
    }
    //需要存左方栅格
    if(RL != LEFT_EDGE && RL != LEFTUP_EDGE && RL != LEFTDOWN_EDGE)
    {
        ROS_ASSERT(_index-1 >= 0 && _index-1 < _states.size());
        if(_states.at(_index - 1) == false)
        {
            _vec.push_back(_index - 1);
            _states.at(_index - 1) = true;
        }
    }
    //需要存右方栅格
    if(RL != RIGHT_EDGE && RL != RIGHTUP_EDGE && RL != RIGHTDOWN_EDGE)
    {
        ROS_ASSERT(_index + 1 >= 0 && _index + 1 < _states.size());
        if(_states.at(_index + 1) == false)
        {
            _vec.push_back(_index + 1);
            _states.at(_index + 1) = true;
        }
    }
    //需要存下方栅格
    if(RL != DOWN_EDGE && RL != LEFTDOWN_EDGE && RL != RIGHTDOWN_EDGE)
    {
        ROS_ASSERT(_index - _width >= 0 && _index - _width < _states.size());
        if(_states.at(_index - _width) == false)
        {
            _vec.push_back(_index - _width);
            _states.at(_index - _width) = true;
        }
    }
    //需要存右上栅格
    if(RL == LEFTDOWN_EDGE || RL == LEFT_EDGE || RL == DOWN_EDGE || RL == FREE)
    {
        ROS_ASSERT(_index + _width + 1 >= 0 && _index + _width + 1 < _states.size());
        if(_states.at(_index + _width + 1) == false)
        {
            _vec.push_back(_index + _width + 1);
            _states.at(_index + _width + 1) = true;
        }
    }
    //需要存右下栅格
    if(RL == LEFTUP_EDGE || RL == UP_EDGE || RL == LEFT_EDGE || RL == FREE)
    {
        ROS_ASSERT(_index - _width + 1 >= 0 && _index - _width + 1 < _states.size());
        if(_states.at(_index - _width + 1) == false)
        {
            _vec.push_back(_index - _width + 1);
            _states.at(_index - _width + 1) = true;
        }
    }
    //需要存左上栅格
    if(RL == RIGHTDOWN_EDGE || RL == RIGHT_EDGE || RL == DOWN_EDGE || RL == FREE)
    {
        ROS_ASSERT(_index + _width - 1 >= 0 && _index + _width - 1 < _states.size());
        if(_states.at(_index + _width - 1) == false)
        {
            _vec.push_back(_index + _width - 1);
            _states.at(_index + _width - 1) = true;
        }
    }
    //需要存左下栅格
    if(RL == RIGHTUP_EDGE || RL == RIGHT_EDGE || RL == UP_EDGE || RL == FREE)
    {
        ROS_ASSERT(_index - _width - 1 >= 0 && _index - _width - 1 < _states.size());
        if(_states.at(_index - _width - 1) == false)
        {
            _vec.push_back(_index - _width - 1);
            _states.at(_index - _width - 1) = true;
        }
    }

}

void convolution::calcConvolution2x2()
{

    std::vector<int> nearbyIndex = {SELF, 0, 0, 0};
    std::vector<int> nearbyValue(4, 0);//1维长度为4的vector且数值均为0
    std::pair<int, int> result = std::make_pair(0, 0);
    int posi = -1;
    //posi:栅格区域在车体的上面还是下面；

    for(int i = 0; i < grid.data.size(); i++)
    {
        posi = getRelativePosi();
        core.recore(2, posi);
        
        // leaderInCore = getLeaderInCore(i, 2);

        getNearbyIndex2x2(i, &nearbyIndex, grid.info.width, grid.info.height);//where we refresh nearbyIndex
        getNearbyValues(i, &nearbyIndex, &nearbyValue);//where we refresh nearbyValue

        //Condition SELF
        if(i == vehicle_posi_index){grid_after_conv.data.at(i) = grid.data.at(i);continue;}

        result = convolute(&nearbyValue);
        ROS_ASSERT_MSG(result.first <= 4 && result.first >=0, "ERROR : Wrong outcome after single convolution for only (int)[0, 4] is legal.");
        
        // core.conv.at(result.second);
        if(result.first == 4)
        {
            if(core.conv.at(result.second).at(0) == 0)
                grid_after_conv.data.at(i-grid_after_conv.info.width) = 100;
            if(core.conv.at(result.second).at(1) == 0)
                grid_after_conv.data.at(i-grid_after_conv.info.width+1) = 100;
            if(core.conv.at(result.second).at(2) == 0)
                grid_after_conv.data.at(i) = 100;
            if(core.conv.at(result.second).at(3) == 0)
                grid_after_conv.data.at(i+1) = 100;
        }
        // else
        // {
        //     if(core.conv.at(result.second).at(0) == 1)
        //         grid_after_conv.data.at(i-grid_after_conv.info.width) = 0;
        //     if(core.conv.at(result.second).at(1) == 1)
        //         grid_after_conv.data.at(i-grid_after_conv.info.width+1) = 0;
        //     if(core.conv.at(result.second).at(2) == 1)
        //         grid_after_conv.data.at(i) = 0;
        //     if(core.conv.at(result.second).at(3) == 1)
        //         grid_after_conv.data.at(i+1) = 0;
        // }
    }
}

/*
    获取当前栅格相对于车体的位置
    输入：
        index : the index of current grid
        _scale : 2 for 2x2 core while 3 for 3x3 core
    输出：
        位置UP DOWN LEFT ……
*/
int convolution::getRelativePosi()
{
    float yaw = tf::getYaw(recommend_pose.orientation);
    if(yaw > M_PI || yaw < -M_PI)
    {
        yaw = yaw - 2*M_PI*static_cast<int>(yaw/(2*M_PI));
    }

    if((M_PI/4) <= yaw && yaw < (M_PI*3/4)) // 45~135度
        return UP;
    else if((-M_PI*3/4) <= yaw && yaw < (-M_PI/4)) // -135~-45度
        return DOWN;
    else if((-M_PI*1/4) <= yaw && yaw < (M_PI*1/4)) // -45~45度
        return RIGHT;
    else  // 135~180度,-180~-135度
        return LEFT;
}

/*
    _scale : 2 for 2x2 while 3 for 3x3
    position : the relative position between the current index and the vehicle position index
*/
void convolution_core::recore(const int _scale, const int position)
{ 
    ROS_ASSERT_MSG(_scale == 2 || _scale == 3, "ERROR : Fail to recore cause wrong size.");
    if(_scale == 2)
    {   
        conv.clear();
        std::vector<int> case1, case2, case3, case4, case5, case6, case7,case8,case9;
        switch(position)
        {
            case UP: 
                case1 = {1, 0, 1, 0};
                case2 = {1, 0, 1, 1};
                case3 = {0, 0, 1, 1};
                case4 = {0, 1, 1, 1};
                case5 = {0, 1, 0, 1};
                case6 = {0, 0, 1, 0};
                case7 = {0, 0, 0, 1};
                case8 = {0, 0, 1, 0};
                case9 = {0, 0, 0, 1};
                break;
            case DOWN: 
                case1 = {1, 0, 1, 0};
                case2 = {1, 1, 1, 0};
                case3 = {1, 1, 0, 0};
                case4 = {1, 1, 0, 1};
                case5 = {0, 1, 0, 1};
                case6 = {1, 0, 0, 0};
                case7 = {0, 1, 0, 0};
                case8 = {0, 0, 1, 0};
                case9 = {0, 0, 0, 1};
                break;
            case LEFT:
                case1 = {0, 0, 1, 1};
                case2 = {1, 0, 1, 1};
                case3 = {1, 0, 1, 0};
                case4 = {1, 1, 1, 0};
                case5 = {1, 1, 0, 0};
                case6 = {0, 0, 1, 0};
                case7 = {1, 0, 0, 0};
                case8 = {0, 1, 0, 0};
                case9 = {0, 0, 0, 1};
                break;
            case RIGHT:
                case1 = {0, 0, 1, 1};
                case2 = {0, 1, 1, 1};
                case3 = {0, 1, 0, 1};
                case4 = {1, 1, 0, 1};
                case5 = {1, 1, 0, 0};
                case6 = {0, 0, 0, 1};
                case7 = {0, 1, 0, 0};
                case8 = {1, 0, 0, 0};
                case9 = {0, 0, 1, 0};
                break;
        }
        conv.push_back(case1);
        conv.push_back(case2);
        conv.push_back(case3);
        conv.push_back(case4);
        conv.push_back(case5);
        conv.push_back(case6);
        conv.push_back(case7);
        conv.push_back(case8);
        conv.push_back(case9);
    }

}
/*
    【仅适用于2x2卷积核】获取某一个index的周围栅格的index，并存入_nearby_index
    输入：
        i：当前栅格的index
    输出：
        _nearby_index：该栅格周围栅格的index
*/
void convolution::getNearbyIndex2x2(const int i, std::vector<int> *_nearby_index, const int width, const int height)
{
    ROS_ASSERT_MSG(i >= 0, "ERROR : Could not get nearby index cause the current index < 0");
    _nearby_index->at(0) = (i - width >= 0) ? i - width : -1;
    _nearby_index->at(1) = ((i - width >= 0) && ((i + 1) % width != 0)) ? i - width + 1 : -1;
    _nearby_index->at(2) = SELF;
    _nearby_index->at(3) = ((i + 1) % width != 0) ? i + 1 : -1;
}

/*
    【适用于任意阶数卷积核】对于周围栅格，获取这些栅格存储的数据
    输入
        curndex：当前栅格的index
        indexes：周围栅格的index组成的vector
    输出：
        values：周围栅格存储的数据
    注意：indexes与values是一一对应的
*/
void convolution::getNearbyValues(const int curIndex, const std::vector<int>*indexes, std::vector<int> *values)
{
    ROS_ASSERT_MSG(indexes->size() == values->size(), "ERROR : Not equal size of indexes and values.");
    //对于indexes里的全部栅格
    
    for(int i = 0; i < indexes->size(); i++)
    {
        if(indexes->at(i) == -1){values->at(i) = 0;}//若为界外栅格
        else if(indexes->at(i) == vehicle_posi_index){values->at(i) = 0;}//若为车所在栅格
        else if(indexes->at(i) == SELF){
            ROS_ASSERT(curIndex >= 0 && curIndex < grid.data.size());
            values->at(i) = getNearbyValue(grid.data.at(curIndex), thres);//若该栅格恰好为自身
        }
        else{
            // std::cout << indexes->at(i) << std::endl;
            ROS_ASSERT(indexes->at(i) >= 0 && indexes->at(i) < grid.data.size());
            values->at(i) = getNearbyValue(grid.data.at(indexes->at(i)), thres);
        }
    }
}

//对于给定的阈值threshold， 若栅格数据大于之，则返回1，否则返回0
int convolution::getNearbyValue(const int data1, const int threshold)
{
    if(data1 >= threshold){return 1;}
    else { return 0;}
}

/*
    【适用于任意阶数卷积核】卷积计算函数
    输入：
        arr：周围栅格存储的数据
    输出：
        out：int型存储的卷积数值
    注意：arr的长度应当等于卷积核vector的长度。如3x3卷积核，arr和conv的size都应该为9
*/
std::pair<int, int>  convolution::convolute(const std::vector<int> *arr)
{
    std::vector<int> score(core.conv.size(), 0);
    int maxValue = 0;
    int maxIndex = 0;
    // ROS_INFO("lengthof arr = %d, core.conv.size() = %d", arr->size(), core.conv.size());
    // ROS_ASSERT_MSG(arr->size() == core.conv.size(), "ERROR : Error occurs for not matched size of convolution core and nearbyValues.");
    for(int order = 0; order < core.conv.size(); order++)
    {
        std::vector<int> currentCore = core.conv.at(order);
        for(int i = 0; i < currentCore.size(); i++)
        {
            if(currentCore.at(i) == arr->at(i))
            {
                score.at(order) += 1;
            }
        }
    }
    auto maxScore = std::max_element(score.begin(), score.end());
    maxValue = *maxScore;
    maxIndex = std::distance(score.begin(), maxScore);
    return std::make_pair(maxValue, maxIndex);
} 


void convolution::reset()
{
    grid.data.clear();
    grid.header.frame_id = "null";
}


void convolution::calcConvolution3x3()
{
    //需要补充QuickSort
}


/*
    ###################################
    ###
    ###               callback Function
    ###
    ###################################
*/

// void convolution::callbackSubRecommendPose(const geometry_msgs::PoseStamped recommendPose)
// {
//     ROS_INFO("Received: Get the Recommended Pose In GRD.");
//     recommend_pose = recommendPose.pose;
//     recommend_pose.position = recommendPose.pose.position;
//     recommend_pose.orientation = tf::createQuaternionMsgFromYaw(M_PI + tf::getYaw(recommendPose.pose.orientation));
//     ifGetRcmdPose = true;
// }

// void convolution::callbackSubVehiclePose(const geometry_msgs::PoseStamped vehiclePose)
// {
//     ROS_INFO("Received: Get the Start Pose | Vehicle Pose In GRD.");
//     vehicle_position = vehiclePose.pose.position;
//     ifSubVehiclePose = true;
// }

void convolution::callbackSubGrid(const nav_msgs::OccupancyGrid _gridData)
{
    // ROS_INFO("Get  in  callbackConvolution");
    grid  = _gridData;

    for(int j = 0; j < grid.data.size(); j++)
    {
        if(grid.data.at(j) <  0 || grid.data.at(j) >100){grid.data.at(j) = 100;}
    }
    // ROS_INFO("grid.info.weight = %d", grid.info.width);
    grid_after_conv.header = _gridData.header;
    grid_after_conv.info = _gridData.info;
    grid_after_conv.data.resize(_gridData.info.height*_gridData.info.width, 0);
    grid_after_cut.header = _gridData.header;
    grid_after_cut.info = _gridData.info;
    grid_after_cut.data.resize(_gridData.info.height*_gridData.info.width, 0);

    ifSubGrid = true;

}

/*
    #############################################
    ###
    ###                         Normal Fit Sequence
    ###
    #############################################
*/

/*
    类名：法向拟合序列
    功能：对于一段序列，依次选其_num个点，进行法向拟合找到【切点法向】即切点及法向。
                法向拟合采用的是圆拟合与直线拟合结合的方法
    param : _points : 待处理的点集
    param : _origin : 参考点，用于筛选出近这一侧的点
    param : _num : 单次处理点集中点的个数
*/
normalFitSequence::normalFitSequence(std::vector<geometry_msgs::Point> _points, const geometry_msgs::Point _origin, const int _num)
{
    geometry_msgs::Pose _temp_pose;
    std::vector<geometry_msgs::Point>::iterator first, end;

    for(int i = 0; i < _points.size() - _num + 1; i++)
    {
        first = _points.begin() + i;
        end = _points.begin() + i + _num;
        std::vector<geometry_msgs::Point> active_points(first, end);
        
        //拟合计算点
        // _temp_pose = circleFit(active_points, _origin);
        _temp_pose = linearFit(active_points, _origin);
        if(_temp_pose.position.z == -1)
        {
            continue;
        }
        poses.push_back(_temp_pose);
    }

}

geometry_msgs::Pose normalFitSequence::circleFit(const std::vector<geometry_msgs::Point> active_points, const geometry_msgs::Point _origin)
{
    geometry_msgs::Pose _pose;
    if(active_points.size() < 3)
    {
        _pose.position.z = -1;
        return _pose;
    }
    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
    int N = active_points.size();
    for (int i = 0; i < active_points.size(); i++)
    {
        double x = active_points.at(i).x;
        double y = active_points.at(i).y;
        double x2 = x * x;
        double y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }

    double C, D, E, G, H;
    double a, b, c;
    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
    double cent_x = a / (-2);
    double cent_y = b / (-2);
    double radius = sqrt(a * a + b * b - 4 * c) / 2;
    if(radius > 20)
    {
        _pose.position.z = -1;
        return _pose;
    }
    //序列上弦朝向
    double theta_0 = atan2(active_points.back().y - cent_y, active_points.back().x - cent_x);
    //序列下弦朝向
    double theta_1 = atan2(active_points.front().y - cent_y, active_points.front().x - cent_x);
    double theta = (theta_0 + theta_1) / 2;
    double theta_low = theta - M_PI/2;
    double theta_up = theta + M_PI/2;

    _pose.position.x = cent_x + radius * cos(theta);
    _pose.position.y = cent_y + radius * sin(theta);
    _pose.position.z = 0;

    double standard_theta = atan2(_origin.y - _pose.position.y, _origin.x - _pose.position.x);
    if(isInAngularInterval(standard_theta, theta_low, theta_up))
    {
        _pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    }
    else{
        if(theta >= 0)
        {
            theta -= 2*M_PI;
        }
        _pose.orientation = tf::createQuaternionMsgFromYaw(M_PI + theta);
    }

    return _pose;
}
geometry_msgs::Pose normalFitSequence::linearFit(const std::vector<geometry_msgs::Point> active_points, const geometry_msgs::Point _origin)
{
    geometry_msgs::Pose temp_pose;
    //A x + B y = 1
    Eigen::MatrixX2f A;
    A.resize(active_points.size(), 2);
    for(int i = 0; i < active_points.size(); i++)
    {
        A(i, 0) = active_points.at(i).x;
        A(i, 1) = active_points.at(i).y;
    }
    Eigen::Matrix2Xf A_T = A.transpose();
    Eigen::VectorXf B;
    B.resize(active_points.size());
    for(int i = 0; i < B.size(); i++)
    {
        B(i) = 1;
    }

    Eigen::Vector2f x = (A_T * A).inverse() * A_T * B;
    Eigen::Vector2f x_a = A_T * B;

    //A x + B y = 1
    double A0 = x(0);
    double B0 = x(1);
    temp_pose.position.x = x_a(0)/active_points.size();
    temp_pose.position.y = x_a(1)/active_points.size();
    temp_pose.position.z = 0;

    double theta_1 = atan2(B0, A0);
    double theta_2 = theta_1 + M_PI;
    if(theta_2 >= M_PI)
    {
        theta_2 -= 2*M_PI;
    }
    double standard_theta = atan2(_origin.y -  temp_pose.position.y, _origin.x - temp_pose.position.x);

    double theta_low = theta_1 - M_PI/2;
    double theta_up = theta_1 + M_PI/2; 

    if(isInAngularInterval(standard_theta, theta_low, theta_up))
    {
        temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta_1);
    }
    else{
        temp_pose.orientation = tf::createQuaternionMsgFromYaw(theta_2);
    }

    return temp_pose;
}

bool normalFitSequence::isInAngularInterval(const double _angle, const double _low, const double _up)
{
    bool flag = false;
    if(_angle >= _low && _angle < _up){flag = true;}
    if(_angle + 2*M_PI  >= _low && _angle + 2*M_PI < _up){flag = true;}
    if(_angle - 2*M_PI  >= _low && _angle - 2*M_PI < _up){flag = true;}
    return flag;
}
void normalFitSequence::savePointsToFile(const visualization_msgs::Marker& fitted_points, const std::string& file_name) {
    // 检查文件扩展名，如果不是.csv，则添加
    std::string output_file_name = file_name;
    if (output_file_name.substr(output_file_name.find_last_of(".") + 1) != "csv") {
        output_file_name += ".csv";
    }

    std::ofstream out_file(output_file_name); // 使用更新后的文件名创建文件输出流对象

    // 检查文件是否成功打开
    if (!out_file) {
        ROS_ERROR("Failed to open file: %s", output_file_name.c_str());
        return;
    }
    
    // 写入标题
    out_file << "x, y, heading" << std::endl;

    // 遍历fitted_points中的points数组
    for (const auto& point : fitted_points.points) {
        // 写入每个点的x, y, z坐标，坐标之间以逗号分隔
        out_file << point.x << "," << point.y << "," << 0 << std::endl;
    }

    // 关闭文件
    out_file.close();

    // 打印信息到控制台，确认文件已保存
    ROS_INFO("Points saved to %s successfully.", output_file_name.c_str());
}
}//end namespace
