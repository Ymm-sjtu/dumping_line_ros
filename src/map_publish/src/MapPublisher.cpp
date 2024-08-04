// MapPublisher.cpp

#include "MapPublisher.h"
#include <fstream>
#include <iostream>

MapPublisher::MapPublisher(const std::string& json_file_path) {
    // 初始化ROS发布者
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);
    map_pub_after_open_close = nh.advertise<nav_msgs::OccupancyGrid>("map_open_close", 10);
    map_pub_after_open = nh.advertise<nav_msgs::OccupancyGrid>("map_open", 10);

    // 读取JSON文件
    std::ifstream json_file(json_file_path);
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(json_file, root, false)) {
        ROS_ERROR("Failed to parse the JSON file.");
        return;
    }

    // 解析地图信息并填充nav_msgs::OccupancyGrid消息
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.resolution = root["resolution"].asFloat(); // 地图分辨率
    occupancy_grid.info.width = root["width"].asUInt(); // 地图宽度
    occupancy_grid.info.height = root["height"].asUInt(); // 地图高度
    occupancy_grid.info.origin.position.x = root["origin.x"].asFloat();
    occupancy_grid.info.origin.position.y = root["origin.y"].asFloat();
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0; // 不旋转

    origin_x = root["origin.x"].asFloat();
    origin_y = root["origin.y"].asFloat();

    std::cout << "Frame ID: " << occupancy_grid.header.frame_id << std::endl;
    std::cout << "Resolution: " << occupancy_grid.info.resolution << std::endl;
    std::cout << "Width: " << occupancy_grid.info.width << std::endl;
    std::cout << "Height: " << occupancy_grid.info.height << std::endl;
    std::cout << "Origin Position X: " << occupancy_grid.info.origin.position.x << std::endl;
    std::cout << "Origin Position Y: " << occupancy_grid.info.origin.position.y << std::endl;
    std::cout << "Origin Position Z: " << occupancy_grid.info.origin.position.z << std::endl;
    std::cout << "Orientation W: " << occupancy_grid.info.origin.orientation.w << std::endl;

    // 初始化occupancy_grid.data大小和默认值
    occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, 0);

    // 根据JSON文件中的data数组更新地图数据
    processJson(root["data"]);
}

void MapPublisher::processJson(const Json::Value& data) {
    if (data.isArray() && !data.empty() && data[0].isObject() && data[0].isMember("index")) {
        processJsonWithIndex(data);
    } else {
        processJsonWithoutIndex(data);
    }
}

void MapPublisher::processJsonWithIndex(const Json::Value& data) {
    for (const auto& element : data) {
        double height = element["height"].asDouble();
        if (height < 0.01) {
            continue;  // 忽略高度小于0.01的数据
        }

        int index = element["index"].asInt();
        int row = index / occupancy_grid.info.width;
        int col = index % occupancy_grid.info.width;

        if (row < occupancy_grid.info.height && col < occupancy_grid.info.width) {
            occupancy_grid.data[row * occupancy_grid.info.width + col] = 100;  // 假设这里我们将占据状态设置为100
        }
    }
}

void MapPublisher::processJsonWithoutIndex(const Json::Value& data) {
    for (unsigned int i = 0; i < data.size(); i++) {
        int index = data[i].asInt(); // 获取应该被标记为占据的点的索引
        if (index >= 0 && index < occupancy_grid.data.size()) {
            occupancy_grid.data[index] = 100; // 将该点标记为占据
        } else {
            ROS_WARN("Index out of bounds: %d", index);
        }
    }
}

void MapPublisher::occupancyGridToMat(const nav_msgs::OccupancyGrid& grid) {
    int width = grid.info.width;
    int height = grid.info.height;

    // 创建一个CV_8UC1的Mat对象，用于灰度图
    cv::Mat mat(height, width, CV_8UC1);

    // 遍历网格并填充到Mat中
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = x + y * width;
            int8_t occupancy_value = grid.data[index];

            // 将ROS中的占用概率值映射到[0, 255]
            if (occupancy_value == -1) {
                mat.at<uchar>(y, x) = 127; // 未知区域设置为127（灰色）
            } else {
                // 将占用概率映射为灰度值：100 -> 0 (黑色), 0 -> 255 (白色)
                mat.at<uchar>(y, x) = static_cast<uchar>((100 - occupancy_value) * 255 / 100);
            }
        }
    }

    cv_gridmap = mat;
}

bool MapPublisher::Erode(cv::Mat &cv_gridmap){

    //利用opencv进行腐蚀
    cv::Mat erode_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(cv_gridmap, cv_gridmap, erode_element);
    // cv::erode(cv_gridmap, cv_gridmap, erode_element);

    // if (bool_visual_)
    // {
    //     cv::namedWindow("腐蚀后地图", cv::WINDOW_NORMAL);
    //     cv::imshow("腐蚀后地图", cv_gridmap);
    //     cv::waitKey(0);
    // }
    
    return true;
}

bool MapPublisher::Dilate(cv::Mat &cv_gridmap){

    //利用opencv进行膨胀
    cv::Mat dilate_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(cv_gridmap, cv_gridmap, dilate_element);
    // cv::dilate(cv_gridmap, cv_gridmap, dilate_element);


    // if (bool_visual_)
    // {
    //     cv::namedWindow("膨胀后的地图", cv::WINDOW_NORMAL);
    //     cv::imshow("膨胀后的地图", cv_gridmap);
    //     cv::waitKey(0);
    // }

    return true;
}

void MapPublisher::MatToOccupancyGrid(const cv::Mat &cv_gridmap,nav_msgs::OccupancyGrid &occupancy_grid) {
    // 检查输入图像是否为单通道灰度图像
    if (cv_gridmap.empty() || cv_gridmap.type() != CV_8UC1) {
        std::cerr << "Input image must be a non-empty single-channel 8-bit image." << std::endl;
        return;
    }

    // 设置OccupancyGrid的元数据
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.info.resolution = 1; // 地图分辨率
    occupancy_grid.info.width = cv_gridmap.cols; // 地图宽度
    occupancy_grid.info.height = cv_gridmap.rows; // 地图高度
    occupancy_grid.info.origin.position.x = origin_x;
    occupancy_grid.info.origin.position.y = origin_y;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0; // 不旋转

    // 初始化occupancy_grid.data大小和默认值
    occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, 0);

    // 遍历Mat并填充到OccupancyGrid中
    for (int y = 0; y < cv_gridmap.rows; ++y) {
        for (int x = 0; x < cv_gridmap.cols; ++x) {
            int index = x + y * cv_gridmap.cols;
            uchar pixel_value = cv_gridmap.at<uchar>(y, x);

            // 将灰度值映射到[0, 100]
            if (pixel_value == 127) {
                occupancy_grid.data[index] = -1; // 未知区域
            } else {
                occupancy_grid.data[index] = static_cast<int8_t>((255 - pixel_value) * 100 / 255); // 映射为[0, 100]
            }
        }
    }

}

void MapPublisher::publishMap() {
    map_pub.publish(occupancy_grid);
}

void MapPublisher::publishMapAfterOpenClose(){
    // 将OccupancyGrid转换为cv::Mat
    occupancyGridToMat(occupancy_grid);

    //先腐蚀后膨胀，去除内凹点
    Erode(cv_gridmap);  // 腐蚀
    Dilate(cv_gridmap); // 膨胀
 
    // 将cv::Mat转换为OccupancyGrid
    MatToOccupancyGrid(cv_gridmap, occupancy_grid_after_open);

    // 发布开运算后的地图
    map_pub_after_open.publish(occupancy_grid_after_open);

    //先膨胀后腐蚀，去除外凸点
    Dilate(cv_gridmap); // 膨胀
    Erode(cv_gridmap);  // 腐蚀

    // 将cv::Mat转换为OccupancyGrid
    MatToOccupancyGrid(cv_gridmap, occupancy_grid_after_open_close);

    // 发布开运算及闭运算后的地图
    map_pub_after_open_close.publish(occupancy_grid_after_open_close);
}