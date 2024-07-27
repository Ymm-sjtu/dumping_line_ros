// MapPublisher.h

#ifndef MAPPUBLISHER_H
#define MAPPUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <jsoncpp/json/json.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <iostream>

class MapPublisher {
public:
    MapPublisher(const std::string& json_file_path); // 构造函数，接收JSON文件路径
    void publishMap(); // 发布地图的公共接口
    void publishMapAfterErodeDilate(); // 发布腐蚀后的地图的公共接口
    void publishMapAfterDilate(); // 发布膨胀后的地图的公共接口

private:
    float origin_x, origin_y; // 地图原点
    ros::NodeHandle nh;
    ros::Publisher map_pub;
    ros::Publisher map_pub_after_erode_dilate;
    ros::Publisher map_pub_after_dilate;
    nav_msgs::OccupancyGrid occupancy_grid;
    nav_msgs::OccupancyGrid occupancy_grid_after_erode_dilate;
    nav_msgs::OccupancyGrid occupancy_grid_after_dilate;
    cv::Mat cv_gridmap;

    void processJson(const Json::Value& data); // 处理JSON的私有函数
    void processJsonWithIndex(const Json::Value& data); // 处理有索引的JSON数据
    void processJsonWithoutIndex(const Json::Value& data); // 处理无索引的JSON数据
    void occupancyGridToMat(const nav_msgs::OccupancyGrid& grid); // 将OccupancyGrid转换为cv::Mat
    void MatToOccupancyGrid(const cv::Mat &cv_gridmap,nav_msgs::OccupancyGrid &occupancy_grid); // 将cv::Mat转换为OccupancyGrid
    bool Erode(cv::Mat &cv_gridmap); // 腐蚀
    bool Dilate(cv::Mat &cv_gridmap); // 膨胀
};

#endif
