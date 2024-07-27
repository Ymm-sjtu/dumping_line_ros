// main.cpp

#include <ros/ros.h>
#include "MapPublisher.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_publisher_node");

    // // 检查是否提供了JSON文件路径
    // if (argc < 2) {
    //     ROS_ERROR("Usage: rosrun <your_package_name> map_publisher_node <path_to_json>");
    //     return 1;
    // }

    // std::string json_file_path = argv[1];
    std::string json_file_path = "/home/ymm/dumping_line_ws/src/map_publish/json/L1.json";

    MapPublisher mapPublisher(json_file_path);
    ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok()) {
        mapPublisher.publishMap();
        mapPublisher.publishMapAfterErodeDilate();
        mapPublisher.publishMapAfterDilate();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
