#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <iostream>
#include <jsoncpp/json/json.h>

class MapPublisher {
public:
    MapPublisher() {
        // 初始化ROS发布者
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);

        // 读取JSON文件
        std::ifstream json_file("/home/ymm/dumping_line_ws/src/map_publish/json/L1.json");
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

    void publishMap() {
        map_pub.publish(occupancy_grid);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher map_pub;
    nav_msgs::OccupancyGrid occupancy_grid;

    void processJson(const Json::Value& data) {
        if (data.isArray() && !data.empty() && data[0].isObject() && data[0].isMember("index")) {
            processJsonWithIndex(data);
        } else {
            processJsonWithoutIndex(data);
        }
    }

    void processJsonWithIndex(const Json::Value& data) {
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

    void processJsonWithoutIndex(const Json::Value& data) {
        for (unsigned int i = 0; i < data.size(); i++) {
            int index = data[i].asInt(); // 获取应该被标记为占据的点的索引
            if (index >= 0 && index < occupancy_grid.data.size()) {
                occupancy_grid.data[index] = 100; // 将该点标记为占据
            } else {
                ROS_WARN("Index out of bounds: %d", index);
            }
        }
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "map_publisher_node");
    MapPublisher mapPublisher;
    ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok()) {
        mapPublisher.publishMap();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
