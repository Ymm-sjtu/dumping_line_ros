
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_publish_node");
    ros::NodeHandle nh;

    ros::Publisher start_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/start_pose_in_grd", 10);
    ros::Publisher recommend_grd_pub = nh.advertise<geometry_msgs::PoseStamped>("/recommend_grd", 10);

    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok())
    {
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::PoseStamped recommend_pose;

        // 通用设置
        ros::Time now = ros::Time::now();
        start_pose.header.stamp = now;
        recommend_pose.header.stamp = now;

        // 为 /start_pose_in_grd 设置姿态
        start_pose.pose.position.x = (-146.013+414.532);
        start_pose.pose.position.y = (915.545-734.578);
        start_pose.pose.position.z = 0.0;
        start_pose.pose.orientation.x = 0.0;
        start_pose.pose.orientation.y = 0.0;
        start_pose.pose.orientation.z = 0.707;
        start_pose.pose.orientation.w = 0.707;

        // 为 /recommend_grd 设置不同的姿态
        recommend_pose.pose.position.x = (-146.013+414.532);
        recommend_pose.pose.position.y = (915.545-734.578);
        recommend_pose.pose.position.z = 0.0;
        recommend_pose.pose.orientation.x = 0.0;
        recommend_pose.pose.orientation.y = 0.0;
        recommend_pose.pose.orientation.z = 0.707;
        recommend_pose.pose.orientation.w = 0.707;

        // 分别发布到两个话题
        start_pose_pub.publish(start_pose);
        recommend_grd_pub.publish(recommend_pose);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
