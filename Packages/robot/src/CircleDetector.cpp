#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

void callback(const sensor_msgs::LaserScan& msg) {
    ROS_INFO("Angle Min: %f\n", msg.angle_min);
    ROS_INFO("Angle max: %f\n", msg.angle_max);
    ROS_INFO("Angle increment: %f\n", msg.angle_increment);
    ROS_INFO("time increment: %f\n", msg.time_increment);
    ROS_INFO("Scan Time: %f\n", msg.scan_time);
    ROS_INFO("range Min: %f\n", msg.range_min);
    ROS_INFO("range max: %f\n", msg.range_max);
    for (int i = 0; i < sizeof(msg.ranges)/ sizeof(float) ; ++i) {
        ROS_INFO("range %i: %f\n", i, msg.ranges[i]);
        ROS_INFO("intensity %i: %f\n", i, msg.intensities[i]);
    }
    ROS_INFO("--------------------------------------------------------------------");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("base_scan", 1000, callback);
    ros::spin();
    return 0;
}
