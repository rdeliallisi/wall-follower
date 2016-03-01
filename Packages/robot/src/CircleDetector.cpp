#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

void callback(const sensor_msgs::LaserScan& msg) {
    ROS_INFO("%s\n", msg.str().c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("base_scan", 1000, callback);
    ros::spin();
    return 0;
}
