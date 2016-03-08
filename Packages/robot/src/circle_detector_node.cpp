#include <ros/ros.h>
#include "circle_detector.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "CircleDetector");
    
    int rate = 10;

    CircleDetector circle_detector;
    
    ros::Rate r(rate);

    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}