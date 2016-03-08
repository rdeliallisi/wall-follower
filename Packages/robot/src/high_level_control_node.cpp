#include <ros/ros.h>
#include "high_level_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "HighLevelControl");

    HighLevelControl high_level_control;

    ros::Rate r(10.0);
    while (ros::ok())
    {
        high_level_control.ControlledRandomMove();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}