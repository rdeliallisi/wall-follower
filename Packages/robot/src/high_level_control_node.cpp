/** 
 * @file high_level_control_node.cpp
 * @brief This file creates the HighLevelControl ros node and uses the 
 * HighLevelControl class to send messages to the robot
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include <ros/ros.h>
#include "high_level_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "HighLevelControl");

    HighLevelControl high_level_control;

    ros::Rate r(10.0);
    while (ros::ok())
    {

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
