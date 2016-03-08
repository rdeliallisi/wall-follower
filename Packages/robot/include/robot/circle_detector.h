#ifndef CRICLE_DETECTOR_H
#define CRICLE_DETECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Circle{
public:
    float x, y;
};

class CircleDetector {
private:
    ros::NodeHandle node_;
    ros::Subscriber laser_sub_;
    Circle circle_;
    int count_threshold_;

public:

    CircleDetector();

    /**
    * Gets the data from the laser range finder, creates an
    * image out of it and runs openCV HoughLines on it
    * 
    * @param msg Raw data comming from the laser range finder
    */
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * Returns the detected circle
     * @return -1, -1 if no circle is detected at the moment, the coordinates
     * of the circle otherwise
     */
    Circle get_circle();
};


#endif