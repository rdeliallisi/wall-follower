/**
 * @file circle_detector.cpp
 * @brief Header file for the circle detector class.
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */


#ifndef CRICLE_DETECTOR_H
#define CRICLE_DETECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "detect_helpers.h"

using namespace std;
using namespace cv;

/**
 * @brief Defines the CircleDetector class
 * which will help checking if a circle is found or not.
 *
 * @details The results from the laser range scanner are used to check if the
 * obstacle that is seen is in fact a circle.
 * Also, it returns information about the position of the circle which is
 * helpful in detecting it.
 */
class CircleDetector {
private:
    /**
     * @brief The class has as parameters the following:
     */

    /**
     * @brief the node that was created
     */
    ros::NodeHandle node_;

    /**
     * @brief The laser_sub subsrives the node that is seen to the laser range
     * finder
     */

    ros::Subscriber laser_sub_;

    /**
    * @brief the circle_detect_pub publishes the translated lrf input as well as circles, if any.
    */
    ros::Publisher circle_detect_pub_;

    BlurParams blur_params_;

    HoughParams hough_params_;

    void LoadParams();

public:

    /**
     * @brief Default constructor for CircleDetector
     */
    CircleDetector();

    /**
     * @brief Gets the data from the laser range finder, creates an
     * image out of it and runs openCV HoughLines on it
     *
     * @param[in]  msg   msg Raw data comming from the laser range finder
     */
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);


    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param x [description]
     * @param y [description]
     * @param screen_w [description]
     * @param screen_h [description]
     */
    void ConvertCartesianToScreen(int &x, int &y, int screen_w, int screen_h);

    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param x [description]
     * @param y [description]
     * @param range [description]
     * @param base_scan_min_angle [description]
     */
    void ConvertLaserScanToCartesian(int &x, int &y, float range, float base_scan_min_angle);
};

#endif