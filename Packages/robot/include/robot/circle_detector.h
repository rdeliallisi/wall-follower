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
 * @brief Define the ScreenCoordinates class which is used to store the
 * computed values of the LRF data in screen coordinates
 */
class ScreenCoordinates {
public:
    /**
     * x coordinate of the LRF data in screen coordinates
     */
    int x;

    /**
     * y coordinate of the LRF data in screen coordinates
     */
    int y;
};

/**
 * @brief Define the CartesianCoordinates class which is used to store the
 * computed values of the LRF data in cartesian frame
 */
class CartesianCoordinates {
public:
    /**
     * x coordinate of the LRF data in cartesian frame
     */
    float x;

    /**
     * y coordinate of the LRF data in cartesian frame
     */
    float y;
};


/**
 * @brief Define the Circle class which takes the coordinates of the points
 *
 */
class Circle {
public:

    /**
     * x coordinate of the point relative to the laser range scanner
     */
    float x;

    /**
     * y coordinate of the point relative to the laser range scanner
     */
    float y;
};

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
     * @brief The circle is the actual circle that the scanner has found
     */
    Circle circle_;

    /**
     * @brief The LRF data in cartesian coordinates relative to the robot
     */
    CartesianCoordinates cartesian_;

    /**
     * The LRF data in screen coordinates
     */
    ScreenCoordinates screen_;

    CannyParams canny_params_;

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
     * @brief Returns the detected circle
     * @return -1, -1 if no circle is detected at the moment, the coordinates
     * of the circle otherwise
     */
    Circle get_circle();

    /**
     * @brief Returns the screen coordinates of the LRF data
     *
     * @return     { description_of_the_return_value }
     */
    ScreenCoordinates get_screen_coordinates();

    /**
     * @brief Returns the cartesian coordinates of the LRF data
     * @return CatesianCoordinate class instance
     */
    CartesianCoordinates get_cartesian_coordinates();


    /**
     * @brief Converts cartesian coordinates to screen coordinates to get rid of
     * negative and floating point values
     *
     * @param[in]  screen_width
     * @param[in]  screen_height
     */
    void ConvertCartesianToScreen(int screen_width, int screen_height);

    /**
     * @brief Converts the LRF data into cartesian coordinates
     *
     * @param[in]  range   The distance of the point detected from the LRF
     * @param[in]  base_scan_min_angle  The angle of the current coordinate point
     */
    void ConvertLaserScanToCartesian(float range, float base_scan_min_angle);

    /**
     * @brief Converts the LRF data into cartesian coordinates
     *
     * @param[in]  base_scan_min_angle  The angle of the current coordinate point
     * @param[in]  msg  The LRF data input in LaserScan format
     */
    void ConvertLaserScanToCartesian(int i, float base_scan_min_angle,
                                     const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
    * @brief Renders the circle on the map image
    */
    void RenderImage(vector<Vec3f> circles, cv::Mat image);
};

#endif