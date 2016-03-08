#ifndef CIRCLE_DETECTOR_H
#define CIRCLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

/**
 * @brief Detects half-circle
 *
 * @date March 2016
 */
class CircleDetector {

public:

    /**
     * Ros node to which the class is attached
    */
    ros::NodeHandle node_;

    /**
     * Used to get the data from the laser range finder
     */
    ros::Subscriber laser_sub_;


    CircleDetector();

    /**
     * @brief creates an image which reflects the laser scan data
     * 
     * @param screen_width 
     * @param screen_height 
     * @param msg [laser_scan]
     * @return cv::Mat 
     */
    cv::Mat create_image(int screen_width, int screen_height, 
    const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
    * @brief Creates a white image of given dimensions
    * 
    * @param screen_width 
    * @param screen_height 
    * 
    * @return image cv::Mat
    */
    cv::Mat create_image(int screen_width, int screen_height);


    /**
     * @brief callback function for laser_scan
     * @details Gets the date from the laser range finder, creates an
     * image out of it and runs openCV HoughCircles on it
     * 
     * @param msg sensor_msgs::LaserScan
     */
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief applies HoughCircles and finds the goal
     * 
     * @param image 
     * @return circles  
     **/
    std::vector<cv::Vec3f> find_circles(cv::Mat image);


};

#endif