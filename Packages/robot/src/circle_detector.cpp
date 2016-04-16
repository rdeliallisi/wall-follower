/**
 * @file circle_detector.cpp
 * @brief This file contains the implementation of the circle detector class.
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "robot/circle_detect_msg.h"
#include "circle_detector.h"

#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "detect_helpers.h"

//Define the constructor for the CircleDetector class
CircleDetector::CircleDetector() : node_() {
    //subscribe the node
    laser_sub_ = node_.subscribe("base_scan", 100,
                                 &CircleDetector::LaserCallback, this);
    circle_detect_pub_ = node_.advertise<robot::circle_detect_msg>(
                             "circle_detect", 100);
    //Method that loads the parameters
    LoadParams();
}

//Define a method to convert the Cartesian coordinates to screen coordinates
void CircleDetector::ConvertCartesianToScreen(int &x, int &y, int screen_w, int screen_h) {
    //convert the data of the x Cartesian coordinate to screen coordinate
    x = static_cast<int>(x + screen_w / 2);
    //convert the data of the y Cartesian coordinate to screen coordinate
    y = static_cast<int>(-y + screen_h / 2);
}

//Define a method to convert the data received from the laser to Cartesian coordinates
void CircleDetector::ConvertLaserScanToCartesian(int &x, int &y, float range, float base_scan_min_angle) {
    //initialise the scale factor variable
    const int scale_factor = 100;
    //convert the data of the x coordinate to Cartesian coordinate
    x = static_cast<int>((range * sin(base_scan_min_angle)) * scale_factor);
    //convert the data of the y coordinate to Cartesian coordinate
    y = static_cast<int>((range * cos(base_scan_min_angle)) * scale_factor);
}

//Define a method which loads the parameters
void CircleDetector::LoadParams() {
    //Firstly, the loaded variable is assigned to be true
    bool loaded = true;

    // These are loaded from the params in the launch file
    if (!node_.getParam("/blur_kernel_size",
                        blur_params_.kernel_size_)) {
        loaded = false;
    }

    if (!node_.getParam("/blur_sigma",
                        blur_params_.sigma_)) {
        loaded = false;
    }

    if (!node_.getParam("/hough_threshold_1",
                        hough_params_.threshold_1_)) {
        loaded = false;
    }

    if (!node_.getParam("/hough_threshold_2",
                        hough_params_.threshold_2_)) {
        loaded = false;
    }

    if (!node_.getParam("/hough_dp",
                        hough_params_.dp_)) {
        loaded = false;
    }

    if (!node_.getParam("/hough_min_dist",
                        hough_params_.min_dist_)) {
        loaded = false;
    }

    if (!node_.getParam("/hough_min_radius",
                        hough_params_.min_radius_)) {
        loaded = false;
    }

    if (!node_.getParam("/hough_max_radius",
                        hough_params_.max_radius_)) {
        loaded = false;
    }

    if (!loaded) {
        ros::shutdown();
    }
}

//Define the LaserCallBack method which turns the maze into an image and then applies Hough Transform
void CircleDetector::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    LoadParams();
    size_t data_points = msg->ranges.size();

    //create image
    int screen_width = 1000;
    int screen_height = 1000;
    const float lrf_max_range = 2;

    cv::Mat image;
    image.create(screen_width, screen_height, CV_8UC1);
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            image.at<uchar>(i, j) = static_cast<uchar>(0);
        }
    }

    //convert laser_scan data to image
    float base_scan_min_angle = msg->angle_min;
    for (int i = 0; i < data_points; ++i) {
        float range = msg->ranges[data_points - 1 - i];
        base_scan_min_angle += msg->angle_increment;
        if (range < lrf_max_range) {
            int x, y;
            ConvertLaserScanToCartesian(x, y, range, base_scan_min_angle);
            ConvertCartesianToScreen(x, y, screen_width, screen_height);

            if (x >= 0 && y >= 0) {
                //Swap places to adapt to OpenCV coordinate system
                image.at<uchar>(y, x) = static_cast<uchar>(255);
            } else {
                //Coordinates are out of bound because of roundoff errors
                ROS_INFO("Round off error: Coordinates out of bound");
            }
        }
    }

    //compute Hough Transform
    cv::Mat destination;
    cv::GaussianBlur(image, destination,
                     Size(blur_params_.kernel_size_, blur_params_.kernel_size_),
                     blur_params_.sigma_, blur_params_.sigma_);

    vector<Vec3f> circles;
    cv::HoughCircles(destination, circles, CV_HOUGH_GRADIENT,
                     hough_params_.dp_, hough_params_.min_dist_,
                     hough_params_.threshold_1_, hough_params_.threshold_2_,
                     hough_params_.min_radius_, hough_params_.max_radius_);
    // declare the x and y coordinates of the circle

    double circle_x, circle_y;
    for (int i = 0; i < circles.size(); i++) {
        circle_x = (circles[0][0] - screen_width / 2) / 100;
        circle_y = -((circles[0][1] - screen_height / 2) / 100);

        // Planar distance to the center of the circle ignoring obstacles
        double center_distance = sqrt(circle_x * circle_x + circle_y * circle_y);
        // Angle to the center of the circle relative to normal cartesian system
        double center_angle = acos(circle_x / center_distance) / M_PI * 180;
        // Index of the angle in the ranges vector
        int index = (center_angle + 30) * 3;
        // Check if we might get an out of bound index after shifting by 20 deg
        if (index < 60 || index >= 660) {
            circles.erase(circles.begin() + i);
            break;
        }
        // Distance from LRF in the direction of the circle center
        double center_lrf = msg->ranges[index];
        if(center_lrf > center_distance) {
            circles.erase(circles.begin() + i);
        }
    }

     for ( size_t i = 0; i < circles.size(); i++ ) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle( destination, center, 3, Scalar(255), -1);
        cv::circle( destination, center, radius, Scalar(255), 1 );
    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", destination );
    waitKey(10);

    //If the circle is found then the coordinates are converted to screen coordinates
    if (circles.size() == 1) {
        circle_x = (circles[0][0] - screen_width / 2) / 100;
        circle_y = -((circles[0][1] - screen_height / 2) / 100);
    }
    //If the circle is not found, then the x and y coordinates are set to -10 because this
    //is a value that will never be achieved
    else {
        circle_x = -10;
        circle_y = -10;
    }
    //Setting the values that will be published
    robot::circle_detect_msg pub_msg;
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.header.frame_id = "/robot";
    pub_msg.circle_x = circle_x;
    pub_msg.circle_y = circle_y;
    pub_msg.ranges = msg->ranges;
    circle_detect_pub_.publish(pub_msg);
}
