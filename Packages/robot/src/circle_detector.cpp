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
    LoadParams();
    LoadTopics();
}

void CircleDetector::LoadTopics() {
    bool loaded = true;
    std::string laser_topic, circle_topic;

    if (!node_.getParam("laser_topic",
                        laser_topic)) {
        loaded = false;
    }

    if (!node_.getParam("circle_topic",
                        circle_topic)) {
        loaded = false;
    }

    if (loaded == false) {
        ROS_INFO("Topics failed to load!");
        ros::shutdown();
    }

    //subscribe the node
    laser_sub_ = node_.subscribe(laser_topic, 100,
                                 &CircleDetector::LaserCallback, this);
    circle_detect_pub_ = node_.advertise<robot::circle_detect_msg>(
                             circle_topic, 100);
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
    //initialize the scale factor variable
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
        ROS_INFO("Failed to load params!");
        ros::shutdown();
    }
}

cv::Mat CircleDetector::CreateImage(const sensor_msgs::LaserScan::ConstPtr& msg) {
    size_t data_points = msg->ranges.size();

    //create image
    int screen_width = 1000;
    int screen_height = 1000;
    const float lrf_max_range = 2;

    cv::Mat image(screen_width, screen_height, CV_8UC1, Scalar(0));

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
                ROS_INFO("Round off error: Coordinates out of bounds!");
            }
        }
    }

    return image;
}

vector<Vec3f> CircleDetector::FindCircles(cv::Mat& image) {
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

    return circles;
}

//Define the LaserCallBack method which turns the maze into an image and then applies Hough Transform
void CircleDetector::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    LoadParams();
    cv::Mat image = CreateImage(msg);

    //compute Hough Transform
    vector<Vec3f> circles = FindCircles(image);

    // declare the x and y coordinates of the circle
    double circle_x, circle_y;
    TransformCircle(circle_x, circle_y, image, circles);

    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    PublishCircle(circle_x, circle_y, ranges);
}

void CircleDetector::TransformCircle(double& circle_x, double& circle_y,
                                     cv::Mat& image, std::vector<Vec3f>& circles) {
    //If the circle is found then the coordinates are converted to screen coordinates
    if (circles.size() == 1) {
        circle_x = (circles[0][0] - image.rows / 2) / 100;
        circle_y = -((circles[0][1] - image.cols / 2) / 100);
    }
    //If the circle is not found, then the x and y coordinates are set to -10 because this
    //is a value that will never be achieved
    else {
        circle_x = -10;
        circle_y = -10;
    }
}

void CircleDetector::PublishCircle(double circle_x, double circle_y, std::vector<float>& ranges) {
    //Setting the values that will be published
    robot::circle_detect_msg pub_msg;
    pub_msg.header.stamp = ros::Time::now();
    pub_msg.header.frame_id = "/robot";
    pub_msg.circle_x = circle_x;
    pub_msg.circle_y = circle_y;
    pub_msg.ranges = ranges;
    circle_detect_pub_.publish(pub_msg);
}
