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

#include "math.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "detect_helpers.h"

CircleDetector::CircleDetector() : node_() , circle_(), cartesian_() {
    circle_.x = -10;
    circle_.y = -10;
    cartesian_.x = 0;
    cartesian_.y = 0;
    laser_sub_ = node_.subscribe("base_scan", 100,
                                 &CircleDetector::LaserCallback, this);
    circle_detect_pub_ = node_.advertise<robot::circle_detect_msg>(
                             "circle_detect", 100);
    LoadParams();
}

Circle CircleDetector::get_circle() {
    return circle_;
}

CartesianCoordinates CircleDetector::get_cartesian_coordinates() {
    return cartesian_;
}

ScreenCoordinates CircleDetector::get_screen_coordinates() {
    return screen_;
}

void CircleDetector::ConvertLaserScanToCartesian(int i, float base_scan_min_angle,
        const sensor_msgs::LaserScan::ConstPtr& msg) {

    size_t data_points = msg->ranges.size();

    cartesian_.x = (msg->ranges[data_points - 1 - i] *
                    sin(base_scan_min_angle)) * 100;
    cartesian_.y = (msg->ranges[data_points - 1 - i] *
                    cos(base_scan_min_angle)) * 100;
}

void CircleDetector::ConvertCartesianToScreen(int screen_width, int screen_height) {
    screen_.x = static_cast<int>(cartesian_.x + screen_width / 2);
    screen_.y = static_cast<int>(-cartesian_.y + screen_height / 2);
}

void CircleDetector::ConvertLaserScanToCartesian(float range, float base_scan_min_angle) {
    const int scale_factor = 100;
    cartesian_.x = (range * sin(base_scan_min_angle)) * scale_factor;
    cartesian_.y = (range * cos(base_scan_min_angle)) * scale_factor;
}

void CircleDetector::LoadParams() {
    bool loaded = true;

    if (!node_.getParam("/CircleDetector/canny_threshold_1",
                        canny_params_.threshold_1_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/canny_threshold_2",
                        canny_params_.threshold_2_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/blur_kernel_size",
                        blur_params_.kernel_size_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/blur_sigma",
                        blur_params_.sigma_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/hough_threshold_1",
                        hough_params_.threshold_1_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/hough_threshold_2",
                        hough_params_.threshold_2_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/hough_dp",
                        hough_params_.dp_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/hough_min_dist",
                        hough_params_.min_dist_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/hough_min_radius",
                        hough_params_.min_radius_)) {
        loaded = false;
    }

    if (!node_.getParam("/CircleDetector/hough_max_radius",
                        hough_params_.max_radius_)) {
        loaded = false;
    }

    if (!loaded) {
        ros::shutdown();
    }
}

void CircleDetector::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    LoadParams();

    size_t data_points = msg->ranges.size();

    //create image
    int screen_width = 1000;
    int screen_height = 1000;

    cv::Mat image;
    image.create(screen_width, screen_height, CV_8UC1);
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            image.at<uchar>(i, j) = static_cast<uchar>(255);
        }
    }

    //convert laser_scan data to image
    float base_scan_min_angle = msg->angle_min;
    for (int i = 0; i < data_points; ++i) {

        const float lrf_max_range = 5;
        float range = msg->ranges[data_points - 1 - i];
        if (range < lrf_max_range) {
            ConvertLaserScanToCartesian(range, base_scan_min_angle);
            ConvertCartesianToScreen(screen_width, screen_height);
            base_scan_min_angle += msg->angle_increment;

            if (screen_.x >= 0 && screen_.y >= 0) {
                image.at<uchar>(screen_.y, screen_.x) = static_cast<uchar>(0);
            } else {
                // Coordinates are out of bound because of roundoff errors
                ROS_INFO("Round off error: Coordinates out of bound");
            }
        }
    }
    //compute Hough Transform
    cv::Mat destination;
    cv::Canny(image, destination, canny_params_.threshold_1_,
              canny_params_.threshold_2_);
    cv::GaussianBlur(destination, destination,
                     Size(blur_params_.kernel_size_, blur_params_.kernel_size_),
                     blur_params_.sigma_, blur_params_.sigma_);

    vector<Vec3f> circles;
    cv::HoughCircles(destination, circles, CV_HOUGH_GRADIENT,
                     hough_params_.dp_, hough_params_.min_dist_,
                     hough_params_.threshold_1_, hough_params_.threshold_2_,
                     hough_params_.min_radius_, hough_params_.max_radius_);

    //RenderImage(circles, destination);

        robot::circle_detect_msg pub_msg;
        pub_msg.header.stamp = ros::Time::now();
        pub_msg.header.frame_id = "/robot";
        pub_msg.circle_x = circle_.x;
        pub_msg.circle_y = circle_.y;
        pub_msg.ranges = msg->ranges;
        circle_detect_pub_.publish(pub_msg);
}

void CircleDetector::RenderImage(vector<Vec3f> circles, cv::Mat image) {
    for ( size_t i = 0; i < circles.size(); i++ ) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle( image, center, 3, Scalar(255), -1);
        cv::circle( image, center, radius, Scalar(255), 1 );
    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", image );
    waitKey(10);
}
