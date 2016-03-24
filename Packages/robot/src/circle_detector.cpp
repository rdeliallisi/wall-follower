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
#include "circle_detector.h"

#include "math.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace std;
using namespace cv;


CircleDetector::CircleDetector() : node_() , circle_() {
    count_threshold_ = 0;
    circle_.x = -10;
    circle_.y = -10;
    laser_sub_ = node_.subscribe("base_scan", 100, 
        &CircleDetector::LaserCallback, this);
}

Circle CircleDetector::get_circle() {
    return circle_;
}

void CircleDetector::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    int data_points = msg->ranges.size();

    //create image
    int screen_width = 1000;
    int screen_height = 1000;

    cv::Mat image;
    image.create(screen_width, screen_height, CV_8UC1);
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            image.at<uchar>(i, j) = (uchar)255;
        }
    }

    //convert laser_scan data to image
    float base_scan_min_angle = msg->angle_min;
    for (int i = 0; i < data_points; ++i) {
        //calculate cartesian coordinates
        float cartesian_x = (msg->ranges[data_points - 1 - i] * 
            sin(base_scan_min_angle)) * 100;
        float cartesian_y = (msg->ranges[data_points - 1 - i] * 
            cos(base_scan_min_angle)) * 100;
        base_scan_min_angle += msg->angle_increment;

        //convert to screen coordinates
        int screen_x = (int)((cartesian_x + screen_width / 2));
        int screen_y = (int)((-cartesian_y + screen_height / 2));

        if (screen_x > 0 && screen_y > 0) {
            image.at<uchar>(screen_y, screen_x) = (uchar)0;
        } else {
            // Coordinates are out of bound becaus of roundoff errors
        }
    }

    //compute Hough Transform
    Mat destination;
    Canny(image, destination, 200, 20);
    GaussianBlur(destination, destination, Size(7, 7), 2, 2 );

    vector<Vec3f> circles;
    HoughCircles(destination, circles, CV_HOUGH_GRADIENT, 1, 100, 200, 15, 10,
    20);

    if (circles.size() != 1) {
        count_threshold_ = 0;
        circle_.x = circle_.y = -10;
    } else {
        count_threshold_++;

        if (count_threshold_ > 0) {
            circle_.x = (circles[0][0] - screen_width / 2) / 100.0;
            circle_.y = -(circles[0][1] - screen_height / 2) / 100.0;

        }
    }

    //Draw the circles detected and display them
    // for ( size_t i = 0; i < circles.size(); i++ ) {
    //     Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    //     int radius = cvRound(circles[i][2]);
    //     cv::circle( image, center, 3, Scalar(0, 255, 255), -1);
    //     cv::circle( image, center, radius, Scalar(0, 0, 255), 1 );
    // }

    // namedWindow( "Display window", WINDOW_AUTOSIZE );
    // imshow( "Display window", image );
    // waitKey(-1);
}
