#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "math.h"

#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace std;
using namespace cv;

/**
 * @brief Creates a white image of given dimensions
 * 
 * @param screen_width 
 * @param screen_height 
 * 
 * @return image cv::Mat
 */
cv::Mat create_image(int screen_width, int screen_height) {
    cv::Mat image;
    image.create(screen_width, screen_height, CV_8UC1);
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            image.at<uchar>(i, j) = (uchar)255;
        }
    }
    return image;
}

/**
 * @brief callback function for laser_scan
 * @details Gets the date from the laser range finder, creates an
 * image out of it and runs openCV HoughCircles on it
 * 
 * @param msg sensor_msgs::LaserScan
 */
void callback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    int data_points = msg->ranges.size();

    //create image
    int screen_width = 1000;
    int screen_height = 1000;
    cv::Mat image = create_image(screen_width, screen_height);

    //convert laser_scan data to image
    float base_scan_min_angle = msg->angle_min;
    for (int i = 0; i < data_points; ++i) {
        //calculate cartesian coordinates
        float cartesian_x = (msg->ranges[data_points - 1 - i] * sin(base_scan_min_angle)) * 100;
        float cartesian_y = (msg->ranges[data_points - 1 - i] * cos(base_scan_min_angle)) * 100;
        base_scan_min_angle += msg->angle_increment;

        //convert to screen coordinates
        int screen_x = (int)((cartesian_x + screen_width / 2));
        int screen_y = (int)((-cartesian_y + screen_height / 2));

        if (screen_x > 0 && screen_y > 0) {
            image.at<uchar>(screen_y, screen_x) = (uchar)0;
        } else {
            cout << screen_x << endl;
            cout << screen_y << endl;
        }
    }

    //compute Hough Transform
    Mat destination;
    Canny(image, destination, 200, 20);
    GaussianBlur(destination, destination, Size(7, 7), 2, 2 );

    vector<Vec3f> circles;
    HoughCircles(destination, circles, CV_HOUGH_GRADIENT, 1, 100, 200, 15, 10, 20);

    cout << "Circles: " << circles.size() << endl;

    // Draw the circles detected
    for ( size_t i = 0; i < circles.size(); i++ ) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle( image, center, 3, Scalar(0, 255, 255), -1);
        cv::circle( image, center, radius, Scalar(0, 0, 255), 1 );
    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", image );
    waitKey(-1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_detector");
    ros::NodeHandle n;

    ros::Rate loop_rate(50);
    ros::Subscriber sub = n.subscribe("base_scan", 1000, callback);
    ros::spin();
    loop_rate.sleep();
    return 0;
}
