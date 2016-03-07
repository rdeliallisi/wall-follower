#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "math.h"

#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#define number_of_values 719

using namespace std;
using namespace cv;

//TODO(atabakhafeez): Check and return correct values for coordinates
/**
* Gets the data from the laser range finder, creates an 
* image out of it and runs openCV HoughLines on it
* @param msg Raw data comming from the laser range finder
*/
void callback(const sensor_msgs::LaserScan& msg) {

    //create image
    int screen_width = 1000;
    int screen_height = 1000;
    cv::Mat image;
    image.create(screen_width, screen_height, CV_8UC1);
    for(int i = 0; i < image.rows; i++){
        for(int j = 0; j < image.cols; j++){
        image.at<uchar>(i,j) = (uchar)255;
        }
    }
                        
    //convert laser_scan data to image
    float base_scan_min_angle = -2.094395;
    for (int i = 0; i < number_of_values; ++i) {
        //calculate cartesian coordinates
        float cartesian_x = (msg.ranges[i] * sin(base_scan_min_angle)) * 100;
        float cartesian_y = (msg.ranges[i] * cos(base_scan_min_angle)) * 100;
        base_scan_min_angle += 0.005826;

        //convert to screen coordinates
        int screen_x = (int)((cartesian_x + screen_width/2));
        int screen_y = (int)((-cartesian_y + screen_height/2));

        if (screen_x > 0 && screen_y > 0) {
            image.at<uchar>(screen_y, screen_x) = (uchar)0;
        } else {
            cout << screen_x << endl;
            cout << screen_y << endl;
        }
    }

    //TODO(atabakhafeez): Delete code below. Only used for visual help.
    //namedWindow( "Display window", WINDOW_AUTOSIZE );
    //imshow( "Display window", image );
    //waitKey(0); 

    //compute Hough Transform
    cv::Mat destination;
    cv::Canny(image, destination, 50, 200, 3);

    vector<Vec4i> lines;
    HoughLinesP( destination, lines, 1, CV_PI/180, 100, 30, 10 );

    vector<Vec3f> circles;
    HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, image.rows/4, 200, 10, 0, 200 );

    cout << "number of lines = " << lines.size() << endl;
    cout << "number of circles = " << circles.size() << endl;

    //TODO(atabakhfeez): Check if conversions of coordinates to original form are correct
    for (int i = 0; i < lines.size(); ++i){
        cout << (lines.at(i)[0])/100 << endl;
        float scaled_distance = (lines.at(i)[0])/100;
        float original_x = scaled_distance * cos(lines.at(i)[1]) - 5;
        float original_y = scaled_distance * sin(lines.at(i)[1]) - 5;

        float point_distance = sqrt(original_x * original_x + original_y * original_y);
        cout << "distance = " << point_distance << endl;

        cout << lines.at(i) << endl;
    }

    for (int i = 0; i < circles.size(); ++i) {
        cout << circles.at(i) << endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "line_detector");
    ros::NodeHandle n;

    ros::Rate loop_rate(50);
    ros::Subscriber sub = n.subscribe("base_scan", 1000, callback);
    ros::spin();
    loop_rate.sleep();
    return 0;
}
