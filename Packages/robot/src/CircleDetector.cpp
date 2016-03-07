#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>



#include "math.h"

#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#define number_of_values 719

using namespace std;
using namespace cv;

class CvImage
{
public:
  std_msgs::Header header;
  std::string encoding;
  cv::Mat image_cv;
};

typedef boost::shared_ptr<CvImage> image_bridge;



void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    laser_geometry::LaserProjection projector_;
    ros::Publisher point_cloud_publisher_;
    tf::TransformListener tfListener_;
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);

    sensor_msgs::Image image_; //cache the image message
    pcl::toROSMsg (cloud, image_);

    // Case 1: Always copy, returning a mutable CvImage
    cv::Mat image = cv_bridge::toCvCopy(image_)->image;
    // image_bridge = cv_bridge::toCvCopy(image_);
    // cv::Mat image = image_bridge.image_cv;

    vector<Vec3f> circles;
    HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, image.rows/4, 50, 10, 20, 40 );

    //cout << "number of lines = " << lines.sizea() << endl;
    cout << "number of circles = " << circles.size() << endl;

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ ) 
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle( image, center, 3, Scalar(0,255,255), -1);
        cv::circle( image, center, radius, Scalar(0,0,255), 1 );
    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", image );
    waitKey(0); 



}


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
    cv::Canny(image, destination, 200, 20);
/*
    vector<Vec4i> lines;
    HoughLinesP( destination, lines, 1, CV_PI/180, 100, 30, 10 );



*/
    GaussianBlur( image, image, Size(9, 9), 2, 2 );

    vector<Vec3f> circles;
    HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, image.rows/4, 50, 10, 20, 40 );



        
    //cout << "number of lines = " << lines.size() << endl;
    cout << "number of circles = " << circles.size() << endl;

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ ) 
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle( image, center, 3, Scalar(0,255,255), -1);
        cv::circle( image, center, radius, Scalar(0,0,255), 1 );
    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );
    imshow( "Display window", image );
    waitKey(0); 

    //compute distance transform:
    cv::Mat dt;
    cv::distanceTransform(255-(destination>0), dt, CV_DIST_L2 ,3);
    //cv::namedWindow("distance transform"); cv::imshow("distance transform", dt/255.0f);

    // test for semi-circles:
    float minInlierDist = 2.0f;
    for( size_t i = 0; i < circles.size(); i++ ) 
    {
        // test inlier percentage:
        // sample the circle and check for distance to the next edge
        unsigned int counter = 0;
        unsigned int inlier = 0;

        cv::Point2f center((circles[i][0]), (circles[i][2]));
        float radius = (circles[i][2]);

        // maximal distance of inlier might depend on the size of the circle
        float maxInlierDist = radius/25.0f;
        if(maxInlierDist<minInlierDist) maxInlierDist = minInlierDist;

        //TODO: maybe paramter incrementation might depend on circle size!
        for(float t =0; t<2*3.14159265359f; t+= 0.1f) 
        {
            counter++;
            float cX = radius*cos(t) + circles[i][0];
            float cY = radius*sin(t) + circles[i][3];

            if(dt.at<float>(cY,cX) < maxInlierDist) 
            {
                inlier++;
                cv::circle(image, cv::Point2i(cX,cY),3, cv::Scalar(0,255,0));
            } 
           else
                cv::circle(image, cv::Point2i(cX,cY),3, cv::Scalar(255,0,0));
        }
        std::cout << 100.0f*(float)inlier/(float)counter << " % of a circle with radius " << radius << " detected" << std::endl;
    }

    //TODO(atabakhfeez): Check if conversions of coordinates to original form are correct
    /*for (int i = 0; i < lines.size(); ++i){
        cout << (lines.at(i)[0])/100 << endl;
        float scaled_distance = (lines.at(i)[0])/100;
        float original_x = scaled_distance * cos(lines.at(i)[1]) - 5;
        float original_y = scaled_distance * sin(lines.at(i)[1]) - 5;

        float point_distance = sqrt(original_x * original_x + original_y * original_y);
        cout << "distance = " << point_distance << endl;

        cout << lines.at(i) << endl;
    }*/

    for (int i = 0; i < circles.size(); ++i) {
        cout << circles.at(i) << endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "line_detector");
    ros::NodeHandle n;

    ros::Rate loop_rate(50);
    ros::Subscriber sub = n.subscribe("base_scan", 1000, scanCallback);
    ros::spin();
    loop_rate.sleep();
    return 0;
}
