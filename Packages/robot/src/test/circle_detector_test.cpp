#include <gtest/gtest.h>
#include <ros/ros.h>
#include "circle_detector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//TODO(atabakhafeez): Write complete test case
TEST(CircleDetectTest, CircleDetected) {
	
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "circle_detector_test");
	return RUN_ALL_TESTS();
}

