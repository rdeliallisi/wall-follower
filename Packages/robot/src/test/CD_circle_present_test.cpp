#include <gtest/gtest.h>
#include <ros/ros.h>
#include "circle_detector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

TEST(CircleDetectTest, CircleDetected) {
	CircleDetector circle_detector;
	ASSERT_NE(circle_detector.circle_.x, -10)
	ASSERT_NE(circle_detector.circle_.y, -10)
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "circle_detector_test");
	return RUN_ALL_TESTS();
}
