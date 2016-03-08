#include <gtest/gtest.h>
#include <ros/ros.h>
#include "circle_detector.h"



TEST(CircleDetectTest, CircleDetected) {
	CircleDetector circle_detector;
	ASSERT_EQ(circle_detector.circle_.x, -10)
	ASSERT_EQ(circle_detector.circle_.y, -10)
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "high_level_control_test");
	return RUN_ALL_TESTS();
}