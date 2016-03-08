#include <gtest/gtest.h>
#include <ros/ros.h>
#include "circle_detector.h"



TEST(CircleDetectTest, CircleDetected) {
	CircleDetector circle_detector;
	ASSERT_FLOAT_EQ(circle_detector.get_circle().x, -10);
	ASSERT_FLOAT_EQ(circle_detector.get_circle().y, -10);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "high_level_control_test");
	return RUN_ALL_TESTS();
}