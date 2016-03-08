#include <gtest/gtest.h>
#include <ros/ros.h>
#include "circle_detector_node.cpp"

TEST(CircleDetectTest, CreateImage) {
	CircleDetector circle_detector;
    

}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "circle_detector_node_test");
	return RUN_ALL_TESTS();
}

