/**
 * @file CD_circle_present_test.cpp
 * @brief Test cases for circles when they are present.
 * 
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "circle_detector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

TEST(CircleDetectTest, CircleDetected) {
	CircleDetector circle_detector;

	ros::Rate r(10.0);
	int i = 0;
	while(ros::ok() && i < 20) {
		ros::spinOnce();
		i++;
		r.sleep();
	}

	ASSERT_NE(circle_detector.get_circle().x, -10);
	ASSERT_NE(circle_detector.get_circle().y, -10);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "circle_detector_test");
	return RUN_ALL_TESTS();
}
