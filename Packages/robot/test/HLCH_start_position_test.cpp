/** 
 * @file HLCH_start_position_test.cpp
 * @brief This file tests that the robot goes to the wall in from and follows it
 * starting from a given fixed position
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */


#include <gtest/gtest.h>
#include <ros/ros.h>
#include "high_level_control.h"

TEST(MovementStateTest, InitialState) {
	HighLevelControl high_level_control;
	ASSERT_TRUE(high_level_control.can_continue());
	ASSERT_FALSE(high_level_control.is_close_to_wall());
	ASSERT_FALSE(high_level_control.is_following_wall());
	ASSERT_FALSE(high_level_control.is_turning());
}

TEST(MovementStateTest, MoveUntilFoundWall) {
	HighLevelControl high_level_control;
	ASSERT_TRUE(high_level_control.can_continue());
	ASSERT_FALSE(high_level_control.is_following_wall());

	ros::Rate r(10.0);
	int i = 0;
	while (ros::ok() && i < 10)
	{
		high_level_control.WallFollowMove();

		ros::spinOnce();
		i++;
		r.sleep();
	}

	ASSERT_FALSE(high_level_control.can_continue());
	ASSERT_TRUE(high_level_control.is_following_wall());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "high_level_control_test");
	return RUN_ALL_TESTS();
}