#include <gtest/gtest.h>
#include <ros/ros.h>
#include "high_level_control.h"

TEST(MovementStateTest, GettingAwayFromWall) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(1);
	ASSERT_TRUE(high_level_control.can_continue());
	ASSERT_TRUE(high_level_control.is_following_wall());
	ASSERT_FALSE(high_level_control.is_close_to_wall());

	ros::Rate r(10.0);
	int i = 0;
	while (ros::ok() && i < 20)
	{
		high_level_control.WallFollowMove();

		ros::spinOnce();
		i++;
		r.sleep();
	}

	ASSERT_TRUE(high_level_control.is_close_to_wall());
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "high_level_control_test");
	return RUN_ALL_TESTS();
}