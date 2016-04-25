/**
 * @file HLC_unit_test_real.cpp
 * @brief Unit test for the High Level Control
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
#include "move_helpers.h"

TEST(HlcCreate, InitMoveStatus) {
	HighLevelControl high_level_control;
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_TRUE(move_status.can_continue_);
	ASSERT_FALSE(move_status.is_close_to_wall_);
	ASSERT_FALSE(move_status.is_following_wall_);
	ASSERT_FALSE(move_status.circle_hit_mode_);
	ASSERT_FALSE(move_status.hit_goal_);
	ASSERT_EQ(move_status.last_turn_, 0);
	ASSERT_EQ(move_status.count_turn_, 0);
}

TEST(HlcCreate, InitMoveSpecs) {
	HighLevelControl high_level_control;
	MoveSpecs move_specs = high_level_control.get_move_specs();
	ASSERT_DOUBLE_EQ(move_specs.high_security_distance_, 0.20);
	ASSERT_DOUBLE_EQ(move_specs.low_security_distance_, 0.10);
	ASSERT_DOUBLE_EQ(move_specs.wall_follow_distance_, 0.4);
	ASSERT_DOUBLE_EQ(move_specs.linear_velocity_, 0.5);
	ASSERT_DOUBLE_EQ(move_specs.angular_velocity_, 1);
}

TEST(HlcCanCotinue, NoCase) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.CanContinue(1, 1, 1);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_TRUE(move_status.can_continue_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.CanContinue(1, 1, 1);
	move_status = high_level_control.get_move_status();
	ASSERT_TRUE(move_status.can_continue_);
}

TEST(HlcCanCotinue, HighSecurityTest) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.CanContinue(0.2, 1, 1);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.can_continue_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.CanContinue(1, 0.2, 1);
	move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.can_continue_);
}

TEST(HlcCanCotinue, LowSecurityTest) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.CanContinue(1, -1, 1);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.can_continue_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.CanContinue(-1, 1, 1);
	move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.can_continue_);
}

TEST(HlcCanCotinue, TwoSecurityTest) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.CanContinue(0.2, -1, 1);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.can_continue_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.CanContinue(-1, 0.2, 1);
	move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.can_continue_);
}

TEST(HlcCanCotinue, CloseSecurityTest) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.CanContinue(0.29, -1, 1);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.can_continue_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.CanContinue(-1, 0.29, 1);
	move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.can_continue_);
}

TEST(HlcIsCloseToWall, NotClose) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.IsCloseToWall(0.5, 1, 1);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.is_close_to_wall_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.IsCloseToWall(1, 0.5, 1);
	move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.is_close_to_wall_);
}

TEST(HlcIsCloseToWall, Close) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.IsCloseToWall(0.3, 1, 5);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_TRUE(move_status.is_close_to_wall_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.IsCloseToWall(1, 0.3, 5);
	move_status = high_level_control.get_move_status();
	ASSERT_TRUE(move_status.is_close_to_wall_);
}

TEST(HlcIsCloseToWall, CloseOnOtherSide) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.IsCloseToWall(1, 0.3, 1);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.is_close_to_wall_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.IsCloseToWall(0.3, 1, 1);
	move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.is_close_to_wall_);
}

TEST(HlcIsCloseToWall, CloseMiddle) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	high_level_control.IsCloseToWall(1, 1, 0.3);
	MoveStatus move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.is_close_to_wall_);

	high_level_control.set_turn_type(LEFT);
	high_level_control.IsCloseToWall(1, 1, 0.3);
	move_status = high_level_control.get_move_status();
	ASSERT_FALSE(move_status.is_close_to_wall_);
}

TEST(HlcCanHit, NoneCase) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(NONE);
	std::vector<float> test_vector;
	for (int i = 0; i < 720; i++) {
		test_vector.push_back(1);
	}
	ASSERT_FALSE(high_level_control.CanHit(0.1, 0.1, test_vector));
}

TEST(HlcCanHit, YesCase) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	std::vector<float> right_vector;
	int i;
	for (i = 0; i < 380; i++) {
		right_vector.push_back(0.1);
	}

	for (i = 380; i < 720; i++) {
		right_vector.push_back(5);
	}
	ASSERT_TRUE(high_level_control.CanHit(0, 0.5, right_vector));

	high_level_control.set_turn_type(LEFT);
	std::vector<float> left_vector;
	for (i = 0; i < 340; i++) {
		left_vector.push_back(5);
	}

	for (i = 340; i < 720; i++) {
		left_vector.push_back(0.1);
	}
	ASSERT_TRUE(high_level_control.CanHit(0, 0.5, left_vector));
}

TEST(HlcCanHit, ToFarCase) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	std::vector<float> right_vector;
	int i;
	for (i = 0; i < 380; i++) {
		right_vector.push_back(0.1);
	}

	for (i = 380; i < 720; i++) {
		right_vector.push_back(5);
	}
	ASSERT_FALSE(high_level_control.CanHit(0, 1.1, right_vector));

	high_level_control.set_turn_type(LEFT);
	std::vector<float> left_vector;
	for (i = 0; i < 340; i++) {
		left_vector.push_back(5);
	}

	for (i = 340; i < 720; i++) {
		left_vector.push_back(0.1);
	}
	ASSERT_FALSE(high_level_control.CanHit(0, 1.1, left_vector));
}

TEST(HlcCanHit, BigXCase) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	std::vector<float> right_vector;
	int i;
	for (i = 0; i < 380; i++) {
		right_vector.push_back(0.1);
	}

	for (i = 380; i < 720; i++) {
		right_vector.push_back(5);
	}
	ASSERT_FALSE(high_level_control.CanHit(1, 0.5, right_vector));

	high_level_control.set_turn_type(LEFT);
	std::vector<float> left_vector;
	for (i = 0; i < 340; i++) {
		left_vector.push_back(5);
	}

	for (i = 340; i < 720; i++) {
		left_vector.push_back(0.1);
	}
	ASSERT_FALSE(high_level_control.CanHit(-1, 0.5, left_vector));
}

TEST(HlcCanHit, CornerCaseCase) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	std::vector<float> right_vector;
	int i;
	for (i = 0; i < 380; i++) {
		right_vector.push_back(0.1);
	}

	for (i = 380; i < 720; i++) {
		right_vector.push_back(1);
	}
	ASSERT_FALSE(high_level_control.CanHit(0, 0.5, right_vector));

	high_level_control.set_turn_type(LEFT);
	std::vector<float> left_vector;
	for (i = 0; i < 340; i++) {
		left_vector.push_back(1);
	}

	for (i = 340; i < 720; i++) {
		left_vector.push_back(0.1);
	}
	ASSERT_FALSE(high_level_control.CanHit(0, 0.5, left_vector));
}

TEST(HlcCanHit, InsideCase) {
	HighLevelControl high_level_control;
	high_level_control.set_turn_type(RIGHT);
	std::vector<float> right_vector;
	int i;
	for (i = 0; i < 380; i++) {
		right_vector.push_back(0.6);
	}

	for (i = 380; i < 720; i++) {
		right_vector.push_back(5);
	}
	ASSERT_FALSE(high_level_control.CanHit(0, 0.5, right_vector));

	high_level_control.set_turn_type(LEFT);
	std::vector<float> left_vector;
	for (i = 0; i < 340; i++) {
		left_vector.push_back(5);
	}

	for (i = 340; i < 720; i++) {
		left_vector.push_back(0.6);
	}
	ASSERT_FALSE(high_level_control.CanHit(0, 0.5, left_vector));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "HLC_unit_test_real");
	return RUN_ALL_TESTS();
}
