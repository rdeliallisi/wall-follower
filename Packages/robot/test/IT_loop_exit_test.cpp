/**
 * @file IT_loop_exit_test.cpp
 * @brief Integration test for successful loop exit in hard world
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


TEST(RobotHardIntegrationTestLoopExit, RobotLoopExitSuccess) {
    HighLevelControl high_level_control;
    high_level_control.set_turn_type(LEFT);
    MoveStatus move_status = high_level_control.get_move_status();
    
    ros::NodeHandle n;
    ros::Rate r(10.0);

    int i = 0;
    while(ros::ok() && i <= 400) {
        ros::spinOnce();
        i++;
        r.sleep();
    }

    move_status = high_level_control.get_move_status();
    ASSERT_TRUE(move_status.can_continue_);
    ASSERT_FALSE(move_status.is_close_to_wall_);
    ASSERT_FALSE(move_status.is_following_wall_);
    ASSERT_FALSE(move_status.circle_hit_mode_);
    ASSERT_FALSE(move_status.hit_goal_);    
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "IT_loop_exit_test");
    return RUN_ALL_TESTS();
}
