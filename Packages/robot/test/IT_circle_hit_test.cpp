/**
 * @file IT_circle_hit_test.cpp
 * @brief Integration test for successful detection of circle in easy world
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "high_level_control.h"
#include "move_helpers.h"


struct AnyHelper {
    bool circle_hit_;

    AnyHelper(){
        circle_hit_ = false;
    }

    void cb(const sensor_msgs::LaserScan::ConstPtr &msg) {
        std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
        if(ranges[ranges.size() / 2] < 0.05) {
            circle_hit_ = true;
        }
    }
};

TEST(RobotEasyIntegrationTestCircleHit, RobotCircleHit) {
    HighLevelControl high_level_control;
    high_level_control.set_turn_type(RIGHT);

    ros::NodeHandle n;
    ros::Rate r(10.0);
    AnyHelper h;

    ros::Subscriber test_sub_ = n.subscribe("base_scan", 100, &AnyHelper::cb, &h);
    int i = 0;
    while(ros::ok() && i <= 100) {
        ros::spinOnce();
        i++;
        r.sleep();
    }

    ASSERT_TRUE(h.circle_hit_);
    
    MoveStatus move_status = high_level_control.get_move_status();
    ASSERT_TRUE(move_status.can_continue_);
    ASSERT_TRUE(move_status.is_close_to_wall_);
    ASSERT_TRUE(move_status.is_following_wall_);
    ASSERT_TRUE(move_status.circle_hit_mode_);
    ASSERT_TRUE(move_status.hit_goal_);    
    ASSERT_EQ(move_status.count_turn_, 0);
    ASSERT_EQ(move_status.last_turn_, 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "IT_circle_hit_test");
    return RUN_ALL_TESTS();
}
