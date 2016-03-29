/**
 * @file ST_robot_hard_test.cpp
 * @brief System test for successful completion of goal in hard world
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "robot/circle_detect_msg.h"

struct AnyHelper {
    AnyHelper() : circle_x(-10), circle_y(-10) {
    }

    void cb(const robot::circle_detect_msg::ConstPtr& msg) {
        circle_x = msg->circle_x;
        circle_y = msg->circle_y;
    }

    float circle_x;
    float circle_y;
};

TEST(RobotHardSystemTest, RobotSuccess) {
    ros::Rate r(10.0);
    ros::NodeHandle n;
    AnyHelper h;
    ros::Subscriber test_sub_ = n.subscribe("circle_detect", 100, &AnyHelper::cb, &h);
    while(ros::ok() && h.circle_x >= -0.1 && h.circle_x <= 0.1) {
        ros::spinOnce();
        r.sleep();
    }
    ASSERT_LE(h.circle_x, 0.1);
    ASSERT_GE(h.circle_x, -0.1);
    ASSERT_LE(h.circle_y, 0.1);
    ASSERT_GE(h.circle_y, -0.1);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "robot_hard_system_test");
    return RUN_ALL_TESTS();
}
