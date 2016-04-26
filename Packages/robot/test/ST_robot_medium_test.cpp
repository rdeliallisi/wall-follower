/**
 * @file ST_robot_medium_test.cpp
 * @brief System test for successful completion of goal in medium world
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

struct AnyHelper {
    AnyHelper() : linear_velocity(-1), angular_velocity(-1) {
    }

    void cb(const geometry_msgs::Twist::ConstPtr& msg) {
        linear_velocity = msg->linear.x;
        angular_velocity = msg->angular.z;
    }

    float linear_velocity;
    float angular_velocity;
};

TEST(RobotEasySystemTest, RobotSuccess) {
    ros::NodeHandle n;
    ros::Rate r(10.0);
    AnyHelper h;
    ros::Subscriber test_sub_ = n.subscribe("cmd_vel", 100, &AnyHelper::cb, &h);
    while (ros::ok()) {
        if (h.linear_velocity < 0.00001  && h.linear_velocity > -0.00001
                && h.angular_velocity < 0.00001 && h.angular_velocity > -0.00001)
            break;
        ros::spinOnce();
        r.sleep();
    }
    ASSERT_FLOAT_EQ(0, h.linear_velocity);
    ASSERT_FLOAT_EQ(0, h.angular_velocity);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "robot_medium_system_test");
    return RUN_ALL_TESTS();
}
