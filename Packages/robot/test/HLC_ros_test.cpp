#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "high_level_control.h"
#include "move_helpers.h"

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

TEST (HlcMove, ZeroCoordinates) {
	ros::NodeHandle n;
	ros::Rate r(10.0);
	HighLevelControl high_level_control;
	AnyHelper h;
	ros::Subscriber test_sub = n.subscribe("cmd_vel", 100, &AnyHelper::cb, &h);
	high_level_control.Move(0, 0);
	int i = 0;
	while (ros::ok() && i < 5) {
		ros::spinOnce();
		r.sleep();
		i++;
	}
	ASSERT_FLOAT_EQ(h.linear_velocity, 0);
	ASSERT_FLOAT_EQ(h.angular_velocity, 0);
}

TEST(HlcMove, PositiveCoordinates) {
	ros::NodeHandle n;
	ros::Rate r(10.0);
	HighLevelControl high_level_control;
	AnyHelper h;
	ros::Subscriber test_sub = n.subscribe("cmd_vel", 100, &AnyHelper::cb, &h);
	high_level_control.Move(10, 5);
	int i = 0;
	while (ros::ok() && i < 5) {
		ros::spinOnce();
		r.sleep();
		i++;
	}
	ASSERT_FLOAT_EQ(h.linear_velocity, 10);
	ASSERT_FLOAT_EQ(h.angular_velocity, 5);
}

TEST(HlcMove, NegativeCoordinates) {
	ros::NodeHandle n;
	ros::Rate r(10.0);
	HighLevelControl high_level_control;
	AnyHelper h;
	ros::Subscriber test_sub = n.subscribe("cmd_vel", 100, &AnyHelper::cb, &h);
	high_level_control.Move(-10, -10);
	int i = 0;
	while (ros::ok() && i < 5) {
		ros::spinOnce();
		r.sleep();
		i++;
	}
	ASSERT_FLOAT_EQ(h.linear_velocity, -10);
	ASSERT_FLOAT_EQ(h.angular_velocity, -10);
}



int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "HLC_ros_test");
	return RUN_ALL_TESTS();
}