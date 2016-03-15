/** 
 * @file high_level_control.cpp
 * @brief This file provides the implementation of the HighLevelControl class
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "high_level_control.h"

HighLevelControl::HighLevelControl() : node_() {
    security_distance_ = 0.25;
    wall_follow_distance_ = 0.35;
    linear_velocity_ = 0.4;
    angular_velocity_ = 1;

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    laser_sub_ = node_.subscribe("base_scan", 100, &HighLevelControl::LaserCallback, this);
    can_continue_ = true;
    is_close_to_wall_ = false;
    is_turning_ = false;
    is_following_wall_ = false;
}

void HighLevelControl::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    std::vector<float>::iterator it = std::min_element(ranges.begin(), ranges.end());
   
    if (*it > security_distance_) {
        can_continue_ = true;
    } else {
        can_continue_ = false;
    }

    if (is_following_wall_) {
        int low_lim, high_lim;

        if (turn_type_ == 1) {
            low_lim = 50;
            high_lim = 150;
        } else if (turn_type_ == -1) {
            low_lim = 520;
            high_lim = 670;
        } else {
            // This case shoud never happen
            ros::shutdown();
        }

        std::vector<float>::iterator wall_it = std::min_element(ranges.begin() + low_lim,
                                               ranges.begin() + high_lim);

        if (*wall_it < wall_follow_distance_) {
            is_close_to_wall_ = true;
        } else {
            is_close_to_wall_ = false;
        }
    }
}

void HighLevelControl::set_security_distance(double security_distance) {
    security_distance_ = security_distance;
}

void HighLevelControl::set_linear_velocity(double linear_velocity) {
    linear_velocity_ = linear_velocity;
}

void HighLevelControl::set_angular_velocity(double angular_velocity) {
    angular_velocity_ = angular_velocity;
}

void HighLevelControl::set_turn_type(int turn_type) {
    turn_type_ = turn_type;
    is_following_wall_ = true;
}

void HighLevelControl::WallFollowMove() {
    if (!can_continue_ && !is_following_wall_) {
        srand(time(NULL));

        // If we are in override mode don't change the type of the wall
        if(turn_type_ != 1 && turn_type_ != -1)
            turn_type_ = rand() % 2 == 0 ? -1 : 1;

        is_following_wall_ = true;
    } else if (can_continue_ && !is_following_wall_) {
        Move(linear_velocity_, 0);
    } else {
        if (can_continue_ && is_close_to_wall_) {
            Move(linear_velocity_, 0);
        } else if (!can_continue_) {
            Move(0, turn_type_ * angular_velocity_);
        } else if (!is_close_to_wall_) {
            Move(0, -turn_type_ * angular_velocity_);
        }
    }
}

void HighLevelControl::ControlledRandomMove() {
    if (can_continue_) {
        Move(linear_velocity_, 0);
        is_turning_ = false;
    } else {
        if (!is_turning_) {
            turn_type_ = rand() % 2 == 0 ? -1 : 1;
            is_turning_ = true;
        }

        Move(0, angular_velocity_ * turn_type_);
    }
}

void HighLevelControl::TotalRandomMove() {
    if(can_continue_) {
        Move(linear_velocity_, 0);
    } else {
        srand(time(NULL));
        int turn_factor = rand() % 20 - 10;
        Move(0, turn_factor * angular_velocity_);
    }
}

//TODO: To be put into Low Level Control
void HighLevelControl::Move(double linear_velocity, double angular_velocity) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;
    cmd_vel_pub_.publish(msg);
}
