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
    InitialiseMoveSpecs();
    InitialiseMoveStatus();

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    laser_sub_ = node_.subscribe("base_scan", 100, &HighLevelControl::LaserCallback, this);
}

void HighLevelControl::InitialiseMoveSpecs() {
    move_specs_.security_distance_ = 0.275;
    move_specs_.wall_follow_distance_ = 0.4;
    move_specs_.linear_velocity_ = 0.4;
    move_specs_.angular_velocity_ = 1;
    move_specs_.turn_type_ = NONE;
}

void HighLevelControl::InitialiseMoveStatus() {
    move_status_.can_continue_ = true;
    move_status_.is_close_to_wall_ = false;
    move_status_.is_turning_ = false;
    move_status_.is_following_wall_ = false;
}

void HighLevelControl::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    std::vector<float>::iterator it = std::min_element(ranges.begin(), ranges.end());

    if (*it > move_specs_.security_distance_) {
        move_status_.can_continue_ = true;
    } else {
        move_status_.can_continue_ = false;
    }

    if (move_status_.is_following_wall_) {
        int low_lim, high_lim;

        if (move_specs_.turn_type_ == LEFT) {
            low_lim = 0;
            high_lim = 250;
        } else if (move_specs_.turn_type_ == RIGHT) {
            low_lim = 470;
            high_lim = 720;
        } else {
            // This case shoud never happen
            ros::shutdown();
        }

        std::vector<float>::iterator wall_it = std::min_element(ranges.begin() + low_lim,
                                               ranges.begin() + high_lim);

        if (*wall_it < move_specs_.wall_follow_distance_) {
            move_status_.is_close_to_wall_ = true;
        } else {
            move_status_.is_close_to_wall_ = false;
        }
    }
}

void HighLevelControl::set_security_distance(double security_distance) {
    move_specs_.security_distance_ = security_distance;
}

void HighLevelControl::set_linear_velocity(double linear_velocity) {
    move_specs_.linear_velocity_ = linear_velocity;
}

void HighLevelControl::set_angular_velocity(double angular_velocity) {
    move_specs_.angular_velocity_ = angular_velocity;
}

void HighLevelControl::set_turn_type(TurnType turn_type) {
    move_specs_.turn_type_ = turn_type;
    move_status_.is_following_wall_ = true;
}

void HighLevelControl::WallFollowMove() {
    if (!move_status_.can_continue_ && !move_status_.is_following_wall_) {
        srand(time(NULL));

        // If we are in override mode don't change the type of the wall
        if (move_specs_.turn_type_ == NONE) {
            move_specs_.turn_type_ = rand() % 2 == 0 ? RIGHT : LEFT;
        }

        move_status_.is_following_wall_ = true;
    } else if (move_status_.can_continue_ && !move_status_.is_following_wall_) {
        Move(move_specs_.linear_velocity_, 0);
    } else {
        if (move_status_.can_continue_ && move_status_.is_close_to_wall_) {
            Move(move_specs_.linear_velocity_, 0);
        } else if (!move_status_.can_continue_) {
            Move(0, (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_);
        } else if (!move_status_.is_close_to_wall_) {
            Move(0, -1 * (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_);
        }
    }
}

//TODO: To be put into Low Level Control
void HighLevelControl::Move(double linear_velocity, double angular_velocity) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;
    cmd_vel_pub_.publish(msg);
}
