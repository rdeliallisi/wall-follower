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
    bool loaded = true;

    if (!node_.getParam("/HighLevelControl/high_security_distance",
                        move_specs_.high_security_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/low_security_distance",
                        move_specs_.low_security_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/wall_follow_distance",
                        move_specs_.wall_follow_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/max_linear_velocity",
                        move_specs_.max_linear_velocity_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/min_linear_velocity",
                        move_specs_.min_linear_velocity_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/angular_velocity",
                        move_specs_.angular_velocity_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/right_range_low_lim",
                        move_specs_.right_range_.low_lim_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/right_range_high_lim",
                        move_specs_.right_range_.high_lim_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/left_range_low_lim",
                        move_specs_.left_range_.low_lim_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/left_range_high_lim",
                        move_specs_.left_range_.high_lim_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/center_range_low_lim",
                        move_specs_.center_range_.low_lim_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/center_range_high_lim",
                        move_specs_.center_range_.high_lim_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/forward_range_low_lim",
                        move_specs_.forward_range_.low_lim_)) {
        loaded = false;
    }

    if (!node_.getParam("/HighLevelControl/forward_range_high_lim",
                        move_specs_.forward_range_.high_lim_)) {
        loaded = false;
    }

    move_specs_.linear_velocity_ = move_specs_.min_linear_velocity_;
    move_specs_.turn_type_ = NONE;

    if (loaded == false) {
        ros::shutdown();
    }
}

void HighLevelControl::InitialiseMoveStatus() {
    move_status_.can_continue_ = true;
    move_status_.is_close_to_wall_ = false;
    move_status_.is_following_wall_ = false;
}

void HighLevelControl::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    NormalMovement(ranges);
}

void HighLevelControl::NormalMovement(std::vector<float>& ranges) {
    double right_min_distance, left_min_distance, center_min_distance, forward_min_distance;

    right_min_distance = GetMin(ranges, move_specs_.right_range_.low_lim_,
                                move_specs_.right_range_.high_lim_);

    left_min_distance = GetMin(ranges, move_specs_.left_range_.low_lim_,
                               move_specs_.left_range_.high_lim_);

    center_min_distance = GetMin(ranges, move_specs_.center_range_.low_lim_,
                                 move_specs_.center_range_.high_lim_);

    forward_min_distance = GetMin(ranges, move_specs_.forward_range_.low_lim_,
                                  move_specs_.forward_range_.high_lim_);

    CanContinue(right_min_distance, left_min_distance, center_min_distance);

    SetLinearVelocity(forward_min_distance);

    IsCloseToWall(right_min_distance, left_min_distance, center_min_distance);
}

void HighLevelControl::SetLinearVelocity(double min_center_distance) {
    double limit = move_specs_.max_linear_velocity_ / 5.0
                   + move_specs_.high_security_distance_;

    if (min_center_distance < limit) {
        move_specs_.linear_velocity_ = move_specs_.min_linear_velocity_;
    } else {
        move_specs_.linear_velocity_ = move_specs_.max_linear_velocity_;
    }
}

double HighLevelControl::GetMin(std::vector<float>& ranges, int start, int finish) {
    std::vector<float>::iterator min = std::min_element(ranges.begin() + start,
                                       ranges.begin() + finish);
    return *min;
}

void HighLevelControl::CanContinue(double right_min_distance, double left_min_distance,
                                   double center_min_distance) {
    double priority_min, secondary_min;
    if (move_specs_.turn_type_ == RIGHT) {
        priority_min = center_min_distance < right_min_distance ? center_min_distance :
                       right_min_distance;

        secondary_min = left_min_distance;
    } else if (move_specs_.turn_type_ == LEFT) {
        priority_min = center_min_distance < left_min_distance ? center_min_distance :
                       left_min_distance;

        secondary_min = right_min_distance;
    } else {
        priority_min = secondary_min = Min(right_min_distance, left_min_distance,
                                           center_min_distance);
    }

    if (priority_min > move_specs_.high_security_distance_
            && secondary_min > move_specs_.low_security_distance_) {
        move_status_.can_continue_ = true;
    } else {
        move_status_.can_continue_ = false;
    }
}

void HighLevelControl::IsCloseToWall(double right_min_distance, double left_min_distance,
                                   double center_min_distance) {
    if (move_status_.is_following_wall_) {
        double min;
        if(move_specs_.turn_type_ == RIGHT) {
            min = right_min_distance;
        } else if(move_specs_.turn_type_ == LEFT) {
            min = left_min_distance;
        } else {
            // This case should never happen
            ros::shutdown();
        }

        if(min < move_specs_.wall_follow_distance_) {
            move_status_.is_close_to_wall_ = true;
        } else {
            move_status_.is_close_to_wall_ = false;
        }
    }
}

double HighLevelControl::Min(double right_min_distance, double left_min_distance,
                             double center_min_distance) {
    double min;
    if (right_min_distance < center_min_distance) {
        if (right_min_distance < left_min_distance) {
            min = right_min_distance;
        } else {
            min = left_min_distance;
        }
    } else {
        if (center_min_distance < left_min_distance) {
            min = center_min_distance;
        } else {
            min = left_min_distance;
        }
    }
    return min;
}

void HighLevelControl::WallFollowMove() {
    if (!move_status_.can_continue_ && !move_status_.is_following_wall_) {
        srand(time(NULL));
        move_specs_.turn_type_ = rand() % 2 == 0 ? RIGHT : LEFT;
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

void HighLevelControl::Move(double linear_velocity, double angular_velocity) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;
    cmd_vel_pub_.publish(msg);
}
