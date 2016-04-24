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
#include <cmath>
#include "robot/circle_detect_msg.h"
#include "high_level_control.h"
#include "util_functions.h"

HighLevelControl::HighLevelControl() : node_() {
    InitialiseMoveSpecs();
    InitialiseMoveStatus();
    InitialiseTopicConnections();
}

void HighLevelControl::InitialiseTopicConnections() {
    bool loaded = true;
    std::string publish_topic, laser_topic, circle_topic;

    if (!node_.getParam("publish_topic",
                        publish_topic)) {
        loaded = false;
    }

    if (!node_.getParam("laser_topic",
                        laser_topic)) {
        loaded = false;
    }

    if (!node_.getParam("circle_topic",
                        circle_topic)) {
        loaded = false;
    }

    if (loaded == false) {
        ROS_INFO("Topics failed to load!");
        ros::shutdown();
    }

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>(publish_topic, 100);
    laser_sub_ = node_.subscribe(laser_topic, 100, &HighLevelControl::LaserCallback, this);
    circle_sub_ = node_.subscribe(circle_topic, 100, &HighLevelControl::CircleCallback, this);
}

void HighLevelControl::InitialiseMoveSpecs() {
    bool loaded = true;

    if (!node_.getParam("high_security_distance",
                        move_specs_.high_security_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/low_security_distance",
                        move_specs_.low_security_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/wall_follow_distance",
                        move_specs_.wall_follow_distance_)) {
        loaded = false;
    }

    if (!node_.getParam("/linear_velocity",
                        move_specs_.linear_velocity_)) {
        loaded = false;
    }

    if (!node_.getParam("/angular_velocity",
                        move_specs_.angular_velocity_)) {
        loaded = false;
    }

    move_specs_.turn_type_ = NONE;

    if (loaded == false) {
        ROS_INFO("Params failed to load!");
        ros::shutdown();
    }
}

void HighLevelControl::InitialiseMoveStatus() {
    move_status_.can_continue_ = true;
    move_status_.is_close_to_wall_ = false;
    move_status_.is_following_wall_ = false;
    move_status_.circle_hit_mode_ = false;
    move_status_.hit_goal_ = false;
    move_status_.count_turn_ = 0;
    move_status_.last_turn_ = 0;
}

void HighLevelControl::LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());

    if (!move_status_.circle_hit_mode_) {
        Update(ranges);
        WallFollowMove();
    } else {
        HitCircle(ranges);
    }

    // Log robot status
    ROS_INFO("can_continue:%d, is_following_wall:%d, is_close_to_wall:%d, turn_type:%d\n",
             move_status_.can_continue_, move_status_.is_following_wall_, move_status_.is_close_to_wall_,
             move_specs_.turn_type_);
}

void HighLevelControl::CircleCallback(const robot::circle_detect_msg::ConstPtr& msg) {
    std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
    circle_x_ = msg->circle_x;
    circle_y_ = msg->circle_y;
    // If true stay in the mode else check if we can hit circle
    move_status_.circle_hit_mode_ = move_status_.circle_hit_mode_ ? true :
                                    CanHit(msg->circle_x, msg->circle_y,
                                           ranges);

    // Log circle coordinates
    ROS_INFO("circle_x:%lf, circle_y:%lf", circle_x_, circle_y_);
}

bool HighLevelControl::CanHit(double circle_x, double circle_y, std::vector<float>& ranges) {
    // Cannot hit circle if not in wall following mode
    if (move_specs_.turn_type_ == NONE) {
        return false;
    }

    int size = ranges.size();

    // Distance to the wall 20 deg on the opposite side of the circle
    double wall_20;
    // Planar distance to the center of the circle ignoring obstacles
    double center_distance = sqrt(circle_x * circle_x + circle_y * circle_y);
    // Angle to the center of the circle relative to normal Cartesian system
    double center_angle = acos(circle_x / center_distance) / M_PI * 180;
    // Index of the angle in the ranges vector
    int index = (int)((center_angle + 30) / 240.0 * size);
    // Check if we might get an out of bound index after shifting by 20 deg
    int deg20 = (int) (20.0 / 240.0 * size);
    if (index < deg20 || index >= size - deg20)
        return false;
    // Distance from LRF in the direction of the circle center
    double center_lrf = ranges[index];
    if (move_specs_.turn_type_ == RIGHT) {
        // Distance 20 deg left from the center
        wall_20 = ranges[index + deg20];
    } else if (move_specs_.turn_type_ == LEFT) {
        // Distance 20 deg right from the center
        wall_20 = ranges[index - deg20];
    } else {
        return false;
    }

    // Threshold to detect if we have fake circle at a corner. If this is the
    // case wall_20 will be smaller than the threshold
    double threshold = center_distance + 1;
    // If the threshold condition is true, if the circle is to the front of the
    // robot and close enough then we go into circle hit mode
    if (wall_20 > threshold && center_lrf < center_distance && circle_x > -0.5
            && circle_x < 0.5 && circle_y < 1 && circle_y > 0) {
        return true;
    }

    return false;
}

void HighLevelControl::Update(std::vector<float>& ranges) {
    double right_min_distance, left_min_distance, center_min_distance;

    int size = ranges.size();

    // 75 degree range to the right
    right_min_distance = GetMin(ranges, 0, (int)(75.0 / 240.0 * size));

    // 75 degree range in front
    center_min_distance = GetMin(ranges, (int)(75.0 / 240.0 * size), (int)(165.0 / 240.0 * size));

    // 75 degree range to the left
    left_min_distance = GetMin(ranges, (int)(165.0 / 240.0 * size), size);

    ROS_INFO("right:%lf, left:%lf, center:%lf\n", right_min_distance, left_min_distance, center_min_distance);

    CanContinue(right_min_distance, left_min_distance, center_min_distance);

    IsCloseToWall(right_min_distance, left_min_distance, center_min_distance);
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
        if (move_specs_.turn_type_ == RIGHT) {
            min = right_min_distance;
        } else if (move_specs_.turn_type_ == LEFT) {
            min = left_min_distance;
        } else {
            // This case should never happen
            ROS_INFO("Robot has no turn type after being attached to wall!");
            ros::shutdown();
        }

        if (min < move_specs_.wall_follow_distance_) {
            move_status_.is_close_to_wall_ = true;
        } else {
            move_status_.is_close_to_wall_ = false;
        }
    }
}

void HighLevelControl::HitCircle(std::vector<float>& ranges) {

    if (move_status_.hit_goal_) {
        GoToCircle(ranges);
        return;
    }

    AlignRobot(ranges);
}

void HighLevelControl::GoToCircle(std::vector<float>& ranges) {

    if(ranges[ranges.size() / 2] < 0.05) {
        ROS_INFO("Goal Reached!");
        ros::shutdown();
    }

    if (circle_x_ < -9 || (circle_x_ <= 0.025 && circle_x_ >= -0.025)) {
        Move(move_specs_.linear_velocity_ , 0);
    } else if (circle_x_ > 0.1) {
        Move(0, -move_specs_.angular_velocity_);
    } else if (circle_x_ < -0.1) {
        Move(0, move_specs_.angular_velocity_);
    } else {
        Move(move_specs_.linear_velocity_ , 0);
    }
}

void HighLevelControl::AlignRobot(std::vector<float>& ranges) {
    int size = ranges.size();

    // 0 deg if right wall and 240 if left wall
    double back_value;
    // 60 deg if right wall and 180 if left wall
    double front_value;
    if (move_specs_.turn_type_ == RIGHT) {
        back_value = ranges[0];
        front_value = ranges[(int)(60.0 / 240.0 * size) - 1];
    } else if (move_specs_.turn_type_ == LEFT) {
        back_value = ranges[size - 1];
        front_value = ranges[(int)(180.0 / 240.0 * size) - 1];
    } else {
        // Cannot hit circle if not in wall following mode
        ROS_INFO("The robot has no turn type while trying to align to the wall!\n");
        ros::shutdown();
    }

    // The difference of the values must be very small such that the robot
    // is aligned to the wall it is following. If not turn appropriately.
    double diff = front_value - back_value;
    if (diff <= 0.025 && diff >= -0.025) {
        move_status_.hit_goal_ = true;
    } else if (diff > 0.025) {
        Move(0, -1 * (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_ / 4);
    } else {
        Move(0, (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_ / 4);
    }
}

void HighLevelControl::WallFollowMove() {
    if (!move_status_.can_continue_ && !move_status_.is_following_wall_) {
        srand(time(NULL));

        // 50% left mode, 50% right mode
        move_specs_.turn_type_ = rand() % 10000 > 5000 ? RIGHT : LEFT;
        move_status_.is_following_wall_ = true;
    } else if (move_status_.can_continue_ && !move_status_.is_following_wall_) {
        Move(move_specs_.linear_velocity_, 0);
    } else {
        if (move_status_.can_continue_ && move_status_.is_close_to_wall_) {
            Move(move_specs_.linear_velocity_, 0);

            // Update loop breaking status
            move_status_.last_turn_ = 0;
            move_status_.count_turn_ = 0;
        } else if (!move_status_.can_continue_) {
            Move(0, (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_);

            // Update loop breaking status
            if (move_status_.last_turn_ == 0 || move_status_.last_turn_ == -1) {
                move_status_.count_turn_++;
            }
            move_status_.last_turn_ = 1;
        } else if (!move_status_.is_close_to_wall_) {
            Move(0, -1 * (move_specs_.turn_type_ - 1) * move_specs_.angular_velocity_);

            // Update loop breaking status
            if (move_status_.last_turn_ == 0 || move_status_.last_turn_ == 1) {
                move_status_.count_turn_++;
            }
            move_status_.last_turn_ = -1;
        }
    }

    BreakLoop();
}

void HighLevelControl::BreakLoop() {
    // In case of a turn loop break out after 10 oposite turns in a row.
    if (move_status_.count_turn_ > 5) {
        if (move_specs_.turn_type_ == RIGHT) {
            // Short right turn
            Move(move_specs_.linear_velocity_, -move_specs_.angular_velocity_);
        } else if (move_specs_.turn_type_ == LEFT) {
            // Short left turn
            Move(move_specs_.linear_velocity_, move_specs_.angular_velocity_);
        } else {
            //Case should not happen
        }
        move_status_.count_turn_ = 0;
    }
}

void HighLevelControl::Move(double linear_velocity, double angular_velocity) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity;
    msg.angular.z = angular_velocity;
    cmd_vel_pub_.publish(msg);
}
