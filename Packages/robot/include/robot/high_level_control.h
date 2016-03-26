/**
 * @file high_level_control.h
 * @brief This file defines the HighLevelControl clas
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#ifndef HIGH_LEVEL_CONTROL_H
#define HIGH_LEVEL_CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "move_helpers.h"

/**
 * @brief Breaks down abstract movement commands into smaller ones, that can be
 * executed by the Low Level Control
 *
 * Usage:
 *     HighLevelControl high_level_control;
 *     high_level_control.FollowWallMove();
 *
 * @author Rubin Deliallisi (rdeliallisi)
 * @date March 2016
 */

class HighLevelControl {

private:

    // TODO(rdeliallisi): To be put into Low Level Control
    /**
     * @brief Ros node to which the class is attached
    */
    ros::NodeHandle node_;

    // TODO(rdeliallisi): To be put into Low Level Control
    /**
     * @brief Used to send messages to the actual robot
     */
    ros::Publisher cmd_vel_pub_;

    // TODO(rdeliallisi): To be put into Object Detection
    /**
     * @brief Used to get the data from the laser range finder
     */
    ros::Subscriber laser_sub_;

    /**
     * @brief Contains constants that define the robot moving behavior
     */
    MoveSpecs move_specs_;

    /**
     * @brief Contains the corrent movement status
     */
    MoveStatus move_status_;

    // TODO(rdeliallisi): To be put into Object Detection
    /**
     * @brief Gets the data from the laser range finder, examines them and
     * updates the relevant class variables
     * @param msg Raw data comming from the laser range finder
     */
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Send the movement command to the robot using ROS nodes and topics
     * @param linear_speed  Linear Speed that the robot should move with
     * @param angular_speed Angular Speed that the robot should rotate with
     */
    void Move(double linear_speed, double angular_speed);

    /**
     * @brief [brief description]
     * @details [long description]
     */
    void InitialiseMoveSpecs();

    /**
     * @brief [brief description]
     * @details [long description]
     */
    void InitialiseMoveStatus();

    /**
     * @brief [brief description]
     * @details [long description]
     *
     * @param ranges [description]
     */
    void NormalMovement(std::vector<float>& ranges);

    /**
     * @brief [brief description]
     * @details [long description]
     *
     * @param right_min_distance [description]
     * @param left_min_distance [description]
     * @param center_min_distance [description]
     */
    void CanContinue(double right_min_distance, double left_min_distance,
                     double center_min_distance);

    /**
     * @brief [brief description]
     * @details [long description]
     *
     * @param right_min_distance [description]
     * @param left_min_distance [description]
     * @param center_min_distance [description]
     */
    void IsCloseToWall(double right_min_distance, double left_min_distance,
                       double center_min_distance);

public:
    HighLevelControl();

    /**
     * @brief Checks if the robot can continue waking forward or not
     * @return Returns the value of can_continue_
     */
    bool can_continue() {
        return move_status_.can_continue_;
    }

    /**
     * @brief Checks if the robot is close enough to the wall it is anchored to
     * @return Returns the value of is_close_to_wall_
     */
    bool is_close_to_wall() {
        return move_status_.is_close_to_wall_;
    }

    /**
     * @brief Checks if the robot is anchored and following a specific wall
     * @return Returns the value of is_following_wall_
     */
    bool is_following_wall() {
        return move_status_.is_following_wall_;
    }

    /**
     * @brief Moves the robot so that it always follows a wall
     */
    void WallFollowMove();
};

#endif
