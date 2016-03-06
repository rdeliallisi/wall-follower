#ifndef HIGH_LEVEL_CONTROL_H
#define HIGH_LEVEL_CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/**
 * Breaks down abstract movement commands into smaller ones,
 * that can be executed by the Low Level Control
 * Usage:
 *     HighLevelControl high_level_control;
 *     high_level_control.move();
 */
class HighLevelControl {

private:

    // TODO(rdeliallisi): To be put into Low Level Control
    // Ros node to which the class is attached
    ros::NodeHandle node_;

    // TODO(rdeliallisi): To be put into Low Level Control
    // Used to send messages to the actual robot
	ros::Publisher cmd_vel_pub_;

    // TODO(rdeliallisi): To be put into Object Detection
    // Used to get the data from the laser range finder
    ros::Subscriber laser_sub_;

    // The type of turn the robot is currently in when turning
    // -2 for right 2 for left
    int turn_type_;

    // Minimum proximity distance that the robot can have from a wall
    double security_distance_;

    // Maximum distance the robot can be away from the wall it is
    // following
    double wall_follow_distance_;

    double linear_velocity_;

    double angular_velocity_;

    // Checks if the robot can walk straight or not
	bool can_continue_;

    // Checks if robot is close enough to the wall it is following
    bool is_close_to_wall_;

    bool is_turning_;

    // TODO(rdeliallisi): To be put into Object Detection
    /**
     * Gets the data from the laser range finder, examines them and 
     * updates the relevant class variables
     * @param msg Raw data comming from the laser range finder
     */
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

public:
    HighLevelControl();

    void set_security_distance(double security_distance);

    void set_angular_velocity(double angular_velocity);

    void set_linear_velocity(double linear_velocity);

    /**
     * Moves the robot so that it always follows a wall
     */
    void WallFollowMove();

    /**
     * Moves the robot randomly using the specified linear and
     * angular velocity
     */
    void RandomMove();

    //TODO(rdeliallisi): To be put into Low Level Control
    /**
     * Send the movement command to the robot using ROS nodes and
     * topics
     * @param linear_speed  Linear Speed that the robot should move with
     * @param angular_speed Angular Speed that the robot should rotate with
     */
    void Move(double linear_speed, double angular_speed);
};

#endif