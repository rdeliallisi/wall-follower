/**
 * @file high_level_control.h
 * @brief This file defines the HighLevelControl class
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
#include "robot/circle_detect_msg.h"
#include "move_helpers.h"

/**
 * @brief Defines the movement of the robot such as the wall following and the
 * gets data from the CircleDetector about the position of the circle and hits
 * it
 *
 * Usage:
 *     HighLevelControl high_level_control;
 */
class HighLevelControl {

private:

	/**
	 * @brief Ros node to which the class is attached
	 */
	ros::NodeHandle node_;

	/**
	 * @brief Used to send messages to the actual robot
	 */
	ros::Publisher cmd_vel_pub_;

	/**
	 * @brief Used to get the data from the laser range finder
	 */
	ros::Subscriber laser_sub_;

	/**
	 * @brief Used to get the position of the circle
	 */
	ros::Subscriber circle_sub_;

	/**
	 * @brief Contains constants that define the robot moving behavior
	 */
	MoveSpecs move_specs_;

	/**
	 * @brief Contains the current movement status
	 */
	MoveStatus move_status_;

	/**
	 * @brief Gets the data from the laser range finder, examines them and
	 * updates the relevant class variables
	 *
	 * @param msg Raw data comming from the laser range finder
	 */
	void LaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

	void CircleCallback(const robot::circle_detect_msg::ConstPtr& msg);

	/**
	 * @brief Analyses the ranges given by the laser range finder and updates the
	 * minimum distances on the left, right and center of the robot
	 *
	 * @param ranges The laser range finder ranges in std::vector<float> format
	 */
	void Update(std::vector<float>& ranges);

	/**
	 * @brief Initialises the movement specifications by getting the parameters
	 * from the config file
	 */
	void InitialiseMoveSpecs();

	/**
	 * @brief Initialises the movement status of the robot once the robot is
	 * initialised
	 */
	void InitialiseMoveStatus();

public:
	/**
	 * @brief The default constructor for the HighLevelControl class
	 */
	HighLevelControl();

	/**
	 * @brief Moves the robot so that it always follows a wall
	 */
	void WallFollowMove();

	/**
	 * @brief Defines the movement of the robot when there is no wall nearby and
	 * checks for the nearest wall
	 *
	 * @param ranges The laser range finder ranges in std::vector<float> format
	 */
	void HitCircle(std::vector<float>& ranges);

	/**
	 * @brief Checks if the robots path is clear and it can hit the circle
	 *
	 * @param circle_x The x-coordinate of the circle in cartesian coordinates
	 * relative to the robot
	 * @param circle_y The y-coordinate of the circle in cartesian coordinates
	 * relative to the robot
	 * @param ranges The laser range finder ranges in std::vector<float> format
	 * @return Returns boolean value of whether thr robot can hit the cicle
	 */
	bool CanHit(double circle_x, double circle_y, std::vector<float>& ranges);

	/**
	 * @brief Checks whether the robot can continue in the same path. If the
	 * security distance is close it sets the can_continue_ to false
	 *
	 * @param right_min_distance The minimum distance detected in the ranges to
	 * the right of the robot
	 * @param left_min_distance The minimum distance detected in the ranges to
	 * the left of the robot
	 * @param center_min_distance The minimum distance detected in the ranges
	 * in front of the robot within a range
	 */
	void CanContinue(double right_min_distance, double left_min_distance,
	                 double center_min_distance);

	/**
	 * @brief Checks if the robot is close to the wall by checking the direction
	 * of the turn and minimum distance from the wall on that side
	 *
	 * @param right_min_distance The minimum distance detected in the ranges to
	 * the right of the robot
	 * @param left_min_distance The minimum distance detected in the ranges to
	 * the left of the robot
	 * @param center_min_distance The minimum distance detected in the ranges
	 * in front of the robot within a range
	 */
	void IsCloseToWall(double right_min_distance, double left_min_distance,
	                   double center_min_distance);
	/**
	 * @brief Send the movement command to the robot using ROS nodes and topics
	 *
	 * @param linear_speed  Linear Speed that the robot should move with
	 * @param angular_speed Angular Speed that the robot should rotate with
	 */
	void Move(double linear_speed, double angular_speed);

	/**
	 * @brief Getter for the movement spacifications
	 *
	 * @return Returns MoveSpecs
	 */
	MoveSpecs get_move_specs() {
		return move_specs_;
	}

	/**
	 * @brief Getter for the movement status of the robot
	 *
	 * @return Returns MoveStatus
	 */
	MoveStatus get_move_status() {
		return move_status_;
	}

	/**
	 * @brief Setter for turn type
	 *
	 * @param turn_type The TurnType for the robot
	 */
	void set_turn_type(TurnType turn_type) {
		if (!move_status_.is_following_wall_)
			move_status_.is_following_wall_ = true;
		move_specs_.turn_type_ = turn_type;
	}
};

#endif
