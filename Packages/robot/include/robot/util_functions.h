/**
 * @file util_functions.h
 * @brief Defines utility functions to be used in the high_level_control class
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */


#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

#include <vector>

/**
 * @brief Method to get the minimum distance of the robot within its range
 * 
 * @param right_min_distance Minimum distance to the right
 * @param left_min_distance Minimum distance to the left
 * @param center_min_distance Minimum distance in front on the robot
 * @return Returns the minimum distance of the robot within its range
 */
double Min(double right_min_distance, double left_min_distance,
           double center_min_distance);

/**
 * @brief Method to get the the minimum distance of the robot within a range
 * 
 * @param ranges ranges of data in the laser range finder
 * @param start range start
 * @param finish range finish 
 * @return Returns the minimum distance of the within the start and finish range
 */
double GetMin(std::vector<float>& ranges, int start, int finish);

#endif