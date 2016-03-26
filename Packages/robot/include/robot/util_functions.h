#ifndef UTIL_FUNCTIONS_H
#define UTIL_FUNCTIONS_H

#include <vector>

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param right_min_distance [description]
 * @param left_min_distance [description]
 * @param center_min_distance [description]
 * @return [description]
 */
double Min(double right_min_distance, double left_min_distance,
           double center_min_distance);

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param ranges [description]
 * @param start [description]
 * @param finish [description]
 * @return [description]
 */
double GetMin(std::vector<float>& ranges, int start, int finish);

#endif