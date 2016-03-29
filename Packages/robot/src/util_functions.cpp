#include "util_functions.h"
#include <vector>
#include <algorithm>

double GetMin(std::vector<float>& ranges, int start, int finish) {
    if (ranges.size() <= 0 || start < 0 || finish > ranges.size() ||
            start > finish) {
        return 0;
    }
    std::vector<float>::iterator min = std::min_element(ranges.begin() + start,
                                       ranges.begin() + finish);
    return *min;
}


double Min(double right_min_distance, double left_min_distance,
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