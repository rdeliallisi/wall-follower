/**
 * @file circle_detector.cpp
 * @brief Helper file for the circle detector class.
 *
 * @author Atabak Hafeez [atabakhafeez]
 * @author Maria Ficiu [MariaFiciu]
 * @author Rubin Deliallisi [rdeliallisi]
 * @author Siddharth Shukla [thunderboltsid]
 * @bug No known bugs.
 */

#ifndef DETECT_HELPERS_H
#define DETECT_HELPERS_H

/**
 * @brief Defines the BlurParams structure
 * which has two variables
 */

struct BlurParams {
	
	/**
        * @brief The structure has as variables the following:
        */

       /**
       * @brief the size of the kernel
       */
	
	int kernel_size_;
	
	/**
       * @brief the amount of blur
       */
	
	int sigma_;
};

/**
 * @brief Defines the HoughParams structure
 * which has six variables
 */

struct HoughParams {
	
	/**
        * @brief The structure has as variables the following:
        * 
       * @brief dp_ is the inverse ratio of resolution
       */
	
	int dp_;
      	/**
        * @brief min_dist_ is the minimum distance between detected centers
       */
	int min_dist_;
	/**
        * @brief threshold_1_ is the upper threshold
       */
	int threshold_1_;
	/**
        * @brief threshold_2_ is the threshold for center detection
       */
	int threshold_2_;
	/**
        * @brief min_radius_ is the minimum ratio to be detected
       */
	int min_radius_;
	/**
        * @brief threshold_1_ is the maximum radius to be detected
       */
	int max_radius_;
};

#endif
