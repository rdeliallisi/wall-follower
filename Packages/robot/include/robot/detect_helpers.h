#ifndef DETECT_HELPERS_H
#define DETECT_HELPERS_H

struct CannyParams {
	int threshold_1_;
	int threshold_2_;
};

struct BlurParams {
	int kernel_size_;
	int sigma_;
};

struct HoughParams {
	int dp_;
	int min_dist_;
	int threshold_1_;
	int threshold_2_;
	int min_radius_;
	int max_radius_;
};

#endif