#include <gtest/gtest.h>
#include "util_functions.h"

TEST(MinTest, Positive) {
	double min = Min(3, 100, 1);
	ASSERT_DOUBLE_EQ(min, 1);
}

TEST(MinTest, Negative) {
	double min = Min(-1, -100, -3);
	ASSERT_DOUBLE_EQ(min, -100);
}

TEST(MinTest, AllEqual) {
	double min = Min(0, 0, 0);
	ASSERT_DOUBLE_EQ(min, 0);
}

TEST(MinTest, Mixed) {
	double min = Min(-1, 0, 1);
	ASSERT_DOUBLE_EQ(min, -1);
}

TEST(GetMinTest, OK) {
	std::vector<float> ranges;
	for (int i = 1; i < 10; i++) {
		ranges.push_back(i);
	}
	ASSERT_DOUBLE_EQ(1, GetMin(ranges, 0, ranges.size()));
}

TEST(GetMinTest, EmptyVector) {
	std::vector<float> ranges;
	ASSERT_DOUBLE_EQ(0, GetMin(ranges, 0, 10));
}

TEST(GetMinTest, Negative) {
	std::vector<float> ranges;
	for (int i = 1; i < 10; i++) {
		ranges.push_back(i);
	}
	ASSERT_DOUBLE_EQ(0, GetMin(ranges, -1, ranges.size()));
}

TEST(GetMinTest, WrongLimits) {
	std::vector<float> ranges;
	for (int i = 1; i < 10; i++) {
		ranges.push_back(i);
	}
	ASSERT_DOUBLE_EQ(0, GetMin(ranges, 5, 3));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}