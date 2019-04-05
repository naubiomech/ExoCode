#include "Utils.hpp"
#include "gtest/gtest.h"

TEST(utils_range_test, creation_test) {
	Range* range = new Range(0,1);
	delete range;
}

TEST(utils_range_test, max_test) {
	Range* range = new Range(0,1);
	range->update(0.5);
	range->update(0);
	range->update(0.5);
	range->update(1);
	range->update(0.5);
	EXPECT_EQ(range->getMax(), 1);
	delete range;
}

TEST(utils_range_test, min_test) {
	Range* range = new Range(0,1);
	range->update(0.5);
	range->update(0);
	range->update(0.5);
	range->update(1);
	range->update(0.5);
	EXPECT_EQ(range->getMin(), 0);
	delete range;
}

TEST(utils_range_test, two_min_test) {
	Range* range = new Range(0,1);
	range->update(0.5);
	range->update(1);
	EXPECT_EQ(range->getMin(), 0);
	range->update(0.5);
	range->update(0.2);
	range->update(1);
	EXPECT_EQ(range->getMin(), 0.1);
	delete range;
}

TEST(utils_range_test, two_max_test) {
	Range* range = new Range(0,1);
	range->update(0.5);
	EXPECT_EQ(range->getMax(), 1);
	range->update(0.8);
	range->update(0);
	EXPECT_EQ(range->getMax(), 0.9);
	delete range;
}

TEST(utils_range_test, two_both_test) {
	Range* range = new Range(0,1);
	range->update(0.2);
	range->update(0.8);
	range->update(0);
	EXPECT_EQ(range->getMax(), 0.9);
	EXPECT_EQ(range->getMin(), 0.1);
	delete range;
}

TEST(utils_range_test, reducing_max_theshold_test) {
	Range* range = new Range(0,1.5);
	range->update(0);
	range->update(0.8);
	range->update(0);
	range->update(0.6);
	range->update(0);
	range->update(0.5);
	range->update(0);
	range->update(0.43);
	range->update(0);
	range->update(0.40);
	range->update(0);
	EXPECT_LT(range->getMax(), 0.75);
	delete range;
}
