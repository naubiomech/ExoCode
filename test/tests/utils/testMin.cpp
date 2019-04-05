#include "Utils.hpp"
#include "gtest/gtest.h"

TEST(utils_min_test, creation_test) {
	Min* min = new Min();
	delete min;
}

TEST(utils_min_test, high_initial_test) {
	Min* min = new Min();
	EXPECT_GT(min->getMin(), 10000.0);
	delete min;
}

TEST(utils_min_test, zero_test) {
	Min* min = new Min();
	min->update(0);
	EXPECT_EQ(min->getMin(), 0);
	delete min;
}

TEST(utils_min_test, low_value_test) {
	Min* min = new Min();
	min->update(-100);
	EXPECT_EQ(min->getMin(), -100);
	delete min;
}

TEST(utils_min_test, high_value_test) {
	Min* min = new Min();
	min->update(100);
	EXPECT_EQ(min->getMin(), 100);
	delete min;
}

TEST(utils_min_test, replace_value_test) {
	Min* min = new Min();
	min->update(100);
	min->update(-25);
	EXPECT_EQ(min->getMin(), -25);
	delete min;
}

TEST(utils_min_test, nonreplace_value_test) {
	Min* min = new Min();
	min->update(-100);
	min->update(45);
	EXPECT_EQ(min->getMin(), -100);
	delete min;
}

TEST(utils_min_test, float_test) {
	Min* min = new Min();
	min->update(0.1);
	EXPECT_EQ(min->getMin(), 0.1);
	delete min;
}
