#include "Utils.hpp"
#include "gtest/gtest.h"

TEST(utils_max_test, creation_test) {
	Max* max = new Max();
	delete max;
}

TEST(utils_max_test, low_initial_test) {
	Max* max = new Max();
	EXPECT_LT(max->getMax(), -10000.0);
	delete max;
}

TEST(utils_max_test, zero_test) {
	Max* max = new Max();
	max->update(0);
	EXPECT_EQ(max->getMax(), 0);
	delete max;
}

TEST(utils_max_test, low_value_test) {
	Max* max = new Max();
	max->update(-100);
	EXPECT_EQ(max->getMax(), -100);
	delete max;
}

TEST(utils_max_test, high_value_test) {
	Max* max = new Max();
	max->update(100);
	EXPECT_EQ(max->getMax(), 100);
	delete max;
}

TEST(utils_max_test, replace_value_test) {
	Max* max = new Max();
	max->update(-100);
	max->update(25);
	EXPECT_EQ(max->getMax(), 25);
	delete max;
}

TEST(utils_max_test, nonreplace_value_test) {
	Max* max = new Max();
	max->update(100);
	max->update(-45);
	EXPECT_EQ(max->getMax(), 100);
	delete max;
}

TEST(utils_max_test, float_test) {
	Max* max = new Max();
	max->update(0.1);
	EXPECT_EQ(max->getMax(), 0.1);
	delete max;
}
