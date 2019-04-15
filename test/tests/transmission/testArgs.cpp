#include "gtest/gtest.h"
#include "gmock/gmock.h"

using::testing::ElementsAre;
using::testing::ElementsAreArray;
using::testing::_;
using::testing::WithArgs;
using::testing::Args;

class MockClass{
public:
	MOCK_METHOD1(myCall, void(double* arg));
};

TEST(mytests, argcall){
	double my_var[3] = {0, 1, 2};
	int* my_other_var = new int[3];
	for(int i = 0; i < 3; i ++){
		my_other_var[i] = i;
	}
	MockClass c;
	// EXPECT_CALL(c, myCall(_))
	// 	.With(WithArgs<0>(ElementsAreArray(my_other_var,3)));
	EXPECT_CALL(c, myCall(_))
		.With(Args<0>(ElementsAre(0,1,2)));
	delete my_other_var;
}
