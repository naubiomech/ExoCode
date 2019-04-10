#include "Transmission.hpp"
#include "mockBasicTransmission.hpp"
#include "mockTransceiver.hpp"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

TEST(transmission_base, creation){
	MockBasicTransmission* trans = new MockBasicTransmission();
	delete trans;
}

TEST(transmission_base, process_zero_args){
	MockTransceiver* transceiver = new MockTransceiver();
	MockBasicTransmission* transmission = new MockBasicTransmission(trans, 0, 0);
	EXPECT_CALL(transmission, processData(NULL,NULL)).Times(1);
	EXPECT_EQ(1,0);
	// transmission->process();
	delete transmission;
	delete transceiver;
}
