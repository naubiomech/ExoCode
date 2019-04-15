#include "Transmission.hpp"
#include "mockBasicTransmission.hpp"
#include "mockTransceiver.hpp"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

using::testing::A;
using::testing::Return;
using::testing::SetArrayArgument;
using::testing::ElementsAreArray;

TEST(transmission_base, allocation){
	MockBasicTransmission* trans = new MockBasicTransmission(NULL, 0, 0);
	delete trans;
}

TEST(transmission_base, process_zero_args){
	MockTransceiver* transceiver = new MockTransceiver();
	MockBasicTransmission* transmission = new MockBasicTransmission(transceiver, 0, 0);
	EXPECT_CALL(*transmission, processData(NULL,NULL)).Times(1);
	transmission->process(NULL,NULL);
	delete transmission;
	delete transceiver;
}

TEST(transmission_base, process_one_receive){
	MockTransceiver* transceiver = new MockTransceiver();
	MockBasicTransmission* transmission = new MockBasicTransmission(transceiver, 1, 0);

	double fake_data[] = {0};
	EXPECT_CALL(*transceiver, receiveData(A<double*>(),1))
		.Times(1)
		.WillRepeatedly(SetArrayArgument<0>(fake_data,fake_data+1));

	EXPECT_CALL(*transceiver, receiveFooter())
		.Times(1)
		.WillRepeatedly(Return(true));

	EXPECT_CALL(*transmission, processData(NULL,NULL)).Times(1);

	transmission->process(NULL,NULL);

	EXPECT_THAT(fake_data, ElementsAreArray(transmission->getReceiveData(), 1));

	delete transmission;
	delete transceiver;
}

TEST(transmission_base, process_ten_receive){
	MockTransceiver* transceiver = new MockTransceiver();
	MockBasicTransmission* transmission = new MockBasicTransmission(transceiver, 10, 0);

	const double fake_data[] = {0,1,2,3,4,5,6,7,8,9};
	EXPECT_CALL(*transceiver, receiveData(A<double*>(),10))
		.Times(1)
		.WillRepeatedly(SetArrayArgument<0>(fake_data,fake_data+10));

	EXPECT_CALL(*transceiver, receiveFooter())
		.Times(1)
		.WillRepeatedly(Return(true));

	EXPECT_CALL(*transmission, processData(NULL,NULL))
		.Times(1);

	transmission->process(NULL,NULL);

	EXPECT_THAT(fake_data, ElementsAreArray(transmission->getReceiveData(), 10));
	delete transmission;
	delete transceiver;
}

TEST(transmission_base, process_one_send){
	MockTransceiver* transceiver = new MockTransceiver();
	MockBasicTransmission* transmission = new MockBasicTransmission(transceiver, 0, 1);

	float fake_data[1] = {0};
	EXPECT_CALL(*transceiver, sendHeader())
		.Times(1);

	EXPECT_CALL(*transceiver, sendCommand(A<char>()))
		.Times(1);

	EXPECT_CALL(*transceiver, sendData(A<float*>(),1))
		.Times(1);

	EXPECT_CALL(*transceiver, sendFooter())
		.Times(1);

	EXPECT_CALL(*transmission, processData(NULL,NULL)).Times(1);

	transmission->process(NULL,NULL);

	EXPECT_THAT(fake_data, ElementsAreArray(transmission->getSendData(), 1));
	delete transmission;
	delete transceiver;
}

TEST(transmission_base, DISABLED_process_ten_send){
	MockTransceiver* transceiver = new MockTransceiver();
	MockBasicTransmission* transmission = new MockBasicTransmission(transceiver, 0, 10);

	float fake_data[10] = {0,1,2,3,4,5,6,7,8,9};
	EXPECT_CALL(*transceiver, sendHeader())
		.Times(1);

	EXPECT_CALL(*transceiver, sendCommand(A<char>()))
		.Times(1);

	EXPECT_CALL(*transceiver, sendData(A<float*>(),10))
		.Times(1);

	EXPECT_CALL(*transceiver, sendFooter())
		.Times(1);

	EXPECT_CALL(*transmission, processData(NULL,NULL)).Times(1);

	transmission->process(NULL,NULL);

	EXPECT_THAT(fake_data, ElementsAreArray(transmission->getSendData(), 10));
	delete transmission;
	delete transceiver;
}

TEST(transmission_base, DISABLED_process_one_both){
	MockTransceiver* transceiver = new MockTransceiver();
	MockBasicTransmission* transmission = new MockBasicTransmission(transceiver, 1, 1);

	float fake_data[1] = {0};
	EXPECT_CALL(*transceiver, sendHeader())
		.Times(1);

	EXPECT_CALL(*transceiver, sendCommand(A<char>()))
		.Times(1);

	EXPECT_CALL(*transceiver, sendData(A<float*>(),1))
		.Times(1);

	EXPECT_CALL(*transceiver, sendFooter())
		.Times(1);

	EXPECT_CALL(*transmission, processData(NULL,NULL)).Times(1);

	transmission->process(NULL,NULL);

	EXPECT_THAT(fake_data, ElementsAreArray(transmission->getSendData(), 1));
	delete transmission;
	delete transceiver;
}

TEST(transmission_base, DISABLED_process_ten_both){
	MockTransceiver* transceiver = new MockTransceiver();
	MockBasicTransmission* transmission = new MockBasicTransmission(transceiver, 10, 10);

	float fake_data[10] = {0,1,2,3,4,5,6,7,8,9};
	EXPECT_CALL(*transceiver, receiveData(A<double*>(),10))
		.Times(1)
		.WillRepeatedly(SetArrayArgument<0>(fake_data,fake_data+10));

	EXPECT_CALL(*transceiver, receiveFooter())
		.Times(1)
		.WillRepeatedly(Return(true));

	EXPECT_CALL(*transceiver, sendHeader())
		.Times(1);

	EXPECT_CALL(*transceiver, sendCommand(A<char>()))
		.Times(1);

	EXPECT_CALL(*transceiver, sendData(AllOf(A<float*>()),10))
		.Times(1);

	EXPECT_CALL(*transceiver, sendFooter())
		.Times(1);

	EXPECT_CALL(*transmission, processData(NULL,NULL)).Times(1);

	transmission->process(NULL,NULL);

	EXPECT_THAT(fake_data, ElementsAreArray(transmission->getSendData(), 10));
	EXPECT_THAT(fake_data, ElementsAreArray(transmission->getReceiveData(), 10));
	delete transmission;
	delete transceiver;
}
