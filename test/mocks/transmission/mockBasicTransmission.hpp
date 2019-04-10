#include <gmock/gmock.h>
#include "Transmission.hpp"

class MockBasicTransmission: public Transmission {
public:
	MockBasicTransmission(Transceiver* trans, unsigned int receive_count, unsigned int send_count):
		Transmission(trans, NULL, 0, receive_count, send_count){};
	MOCK_METHOD2(processData, void(ExoMessageBuilder* builder, ExoReport* report));
};
