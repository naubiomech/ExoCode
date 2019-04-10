#include <gmock/gmock.h>
#include "Transmission.hpp"

class MockTransmission: public Transmission {
public:
	MockTransmission(Transceiver* trans, unsigned int receive_count, unsigned int send_count):
		Transmission(Transceiver* trans, NULL, 0, receive_count, send_count){};
	MOCK_METHOD2(processData, void(ExoMessageBuilder* builder, ExoReport* report));
}
