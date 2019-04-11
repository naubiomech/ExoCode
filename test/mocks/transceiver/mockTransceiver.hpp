#include <gmock/gmock.h>
#include "Transceiver.hpp"

class MockTransceiver: public Transceiver {
public:
	MockTransceiver(): Transceiver(){};
	MOCK_METHOD0(dataAvailable, bool());
	MOCK_METHOD0(noDataAvailable, bool());
	MOCK_METHOD0(clear, void());
	MOCK_METHOD0(sendHeader, void());
	MOCK_METHOD1(sendCommand, void(CommandCode code));
	MOCK_METHOD2(sendData, void(float* data, unsigned int bytes_to_send));
	MOCK_METHOD0(sendFooter, void());
	MOCK_METHOD0(receiveHeader, bool());
	MOCK_METHOD0(receiveCommand, CommandCode());
	MOCK_METHOD2(receiveData, void(double* data_output, unsigned int bytes_expected));
	MOCK_METHOD0(receiveFooter, bool());
};
