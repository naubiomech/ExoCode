#include <gmock/gmock.h>
#include "Transceiver.hpp"

class MockTransceiver: public Transceiver {
public:
	MockTransceiver(): Transceiver(){};
	MOCK_METHOD0(dataAvailable, bool());
	MOCK_METHOD0(noDataAvailable, bool());
	MOCK_METHOD0(clear, void());
	MOCK_METHOD0(sendHeader, void());
	MOCK_METHOD2(sendData, void(float* data, unsigned int bytes_to_send));
	MOCK_METHOD(sendFooter, void());
	MOCK_METHOD(receiveHeader, bool());
	MOCK_METHOD(receiveCommand, CommandCode());
	MOCK_METHOD(receiveData, void(double* data_output, unsigned int bytes_expected));
	MOCK_METHOD(receiveFooter, bool());
}
