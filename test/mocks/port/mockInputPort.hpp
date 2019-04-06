#include <gmock/gmock.h>
#include "Port.hpp"

class MockAnalogInputPort : public AnalogInputPort {
public:
	MockAnalogInputPort(): AnalogInputPort(0,1){};
	MOCK_METHOD0(read, double());
};
