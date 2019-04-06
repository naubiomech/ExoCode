#include "Motor.hpp"
#include "gtest/gtest.h"

#include "mockInputPort.hpp"

TEST(motor_test, creation_test) {
	MockAnalogInputPort* in = new MockAnalogInputPort();
	Motor* motor = new Motor(in, NULL, 1);
	delete motor;
}
