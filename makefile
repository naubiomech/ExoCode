# Good source for make file ideas
# https://stackoverflow.com/questions/14639794/getting-make-to-create-object-files-in-a-specific-directory
# https://devhints.io/makefile

SHELL = /bin/sh
OUTPUT = exo

C = gcc
CXX = g++
CFLAGS = -std=gnu99
CXXFLAGS = -std=c++11
CPPFLAGS = -Wall -Wextra -pedantic -Wwrite-strings
LDFLAGS = -pthread -lm
OPTFLAG= -O3
DEBUGFLAG = -ggdb

SOURCES=$(wildcard *.cpp)
OBJECTS=$(patsubst %.cpp, build/%.o, $(SOURCES))

all: build
build/Board.o: Parameters.hpp Exoskeleton.hpp Port.hpp
build/BoardBuilder.o: Board.hpp Port.hpp
build/Commands.o: States.hpp
build/Communications.o: Transceiver.hpp Transmission.hpp
build/Control_Algorithms.o: Utils.hpp Report.hpp States.hpp
build/Control_Module.o: Arduino.hpp States.hpp Control_Algorithms.hpp Shaping_Functions.hpp Utils.hpp Linked_List.hpp
build/ExoBuilder.o: Port.hpp Exoskeleton.hpp Board.hpp Linked_List.hpp Pot.hpp
build/Exoskeleton.o: Arduino.hpp Leg.hpp Report.hpp Communications.hpp Message.hpp
build/FSR.o: Parameters.hpp Utils.hpp Report.hpp Port.hpp Linked_List.hpp
build/IMU.o: Arduino.hpp Parameters.hpp Report.hpp Port.hpp
build/Joint.o: Motor.hpp TorqueSensor.hpp Pot.hpp Report.hpp Port.hpp Control_Module.hpp Message.hpp
build/Leg.o: Arduino.hpp States.hpp Joint.hpp FSR.hpp Report.hpp IMU.hpp Pot.hpp Linked_List.hpp Message.hpp
build/Linked_List.o: Arduino.hpp
build/Message.o: Report.hpp Linked_List.hpp Commands.hpp
build/Motor.o: Arduino.hpp Parameters.hpp Utils.hpp Shaping_Functions.hpp Control_Algorithms.hpp Port.hpp Report.hpp
build/Port.o: Arduino.hpp
build/Pot.o: Port.hpp Report.hpp
build/Report.o: States.hpp Linked_List.hpp
build/Shaping_Functions.o: Utils.hpp
build/States.o: Utils.hpp Linked_List.hpp
build/TorqueSensor.o: Report.hpp Port.hpp Utils.hpp
build/Transceiver.o: Arduino.hpp Port.hpp Command_Codes.hpp Report.hpp Message.hpp
build/Transmission.o: Commands.hpp Command_Codes.hpp Message.hpp Transceiver.hpp Report.hpp

.PHONY: all clean build clear rebuild debug optimize


build: $(OUTPUT)

$(OUTPUT): $(OBJECTS)
	$(CXX) $(LDFLAGS) $^ -o $@

build/%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(CPPFLAGS) $< -o $@

clean:
	rm -f ./*.o ./$(OUTPUT) ./*~ ./*.stackdump ./*# ./build/*.o

clear:
	clear

rebuild: clean build

all: build

debug: CPPFLAGS += $(DEBUGFLAG)
debug: rebuild

optimize: CPPFLAGS += $(OPTFLAG)
optimize: rebuild
