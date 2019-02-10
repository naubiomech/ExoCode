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

.PHONY: all clean build clear rebuild debug optimize

all: build

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
