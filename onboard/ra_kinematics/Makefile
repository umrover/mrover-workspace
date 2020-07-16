CXX = g++

CXXFLAGS = -Wall -Werror -pedantic --std=c++11 -g

all: test

test: arm_state_tests.exe


arm_state_tests.exe: test/arm_state_tests.cpp arm_state.cpp
	$(CXX) $(CXXFLAGS) $^ -o test/executables/$@
	./test/executables/$@

.SUFFIXES:

.PHONY: clean
clean:
	rm -rvf test/executables/*.out test/executables/*.exe test/executables/*.dSYM test/executables/*.stackdump