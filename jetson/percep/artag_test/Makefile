CXX = g++
SOURCES = artag_test_errors.cpp artag_test_framework.cpp main.cpp
OBJECTS  = $(SOURCES:%.cpp=%.o)
CXXFLAGS = -std=c++1y -g -Wall
OPENCV = $(shell pkg-config --libs opencv)
EXECUTABLE = artag_test

test:
	$(CXX) $(CXXFLAGS) $(OPENCV) $(SOURCES) -o $(EXECUTABLE)

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)
	rm -Rf *.dSYM