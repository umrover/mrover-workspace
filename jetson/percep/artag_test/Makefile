CXX = g++
SOURCES = test_framework.cpp main.cpp
OBJECTS  = $(SOURCES:%.cpp=%.o)
CXXFLAGS = -std=c++1y -g -Wall
OPENCV = `pkg-config opencv --cflags --libs`
EXECUTABLE = artag_test

test:
	$(CXX) $(CXXFLAGS) $(SOURCES) -o $(EXECUTABLE)

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)
	rm -Rf *.dSYM