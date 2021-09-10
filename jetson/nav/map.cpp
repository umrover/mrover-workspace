#include <iostream>
#include <vector>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
#include <string>
#include <math.h> 
#include "map.h"

#define PI 3.14159265

using namespace std;

Map::Map(string inputPath) 
: xTotalDisplacement(0.0), yTotalDisplacement(0.0) {
    ReadInputImage(inputPath);
} 

vector<uint8_t> Map::ReadInputImage(string inputPath){
    
}

// temporary paramenters: joystickHeading is in degrees
void Map::updatePosition(double time, double joystickMagnitude, double joystickHeading) {
    // calculate distance rover is to be moved
    // test negative displacements
    xTotalDisplacement += time*joystickMagnitude*cos(joystickHeading*PI / 180.0); 
    yTotalDisplacement += time*joystickMagnitude*sin(joystickHeading*PI / 180.0);
    currentHeading = updateHeading(currentHeading, joystickHeading);

    int number_cells_x = floor(xTotalDisplacement / cellLength);  
    int number_cells_y = floor(yTotalDisplacement / cellLength);  

    currentIndices.first = startIndices.first + number_cells_x; 
    currentIndices.second = startIndices.second + number_cells_y;
}

double updateHeading(const double &oldHeading, const double &joystickHeading){
    double newHeading = oldHeading + joystickHeading;
    if(newHeading >= 360){
        newHeading -= 360;
    }
    return newHeading;
}

void Map::displayLCM(){
    
}

vector<uint8_t> Map::extractObstacles(){
    // after Update position, the rover is at currentIndices (x and y coordinates)
    // from here, get the equilateral triangle of side length 25
    // to "simulate" the camera view of the rover 

}

void Map::colorMap(){
    // look at the corresponding area on the Input Map (which is colored with obstacles) 
    // on the Navigation Map, color the map 
}

void Map::saveMapToImage(){
    
}