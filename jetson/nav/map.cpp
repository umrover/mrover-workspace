#include <iostream>
#include <vector>
// #include <opencv2/opencv.hpp>
// #include <opencv2/aruco.hpp>
#include <string>
#include <math.h> 
#include "map.h"

//libraries for coloring Map
#include <stddef.h>
#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define PI 3.14159265

using namespace std;

Map::Map(string inputPath) 
: xTotalDisplacement(0.0), yTotalDisplacement(0.0) {
    ReadInputImage(inputPath);
} 

vector<uint8_t> Map::ReadInputImage(string inputPath){
    
}

double Map::updateHeading(const double &oldHeading, const double &joystickHeading){
    double newHeading = oldHeading + joystickHeading;
    if(newHeading >= 360){
        newHeading -= 360;
    }
    return newHeading;
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



void Map::displayLCM(){
    
}

void Map::colorMap(){
    // after Update position, the rover is at currentIndices (x and y coordinates)
    // from here, get the equilateral triangle of side length 25
    // to "simulate" the camera view of the rover 

    // look at the corresponding area on the Input Map (which is colored with obstacles) 
    // on the Navigation Map, color the map 
}

void Map::saveMapToImage(){
    
}


// PRIVATE MEMBER FUNCTION DECLARATIONS
void Map::ScanLine(int x1, int x2, int y1, int y2) {
    int sx, sy, dx1, dy1, dx2, dy2, x, y, m, n, k, cnt;

    sx = x2 - x1;
    sy = y2 - y1;

    /*
        3   2   1
        \  |  /
            \ | /
            \|/
    4 --------+--------- 0
            /|\
            / | \
        /  |  \
        5   6   7
            4 -> 0
            5 -> 1
            6 -> 2
            7 -> 3
    */
    if (sy < 0 || sy == 0 && sx < 0)
    {
        k = x1; x1 = x2; x2 = k;
        k = y1; y1 = y2; y2 = k;
        sx = -sx;
        sy = -sy;
    }

    if (sx > 0) dx1 = 1;
    else if (sx < 0) dx1 = -1;
    else dx1 = 0;

    if (sy > 0) dy1 = 1;
    else if (sy < 0) dy1 = -1;
    else dy1 = 0;

    m = abs(sx);
    n = abs(sy);
    dx2 = dx1;
    dy2 = 0;

    if (m < n)
    {
        m = abs(sy);
        n = abs(sx);
        dx2 = 0;
        dy2 = dy1;
    }

    x = x1; y = y1;
    cnt = m + 1;
    k = n / 2;

    while (cnt--) {
        if ((y >= 0) && (y < dimension)){
            if (x < ContourX[y][0]) {
                ContourX[y][0] = x;
            }
            if (x > ContourX[y][1]) {
                ContourX[y][1] = x;
            }

            k += n;
            if (k < m) {
                x += dx2;
                y += dy2;
            }
            else {
                k -= m;
                x += dx1;
                y += dy1;
            }
        }
    }
}

void Map::DrawTriangle(Square p0, Square p1, Square p2) {
  long y;

  for (y = 0; y < dimension; y++) {
    ContourX[y][0] = LONG_MAX; // min X
    ContourX[y][1] = LONG_MIN; // max X
  }

  ScanLine(p0.x, p0.y, p1.x, p1.y);
  ScanLine(p1.x, p1.y, p2.x, p2.y);
  ScanLine(p2.x, p2.y, p0.x, p0.y);

  for (y = 0; y < dimension; y++) {
    if (ContourX[y][1] >= ContourX[y][0]) {
      long x = ContourX[y][0];
      long len = 1 + ContourX[y][1] - ContourX[y][0];

      // Can draw a horizontal line instead of individual pixels here
      while (len--) {
        SetPixel(x++, y, p0.color);
      }
    }
  }
}

Map::Square Map::getTriangleVertex(Map::Square& start){
    Square triangleVertex; 
    return triangleVertex; 
}