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

Map::Map(const vector< vector <vector<uint8_t> > > &inputMap, uint32_t row_start, uint32_t col_start) : 
inputMap(inputMap), navigationMap(3,vector<vector<uint8_t> >(dimension, vector<uint8_t> (dimension, 128))), 
startIndices(row_start,col_start), 
xTotalDisplacement(0.0), yTotalDisplacement(0.0),currentHeading(0.0) {}; 

double Map::updateHeading(double oldHeading, Joystick joystick){
    double newHeading;

    if(joystick.forward_back >=0){
        newHeading = acos(joystick.left_right);
    }else{
        newHeading = acos(joystick.left_right) + PI/2;
    }

    newHeading = oldHeading + joystickHeading;
    if(newHeading >= 2*PI) {
        newHeading -= 2*PI;
    }
    return newHeading;
}

vector< vector <vector<uint8_t> > >& Map::updateNavMap(double time, Joystick joystick) {
    // calculate distance rover is to be moved
    // test negative displacements

    currentHeading = updateHeading(currentHeading, joystick);
    xTotalDisplacement += time*speed*abs(joystick.forward_back)*cos(currentHeading*PI/180.0)); 
    yTotalDisplacement += time*speed*abs(joystick.forward_back)*sin(currentHeading*PI/180.0));

    int number_cells_x = floor(xTotalDisplacement / cellLength);  
    int number_cells_y = floor(yTotalDisplacement / cellLength);  

    currentIndices.x = startIndices.x + number_cells_x; 
    currentIndices.y = startIndices.y + number_cells_y;

    pair<Map::Square, Map::Square> twoVertices = getTriangleVertex(currentIndices);
    DrawTriangle(currentIndices, twoVertices.first, twoVertices.second);
    return navigationMap;
}

void Map::displayLCM(){
    std::cout << "Current x and y indices: " << currentIndices.x << " " << currentIndices.y << std::endl;
    std::cout << "Current heading: " << currentHeading << std::endl;
    std::cout << "Joystick LCM:  forward_back: " << joystick.forward_back << std::endl;
    std::cout << "Joystick LCM:  left_right: " << joystick.left_right << std::endl;
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
        Color c;
        c.r = inputMap[0][x][y];
        c.g = inputMap[1][x][y];
        c.b = inputMap[2][x][y];
        SetPixel(x++, y, c);
      }
    }
  }
}

void Map::SetPixel(int x, int y, Color color) {
    if ((x < 0) || (x >= dimension) || (y < 0) || (y >= dimension)) { return; }
    
    if(navigationMap[0][x][y]==128 && navigationMap[1][x][y]==128 && navigationMap[2][x][y]==128){
        navigationMap[0][y][x] = color.r;
        navigationMap[1][y][x] = color.g; 
        navigationMap[2][y][x] = color.b;
    }
}

pair<Map::Square, Map::Square> Map::getTriangleVertex(Map::Square& start){
    pair<Square, Square> triangleVertices; 
    double theta_1 = getHeading() - 30; // this is in degrees
    double theta_2 = getHeading() + 30; // this is in degrees
    
    //naive approach
    Square vertex_1(25*cos(theta_1*PI/180.0), 25*sin(theta_1*PI/180.0));

    Square vertex_2(25*cos(theta_2*PI/180.0), 25*sin(theta_2*PI/180.0));

    triangleVertices.first = vertex_1;
    triangleVertices.second = vertex_2; 

    return triangleVertices; 
}