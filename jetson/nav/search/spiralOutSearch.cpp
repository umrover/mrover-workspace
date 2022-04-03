#include "spiralOutSearch.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <fstream>
#include <cmath>

SpiralOut::~SpiralOut() {}

void SpiralOut::initializeSearch( Rover* rover, const rapidjson::Document& roverConfig, const double visionDistance )
{
    mSearchPoints.clear();
    ifstream coordinate_file (roverConfig[ "search" ][ "spiralSearchPoints" ].GetString());
    float rho, phi;
    while (coordinate_file >> rho >> phi) {
        Odometry nextSearchPoint = createOdom(rover->roverStatus().path().front().odom, phi, rho, rover);
        mSearchPoints.push_back( nextSearchPoint );
    }
    coordinate_file.close();
} // initializeSearch()