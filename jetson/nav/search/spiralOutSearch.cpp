#include "spiralOutSearch.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <fstream>

using namespace std;

void SpiralOut::initializeSearch(const rapidjson::Document& roverConfig, double visionDistance) {
    mSearchPoints.clear();
    string const& path = roverConfig["search"]["spiralSearchPoints"].GetString();
    ifstream coordinate_file(path);
    if (!coordinate_file) {
        throw runtime_error("Could not open spiral search points file at: " + path);
    }
    float rho, phi;
    while (coordinate_file >> rho >> phi) {
        Odometry currentPoint = mStateMachine.lock()->getCourseState()->getRemainingWaypoints().front().odom;
        Odometry nextSearchPoint = createOdom(currentPoint, phi, rho, mStateMachine.lock()->getRover());
        mSearchPoints.push_back(nextSearchPoint);
    }
    coordinate_file.close();
} // initializeSearch()