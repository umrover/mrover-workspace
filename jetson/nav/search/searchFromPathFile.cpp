#include "searchFromPathFile.hpp"

#include <iostream>
#include <fstream>

#include "stateMachine.hpp"

using namespace std;

void SearchFromPathFile::initializeSearch(const rapidjson::Document& roverConfig, double visionDistance) {
    mSearchPoints.clear();
    ifstream coordinate_file(mPath);
    if (!coordinate_file) {
        throw runtime_error("Could not open spiral search points file at: " + mPath);
    }
    float rho, phi;
    while (coordinate_file >> rho >> phi) {
        Odometry currentPoint = mStateMachine.lock()->getCourseState()->getRemainingWaypoints().front().odom;
        Odometry nextSearchPoint = createOdom(currentPoint, phi, rho, mStateMachine.lock()->getRover());
        mSearchPoints.push_back(nextSearchPoint);
    }
    coordinate_file.close();
} // initializeSearch()
