#include "diamondGateSearch.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <cmath>

DiamondGateSearch::DiamondGateSearch( StateMachine* stateMachine, Rover* rover, const rapidjson::Document& roverConfig )
    : GateStateMachine(stateMachine, rover, roverConfig ) {}

DiamondGateSearch::~DiamondGateSearch() {}

void DiamondGateSearch::initializeSearch()
{
    mGateSearchPoints.clear();
    Odometry currOdom = mPhoebe->roverStatus().odometry();
    double diamondWidth = mPhoebe->roverStatus().path().front().gate_width * 3;

    // TODO: make figure explaining this in drive and link here
    const double height = cos(mPhoebe->roverStatus().target().bearing * PI/180) * mPhoebe->roverStatus().target().distance;
    double width = diamondWidth + sin(mPhoebe->roverStatus().target().bearing * PI/180) * mPhoebe->roverStatus().target().distance;
    double distance = sqrt(pow(height, 2) + pow(width, 2));
    const double turnAngle = atan2(width, height) * 180 / PI;
    double angle = mod(currOdom.bearing_deg + turnAngle, 360); // absolute bearing

    cout << "corner1 start\n";
    cout << "height: " << height << "\nwidth: " << width << "\ndistance " << distance << "\nturnAngle " << turnAngle << "\nangle " << angle << endl;
    cout << "corner1 end\n";

    Odometry corner1 = createOdom(currOdom, angle, distance, mPhoebe);

    const double absolute_bear_to_target = mod(currOdom.bearing_deg + mPhoebe->roverStatus().target().bearing, 360);
    Odometry corner2 = createOdom(currOdom, absolute_bear_to_target, diamondWidth + mPhoebe->roverStatus().target().distance, mPhoebe);

    angle -= 90;
    width = diamondWidth - sin(mPhoebe->roverStatus().target().bearing * PI/180) * mPhoebe->roverStatus().target().distance;
    distance = sqrt(pow(height, 2) + pow(width, 2));
    Odometry corner3 = createOdom(currOdom, angle, distance, mPhoebe);

    cout << corner1.latitude_min << " " << corner1.longitude_min << "\n";
    cout << corner2.latitude_min << " " << corner2.longitude_min << "\n";
    cout << corner3.latitude_min << " " << corner3.longitude_min << "\n";
    cout << currOdom.latitude_min << " " << currOdom.longitude_min << "\n\n";
    mGateSearchPoints.push_back(corner1);
    mGateSearchPoints.push_back(corner2);
    mGateSearchPoints.push_back(corner3);
    mGateSearchPoints.push_back(currOdom);
} // initializeSearch()
