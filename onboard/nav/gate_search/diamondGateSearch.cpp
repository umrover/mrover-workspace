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
    double diamondWidth = mPhoebe->roverStatus().path().front().gate_width * 1.5;
    const double targetBearing = mPhoebe->roverStatus().target().bearing;
    const double targetDist = mPhoebe->roverStatus().target().distance;

    // TODO: make figure explaining this in drive and link here
    double distance = sqrt(pow(targetDist, 2) + pow(diamondWidth, 2));
    double theta = atan2(diamondWidth, targetDist) * 180 / PI;
    double relTurn = theta + targetBearing;
    double angle = mod(currOdom.bearing_deg + relTurn, 360); // absolute bearing

    Odometry corner1 = createOdom(currOdom, angle, distance, mPhoebe);

    const double absolute_bear_to_target = mod(currOdom.bearing_deg + targetBearing, 360);
    Odometry corner2 = createOdom(currOdom, absolute_bear_to_target, diamondWidth + targetDist, mPhoebe);

    relTurn = -1 * theta + targetBearing;
    angle = mod(currOdom.bearing_deg + relTurn, 360);
    Odometry corner3 = createOdom(currOdom, angle, distance, mPhoebe);

    Odometry corner4 = createOdom(currOdom, mod(absolute_bear_to_target + 180, 360), diamondWidth - targetDist, mPhoebe);

    mGateSearchPoints.push_back(corner1);
    mGateSearchPoints.push_back(corner2);
    mGateSearchPoints.push_back(corner3);
    mGateSearchPoints.push_back(corner4);
} // initializeSearch()
