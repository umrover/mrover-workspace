#include "diamondGateSearch.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <utility>

DiamondGateSearch::DiamondGateSearch(std::weak_ptr<StateMachine> sm, const rapidjson::Document& roverConfig)
        : GateStateMachine(move(sm), roverConfig) {}

DiamondGateSearch::~DiamondGateSearch() = default;

void DiamondGateSearch::initializeSearch() {
//    mGateSearchPoints.clear();
//    Odometry currOdom = mRover->odometry();
//    double diamondWidth = mRover->path().front().gate_width * 1.5;
//    const double targetBearing = mRover->leftCacheTarget().bearing;
//    const double targetDist = mRover->leftCacheTarget().distance;
//
//    // TODO: make figure explaining this in drive and link here
//    double distance = sqrt(pow(targetDist, 2) + pow(diamondWidth, 2));
//    double theta = atan2(diamondWidth, targetDist) * 180 / PI;
//    double relTurn = theta + targetBearing;
//    double angle = mod(currOdom.bearing_deg + relTurn, 360); // absolute bearing
//
//    Odometry corner1 = createOdom(currOdom, angle, distance, mRover);
//
//    const double absolute_bear_to_target = mod(currOdom.bearing_deg + targetBearing, 360);
//    Odometry corner2 = createOdom(currOdom, absolute_bear_to_target, diamondWidth + targetDist, mRover);
//
//    relTurn = -1 * theta + targetBearing;
//    angle = mod(currOdom.bearing_deg + relTurn, 360);
//    Odometry corner3 = createOdom(currOdom, angle, distance, mRover);
//
//    Odometry corner4 = createOdom(currOdom, mod(absolute_bear_to_target + 180, 360), diamondWidth - targetDist, mRover);
//
//    mGateSearchPoints.push_back(corner1);
//    mGateSearchPoints.push_back(corner2);
//    mGateSearchPoints.push_back(corner3);
//    mGateSearchPoints.push_back(corner4);
} // initializeSearch()
