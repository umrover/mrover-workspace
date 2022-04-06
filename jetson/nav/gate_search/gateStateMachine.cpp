#include "gateStateMachine.hpp"

#include <utility>
#include <iostream>

#include "utilities.hpp"
#include "environment.hpp"
#include "stateMachine.hpp"
#include "./gate_search/circleGateSearch.hpp"

using Eigen::Vector2d;

// Constructs a GateStateMachine object with mStateMachine
GateStateMachine::GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig) :
        mStateMachine(move(stateMachine)),
        mRoverConfig(roverConfig),
        mLeftDistFilter(roverConfig["gate"]["filterSize"].GetInt()),
        mRightDistFilter(roverConfig["gate"]["filterSize"].GetInt()),
        mLeftBearingFilter(roverConfig["gate"]["filterSize"].GetInt()),
        mRightBearingFilter(roverConfig["gate"]["filterSize"].GetInt()) {
}

GateStateMachine::~GateStateMachine() = default;


// Execute loop through gate state machine.
NavState GateStateMachine::run() {
    const std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    auto rover = sm->getRover();
    switch (rover->currentState()) {
        case NavState::BeginGateSearch: {
            mLeftDistFilter.reset();
            mRightDistFilter.reset();
            mLeftBearingFilter.reset();
            mRightBearingFilter.reset();
            mPath.clear();
            return NavState::GateMakePath;
        }
        case NavState::GateMakePath: {
            std::shared_ptr<Environment> env = sm->getEnv();
            TargetList targets = env->getTargets();
            double rawLeftDist = targets.targetList[LEFT_TARGET_IDX].distance;
            if (rawLeftDist > 0) mLeftDistFilter.push(rawLeftDist);
            double rawRightDist = targets.targetList[RIGHT_TARGET_IDX].distance;
            if (rawRightDist > 0) mRightDistFilter.push(rawRightDist);
            mLeftBearingFilter.push(targets.targetList[LEFT_TARGET_IDX].bearing);
            mRightBearingFilter.push(targets.targetList[RIGHT_TARGET_IDX].bearing);
            double leftDist = mLeftDistFilter.get(0.75);
            double rightDist = mRightDistFilter.get(0.75);
            double leftBearing = degreeToRadian(mLeftBearingFilter.get(0.75));
            double rightBearing = degreeToRadian(mRightBearingFilter.get(0.75));
            if (mLeftDistFilter.full() && mRightDistFilter.full() && mLeftBearingFilter.full() && mRightBearingFilter.full()) {
                Vector2d p1{leftDist * cos(leftBearing), leftDist * sin(leftBearing)};
                Vector2d p2{rightDist * cos(rightBearing), rightDist * sin(rightBearing)};
                Vector2d v = p2 - p1;
                Vector2d m = p1 + v / 2;
                double driveDist = v.dot(m) / v.norm() + mRoverConfig["navThresholds"]["waypointDistance"].GetDouble();
                double currentBearing = rover->odometry().bearing_deg;
                double perpBearing = currentBearing + radianToDegree(atan2(v.y(), v.x()));
                Odometry perpOdometry = createOdom(rover->odometry(), perpBearing, driveDist, rover);
                mPath.push_back(perpOdometry);
                Odometry throughOdometry = createOdom(perpOdometry, perpBearing - 105.0, 3.0, rover);
                mPath.push_back(throughOdometry);
                return NavState::GateDrivePath;
            } else {
                rover->stop();
            }
            return NavState::GateMakePath;
        }
        case NavState::GateDrivePath: {
            if (mPath.empty()) {
                return NavState::Done;
            } else {
                Odometry const& front = mPath.front();
                if (rover->turn(front)) {
                    DriveStatus status = rover->drive(front);
                    if (status == DriveStatus::Arrived) {
                        mPath.pop_front();
                    }
                }
            }
            return NavState::GateDrivePath;
        }
        default: {
            std::cerr << "Entered Unknown NavState in search state machine" << std::endl;
            return NavState::Unknown;
        }
    } // switch
} // run

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig) {
    return std::make_shared<CircleGateSearch>(sm, roverConfig);
} // GateFactory()