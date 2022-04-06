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
    auto rover = mStateMachine.lock()->getRover();
    switch (rover->currentState()) {
        case NavState::BeginGateSearch: {
            mLeftDistFilter.reset();
            mRightDistFilter.reset();
            mLeftBearingFilter.reset();
            mRightBearingFilter.reset();
            return NavState::GateMakePath;
        }
        case NavState::GateMakePath: {
            std::shared_ptr<Environment> env = mStateMachine.lock()->getEnv();
            TargetList targets = env->getTargets();
            mLeftDistFilter.add(targets.targetList[LEFT_TARGET_IDX].distance);
            mRightDistFilter.add(targets.targetList[RIGHT_TARGET_IDX].distance);
            mLeftBearingFilter.add(targets.targetList[LEFT_TARGET_IDX].bearing);
            mRightBearingFilter.add(targets.targetList[RIGHT_TARGET_IDX].bearing);
            double leftDist, rightDist, leftBearing, rightBearing;
            if (mLeftDistFilter.get(0.75, leftDist)
                && mRightDistFilter.get(0.75, rightDist)
                && mLeftBearingFilter.get(0.75, leftBearing)
                && mRightBearingFilter.get(0.75, rightBearing)) {
                Vector2d p1{leftDist * cos(leftBearing), leftDist * sin(leftBearing)};
                Vector2d p2{rightDist * cos(rightBearing), rightDist * sin(rightBearing)};
                Vector2d v = p2 - p1;
                Vector2d m = p1 + v / 2;
                double driveDist = v.dot(m) / v.norm();
                double driveBearing = atan2(v.y(), v.x());
                rover->drive(driveDist, driveBearing);
            } else {
                rover->stop();
            }
            return NavState::GateMakePath;
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