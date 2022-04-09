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
        mConfig(roverConfig) {
}

GateStateMachine::~GateStateMachine() = default;

void GateStateMachine::updateGateTraversalPath(){
    //TODO: update the gatePath vector here with a path to go to
//    std::shared_ptr<Environment> env = mStateMachine.lock()->getEnv();
//    Odometry leftPost = env->getLeftPostLocation();
//    Odometry rightPost = env->getRightPostLocation();
}

// Execute loop through gate state machine.
NavState GateStateMachine::run() {
    const std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    auto rover = sm->getRover();
    switch (rover->currentState()) {
        case NavState::BeginGateSearch: {
            mPath.clear();
            return NavState::GateMakePath;
        }
        case NavState::GateMakePath: {
            std::shared_ptr<Environment> env = sm->getEnv();
            if (env->areTargetFiltersReady()) {
                makeDualSegmentPath(rover, env);
                return NavState::GateTraverse;
            } else {
                rover->stop();
                mPath.clear();
            }
            return NavState::GateMakePath;
        }
        case NavState::GateTraverse: {
            if (mPath.empty()) {
                return NavState::Done;
            } else {
                Odometry const& front = mPath.front();
                if (rover->turn(front, sm->getDtSeconds())) {
                    DriveStatus status = rover->drive(front, sm->getDtSeconds(), mConfig["navThresholds"]["targetDistance"].GetDouble());
                    if (status == DriveStatus::Arrived) {
                        mPath.pop_front();
                    }
                }
            }
            return NavState::GateTraverse;
        }
        default: {
            std::cerr << "Entered Unknown NavState in search state machine" << std::endl;
            return NavState::Unknown;
        }
    } // switch
}

void GateStateMachine::makeDualSegmentPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment>& env) {
    Vector2d p1 = env->getLeftPostRelative();
    Vector2d p2 = env->getRightPostRelative();
    Vector2d v = p2 - p1;
    Vector2d m = p1 + v / 2;
    double driveDist = v.dot(m) / m.norm() + mConfig["navThresholds"]["waypointDistance"].GetDouble();
    double currentBearing = rover->odometry().bearing_deg;
    double perpBearing = currentBearing + radianToDegree(atan2(v.y(), v.x()));
    Odometry perpOdometry = createOdom(rover->odometry(), perpBearing, driveDist, rover);
    mPath.push_back(perpOdometry);
    Odometry throughOdometry = createOdom(perpOdometry, perpBearing - 105.0, 3.0, rover);
    mPath.push_back(throughOdometry);
}
// run

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig) {
    return std::make_shared<GateStateMachine>(sm, roverConfig);
} // GateFactory()