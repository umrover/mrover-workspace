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
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();

    switch (rover->currentState()) {
        case NavState::BeginGateSearch: {
            mPath.clear();
            return NavState::GateMakePath;
        }
        case NavState::GateMakePath: {
            if (env->areTargetFiltersReady()) {
                makeSpiderPath(rover, env);
//                makeDualSegmentPath(rover, env);
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
                double dt = sm->getDtSeconds();
                if (rover->turn(front, dt)) {
                    DriveStatus status = rover->drive(front, dt, mConfig["navThresholds"]["waypointDistance"].GetDouble());
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
    double driveDist = v.dot(m) / m.norm();
    double deltaBearing = radianToDegree(atan2(v.y(), v.x()));
    if (driveDist < 0.0) deltaBearing = deltaBearing - 180.0;
    double perpBearing = rover->odometry().bearing_deg + deltaBearing;
    double finalDriveDist = fabs(driveDist) + mConfig["navThresholds"]["waypointDistance"].GetDouble();
    Odometry perpOdometry = createOdom(rover->odometry(), perpBearing, finalDriveDist, rover);
    mPath.push_back(perpOdometry);
    double rotateBearing = perpBearing - (driveDist > 0 ? 100.0 : -100);
    Odometry throughOdometry = createOdom(perpOdometry, rotateBearing, m.norm() + 2.0, rover);
    mPath.push_back(throughOdometry);
}


void GateStateMachine::makeSpiderPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment>& env) {
    Vector2d p1 = env->getLeftPostRelative();
    Vector2d p2 = env->getRightPostRelative();
    Vector2d center = (p1 + p2) / 2;
    // TODO make this a constant
    double approachDistance = 2.0;
    Vector2d postDir = p2 - p1;
    Vector2d perp = {-postDir.y(), postDir.x()};
    perp = perp / perp.norm();
    Vector2d approachPoints[2] = {(perp * approachDistance) + center,
                                  (perp * -approachDistance) + center};
    Vector2d prepPoints[4] = {(2 * approachDistance * perp) + p1,
                              (2 * approachDistance * perp) + p2,
                              (-2 * approachDistance * perp) + p1,
                              (-2 * approachDistance * perp) + p2};

    // TODO: add logic to go to farthest point along the path that doesn't collid with gate

    // find closest prep point
    double min_norm = -1.0;
    Vector2d prepPoint;
    for (auto& i: prepPoints) {
        double dist = i.norm();
        if (min_norm == -1.0 || dist < min_norm) {
            min_norm = dist;
            prepPoint = i;
        }
    }

    // find the closest approach point to prep point and set the other one as a victory point (we're through the gate)
    min_norm = -1.0;
    Vector2d approachPoint;
    Vector2d victoryPoint;
    double distance1 = (approachPoints[0] - prepPoint).norm();
    double distance2 = (approachPoints[1] - prepPoint).norm();
    if (distance1 < distance2) {
        approachPoint = approachPoints[0];
        victoryPoint = approachPoints[1];
    } else {
        approachPoint = approachPoints[1];
        victoryPoint = approachPoints[0];
    }
    Odometry cur = rover->odometry();
//    std::cout << prepPoint.x() << ", " << prepPoint.y() << " , " << approachPoint.x() << " , " << approachPoint.y()) << std::endl;
    mPath.push_back(createOdom(cur, prepPoint, rover));
    mPath.push_back(createOdom(cur, approachPoint, rover));
    mPath.push_back(createOdom(cur, center, rover));
    mPath.push_back(createOdom(cur, victoryPoint, rover));
}

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig) {
    return std::make_shared<GateStateMachine>(sm, roverConfig);
} // GateFactory()