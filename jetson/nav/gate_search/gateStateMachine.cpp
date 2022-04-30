#include "gateStateMachine.hpp"

#include <utility>
#include <iostream>

#include "utilities.hpp"
#include "environment.hpp"
#include "stateMachine.hpp"

using Eigen::Vector2d;

// Constructs a GateStateMachine object with mStateMachine
GateStateMachine::GateStateMachine(std::weak_ptr<StateMachine> stateMachine, const rapidjson::Document& roverConfig) :
        mStateMachine(move(stateMachine)),
        mConfig(roverConfig) {
}

GateStateMachine::~GateStateMachine() = default;

void GateStateMachine::updateGateTraversalPath() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    makeSpiderPath(sm->getRover(), sm->getEnv());
    publishGatePath();
}

// TODO: still needed?
Odometry GateStateMachine::getPointToFollow(Odometry curRoverLocation) {
    return mPath[mPathIndex];
}

/***
 * @return Whether we are "parallel" to the gate,
 * which in this case means we are approaching the gate from the side.
 * If this value is false it means we can drive straight through the gate
 * since we are head on.
 */
bool GateStateMachine::isParallelToGate() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    Vector2d p1 = sm->getEnv()->getPostOneOffsetInCartesian(sm->getRover()->odometry());
    Vector2d p2 = sm->getEnv()->getPostTwoOffsetInCartesian(sm->getRover()->odometry());
    Vector2d center = (p1 + p2) / 2;
    Vector2d postDir = p2 - p1;
    center.normalize();
    postDir.normalize();
    double gateAlignment = center.dot(postDir);
    return std::fabs(gateAlignment) > 0.75;
}

/***
 * Main gate search machine logic:
 * If we have just began, check how aligned we are with the gate.
 * This decides what paths to include in our gate path.
 * If we are traversing the gate, try to go to the next post.
 * If we are past the prep point try to remake the point from new readings.
 *
 * @return Next state
 */
NavState GateStateMachine::run() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();

    publishGatePath();
    switch (rover->currentState()) {
        case NavState::BeginGateSearch: {
            // If we are parallel to the gate, we can't simply align a bit then drive through.
            // We have to navigate around the closest point, so include the prep point,
            // which is always index zero.
            mPathIndex = isParallelToGate() ? 0 : 1;
            updateGateTraversalPath();
            return NavState::GateTraverse;
        }
        case NavState::GateTraverse: {
            if (mPathIndex == mPath.size()) {
                return NavState::Done;
            } else {
                Odometry const& toFollow = getPointToFollow(rover->odometry());
                double dt = sm->getDtSeconds();
                if (rover->drive(toFollow, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                    std::cout << "At gate path index: " << mPathIndex << std::endl;
                    ++mPathIndex;
                }
            }
            // This check avoids the situation where you approach the gate parallel
            // and rapidly try to switch approaching from either side
            if (mPathIndex > 1) {
                if (env->hasNewPostUpdate() && env->hasGateLocation()) {
                    updateGateTraversalPath();
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

/***
 * Spider path include four points:
 * Prep point if we are not aligned.
 * Line up so we are in front of the gate.
 * Drive to the center.
 * Drive past the center to the end.
 */
void GateStateMachine::makeSpiderPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment> const& env) {
    Vector2d p1 = env->getPostOneOffsetInCartesian(rover->odometry());
    Vector2d p2 = env->getPostTwoOffsetInCartesian(rover->odometry());
    Vector2d center = (p1 + p2) / 2;
    // TODO: make this config
    double approachDistance = 3.0;
    Vector2d postDir = (p2 - p1).normalized();
    Vector2d perp = {-postDir.y(), postDir.x()};
    Vector2d approachPoints[2] = {(perp * approachDistance) + center,
                                  (perp * -approachDistance) + center};
    Vector2d prepPoints[4] = {(2 * approachDistance * perp) + p1,
                              (2 * approachDistance * perp) + p2,
                              (-2 * approachDistance * perp) + p1,
                              (-2 * approachDistance * perp) + p2};

    // Find closest prep point
    double minNorm = -1.0;
    Vector2d prepPoint;
    for (auto& i: prepPoints) {
        double dist = i.norm();
        if (minNorm == -1.0 || dist < minNorm) {
            minNorm = dist;
            prepPoint = i;
        }
    }

    // Find the closest approach point to prep point and set the other one as a victory point (we're through the gate)
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
    Odometry prepOdom = createOdom(cur, prepPoint, rover);
    Odometry approachOdom = createOdom(cur, approachPoint, rover);
    Odometry centerOdom = createOdom(cur, center, rover);
    Odometry victoryOdom = createOdom(cur, victoryPoint, rover);
    mPath.clear();
    // Near 1 if we are parallel to gate finish line
    mPath.push_back(prepOdom);
    mPath.push_back(approachOdom);
    mPath.push_back(centerOdom);
    mPath.push_back(victoryOdom);
} // makeSpiderPath()

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig) {
    return std::make_shared<GateStateMachine>(sm, roverConfig);
} // GateFactory()

// Sends search path rover takes when trying to find posts
void GateStateMachine::publishGatePath() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    mProjectedPoints.points.assign(mPath.begin(), mPath.end());
    mProjectedPoints.points.push_back(env->getPostOneLocation());
    mProjectedPoints.points.push_back(env->getPostTwoLocation());
    mProjectedPoints.pattern_size = static_cast<int32_t>(mProjectedPoints.points.size());
    mProjectedPoints.path_type = "gate-path";
    std::string gatePathChannel = mConfig["lcmChannels"]["gatePathChannel"].GetString();
    sm->getLCM().publish(gatePathChannel, &mProjectedPoints);

} // publishGatePath()

void GateStateMachine::makeDualSegmentPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment>& env) {
    Vector2d p1 = env->getPostOneOffsetInCartesian(rover->odometry());
    Vector2d p2 = env->getPostTwoOffsetInCartesian(rover->odometry());
    Vector2d v = p2 - p1;
    Vector2d m = p1 + v / 2;
    double driveDist = v.dot(m) / v.norm();
    double deltaBearing = radianToDegree(atan2(v.y(), v.x()));
    if (driveDist < 0.0) deltaBearing = deltaBearing - 180.0;
    double perpBearing = rover->odometry().bearing_deg + deltaBearing;
    double finalDriveDist = fabs(driveDist) + mConfig["navThresholds"]["waypointDistance"].GetDouble();
    Odometry perpOdometry = createOdom(rover->odometry(), perpBearing, finalDriveDist, rover);
    mPath.push_back(perpOdometry);
    double rotateBearing = perpBearing - (driveDist > 0 ? 105.0 : -105);
    Odometry throughOdometry = createOdom(perpOdometry, rotateBearing, m.norm() + 2.0, rover);
    mPath.push_back(throughOdometry);
} // makeDualSegmentPath()
