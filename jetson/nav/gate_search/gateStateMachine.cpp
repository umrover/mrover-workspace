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

//gets the point that the rover should follow off the path
//the farthest along point on the path that doesn't intersect with the gate
Odometry GateStateMachine::getPointToFollow(Odometry curRoverLocation) {
    //todo: optimize away the prep point
    return mPath[mPathIndex];
}

// Execute loop through gate state machine.
NavState GateStateMachine::run() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();

    publishGatePath();
    switch (rover->currentState()) {
        case NavState::BeginGateSearch: {
            mPathIndex = 0;
            updateGateTraversalPath();
            return NavState::GateTraverse;
        }
        case NavState::GateTraverse: {
            if (mPathIndex == mPath.size()) {
//                std::exit(1);
                return NavState::Done;
            } else {
                Odometry const& toFollow = getPointToFollow(rover->odometry());
                double dt = sm->getDtSeconds();
                if (rover->drive(toFollow, mConfig["navThresholds"]["waypointDistance"].GetDouble(), dt)) {
                    std::cout << "At gate path index: " << mPathIndex << std::endl;
                    ++mPathIndex;
                }
            }
            if (mPathIndex > 0) {
                // This avoids the situation where you approach the gate
                // perpendicularly and try to rapidly switch between
                // approaching either side
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

void printPoint(Vector2d p) {
    std::cout << "(" << p.x() << " , " << p.y() << "),";
}

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
}


void GateStateMachine::makeSpiderPath(std::shared_ptr<Rover> const& rover, std::shared_ptr<Environment> const& env) {
    Vector2d p1 = env->getPostOneOffsetInCartesian(rover->odometry());
    Vector2d p2 = env->getPostTwoOffsetInCartesian(rover->odometry());
    Vector2d center = (p1 + p2) / 2;
    // TODO make this a constant
    double approachDistance = 2.0;
    Vector2d postDir = (p2 - p1).normalized();
    Vector2d perp = {-postDir.y(), postDir.x()};
    Vector2d approachPoints[2] = {(perp * approachDistance) + center,
                                  (perp * -approachDistance) + center};
    Vector2d prepPoints[4] = {(2 * approachDistance * perp) + p1,
                              (2 * approachDistance * perp) + p2,
                              (-2 * approachDistance * perp) + p1,
                              (-2 * approachDistance * perp) + p2};

    // find closest prep point
    double minNorm = -1.0;
    Vector2d prepPoint;
    for (auto& i: prepPoints) {
        double dist = i.norm();
        if (minNorm == -1.0 || dist < minNorm) {
            minNorm = dist;
            prepPoint = i;
        }
    }

    // find the closest approach point to prep point and set the other one as a victory point (we're through the gate)
    minNorm = -1.0;
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
    Odometry prepOdom = createOdom(cur, prepPoint, rover);
    Odometry approachOdom = createOdom(cur, approachPoint, rover);
    Odometry centerOdom = createOdom(cur, center, rover);
    Odometry victoryOdom = createOdom(cur, victoryPoint, rover);
    mPath.clear();
    // Near 1 if we are parallel to gate finish line
    double gateAlignment = center.normalized().dot(postDir);
    if (std::fabs(gateAlignment) > 0.75)
        mPath.push_back(prepOdom);
    mPath.push_back(approachOdom);
    mPath.push_back(centerOdom);
    mPath.push_back(victoryOdom);

    // std::cout << "points = (";
    // printPoint(p1);
    // printPoint(p2);
    // printPoint(prepPoint);
    // printPoint(approachPoint);
    // printPoint(center);
    // printPoint(victoryPoint);
    // std::cout << ")" << std::endl;
}

// Creates an GateStateMachine object
std::shared_ptr<GateStateMachine> GateFactory(const std::weak_ptr<StateMachine>& sm, const rapidjson::Document& roverConfig) {
    return std::make_shared<GateStateMachine>(sm, roverConfig);
} // GateFactory()

// Sends search path rover takes when trying to find posts
void GateStateMachine::publishGatePath() {
    // Construct vector from deque
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    mProjectedPoints.points.assign(mPath.begin(), mPath.end());
    mProjectedPoints.points.push_back(env->getPostOneLocation());
    mProjectedPoints.points.push_back(env->getPostTwoLocation());
    mProjectedPoints.pattern_size = static_cast<int32_t>(mProjectedPoints.points.size());
    mProjectedPoints.path_type = "gate-path";

    std::string gatePathChannel = mConfig["lcmChannels"]["gatePathChannel"].GetString();
    sm->getLCM().publish(gatePathChannel, &mProjectedPoints);

} // publishSearchPoints()
