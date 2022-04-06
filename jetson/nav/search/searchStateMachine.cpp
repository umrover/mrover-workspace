#include "searchStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "searchFromPathFile.hpp"

#include <cmath>
#include <utility>
#include <iostream>

// Constructs an SearchStateMachine object with mStateMachine, mConfig, and mRover
SearchStateMachine::SearchStateMachine(std::weak_ptr<StateMachine> sm, const rapidjson::Document& roverConfig)
        : mStateMachine(move(sm)), mRoverConfig(roverConfig) {}

// Runs the search state machine through one iteration. This will be called by
// StateMachine  when NavState is in a search state. This will call the corresponding
// function based on the current state and return the next NavState
NavState SearchStateMachine::run() {
    switch (mStateMachine.lock()->getRover()->currentState()) {
        case NavState::SearchTurn: {
            return executeSearchTurn();
        }

        case NavState::SearchDrive: {
            return executeSearchDrive();
        }

        case NavState::TurnToTarget: {
            return executeTurnToTarget();
        }

        case NavState::DriveToTarget: {
            return executeDriveToTarget();
        }

        default: {
            std::cerr << "Entered Unknown NavState in search state machine" << std::endl;
            return NavState::Unknown;
        }
    } // switch
} // run()

// Executes the logic for turning while searching.
// If no remaining search points, it proceeds to change search algorithms.
// If the rover detects the target, it proceeds to the target.
// If the rover finishes turning, it proceeds to driving while searching.
// Else the rover keeps turning to the next Waypoint.
NavState SearchStateMachine::executeSearchTurn() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();

    if (mSearchPoints.empty()) {
        return NavState::BeginSearch;
    }

    // You may as well just go to the left post always
    double distance = sm->getRover()->leftCacheTarget().distance;
    bool isWantedTarget = sm->getCourseState()->getRemainingWaypoints().front().id == sm->getRover()->leftCacheTarget().id;
    if (distance >= 0 && isWantedTarget) {
        return NavState::TurnToTarget;
    }

    Odometry& nextSearchPoint = mSearchPoints.front();
    if (sm->getRover()->turn(nextSearchPoint)) {
        return NavState::SearchDrive;
    }
    return NavState::SearchTurn;
} // executeSearchTurn()

// Executes the logic for driving while searching.
// If the rover detects the target, it proceeds to the target.
// If the rover detects an obstacle and is within the obstacle 
// distance threshold, it proceeds to obstacle avoidance.
// If the rover finishes driving, it proceeds to turning to the next Waypoint.
// If the rover is still on course, it keeps driving to the next Waypoint.
// Else the rover turns to the next Waypoint or turns back to the current Waypoint
NavState SearchStateMachine::executeSearchDrive() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Rover> rover = sm->getRover();

    int16_t frontId = sm->getCourseState()->getRemainingWaypoints().front().id;
    if (rover->leftCacheTarget().distance >= 0 && frontId == rover->leftCacheTarget().id) {
        return NavState::TurnToTarget;
    }

    if (isObstacleDetected(rover, sm->getEnv()) && isObstacleInThreshold(rover, sm->getEnv(), mRoverConfig)) {
        Obstacle obstacle = sm->getEnv()->getObstacle();
        sm->updateObstacleElements(obstacle.bearing, obstacle.rightBearing, obstacle.distance);
        return NavState::SearchTurnAroundObs;
    }
    DriveStatus driveStatus = rover->drive(mSearchPoints.front());

    if (driveStatus == DriveStatus::Arrived) {
        mSearchPoints.pop_front();
        return NavState::SearchTurn;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::SearchDrive;
    }
    return NavState::SearchTurn;
} // executeSearchDrive()

// Executes the logic for turning to the target.
// If the rover loses the target, will continue to turn using last known angles.
// If the rover finishes turning to the target, it goes into waiting state to
// give CV time to relocate the target
// Else the rover continues to turn to the target.
NavState SearchStateMachine::executeTurnToTarget() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Rover> rover = sm->getRover();

    if (rover->leftCacheTarget().distance == mRoverConfig["navThresholds"]["noTargetDist"].GetDouble()) {
        return NavState::SearchTurn;
    }
    if (rover->turn(rover->leftCacheTarget().bearing + rover->odometry().bearing_deg)) {
        return NavState::DriveToTarget;
    }
    return NavState::TurnToTarget;
} // executeTurnToTarget()

// Executes the logic for driving to the target.
// If the rover loses the target, it continues with search by going to
// the last point before the rover turned to the target
// If the rover detects an obstacle and is within the obstacle 
// distance threshold, it proceeds to go around the obstacle.
// If the rover finishes driving to the target, it moves on to the next Waypoint.
// If the rover is on course, it keeps driving to the target.
// Else, it turns back to face the target.
NavState SearchStateMachine::executeDriveToTarget() {
    std::shared_ptr<StateMachine> sm = mStateMachine.lock();
    std::shared_ptr<Environment> env = sm->getEnv();
    std::shared_ptr<Rover> rover = sm->getRover();

    // Definitely cannot find the target
    if (rover->leftCacheTarget().distance == mRoverConfig["navThresholds"]["noTargetDist"].GetDouble()) {
        std::cerr << "Lost the target\n";
        return NavState::SearchTurn;
    }

    // Obstacle Detected
    if (isObstacleDetected(rover, env)
        && !isTargetReachable(rover, env, mRoverConfig)
        && isObstacleInThreshold(rover, env, mRoverConfig)) {
        Obstacle obstacle = env->getObstacle();
        sm->updateObstacleElements(obstacle.bearing, obstacle.rightBearing, obstacle.distance);
        return NavState::SearchTurnAroundObs;
    }

    DriveStatus driveStatus;

    double distance = rover->leftCacheTarget().distance;
    double bearing = rover->leftCacheTarget().bearing + rover->odometry().bearing_deg;

    driveStatus = rover->drive(distance, bearing, mRoverConfig["navThresholds"]["targetDistance"].GetDouble());

    if (driveStatus == DriveStatus::Arrived) {
        mSearchPoints.clear();
        Waypoint completed = sm->getCourseState()->completeCurrentWaypoint();
        return completed.gate ? NavState::BeginGateSearch : NavState::Turn;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return NavState::DriveToTarget;
    }
    return NavState::TurnToTarget;
} // executeDriveToTarget()

// add intermediate points between the existing search points in a path generated by a search algorithm.
// The maximum separation between any points in the search point list is determined by the rover's sight distance.
void SearchStateMachine::insertIntermediatePoints() {
    double visionDistance = mRoverConfig["computerVision"]["visionDistance"].GetDouble();
    const double maxDifference = 2 * visionDistance;

    for (int i = 0; i < int(mSearchPoints.size()) - 1; ++i) {
        Odometry point1 = mSearchPoints.at(i);
        Odometry point2 = mSearchPoints.at(i + 1);
        double distance = estimateNoneuclid(point1, point2);
        if (distance > maxDifference) {
            int numPoints = int(ceil(distance / maxDifference) - 1);
            double newDifference = distance / (numPoints + 1);
            double bearing = calcBearing(point1, point2);
            for (int j = 0; j < numPoints; ++j) {
                Odometry startPoint = mSearchPoints.at(i);
                Odometry newOdom = createOdom(startPoint, bearing, newDifference, mStateMachine.lock()->getRover());
                auto insertPosition = mSearchPoints.begin() + i + 1;
                mSearchPoints.insert(insertPosition, newOdom);
                ++i;
            }
        }
    }
} // insertIntermediatePoints()

// The search factory allows for the creation of search objects and
// an ease of transition between search algorithms
std::shared_ptr<SearchStateMachine>
SearchFactory(const std::weak_ptr<StateMachine>& sm, SearchType type, const std::shared_ptr<Rover>& rover, const rapidjson::Document& roverConfig) {
    std::shared_ptr<SearchStateMachine> search = nullptr;
    switch (type) {
        case SearchType::FROM_PATH_FILE:
            search = std::make_shared<SearchFromPathFile>(sm, roverConfig, "jetson/nav/search/spiral_search_points.txt");
            break;
        default:
            std::cerr << "Unknown Search Type. Defaulting to Spiral\n";
            break;
    }
    return search;
} // SearchFactory

/******************/
/* TODOS */
/******************/
// save location of target then go around object? ( Execute drive to target )
// look into removing the waiting when turning to target or at least doing this a better way. This should at very least be it's own state
