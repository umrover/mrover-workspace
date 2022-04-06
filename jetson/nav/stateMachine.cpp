#include "stateMachine.hpp"

#include <map>
#include <utility>
#include <iostream>

#include "utilities.hpp"
#include "rover_msgs/NavStatus.hpp"
#include "gate_search/circleGateSearch.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"

// Constructs a StateMachine object with the input lcm object.
// Reads the configuration file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine(
        rapidjson::Document& config,
        std::shared_ptr<Rover> rover, std::shared_ptr<Environment> env, std::shared_ptr<CourseProgress> courseProgress,
        lcm::LCM& lcmObject
) : mConfig(config), mRover(move(rover)), mEnv(move(env)), mCourseProgress(move(courseProgress)),
    mLcmObject(lcmObject) {
    mSearchStateMachine = SearchFactory(weak_from_this(), SearchType::FROM_PATH_FILE, mRover, mConfig);
    mGateStateMachine = GateFactory(weak_from_this(), mConfig);
    mObstacleAvoidanceStateMachine = ObstacleAvoiderFactory(weak_from_this(),
                                                            ObstacleAvoidanceAlgorithm::SimpleAvoidance, mRover,
                                                            mConfig);
} // StateMachine()

void StateMachine::setSearcher(SearchType type, const std::shared_ptr<Rover>& rover,
                               const rapidjson::Document& roverConfig) {
    mSearchStateMachine = SearchFactory(weak_from_this(), type, rover, roverConfig);
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleDistance(double distance) {
    mObstacleAvoidanceStateMachine->updateObstacleDistance(distance);
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleElements(double leftBearing, double rightBearing, double distance) {
    mObstacleAvoidanceStateMachine->updateObstacleAngle(leftBearing, rightBearing);
    updateObstacleDistance(distance);
}

// Runs the state machine through one iteration. The state machine will
// run if the state has changed or if the rover's status has changed.
// Will call the corresponding function based on the current state.
void StateMachine::run() {
    mRover->updateTargets(mEnv, mCourseProgress);

    publishNavState();
    NavState nextState = NavState::Unknown;

    if (!mRover->autonState().is_auton) {
        nextState = NavState::Off;
        mRover->setState(executeOff()); // turn off immediately
        mCourseProgress->clearProgress();
        if (nextState != mRover->currentState()) {
            mRover->setState(nextState);
        }
        return;
    }
    switch (mRover->currentState()) {
        case NavState::Off: {
            nextState = executeOff();
            break;
        }

        case NavState::Done: {
            nextState = executeDone();
            break;
        }


        case NavState::Turn: {
            nextState = executeTurn();
            break;
        }


        case NavState::Drive: {
            nextState = executeDrive();
            break;
        }


        case NavState::SearchTurn:
        case NavState::SearchDrive:
        case NavState::TurnToTarget:
        case NavState::DriveToTarget: {
            nextState = mSearchStateMachine->run();
            break;
        }

        case NavState::TurnAroundObs:
        case NavState::SearchTurnAroundObs:
        case NavState::DriveAroundObs:
        case NavState::SearchDriveAroundObs: {
            nextState = mObstacleAvoidanceStateMachine->run();
            break;
        }

        case NavState::ChangeSearchAlg: {
            double visionDistance = mConfig["computerVision"]["visionDistance"].GetDouble();
            setSearcher(SearchType::FROM_PATH_FILE, mRover, mConfig);

            mSearchStateMachine->initializeSearch(mConfig, visionDistance);
            nextState = NavState::SearchTurn;
            break;
        }

        case NavState::GatePrepare:
        case NavState::GateMakePath: {
            nextState = mGateStateMachine->run();
            break;
        }

        case NavState::Unknown: {
            throw std::runtime_error("Entered unknown state.");
        }
    } // switch

    if (nextState != mRover->currentState()) {
        mRover->setState(nextState);
        mRover->bearingPid().reset();
    }
    std::cerr << std::flush;
} // run()

// Publishes the current navigation state to the nav status lcm channel.
void StateMachine::publishNavState() const {
    NavStatus navStatus{
            .nav_state_name = stringifyNavState(),
            .completed_wps = static_cast<int32_t>(mCourseProgress->getRemainingWaypoints().size()),
            .total_wps = mCourseProgress->getCourse().num_waypoints
    };
    const std::string& navStatusChannel = mConfig["lcmChannels"]["navStatusChannel"].GetString();
    mLcmObject.publish(navStatusChannel, &navStatus);
} // publishNavState()

// Executes the logic for off. If the rover is turned on, it updates
// the roverStatus. If the course is empty, the rover is done  with
// the course otherwise it will turn to the first waypoint. Else the
// rover is still off.
NavState StateMachine::executeOff() {
    if (mRover->autonState().is_auton) {
        NavState nextState = mCourseProgress->getCourse().num_waypoints ? NavState::Turn : NavState::Done;
        mCourseProgress->clearProgress();
        return nextState;
    }
    mRover->stop();
    return NavState::Off;
} // executeOff()

// Executes the logic for the done state. Stops and turns off the
// rover.
NavState StateMachine::executeDone() {
    mRover->stop();
    return NavState::Done;
} // executeDone()

// Executes the logic for the turning. If the rover is turned off, it
// proceeds to Off. If the rover finishes turning, it drives to the
// next Waypoint. Else the rover keeps turning to the Waypoint.
NavState StateMachine::executeTurn() {
    if (mCourseProgress->getRemainingWaypoints().empty()) {
        return NavState::Done;
    }

    Odometry const& nextPoint = mCourseProgress->getRemainingWaypoints().front().odom;
//    if (estimateNoneuclid(mRover->odometry(), nextPoint) < mConfig["navThresholds"]["waypointDistance"].GetDouble()
//        || mRover->turn(nextPoint)) {
    if (mRover->turn(nextPoint)) {
        return NavState::Drive;
    }

    return NavState::Turn;
} // executeTurn()

// Executes the logic for driving. If the rover is turned off, it
// proceeds to Off. If the rover finishes driving, it either starts
// searching for a target (dependent the search parameter of
// the Waypoint) or it turns to the next Waypoint. If the rover
// detects an obstacle and is within the obstacle distance threshold, 
// it goes to turn around it. Else the rover keeps driving to the next Waypoint.
NavState StateMachine::executeDrive() {
    Waypoint const& nextWaypoint = mCourseProgress->getRemainingWaypoints().front();
    double distance = estimateNoneuclid(mRover->odometry(), nextWaypoint.odom);

    if (isObstacleDetected(mRover, getEnv())
        && !isWaypointReachable(distance)
        && isObstacleInThreshold(mRover, getEnv(), mConfig)) {
        mObstacleAvoidanceStateMachine->updateObstacleElements(mEnv->getObstacle().bearing,
                                                               mEnv->getObstacle().rightBearing,
                                                               getOptimalAvoidanceDistance());
        return NavState::TurnAroundObs;
    }

//    if ((nextWaypoint.search || nextWaypoint.gate)
//        && mRover->leftCacheTarget().id == nextWaypoint.id
//        && distance <= mConfig["navThresholds"]["waypointRadius"].GetDouble()) {
//        return NavState::TurnToTarget;
//    }

    DriveStatus driveStatus = mRover->drive(nextWaypoint.odom);

    if (driveStatus == DriveStatus::Arrived) {
        if (nextWaypoint.search || nextWaypoint.gate) {
            return NavState::ChangeSearchAlg;
        }
        mCourseProgress->completeCurrentWaypoint();
        return NavState::Turn;
    }
    if (driveStatus == DriveStatus::OnCourse) {

        return NavState::Drive;
    }

    return NavState::Turn;
} // executeDrive()

// Gets the string representation of a nav state.
std::string StateMachine::stringifyNavState() const {
    static const std::unordered_map<NavState, std::string> navStateNames =
            {
                    {NavState::Off,                  "Off"},
                    {NavState::Done,                 "Done"},
                    {NavState::Turn,                 "Turn"},
                    {NavState::Drive,                "Drive"},
                    {NavState::ChangeSearchAlg,      "Change Search Algorithm"},
                    {NavState::SearchTurn,           "Search Turn"},
                    {NavState::SearchDrive,          "Search Drive"},
                    {NavState::TurnToTarget,         "Turn to Target"},
                    {NavState::DriveToTarget,        "Drive to Target"},
                    {NavState::TurnAroundObs,        "Turn Around Obstacle"},
                    {NavState::DriveAroundObs,       "Drive Around Obstacle"},
                    {NavState::SearchTurnAroundObs,  "Search Turn Around Obstacle"},
                    {NavState::SearchDriveAroundObs, "Search Drive Around Obstacle"},
                    {NavState::GatePrepare,          "Gate Prepare"},
                    {NavState::GateMakePath,         "Gate Make Path"},

                    {NavState::Unknown,              "Unknown"}
            };

    return navStateNames.at(mRover->currentState());
} // stringifyNavState()

// Returns the optimal angle to avoid the detected obstacle.
double StateMachine::getOptimalAvoidanceDistance() const {
    return mEnv->getObstacle().distance + mConfig["navThresholds"]["waypointDistance"].GetDouble();
} // optimalAvoidanceAngle()

bool StateMachine::isWaypointReachable(double distance) {
    return isLocationReachable(mRover, mEnv, mConfig, distance, mConfig["navThresholds"]["waypointDistance"].GetDouble());
}

std::shared_ptr<Environment> StateMachine::getEnv() {
    return mEnv;
}

std::shared_ptr<CourseProgress> StateMachine::getCourseState() {
    return mCourseProgress;
}

std::shared_ptr<Rover> StateMachine::getRover() {
    return mRover;
}
