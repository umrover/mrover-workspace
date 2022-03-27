#include "stateMachine.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <map>
#include "rapidjson/istreamwrapper.h"

#include "rover_msgs/NavStatus.hpp"
#include "utilities.hpp"
#include "search/spiralOutSearch.hpp"
#include "search/spiralInSearch.hpp"
#include "search/lawnMowerSearch.hpp"
#include "obstacle_avoidance/simpleAvoidance.hpp"
#include "gate_search/diamondGateSearch.hpp"

// Constructs a StateMachine object with the input lcm object.
// Reads the configuartion file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
StateMachine::StateMachine(std::shared_ptr<Environment> env, std::shared_ptr<CourseProgress> courseState, lcm::LCM& lcmObject) :
        mEnv(move(env)), mCourseProgress(move(courseState)),
        mLcmObject(lcmObject),
        mTotalWaypoints(0), mCompletedWaypoints(0) {
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/nav/config.json";
    configFile.open(configPath);
    if (!configFile) {
        throw runtime_error("Could not open config file at: " + configPath);
    }
    rapidjson::IStreamWrapper isw(configFile);
    mRoverConfig.ParseStream(isw);
    mRover = make_shared<Rover>(mRoverConfig, lcmObject);
    mSearchStateMachine = SearchFactory(weak_from_this(), SearchType::SPIRALOUT, mRover, mRoverConfig);
    mGateStateMachine = GateFactory(weak_from_this(), mRover, mRoverConfig);
    mObstacleAvoidanceStateMachine = ObstacleAvoiderFactory(weak_from_this(), ObstacleAvoidanceAlgorithm::SimpleAvoidance, mRover, mRoverConfig);
} // StateMachine()

// Destructs the StateMachine object. Deallocates memory for the Rover
// object.
StateMachine::~StateMachine() = default;

void StateMachine::setSearcher(SearchType type, shared_ptr<Rover> rover, const rapidjson::Document& roverConfig) {
    assert(mSearchStateMachine);
    mSearchStateMachine = SearchFactory(weak_from_this(), type, rover, roverConfig);
}

void StateMachine::updateCompletedPoints() {
    mCompletedWaypoints += 1;
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleAngle(double bearing, double rightBearing) {
    mObstacleAvoidanceStateMachine->updateObstacleAngle(bearing, rightBearing);
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleDistance(double distance) {
    mObstacleAvoidanceStateMachine->updateObstacleDistance(distance);
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void StateMachine::updateObstacleElements(double bearing, double rightBearing, double distance) {
    updateObstacleAngle(bearing, rightBearing);
    updateObstacleDistance(distance);
}

// Runs the state machine through one iteration. The state machine will
// run if the state has changed or if the rover's status has changed.
// Will call the corresponding function based on the current state.
void StateMachine::run() {
    publishNavState();
    NavState nextState = NavState::Unknown;

    if (!mRover->roverStatus().autonState().is_auton) {
        nextState = NavState::Off;
        mRover->roverStatus().currentState() = executeOff(); // turn off immediately
        mCourseProgress->clearProgress();
        if (nextState != mRover->roverStatus().currentState()) {
            mRover->roverStatus().currentState() = nextState;
            mStateChanged = true;
        }
        return;
    }
    switch (mRover->roverStatus().currentState()) {
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


        case NavState::SearchFaceNorth:
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
            static double visionDistance = mRoverConfig["computerVision"]["visionDistance"].GetDouble();
            setSearcher(SearchType::SPIRALOUT, mRover, mRoverConfig);

            mSearchStateMachine->initializeSearch(mRover, mRoverConfig, visionDistance);
            nextState = NavState::SearchTurn;
            break;
        }

        case NavState::GateSpin:
        case NavState::GateSpinWait:
        case NavState::GateTurn:
        case NavState::GateDrive:
        case NavState::GateTurnToCentPoint:
        case NavState::GateDriveToCentPoint:
        case NavState::GateFace:
        case NavState::GateDriveThrough:
        case NavState::GateTurnToFarPost:
        case NavState::GateDriveToFarPost:
        case NavState::GateTurnToGateCenter: {
            nextState = mGateStateMachine->run();
            break;
        }

        case NavState::Unknown: {
            cerr << "Entered unknown state.\n";
            exit(1);
        }
    } // switch

    if (nextState != mRover->roverStatus().currentState()) {
        mStateChanged = true;
        mRover->roverStatus().currentState() = nextState;
        mRover->distancePid().reset();
        mRover->bearingPid().reset();
    }
    cerr << flush;
} // run()

// Updates the auton state (on/off) of the rover's status.
void StateMachine::updateRoverStatus(AutonState autonState) {
    mNewRoverStatus.autonState() = autonState;
} // updateRoverStatus( AutonState )

// Updates the odometry information of the rover's status.
void StateMachine::updateRoverStatus(Odometry odometry) {
    mNewRoverStatus.odometry() = odometry;
} // updateRoverStatus( Odometry )


// Publishes the current navigation state to the nav status lcm channel.
void StateMachine::publishNavState() const {
    NavStatus navStatus;
    navStatus.nav_state_name = stringifyNavState();
    navStatus.completed_wps = mCompletedWaypoints;
    navStatus.total_wps = mTotalWaypoints;
    const string& navStatusChannel = mRoverConfig["lcmChannels"]["navStatusChannel"].GetString();
    mLcmObject.publish(navStatusChannel, &navStatus);
} // publishNavState()

// Executes the logic for off. If the rover is turned on, it updates
// the roverStatus. If the course is empty, the rover is done  with
// the course otherwise it will turn to the first waypoing. Else the
// rover is still off.
NavState StateMachine::executeOff() {
    if (mRover->roverStatus().autonState().is_auton) {
        mCompletedWaypoints = 0;
        mTotalWaypoints = mCourseProgress->getRemainingWaypoints().size();

        if (!mTotalWaypoints) {
            return NavState::Done;
        }
        return NavState::Turn;
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
    double distance = estimateNoneuclid(mRover->roverStatus().odometry(), nextWaypoint.odom);

    if (isObstacleDetected(mRover, getEnv())
        && !isWaypointReachable(distance)
        && isObstacleInThreshold(mRover, getEnv(), mRoverConfig)) {
        mObstacleAvoidanceStateMachine->updateObstacleElements(mEnv->getObstacle().bearing,
                                                               mEnv->getObstacle().rightBearing,
                                                               getOptimalAvoidanceDistance());
        return NavState::TurnAroundObs;
    }
    DriveStatus driveStatus = mRover->drive(nextWaypoint.odom);
    if (driveStatus == DriveStatus::Arrived) {
        if (nextWaypoint.search || nextWaypoint.gate) {
            return NavState::ChangeSearchAlg;
        }
        mCourseProgress->completeCurrentWaypoint();

        ++mCompletedWaypoints;
        return NavState::Turn;
    }
    if (driveStatus == DriveStatus::OnCourse) {

        return NavState::Drive;
    }

    return NavState::Turn;
} // executeDrive()

// Gets the string representation of a nav state.
string StateMachine::stringifyNavState() const {
    static const map<NavState, std::string> navStateNames =
            {
                    {NavState::Off,                  "Off"},
                    {NavState::Done,                 "Done"},
                    {NavState::Turn,                 "Turn"},
                    {NavState::Drive,                "Drive"},
                    {NavState::SearchFaceNorth,      "Search Face North"},
                    {NavState::ChangeSearchAlg,      "Change Search Algorithm"},
                    {NavState::SearchTurn,           "Search Turn"},
                    {NavState::SearchDrive,          "Search Drive"},
                    {NavState::TurnToTarget,         "Turn to Target"},
                    {NavState::DriveToTarget,        "Drive to Target"},
                    {NavState::TurnAroundObs,        "Turn Around Obstacle"},
                    {NavState::DriveAroundObs,       "Drive Around Obstacle"},
                    {NavState::SearchTurnAroundObs,  "Search Turn Around Obstacle"},
                    {NavState::SearchDriveAroundObs, "Search Drive Around Obstacle"},
                    {NavState::GateSpin,             "Gate Spin"},
                    {NavState::GateSpinWait,         "Gate Spin Wait"},
                    {NavState::GateTurn,             "Gate Turn"},
                    {NavState::GateDrive,            "Gate Drive"},
                    {NavState::GateTurnToCentPoint,  "Gate Turn to Center Point"},
                    {NavState::GateDriveToCentPoint, "Gate Drive to Center Point"},
                    {NavState::GateFace,             "Gate Face"},
                    {NavState::GateTurnToFarPost,    "Gate Turn to Far Post"},
                    {NavState::GateDriveToFarPost,   "Gate Drive to Far Post"},
                    {NavState::GateTurnToGateCenter, "Gate Turn to Gate Center"},
                    {NavState::GateDriveThrough,     "Gate Drive Through"},

                    {NavState::Unknown,              "Unknown"}
            };

    return navStateNames.at(mRover->roverStatus().currentState());
} // stringifyNavState()

// Returns the optimal angle to avoid the detected obstacle.
double StateMachine::getOptimalAvoidanceDistance() const {
    return mEnv->getObstacle().distance + mRoverConfig["navThresholds"]["waypointDistance"].GetDouble();
} // optimalAvoidanceAngle()

bool StateMachine::isWaypointReachable(double distance) {
    return isLocationReachable(mRover, mEnv, mRoverConfig, distance, mRoverConfig["navThresholds"]["waypointDistance"].GetDouble());
}

std::shared_ptr<Environment> StateMachine::getEnv() {
    return mEnv;
}

std::shared_ptr<CourseProgress> StateMachine::getCourseState() {
    return mCourseProgress;
}

// isWaypointReachable

// TODOS:
// [drive to target] obstacle and target
// all of code, what to do in cases of both target and obstacle
// thresholds based on state? waypoint vs target
