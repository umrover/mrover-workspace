#include "obstacleAvoidanceStateMachine.hpp"

#include <iostream>
#include <utility>

#include "stateMachine.hpp"
#include "simpleAvoidance.hpp"

// Constructs an ObstacleAvoidanceStateMachine object with mStateMachine, mConfig, and mRover
ObstacleAvoidanceStateMachine::ObstacleAvoidanceStateMachine(weak_ptr<StateMachine> stateMachine_, shared_ptr<Rover> rover,
                                                             const rapidjson::Document& roverConfig)
        : mStateMachine(move(stateMachine_)), mJustDetectedObstacle(false), mRover(move(rover)), mRoverConfig(roverConfig) {}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleAngle(double bearing, double rightBearing) {
    mOriginalObstacleAngle = min(bearing, rightBearing);
}

// Allows outside objects to set the original obstacle distance
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleDistance(double distance) {
    mOriginalObstacleDistance = distance;
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleElements(double bearing, double rightBearing, double distance) {
    updateObstacleAngle(bearing, rightBearing);
    updateObstacleDistance(distance);
}

// Runs the avoidance state machine through one iteration. This will be called by StateMachine
// when NavState is in an obstacle avoidance state. This will call the corresponding function based
// on the current state and return the next NavState
NavState ObstacleAvoidanceStateMachine::run() {
    switch (mRover->currentState()) {
        case NavState::TurnAroundObs:
        case NavState::SearchTurnAroundObs: {
            return executeTurnAroundObs(mRover, mRoverConfig);
        }

        case NavState::DriveAroundObs:
        case NavState::SearchDriveAroundObs: {

            return executeDriveAroundObs(mRover, mRoverConfig);

        }

        default: {
            cerr << "Entered unknown NavState in obstacleAvoidanceStateMachine" << endl;
            return NavState::Unknown;
        }
    } // switch
}

// Checks that both rover is in search state and that target is detected
bool ObstacleAvoidanceStateMachine::isTargetDetected() {
    // Second check is to see if we have either a valid target, or if we have a valid
    // cached target to view
    return (mRover->currentState() == NavState::SearchTurnAroundObs &&
            (mRover->leftCacheTarget().distance >= 0));
}

// The obstacle avoidance factory allows for the creation of obstacle avoidance objects and
// an ease of transition between obstacle avoidance algorithms
shared_ptr<ObstacleAvoidanceStateMachine> ObstacleAvoiderFactory(
        weak_ptr<StateMachine> roverStateMachine, ObstacleAvoidanceAlgorithm algorithm, shared_ptr<Rover> rover,
        const rapidjson::Document& roverConfig
) {
    shared_ptr<ObstacleAvoidanceStateMachine> avoid = nullptr;
    switch (algorithm) {
        case ObstacleAvoidanceAlgorithm::SimpleAvoidance:
            avoid = make_shared<SimpleAvoidance>(roverStateMachine, rover, roverConfig);
            break;

        default:
            cerr << "Unkown Search Type. Defaulting to original\n";
            avoid = make_shared<SimpleAvoidance>(roverStateMachine, rover, roverConfig);
            break;
    } // switch
    return avoid;
} // ObstacleAvoiderFactory

