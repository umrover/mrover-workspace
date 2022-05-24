#include "obstacleAvoidanceStateMachine.hpp"

#include <iostream>
#include <utility>

#include "stateMachine.hpp"
#include "simpleAvoidance.hpp"

// Constructs an ObstacleAvoidanceStateMachine object with mStateMachine, mConfig, and mRover
ObstacleAvoidanceStateMachine::ObstacleAvoidanceStateMachine
        (std::weak_ptr<StateMachine> sm, std::shared_ptr<Rover> rover, const rapidjson::Document& roverConfig)
        : mStateMachine(move(sm)), mJustDetectedObstacle(false), mRover(move(rover)), mConfig(roverConfig) {}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleAngle(double bearing, double rightBearing) {
    mOriginalObstacleAngle = std::min(bearing, rightBearing);
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
            return executeTurnAroundObs(mRover, mConfig);
        }

        case NavState::DriveAroundObs:
        case NavState::SearchDriveAroundObs: {

            return executeDriveAroundObs(mRover, mConfig);

        }

        default: {
            std::cerr << "Entered unknown NavState in obstacleAvoidanceStateMachine" << std::endl;
            return NavState::Unknown;
        }
    } // switch
}

// Checks that both rover is in search state and that target is detected
bool ObstacleAvoidanceStateMachine::isTargetDetected() {
    // Second check is to see if we have either a valid target, or if we have a valid
    // cached target to view
    return (mRover->currentState() == NavState::SearchTurnAroundObs &&
            (true)); //doesn't matter since this isn't used anyways will be replaced w obstacle update
}

// The obstacle avoidance factory allows for the creation of obstacle avoidance objects and
// an ease of transition between obstacle avoidance algorithms
std::shared_ptr<ObstacleAvoidanceStateMachine> ObstacleAvoiderFactory(
        std::weak_ptr<StateMachine> roverStateMachine, ObstacleAvoidanceAlgorithm algorithm, std::shared_ptr<Rover> rover,
        const rapidjson::Document& roverConfig
) {
    std::shared_ptr<ObstacleAvoidanceStateMachine> avoid = nullptr;
    switch (algorithm) {
        case ObstacleAvoidanceAlgorithm::SimpleAvoidance:
            avoid = std::make_shared<SimpleAvoidance>(roverStateMachine, rover, roverConfig);
            break;

        default:
            std::cerr << "Unkown Search Type. Defaulting to original\n";
            avoid = std::make_shared<SimpleAvoidance>(roverStateMachine, rover, roverConfig);
            break;
    } // switch
    return avoid;
} // ObstacleAvoiderFactory

