#include "obstacleAvoidanceStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "simpleAvoidance.hpp"
#include <cmath>
#include <iostream>

// Constructs an ObstacleAvoidanceStateMachine object with roverStateMachine
ObstacleAvoidanceStateMachine::ObstacleAvoidanceStateMachine( StateMachine* stateMachine_)
    : roverStateMachine( stateMachine_ ),
      mJustDetectedObstacle( false ) {}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleAngle( double bearing )
{
    mOriginalObstacleAngle = bearing;
}

// Allows outside objects to set the original obstacle distance
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleDistance( double distance )
{
    mOriginalObstacleDistance = distance;
}

// Allows outside objects to set the original obstacle angle
// This will allow the variable to be set before the rover turns
void ObstacleAvoidanceStateMachine::updateObstacleElements( double bearing, double distance )
{
    updateObstacleAngle( bearing );
    updateObstacleDistance( distance );
}

// Runs the avoidance state machine through one iteration. This will be called by StateMachine
// when NavState is in an obstacle avoidance state. This will call the corresponding function based
// on the current state and return the next NavState
NavState ObstacleAvoidanceStateMachine::run( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    switch ( phoebe->roverStatus().currentState() )
    {
        case NavState::TurnAroundObs:
        case NavState::SearchTurnAroundObs:
        {
            return executeTurnAroundObs( phoebe, roverConfig );
        }

        case NavState::DriveAroundObs:
        case NavState::SearchDriveAroundObs:
        {
            return executeDriveAroundObs( phoebe );
        }

        default:
        {
            cerr << "Entered unknown NavState in obstacleAvoidanceStateMachine" << endl;
            return NavState::Unknown;
        }
    } // switch
}

// Checks that both rover is in search state and that tenis ball is detected
bool ObstacleAvoidanceStateMachine::isTennisBallDetected ( Rover* phoebe )
{
    return ( phoebe->roverStatus().currentState() == NavState::SearchTurnAroundObs &&
             phoebe->roverStatus().tennisBall().found );
}

// Checks to see if tennis ball is reachable before hitting obstacle
// Tennis ball must be closer than obstacle and must be in same direction ( in other words,
// the rover should be turning the same way to go around the obstacle as to get to the ball )
bool ObstacleAvoidanceStateMachine::isTennisBallReachable( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    double distanceToBall = phoebe->roverStatus().tennisBall().distance;
    double bearingToBall = phoebe->roverStatus().tennisBall().bearing;
    double distanceToObstacle = phoebe->roverStatus().obstacle().distance;
    double bearingToObstacle = phoebe->roverStatus().obstacle().bearing;
    double tennisBallThreshold = roverConfig[ "navThresholds" ][ "tennisBallDistance" ].GetDouble();
    double bearingThreshold = roverConfig[ "navThresholds" ][ "bearingThreshold" ].GetDouble();


    return distanceToObstacle > distanceToBall - tennisBallThreshold ||
                ( bearingToBall < 0 && bearingToObstacle < 0 ) ||
                ( bearingToBall > 0 && bearingToObstacle > 0 ) ||
                fabs(bearingToObstacle) < fabs(bearingToBall) - bearingThreshold;
}

// The obstacle avoidance factory allows for the creation of obstacle avoidance objects and
// an ease of transition between obstacle avoidance algorithms
ObstacleAvoidanceStateMachine* ObstacleAvoiderFactory ( StateMachine* roverStateMachine,
                                                        ObstacleAvoidanceAlgorithm algorithm )
{
    ObstacleAvoidanceStateMachine* avoid = nullptr;
    switch ( algorithm )
    {
        case ObstacleAvoidanceAlgorithm::SimpleAvoidance:
            avoid = new SimpleAvoidance( roverStateMachine );
            break;

        default:
            std::cerr << "Unkown Search Type. Defaulting to original\n";
            avoid = new SimpleAvoidance( roverStateMachine );
            break;
    } // switch
    return avoid;
} // ObstacleAvoiderFactory

