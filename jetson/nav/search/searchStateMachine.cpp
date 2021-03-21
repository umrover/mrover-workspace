#include "searchStateMachine.hpp"

#include "stateMachine.hpp"
#include "utilities.hpp"
#include "spiralOutSearch.hpp"
#include "spiralInSearch.hpp"
#include "lawnMowerSearch.hpp"

#include <iostream>
#include <time.h>
#include <cmath>

// Constructs an SearchStateMachine object with roverStateMachine, mRoverConfig, and mRover
SearchStateMachine::SearchStateMachine(StateMachine* roverStateMachine, Rover* rover, const rapidjson::Document& roverConfig)
    : roverStateMachine( roverStateMachine ) 
    , mRover( rover ) 
    , mRoverConfig( roverConfig ) {}


// Runs the search state machine through one iteration. This will be called by
// StateMachine  when NavState is in a search state. This will call the corresponding
// function based on the current state and return the next NavState
NavState SearchStateMachine::run()
{
    switch ( mRover->roverStatus().currentState() )
    {
        case NavState::SearchSpin:
        {
            return executeSearchSpin();
        }

        case NavState::SearchSpinWait:
        case NavState::TurnedToTargetWait:
        {
            return executeRoverWait();
        }

        case NavState::SearchTurn:
        {
            return executeSearchTurn();
        }

        case NavState::SearchDrive:
        {
            return executeSearchDrive();
        }

        case NavState::TurnToTarget:
        {
            return executeTurnToTarget();
        }

        case NavState::DriveToTarget:
        {
            return executeDriveToTarget();
        }

        default:
        {
            cerr << "Entered Unknown NavState in search state machine" << endl;
            return NavState::Unknown;
        }
    } // switch
} // run()

// Executes the logic for a search spin. If at a multiple of
// waitStepSize, the rover will go to SearchSpinWait. If the rover
// detects the target, it proceeds to the target. If finished with a 360,
// the rover moves on to the next phase of the search. Else continues
// to search spin.
NavState SearchStateMachine::executeSearchSpin()
{
    // degrees to turn to before performing a search wait.
    double waitStepSize = mRoverConfig[ "search" ][ "searchWaitStepSize" ].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; //initialize, is corrected on first call

    if( mRover->roverStatus().target().distance >= 0 )
    {
        updateTargetDetectionElements( mRover->roverStatus().target().bearing,
                                           mRover->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }
    if ( nextStop == 0 )
    {
        //get current angle and set as origAngle
        mOriginalSpinAngle = mRover->roverStatus().odometry().bearing_deg; //doublecheck
        nextStop = mOriginalSpinAngle;
    }
    if( mRover->turn( nextStop ) )
    {
        if( nextStop - mOriginalSpinAngle >= 360 )
        {
            nextStop = 0;
            return NavState::SearchTurn;
        }
        nextStop += waitStepSize;
        return NavState::SearchSpinWait;
    }
    return NavState::SearchSpin;
} // executeSearchSpin()


// Executes the logic for waiting during a search spin so that CV can
// look for the target. If the rover detects the target, it proceeds
// to the target. If the rover is done waiting, it continues the search
// spin. Else the rover keeps waiting.
NavState SearchStateMachine::executeRoverWait()
{
    static bool started = false;
    static time_t startTime;

    if( mRover->roverStatus().target().distance >= 0 )
    {
        updateTargetDetectionElements( mRover->roverStatus().target().bearing,
                                       mRover->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }
    if( !started )
    {
        mRover->stop();
        startTime = time( nullptr );
        started = true;
    }
    double waitTime = mRoverConfig[ "search" ][ "searchWaitTime" ].GetDouble();
    if( difftime( time( nullptr ), startTime ) > waitTime )
    {
        started = false;
        if ( mRover->roverStatus().currentState() == NavState::SearchSpinWait )
        {
            return NavState::SearchSpin;
        }
        return NavState::SearchTurn;
    }
    else
    {
        if ( mRover->roverStatus().currentState() == NavState::SearchSpinWait )
        {
            return NavState::SearchSpinWait;
        }
        return NavState::TurnedToTargetWait;
    }
}

// Executes the logic for turning while searching.
// If no remaining search points, it proceeds to change search algorithms.
// If the rover detects the target, it proceeds to the target.
// If the rover finishes turning, it proceeds to driving while searching.
// Else the rover keeps turning to the next Waypoint.
NavState SearchStateMachine::executeSearchTurn()
{
    if( mSearchPoints.empty() )
    {
        return NavState::ChangeSearchAlg;
    }
    if( mRover->roverStatus().target().distance >= 0 )
    {
        updateTargetDetectionElements( mRover->roverStatus().target().bearing,
                                       mRover->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }
    Odometry& nextSearchPoint = mSearchPoints.front();
    if( mRover->turn( nextSearchPoint ) )
    {
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
NavState SearchStateMachine::executeSearchDrive()
{
    if( mRover->roverStatus().target().distance >= 0 )
    {
        updateTargetDetectionElements( mRover->roverStatus().target().bearing,
                                           mRover->roverStatus().odometry().bearing_deg );
        return NavState::TurnToTarget;
    }

    if( isObstacleDetected( mRover )  && isObstacleInThreshold( mRover, mRoverConfig ) )
    {
        roverStateMachine->updateObstacleAngle( mRover->roverStatus().obstacle().bearing );
        roverStateMachine->updateObstacleDistance( mRover->roverStatus().obstacle().distance );
        return NavState::SearchTurnAroundObs;
    }
    const Odometry& nextSearchPoint = mSearchPoints.front();
    DriveStatus driveStatus = mRover->drive( nextSearchPoint );

    if( driveStatus == DriveStatus::Arrived )
    {
        mSearchPoints.pop_front();
        return NavState::SearchSpin;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::SearchDrive;
    }
    return NavState::SearchTurn;
} // executeSearchDrive()

// Executes the logic for turning to the target.
// If the rover loses the target, will continue to turn using last known angles.
// If the rover finishes turning to the target, it goes into waiting state to
// give CV time to relocate the target
// Else the rover continues to turn to to the target.
NavState SearchStateMachine::executeTurnToTarget()
{
    if( mRover->roverStatus().target().distance < 0 )
    {
        cerr << "Lost the target. Continuing to turn to last known angle\n";
        if( mRover->turn( mTargetAngle + mTurnToTargetRoverAngle ) )
        {
            return NavState::TurnedToTargetWait;
        }
        return NavState::TurnToTarget;
    }
    if( mRover->turn( mRover->roverStatus().target().bearing +
                      mRover->roverStatus().odometry().bearing_deg ) )
    {
        return NavState::DriveToTarget;
    }
    updateTargetDetectionElements( mRover->roverStatus().target().bearing,
                                       mRover->roverStatus().odometry().bearing_deg );
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
NavState SearchStateMachine::executeDriveToTarget()
{
    if( mRover->roverStatus().target().distance < 0 )
    {
        cerr << "Lost the target\n";
        return NavState::SearchTurn; //NavState::SearchSpin
    }

    if( isObstacleDetected( mRover ) &&
        !isTargetReachable( mRover, mRoverConfig )  && isObstacleInThreshold( mRover, mRoverConfig ) )
    {
        roverStateMachine->updateObstacleAngle( mRover->roverStatus().obstacle().bearing );
        roverStateMachine->updateObstacleDistance( mRover->roverStatus().obstacle().distance );
        return NavState::SearchTurnAroundObs;
    }

    DriveStatus driveStatus = mRover->drive( mRover->roverStatus().target().distance,
                                             mRover->roverStatus().target().bearing +
                                             mRover->roverStatus().odometry().bearing_deg,
                                             true );
    if( driveStatus == DriveStatus::Arrived )
    {
        mSearchPoints.clear();
        if( mRover->roverStatus().path().front().gate )
        {
            roverStateMachine->mGateStateMachine->mGateSearchPoints.clear();
            const double absAngle = mod(mRover->roverStatus().odometry().bearing_deg +
                                        mRover->roverStatus().target().bearing,
                                        360);
            roverStateMachine->mGateStateMachine->lastKnownPost1.odom = createOdom( mRover->roverStatus().odometry(),
                                                                                    absAngle,
                                                                                    mRover->roverStatus().target().distance,
                                                                                    mRover );
            roverStateMachine->mGateStateMachine->lastKnownPost1.id = mRover->roverStatus().target().id;
            return NavState::GateSpin;
        }
        mRover->roverStatus().path().pop_front();
        roverStateMachine->updateCompletedPoints();
        return NavState::Turn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::DriveToTarget;
    }
    return NavState::TurnToTarget;
} // executeDriveToTarget()

// Sets last known target angle, so if target is lost, we
// continue to turn to that angle
void SearchStateMachine::updateTargetAngle( double bearing )
{
    mTargetAngle = bearing;
} // updateTargetAngle

// Sets last known rover angle while it is turning to the target
// in case target is lost while turning
void SearchStateMachine::updateTurnToTargetRoverAngle( double bearing )
{
    mTurnToTargetRoverAngle = bearing;
} // updateTurnToTargetRoverAngle

// Sets last known angles for both the target and the rover
// these bearings are used to turn towards the target in case target
// is lost while turning
void SearchStateMachine::updateTargetDetectionElements( double target_bearing, double rover_bearing )
{
    updateTargetAngle( target_bearing );
    updateTurnToTargetRoverAngle( rover_bearing );
} // updateTargetDetectionElements

// add intermediate points between the existing search points in a path generated by a search algorithm.
// The maximum separation between any points in the search point list is determined by the rover's sight distance.
void SearchStateMachine::insertIntermediatePoints()
{
    double visionDistance = mRoverConfig[ "computerVision" ][ "visionDistance" ].GetDouble();
    const double maxDifference = 2 * visionDistance;

    for( int i = 0; i < int( mSearchPoints.size() ) - 1; ++i )
    {
        Odometry point1 = mSearchPoints.at( i );
        Odometry point2 = mSearchPoints.at( i + 1 );
        double distance = estimateNoneuclid( point1, point2 );
        if ( distance > maxDifference )
        {
            int numPoints = int( ceil( distance / maxDifference ) - 1 );
            double newDifference = distance / ( numPoints + 1 );
            double bearing = calcBearing( point1, point2 );
            for ( int j = 0; j < numPoints; ++j )
            {
                Odometry startPoint = mSearchPoints.at( i );
                Odometry newOdom = createOdom( startPoint, bearing, newDifference, mRover );
                auto insertPosition = mSearchPoints.begin() + i + 1;
                mSearchPoints.insert( insertPosition, newOdom );
                ++i;
            }
        }
    }
} // insertIntermediatePoints()

// The search factory allows for the creation of search objects and
// an ease of transition between search algorithms
SearchStateMachine* SearchFactory( StateMachine* stateMachine, SearchType type, Rover* rover, const rapidjson::Document& roverConfig )  //TODO
{
    SearchStateMachine* search = nullptr;
    switch (type)
    {
        case SearchType::SPIRALOUT:
            search = new SpiralOut( stateMachine, rover, roverConfig );
            break;

        case SearchType::LAWNMOWER:
            search = new LawnMower( stateMachine, rover, roverConfig );
            break;

        case SearchType::SPIRALIN:
            search = new SpiralIn( stateMachine, rover, roverConfig );
            break;

        default:
            std::cerr << "Unkown Search Type. Defaulting to Spiral\n";
            search = new SpiralOut( stateMachine, rover, roverConfig );
            break;
    }
    return search;
} // SearchFactory

/******************/
/* TODOS */
/******************/
// save location of target then go around object? ( Execute drive to target )
// look into removing the waiting when turning to target or at least doing this a better way. This should at very least be it's own state
