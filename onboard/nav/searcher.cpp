#include "searcher.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>
#include <time.h>

/*****************************************************/
/* Funtions for StateMachine to access Search points */
/*****************************************************/
Odometry Searcher::frontSearchPoint( )
{
    return mSearchPoints.front();
}

void Searcher::popSearchPoint( )
{
    mSearchPoints.pop_front();
    return;
}

/*****************************************************/
/* Searcher Run Fuction */
/*****************************************************/
NavState Searcher::run( Rover * phoebe, const rapidjson::Document& roverConfig )
{
    switch ( phoebe->roverStatus().currentState() )
    {
        case NavState::SearchFaceNorth:
        {
            return executeSearchFaceNorth( phoebe );
        }

        case NavState::SearchSpin:
        {
            return executeSearchSpin( phoebe, roverConfig );
        }

        case NavState::SearchSpinWait:
        {
            return executeSearchSpinWait( phoebe, roverConfig );
        }

        case NavState::SearchTurn:
        {
            return executeSearchTurn( phoebe, roverConfig );
        }

        case NavState::SearchDrive:
        {
            return executeSearchDrive( phoebe );
        }

        case NavState::TurnToBall:
        {
            return executeTurnToBall( phoebe );
        }

        case NavState::DriveToBall:
        {
            return executeDriveToBall( phoebe );
        }

        default:
        {
            return NavState::Unknown;
        }
    } // switch
}

/*****************************************************/
/* Helpers */
/*****************************************************/

// Executes the logic for turning to face north to orient itself for a search.
// If the rover detects the tennis ball, it proceeds to the ball.
// If the rover finishes turning, it proceeds to SearchSpin.
// Else the rover keeps turning to north.
NavState Searcher::executeSearchFaceNorth( Rover * phoebe )
{
    if( phoebe->roverStatus().tennisBall().found )
    {
        mSearchPoints.push_front( phoebe->roverStatus().odometry() );
        return NavState::TurnToBall;
    }
    if( phoebe->turn( 0 ) )
    {
        return NavState::SearchSpin;
    }
    return NavState::SearchFaceNorth;
} // executeSearchFaceNorth

// Executes the logic for a search spin. If at a multiple of
// waitStepSize, the rover will go to SearchSpinWait. If the rover
// detects the ball, it proceeds to the ball. If finished with a 360,
// the rover moves on to the next phase of the search. Else continues
// to search spin.
NavState Searcher::executeSearchSpin( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    // degrees to turn to before performing a search wait.
    double waitStepSize = roverConfig[ "computerVision" ][ "searchWaitStepSize" ].GetDouble();
    static int nextStop = 0; // to force the rover to wait initially
    if( phoebe->roverStatus().tennisBall().found )
    {
        mSearchPoints.push_front( phoebe->roverStatus().odometry() );
        return NavState::TurnToBall;
    }
    if( phoebe->turn( nextStop ) )
    {
        if( nextStop >= 360 )
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
// look for the tennis ball. If the rover detects the ball, it proceeds
// to the ball. If the rover is done waiting, it continues the search
// spin. Else the rover keeps waiting.
NavState Searcher::executeSearchSpinWait( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    static bool started = false;
    static time_t startTime;
    if( phoebe->roverStatus().tennisBall().found )
    {
        mSearchPoints.push_front( phoebe->roverStatus().odometry() );
        return NavState::TurnToBall;
    }
    if( !started )
    {
        startTime = time( nullptr );
        started = true;
    }
    double waitTime = roverConfig[ "computerVision" ][ "searchWaitTime" ].GetDouble();
    if( difftime( time( nullptr ), startTime ) > waitTime )
    {
        started = false;
        return NavState::SearchSpin;
    }
    else
    {
        return NavState::SearchSpinWait;
    }
}

// Executes the logic for turning while searching.
// If no remaining search points, it proceeds to change search algorithms.
// If the rover detects the tennis ball, it proceeds to the ball.
// If the rover finishes turning, it proceeds to driving while searching.
// Else the rover keeps turning to the next Waypoint.
NavState Searcher::executeSearchTurn( Rover* phoebe, const rapidjson::Document& roverConfig )
{
    if( mSearchPoints.empty() )
    {
        return NavState::ChangeSearchAlg;
    }
    if( phoebe->roverStatus().tennisBall().found )
    {
        mSearchPoints.push_front( phoebe->roverStatus().odometry() );
        return NavState::TurnToBall;
    }
    Odometry& nextSearchPoint = mSearchPoints.front();
    if( phoebe->turn( nextSearchPoint ) )
    {
        return NavState::SearchDrive;
    }
    return NavState::SearchTurn;
} // executeSearchTurn()

// Executes the logic for driving while searching.
// If the rover detects the tennis ball, it proceeds to the ball.
// If the rover detects an obstacle, it proceeds to obstacle avoidance.
// If the rover finishes driving, it proceeds to turning to the next Waypoint.
// If the rover is still on course, it keeps driving to the next Waypoint.
// Else the rover turns to the next Waypoint or turns back to the current Waypoint
NavState Searcher::executeSearchDrive( Rover * phoebe )
{
    if( phoebe->roverStatus().tennisBall().found )
    {
        mSearchPoints.push_front( phoebe->roverStatus().odometry() );
        return NavState::TurnToBall;
    }
    if( phoebe->roverStatus().obstacle().detected )
    {
        stateMachine->updateObstacleAngle( phoebe->roverStatus().obstacle().bearing );
        return NavState::SearchTurnAroundObs;
    }
    const Odometry& nextSearchPoint = mSearchPoints.front();
    DriveStatus driveStatus = phoebe->drive( nextSearchPoint );

    if( driveStatus == DriveStatus::Arrived )
    {
        mSearchPoints.pop_front();
        return NavState::SearchTurn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::SearchDrive;
    }
    return NavState::SearchTurn;
} // executeSearchDrive()

// Executes the logic for turning to the tennis ball.
// If the rover loses the ball, it starts to search again.
// If the rover finishes turning to the ball, it drives to the ball.
// Else the rover continues to turn to to the ball.
NavState Searcher::executeTurnToBall( Rover * phoebe )
{
    if( !phoebe->roverStatus().tennisBall().found )
    {
        cerr << "Lost the tennis ball\n";
        return NavState::SearchFaceNorth;
    }
    if( phoebe->turn( phoebe->roverStatus().tennisBall().bearing +
                       phoebe->roverStatus().odometry().bearing_deg ) )
    {
        return NavState::DriveToBall;
    }
    return NavState::TurnToBall;
} // executeTurnToBall()

// Executes the logic for driving to the tennis ball.
// If the rover loses the ball, it starts the search again.
// If the rover detects an obstacle, it proceeds to go around the obstacle.
// If the rover finishes driving to the ball, it moves on to the next Waypoint.
// If the rover is on course, it keeps driving to the ball.
// Else, it turns back to face the ball.
NavState Searcher::executeDriveToBall( Rover * phoebe )
{
    if( !phoebe->roverStatus().tennisBall().found )
    {
        cerr << "Lost the tennis ball\n";
        return NavState::SearchFaceNorth;
    }
    if( phoebe->roverStatus().obstacle().detected )
    {
        stateMachine->updateObstacleAngle( phoebe->roverStatus().obstacle().bearing );
        return NavState::SearchTurnAroundObs;
    }

    DriveStatus driveStatus = phoebe->drive( phoebe->roverStatus().tennisBall().distance,
                                              phoebe->roverStatus().tennisBall().bearing +
                                              phoebe->roverStatus().odometry().bearing_deg );
    if( driveStatus == DriveStatus::Arrived )
    {
        phoebe->roverStatus().path().pop();
        stateMachine->updateCompletedPoints();
        return NavState::Turn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::DriveToBall;
    }
    return NavState::TurnToBall;
} // executeDriveToBall()

/*************************************************************************/
/* TODOS */
/*************************************************************************/
// TODO: save location of ball then go around object? ( Execute drive to ball )
