#include "gateStateMachine.hpp"

#include "utilities.hpp"
#include "stateMachine.hpp"
#include "./gate_search/diamondGateSearch.hpp"
#include <cmath>
#include <iostream>

// Constructs a GateStateMachine object with roverStateMachine
GateStateMachine::GateStateMachine( StateMachine* stateMachine, Rover* rover, const rapidjson::Document& roverConfig )
    : mRoverStateMachine( stateMachine )
    , mRoverConfig( roverConfig )
    , mPhoebe( rover ) {}

GateStateMachine::~GateStateMachine() {}

// Execute loop through gate state machine.
NavState GateStateMachine::run()
{
    switch ( mPhoebe->roverStatus().currentState() )
    {
        case NavState::GateSpin:
        {
            return executeGateSpin();
        }

        case NavState::GateSpinWait:
        {
            return executeGateSpinWait();
        }

        case NavState::GateTurn:
        {
            return executeGateTurn();
        }

        case NavState::GateDrive:
        {
            return executeGateDrive();
        }

        case NavState::GateTurnToCentPoint:
        {
            cout << "turn to cent\n";
            return NavState::GateTurnToCentPoint;
        }

        default:
        {
            cerr << "Entered Unknown NavState in search state machine" << endl;
            return NavState::Unknown;
        }
    } // switch
} // run

//
NavState GateStateMachine::executeGateSpin()
{
    // degrees to turn to before performing a search wait.
    double waitStepSize = mRoverConfig[ "search" ][ "searchWaitStepSize" ].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; //initialize, is corrected on first call

    if( mPhoebe->roverStatus().target2().distance >= 0 ||
        ( mPhoebe->roverStatus().target().distance >= 0 && mPhoebe->roverStatus().target().id != lastKnownPost1.id ))
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if ( nextStop == 0 )
    {
        // get current angle and set as origAngle
        mOriginalSpinAngle = mPhoebe->roverStatus().odometry().bearing_deg; //doublecheck
        nextStop = mOriginalSpinAngle;
    }
    if( mPhoebe->turn( nextStop ) )
    {
        if( nextStop - mOriginalSpinAngle >= 360 )
        {
            nextStop = 0;
            return NavState::GateTurn;
        }
        nextStop += waitStepSize;
        return NavState::GateSpinWait;
    }
    return NavState::GateSpin;
} // executeGateSpin()

//
NavState GateStateMachine::executeGateSpinWait()
{
    static bool started = false;
    static time_t startTime;

    if( mPhoebe->roverStatus().target2().distance >= 0 ||
        ( mPhoebe->roverStatus().target().distance >= 0 && mPhoebe->roverStatus().target().id != lastKnownPost1.id ))
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if( !started )
    {
        mPhoebe->stop();
        startTime = time( nullptr );
        started = true;
    }
    double waitTime = mRoverConfig[ "search" ][ "searchWaitTime" ].GetDouble();
    if( difftime( time( nullptr ), startTime ) > waitTime )
    {
        started = false;
        return NavState::GateSpin;
    }
    return NavState::GateSpinWait;
} // executeGateSpinWait()

//
NavState GateStateMachine::executeGateTurn()
{
    if( mGateSearchPoints.empty() )
    {
        initializeSearch();
    }

    if( mPhoebe->roverStatus().target2().distance >= 0 ||
        ( mPhoebe->roverStatus().target().distance >= 0 && mPhoebe->roverStatus().target().id != lastKnownPost1.id ))
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    Odometry& nextSearchPoint = mGateSearchPoints.front();
    if( mPhoebe->turn( nextSearchPoint ) )
    {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateTurn()

//
NavState GateStateMachine::executeGateDrive()
{
    if( mPhoebe->roverStatus().target2().distance >= 0 ||
        ( mPhoebe->roverStatus().target().distance >= 0 && mPhoebe->roverStatus().target().id != lastKnownPost1.id ))
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    // TODO
    // if( isObstacleDetected( phoebe ) )
    // {
    //     roverStateMachine->updateObstacleAngle( phoebe->roverStatus().obstacle().bearing );
    //     roverStateMachine->updateObstacleDistance( phoebe->roverStatus().obstacle().distance );
    //     return NavState::SearchTurnAroundObs;
    // }
    const Odometry& nextSearchPoint = mGateSearchPoints.front();
    DriveStatus driveStatus = mPhoebe->drive( nextSearchPoint );

    if( driveStatus == DriveStatus::Arrived )
    {
        mGateSearchPoints.pop_front();
        return NavState::GateSpin;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateDrive()

// Update stored location and id for second post.
void GateStateMachine::updatePost2Info()
{
    // todo save second post
    if( mPhoebe->roverStatus().target2().distance >= 0 )
    {
        if( mPhoebe->roverStatus().target().id == lastKnownPost1.id )
        {
            lastKnownPost2.id = mPhoebe->roverStatus().target2().id;
            lastKnownPost2.odom = createOdom( mPhoebe->roverStatus().odometry(),
                                              mPhoebe->roverStatus().target2().bearing,
                                              mPhoebe->roverStatus().target2().distance,
                                              mPhoebe );
        }
        else
        {
            lastKnownPost2.id = mPhoebe->roverStatus().target().id;
            lastKnownPost2.odom = createOdom( mPhoebe->roverStatus().odometry(),
                                              mPhoebe->roverStatus().target().bearing,
                                              mPhoebe->roverStatus().target().distance,
                                              mPhoebe );
        }
    }
    else
    {
        lastKnownPost2.id = mPhoebe->roverStatus().target().id;
        lastKnownPost2.odom = createOdom( mPhoebe->roverStatus().odometry(),
                                          mPhoebe->roverStatus().target().bearing,
                                          mPhoebe->roverStatus().target().distance,
                                          mPhoebe );
    }
} // updatePost2Info()

// Find the point centered in front of the gate.
// Find the angle that the rover should face from that point to face the gate.
// This point should be on the correct side of the gate so that we drive
// through it in the correct direction.
void GateStateMachine::calcCenterPoint()
{

} // calcCenterPoint()

// Creates an GateStateMachine object
GateStateMachine* GateFactory( StateMachine* stateMachine, Rover* phoebe, const rapidjson::Document& roverConfig )
{
    return new DiamondGateSearch( stateMachine, phoebe, roverConfig );
} // GateFactor()
