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
    , mRover( rover ) {}

GateStateMachine::~GateStateMachine() {}

// Execute loop through gate state machine.
NavState GateStateMachine::run()
{
    switch ( mRover->roverStatus().currentState() )
    {
        case NavState::GateSpin:
        {
            return executeGateSpin();
        }

        case NavState::GateSearchGimbal:
        {
            return executeGateSearchGimbal();
        }

        case NavState::GateWait:
        {
            return executeGateWait();
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
            return executeGateTurnToCentPoint();
        }

        case NavState::GateFace:
        {
            return executeGateFace();
        }

        case NavState::GateDriveToCentPoint:
        {
            return executeGateDriveToCentPoint();
        }

        case NavState::GateTurnToFarPost:
        {
            return executeGateTurnToFarPost();
        }

        case NavState::GateDriveToFarPost:
        {
            return executeGateDriveToFarPost();
        }

        case NavState::GateTurnToGateCenter:
        {
            return executeGateTurnToGateCenter();
        }

        case NavState::GateDriveThrough:
        {
            return executeGateDriveThrough();
        }

        default:
        {
            cerr << "Entered Unknown NavState in search state machine" << endl;
            return NavState::Unknown;
        }
    } // switch
} // run

// Perform spin search for a waypoint
NavState GateStateMachine::executeGateSpin()
{
    // degrees to turn to before performing a search wait.
    double waitStepSize = mRoverConfig[ "search" ][ "searchWaitStepSize" ].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double mOriginalSpinAngle = 0; //initialize, is corrected on first call

    if( mRover->roverStatus().rightCacheTarget().distance >= 0 ||
        ( mRover->roverStatus().leftCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id != lastKnownRightPost.id ) )
    {
        mRover->roverStatus().getLeftMisses() = 0; // reset
        mRover->roverStatus().getRightMisses() = 0; // reset
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    if( nextStop == 0 )
    {
        // get current angle and set as origAngle
        mOriginalSpinAngle = mRover->roverStatus().odometry().bearing_deg; //doublecheck
        nextStop = mOriginalSpinAngle;
    }
    if( mRover->turn( nextStop ) )
    {
        if( nextStop - mOriginalSpinAngle >= 360 )
        {
            nextStop = 0;
            return NavState::GateTurn;
        }
        nextStop += waitStepSize;
        return NavState::GateWait;
    }
    return NavState::GateSpin;
} // executeGateSpin()

//Executes the logic for a gimbal gate search. The main objective of a gimbal gate search is to spin the gimbal
//to positive "gimbalSearchAngleMag" (150) then to -150 then to 0. Every "wait step size" we stop the gimbal
//in order to give it time to find the target.
NavState GateStateMachine::executeGateSearchGimbal()
{
    //initially set the waitstepsize to be the same as the gimbalSearchAngleMag so we just go straight to
    //the extremity without waiting.
    static double waitStepSize = mRoverConfig[ "search" ][ "gimbalSearchAngleMag" ].GetDouble();
    static double nextStop = 0; // to force the rover to wait initially
    static double phase = 0; // if 0, go to +150. if 1 go to -150, if 2 go to 0
    static double desired_yaw = mRoverConfig[ "search" ][ "gimbalSearchAngleMag" ].GetDouble(); 

    //if target aquired, go to it
    if( mRover->roverStatus().rightTarget().distance >= 0 ||
        ( mRover->roverStatus().leftTarget().distance >= 0 && mRover->roverStatus().leftTarget().id != lastKnownRightPost.id ) )
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    //set the desired_yaw to wherever the next stop on the gimbals path is
    //enter the if if the gimbal is at the next stop
    if( mRover->gimbal().setDesiredGimbalYaw( nextStop ) )
    {   
        //if the next stop is at the desired_yaw for the phase (150, -150, 0)
        if ( nextStop == desired_yaw )
        {
            //if there are more phases, increment the phase
            if ( phase <= 2 )
                ++phase;

            //if the phase is one, set the waitstepsize to the specified config value and flip desired yaw
            //goal of this phase is to go in waitstepsize increments from positive gimbalSearchAngleMag to 
            //negative gimbalSearchAngleMag
            if ( phase == 1 ) {
                waitStepSize = -mRoverConfig[ "search" ][ "gimbalSearchWaitStepSize" ].GetDouble();
                desired_yaw *= -1;
            }
            //Go straight to zero, set the waitstep size to the difference between 0 and currentPosition
            else if ( phase == 2 ) 
            {
                waitStepSize = 0 - nextStop;
                desired_yaw = 0;
            }
        }

        //if we are done with all phases
        if ( phase == 3 )
        {
            //reset static vars
            waitStepSize = mRoverConfig[ "search" ][ "gimbalSearchAngleMag" ].GetDouble();
            nextStop = 0;
            phase = 0;
            desired_yaw = mRoverConfig[ "search" ][ "gimbalSearchAngleMag" ].GetDouble( );
            //Turn to next search point
            return NavState::GateTurn;
        }
        //set the next stop for the gimbal to increment by the waitStepSize
        nextStop += waitStepSize;
        //we are at our stopping point for the camera so go into search gimbal wait
        return NavState::GateWait;
    }

    mRover->publishGimbal( );

    return NavState::GateSearchGimbal;
}//executeGateSearchGimbal()

// Wait for predetermined time before performing GateSpin
NavState GateStateMachine::executeGateWait()
{
    static bool started = false;
    static time_t startTime;

    if( mRover->roverStatus().rightCacheTarget().distance >= 0 ||
        ( mRover->roverStatus().leftCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id != lastKnownRightPost.id ) )
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
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
        //if using gimbal switch to it.
        if ( mRoverConfig["search"]["useGimbal"].GetBool() )
        {
            return NavState::GateSearchGimbal;
        }
        else
        {
            return NavState::GateSpin;
        }
    }
    return NavState::GateWait;
} // executeGateWait()

// Turn to determined waypoint
NavState GateStateMachine::executeGateTurn()
{
    if( mGateSearchPoints.empty() )
    {
        initializeSearch();
    }

    if( mRover->roverStatus().rightCacheTarget().distance >= 0 ||
        ( mRover->roverStatus().leftCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id != lastKnownRightPost.id ) )
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    Odometry& nextSearchPoint = mGateSearchPoints.front();
    if( mRover->turn( nextSearchPoint ) )
    {
        return NavState::GateDrive;
    }
    return NavState::GateTurn;
} // executeGateTurn()

// Drive to determined waypoint
NavState GateStateMachine::executeGateDrive()
{
    if( mRover->roverStatus().rightCacheTarget().distance >= 0 ||
        ( mRover->roverStatus().leftCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id != lastKnownRightPost.id ) )
    {
        updatePost2Info();
        calcCenterPoint();
        return NavState::GateTurnToCentPoint;
    }

    const Odometry& nextSearchPoint = mGateSearchPoints.front();
    DriveStatus driveStatus = mRover->drive( nextSearchPoint );

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

// Turn to center of the two gate posts
NavState GateStateMachine::executeGateTurnToCentPoint()
{
    if( mRover->turn( centerPoint1 ) )
    {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateTurnToCentPoint()

// Drive to the center point defined by the two posts
NavState GateStateMachine::executeGateDriveToCentPoint()
{
    DriveStatus driveStatus = mRover->drive( centerPoint1 );

    if( driveStatus == DriveStatus::Arrived )
    {
        return NavState::GateFace;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateDriveToCentPoint()

// Turn to the face of the gate posts 
NavState GateStateMachine::executeGateFace()
{
    if( mRover->turn( centerPoint2 ) )
    {
        return NavState::GateTurnToFarPost;
    }
    return NavState::GateFace;
} // executeGateFace()

// Turn to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateTurnToFarPost()
{
    if( mRover->roverStatus().rightCacheTarget().distance > 0 ) 
    {
        if( mRover->roverStatus().leftCacheTarget().distance < mRover->roverStatus().rightCacheTarget().distance ) 
        {
            if( mRover->turn( mRover->roverStatus().rightCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg ) )
            {
                return NavState::GateDriveToFarPost;
            }
        }
        else 
        {
            if( mRover->turn( mRover->roverStatus().leftCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg ) ) 
            {
                return NavState::GateDriveToFarPost;
            }   
        }
    }
    else
    {
        if( mRover->turn( mRover->roverStatus().leftCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg ) ) 
        {
            return NavState::GateDriveToFarPost;
        }
    }
    return NavState::GateTurnToFarPost;
} // executeGateTurnToFarPost()

// Drive to furthest post (or the only post if only one is available)
NavState GateStateMachine::executeGateDriveToFarPost()
{
    // Minor adjustment to gate targeting, due to issue of driving through a 
    // post when driving through the wrong direction
    double gateAdjustmentDist = mRoverConfig[ "gateAdjustment" ][ "adjustmentDistance" ].GetDouble();

    // Set to first target, since we should have atleast one in sight/detected
    double distance = mRover->roverStatus().leftCacheTarget().distance - gateAdjustmentDist;
    double bearing = mRover->roverStatus().leftCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg;

    if( mRover->roverStatus().rightCacheTarget().distance > 0 &&
        mRover->roverStatus().leftCacheTarget().distance < mRover->roverStatus().rightCacheTarget().distance ) 
    {
        // Set our variables to drive to target/post 2, which is farther away
        distance = mRover->roverStatus().rightCacheTarget().distance - gateAdjustmentDist;
        bearing = mRover->roverStatus().rightCacheTarget().bearing + mRover->roverStatus().odometry().bearing_deg;
    }

    DriveStatus driveStatus = mRover->drive( distance, bearing, true );

    if( driveStatus == DriveStatus::Arrived )
    {
        return NavState::GateTurnToGateCenter;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveToFarPost;
    }
    return NavState::GateDriveToFarPost;
} // executeGateDriveToFarPost()

// Execute turn back to center point for driving through the gate
NavState GateStateMachine::executeGateTurnToGateCenter()
{
    if( mRover->turn( centerPoint2 ) ) 
    {
        return NavState::GateDriveThrough;
    }
    return NavState::GateTurnToGateCenter;
} // executeGateTurnToGateCenter()

// Drive through gate posts
NavState GateStateMachine::executeGateDriveThrough()
{
    DriveStatus driveStatus = mRover->drive( centerPoint2 );

    if( driveStatus == DriveStatus::Arrived )
    {
        if( !isCorrectGateDir ) // Check if we drove through the incorrect direction
        {
            const Odometry temp = centerPoint1;
            centerPoint1 = centerPoint2;
            centerPoint2 = temp;
            isCorrectGateDir = true;
            return NavState::GateSpin;
        }
        mRover->roverStatus().path().pop_front();
        mRoverStateMachine->updateCompletedPoints();
        return NavState::Turn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return NavState::GateDriveThrough;
    }
    return NavState::GateDriveThrough;
} // executeGateDriveThrough()

// Update stored location and id for second post.
void GateStateMachine::updatePost2Info()
{
    if( mRover->roverStatus().rightCacheTarget().distance >= 0 && mRover->roverStatus().leftCacheTarget().id == lastKnownRightPost.id )
    {
        const double targetAbsAngle = mod( mRover->roverStatus().odometry().bearing_deg +
                                          mRover->roverStatus().rightCacheTarget().bearing,
                                          360 );
        lastKnownLeftPost.odom = createOdom( mRover->roverStatus().odometry(),
                                          targetAbsAngle,
                                          mRover->roverStatus().rightCacheTarget().distance,
                                          mRover );
        lastKnownLeftPost.id = mRover->roverStatus().rightCacheTarget().id;
    }
    else
    {
        const double targetAbsAngle = mod( mRover->roverStatus().odometry().bearing_deg +
                                          mRover->roverStatus().leftCacheTarget().bearing,
                                          360 );
        lastKnownLeftPost.odom = createOdom( mRover->roverStatus().odometry(),
                                          targetAbsAngle,
                                          mRover->roverStatus().leftCacheTarget().distance,
                                          mRover );
        lastKnownLeftPost.id = mRover->roverStatus().leftCacheTarget().id;
    }
} // updatePost2Info()

// Find the point centered in front of the gate.
// Find the angle that the rover should face from that point to face the gate.
// This point should be on the correct side of the gate so that we drive
// through it in the correct direction.
void GateStateMachine::calcCenterPoint()
{
    const Odometry& currOdom = mRover->roverStatus().odometry();
    const double distFromGate = 3;
    const double gateWidth = mRover->roverStatus().path().front().gate_width;
    const double tagToPointAngle = radianToDegree( atan2( distFromGate, gateWidth / 2 ) );
    const double gateAngle = calcBearing( lastKnownRightPost.odom, lastKnownLeftPost.odom );
    const double absAngle1 = mod( gateAngle + tagToPointAngle, 360 );
    const double absAngle2 = mod( absAngle1 + 180, 360 );
    const double tagToPointDist = sqrt( pow( gateWidth / 2, 2 ) + pow( distFromGate, 2 ) );
    
    // Assuming that CV works well enough that we don't pass through the gate before
    // finding the second post. Thus, centerPoint1 will always be closer.
    centerPoint1 = createOdom( lastKnownRightPost.odom, absAngle1, tagToPointDist, mRover );
    centerPoint2 = createOdom( lastKnownLeftPost.odom, absAngle2, tagToPointDist, mRover );
    const double cp1Dist = estimateNoneuclid( currOdom, centerPoint1 );
    const double cp2Dist = estimateNoneuclid( currOdom, centerPoint2 );
    if( lastKnownRightPost.id % 2 )
    {
        isCorrectGateDir = true;
    }
    else
    {
        isCorrectGateDir = false;
    }
    if( cp1Dist > cp2Dist )
    {
        const Odometry temp = centerPoint1;
        centerPoint1 = centerPoint2;
        centerPoint2 = temp;
        isCorrectGateDir = !isCorrectGateDir;
    }

} // calcCenterPoint()

// Creates an GateStateMachine object
GateStateMachine* GateFactory( StateMachine* stateMachine, Rover* rover, const rapidjson::Document& roverConfig )
{
    return new DiamondGateSearch( stateMachine, rover, roverConfig );
} // GateFactory()