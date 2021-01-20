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
            return executeGateTurnToCentPoint();
        }

        case NavState::GateDriveToCentPoint:
        {
            return executeGateDriveToCentPoint();
        }

        case NavState::GateFace:
        {
            return executeGateFace();
        }

        case NavState::GateShimmy:
        {
            return executeGateShimmy();
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

NavState GateStateMachine::executeGateTurnToCentPoint()
{
    if( mPhoebe->turn( centerPoint1 ) )
    {
        return NavState::GateDriveToCentPoint;
    }
    return NavState::GateTurnToCentPoint;
} // executeGateTurnToCentPoint()

NavState GateStateMachine::executeGateDriveToCentPoint()
{
    // TODO: Obstacle Avoidance?
    DriveStatus driveStatus = mPhoebe->drive( centerPoint1 );

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

NavState GateStateMachine::executeGateFace()
{
    if( mPhoebe->turn( centerPoint2 ) )
    {
        return NavState::GateShimmy;
    }
    return NavState::GateFace;
} // executeGateFace()

NavState GateStateMachine::executeGateShimmy()
{
    static int direction = 1; // 1 = forward, -1 = backwards
    const double fovDepth = mRoverConfig["computerVision"]["visionDistance"].GetDouble();
    const double fovAngle = mRoverConfig["computerVision"]["fieldOfViewSafeAngle"].GetDouble();
    const Odometry currOdom = mPhoebe->roverStatus().odometry();

    // If we are centered
    const double targetAnglesDiff = mPhoebe->roverStatus().target().bearing +
                                    mPhoebe->roverStatus().target2().bearing;
    if(targetAnglesDiff < mRoverConfig["navThresholds"]["gateCenteredAngleDiff"].GetDouble())
    {
        direction = 1;
        return NavState::GateDriveThrough;
    }

    // If we need to switch directions
    const bool visibleTargetAngles = mPhoebe->roverStatus().target().bearing > fovAngle / 2 &&
                                     mPhoebe->roverStatus().target2().bearing < fovAngle / 2;
    const bool visibleTargetDists = mPhoebe->roverStatus().target().distance < fovDepth &&
                                    mPhoebe->roverStatus().target2().distance < fovDepth;
    if(!visibleTargetAngles || !visibleTargetDists)
    {
        mPhoebe->stop();
        direction = direction == 1 ? -1 : 1;
        return NavState::GateFace;
    }

    // Otherwise keep driving
    const double gateWidth = mPhoebe->roverStatus().path().front().gate_width;
    const double gateAngle = calcBearing(lastKnownPost1.odom, lastKnownPost2.odom); // Angle from post 1 to post 2
    const Odometry gateCent = createOdom(lastKnownPost1.odom, gateAngle, gateWidth / 2, mPhoebe);
    const double roverToGateCentAngle = calcBearing(currOdom, gateCent); // ablsolute angle
    mPhoebe->drive(direction, roverToGateCentAngle); // TODO: drive straight when going backwards
    return NavState::GateShimmy;
} // executeGateShimmy()

NavState GateStateMachine::executeGateDriveThrough()
{
    // TODO: Obstacle Avoidance?
    DriveStatus driveStatus = mPhoebe->drive( centerPoint2 );

    if( driveStatus == DriveStatus::Arrived )
    {
        if(!CP1ToCP2CorrectDir)
        {
            const Odometry temp = centerPoint1;
            centerPoint1 = centerPoint2;
            centerPoint2 = temp;
            CP1ToCP2CorrectDir = true;
            return NavState::GateFace;
        }
        mPhoebe->roverStatus().path().pop_front();
        mRoverStateMachine->updateCompletedPoints();
        return NavState::Turn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        // TODO
        return NavState::GateDriveThrough;
    }
    return NavState::GateDriveThrough;
} // executeGateDriveThrough()

// Update stored location and id for second post.
void GateStateMachine::updatePost2Info()
{
    if(mPhoebe->roverStatus().target2().distance >= 0 && mPhoebe->roverStatus().target().id == lastKnownPost1.id)
    {
        const double targetAbsAngle = mod(mPhoebe->roverStatus().odometry().bearing_deg +
                                          mPhoebe->roverStatus().target2().bearing,
                                          360);
        lastKnownPost2.odom = createOdom( mPhoebe->roverStatus().odometry(),
                                          targetAbsAngle,
                                          mPhoebe->roverStatus().target2().distance,
                                          mPhoebe );
        lastKnownPost2.id = mPhoebe->roverStatus().target2().id;
    }
    else
    {
        const double targetAbsAngle = mod(mPhoebe->roverStatus().odometry().bearing_deg +
                                          mPhoebe->roverStatus().target().bearing,
                                          360);
        lastKnownPost2.odom = createOdom( mPhoebe->roverStatus().odometry(),
                                          targetAbsAngle,
                                          mPhoebe->roverStatus().target().distance,
                                          mPhoebe );
        lastKnownPost2.id = mPhoebe->roverStatus().target().id;
    }
} // updatePost2Info()

// Find the point centered in front of the gate.
// Find the angle that the rover should face from that point to face the gate.
// This point should be on the correct side of the gate so that we drive
// through it in the correct direction.
void GateStateMachine::calcCenterPoint()
{
    const Odometry& currOdom = mPhoebe->roverStatus().odometry();
    const double distFromGate = 3;
    const double gateWidth = mPhoebe->roverStatus().path().front().gate_width;
    const double tagToPointAngle = radianToDegree(atan2(distFromGate, gateWidth / 2));
    const double gateAngle = calcBearing(lastKnownPost1.odom, lastKnownPost2.odom);
    const double absAngle1 = mod(gateAngle + tagToPointAngle, 360);
    const double absAngle2 = mod(absAngle1 + 180, 360);
    const double tagToPointDist = sqrt(pow(gateWidth / 2, 2) + pow(distFromGate, 2));
    // Assuming that CV works well enough that we don't pass through the gate before
    // finding the second post. Thus, centerPoint1 will always be closer.
    // TODO: verify this
    centerPoint1 = createOdom(lastKnownPost1.odom, absAngle1, tagToPointDist, mPhoebe);
    centerPoint2 = createOdom(lastKnownPost2.odom, absAngle2, tagToPointDist, mPhoebe);
    const double cp1Dist = estimateNoneuclid(currOdom, centerPoint1);
    const double cp2Dist = estimateNoneuclid(currOdom, centerPoint2);
    if(lastKnownPost1.id % 2)
    {
        CP1ToCP2CorrectDir = true;
    }
    else
    {
        CP1ToCP2CorrectDir = false;
    }
    if(cp1Dist > cp2Dist)
    {
        const Odometry temp = centerPoint1;
        centerPoint1 = centerPoint2;
        centerPoint2 = temp;
        CP1ToCP2CorrectDir = !CP1ToCP2CorrectDir;
    }

} // calcCenterPoint()

// Creates an GateStateMachine object
GateStateMachine* GateFactory( StateMachine* stateMachine, Rover* phoebe, const rapidjson::Document& roverConfig )
{
    return new DiamondGateSearch( stateMachine, phoebe, roverConfig );
} // GateFactor()
