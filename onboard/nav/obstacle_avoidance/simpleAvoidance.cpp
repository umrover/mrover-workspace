#include "simpleAvoidance.hpp"

#include "stateMachine.hpp"
#include "utilities.hpp"

#include <iostream>
#include <cmath>

// Constructs a SimpleAvoidance object with the input roverStateMachine.
// SimpleAvoidance is abstacted from ObstacleAvoidanceStateMachine object so it creates an
// ObstacleAvoidanceStateMachine object with the roverStateMachine. The SimpleAvoidance object will
// execute the logic for the simple avoidance algorithm
SimpleAvoidance::SimpleAvoidance( StateMachine* roverStateMachine )
    : ObstacleAvoidanceStateMachine( roverStateMachine ) {}

// Destructs the SimpleAvoidance object.
SimpleAvoidance::~SimpleAvoidance() {}

// Turn away from obstacle until it is no longer detected.
// If in search state and target is both detected and reachable, return NavState TurnToTarget.
// ASSUMPTION: There is no rock that is more than 8 meters (pathWidth * 2) in diameter
NavState SimpleAvoidance::executeTurnAroundObs( Rover* phoebe,
                                                const rapidjson::Document& roverConfig )
{
    if( isTargetDetected ( phoebe ) && isTargetReachable( phoebe, roverConfig ) )
    {
        return NavState::TurnToTarget;
    }
    if( !phoebe->roverStatus().obstacle().detected )
    {
        double distanceAroundObs = mOriginalObstacleDistance /
                                   cos( fabs( degreeToRadian( mOriginalObstacleAngle ) ) );
        mObstacleAvoidancePoint = createAvoidancePoint( phoebe, distanceAroundObs );
        if( phoebe->roverStatus().currentState() == NavState::TurnAroundObs )
        {
            return NavState::DriveAroundObs;
        }
        mJustDetectedObstacle = false;
        return NavState::SearchDriveAroundObs;
    }

    double obstacleBearing = phoebe->roverStatus().obstacle().bearing;
    if( mJustDetectedObstacle &&
        ( obstacleBearing < 0 ? mLastObstacleAngle >= 0 : mLastObstacleAngle < 0 ) ) {
        obstacleBearing *= -1;
    }

    double desiredBearing = mod( phoebe->roverStatus().odometry().bearing_deg + obstacleBearing, 360 );
    mJustDetectedObstacle = true;
    mLastObstacleAngle = obstacleBearing;
    phoebe->turn( desiredBearing );
    return phoebe->roverStatus().currentState();
} // executeTurnAroundObs()

// Drives to dummy waypoint. Once arrived, rover will drive to original waypoint
// ( original waypoint is the waypoint before obstacle avoidance was triggered )
NavState SimpleAvoidance::executeDriveAroundObs( Rover* phoebe )
{
    if( phoebe->roverStatus().obstacle().detected )
    {
        if( phoebe->roverStatus().currentState() == NavState::DriveAroundObs )
        {
            return NavState::TurnAroundObs;
        }
        return NavState::SearchTurnAroundObs;
    }

    DriveStatus driveStatus = phoebe->drive( mObstacleAvoidancePoint );
    if( driveStatus == DriveStatus::Arrived )
    {
        if( phoebe->roverStatus().currentState() == NavState::DriveAroundObs )
        {
            return NavState::Turn;
        }
        return NavState::SearchTurn;
    }
    if( driveStatus == DriveStatus::OnCourse )
    {
        return phoebe->roverStatus().currentState();
    }
    if( phoebe->roverStatus().currentState() == NavState::DriveAroundObs )
    {
        return NavState::TurnAroundObs;
    }
    return NavState::SearchTurnAroundObs;
} // executeDriveAroundObs()

// Create the odometry point used to drive around an obstacle
Odometry SimpleAvoidance::createAvoidancePoint( Rover* phoebe, const double distance )
{
    Odometry avoidancePoint = phoebe->roverStatus().odometry();
    double totalLatitudeMinutes = avoidancePoint.latitude_min +
        cos( degreeToRadian( avoidancePoint.bearing_deg ) ) * distance * LAT_METER_IN_MINUTES;
    double totalLongitudeMinutes = avoidancePoint.longitude_min +
        sin( degreeToRadian( avoidancePoint.bearing_deg ) ) * distance * phoebe->longMeterInMinutes();
    avoidancePoint.latitude_deg += totalLatitudeMinutes / 60;
    avoidancePoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes) / 60 ) * 60 );
    avoidancePoint.longitude_deg += totalLongitudeMinutes / 60;
    avoidancePoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

    return avoidancePoint;


} // createAvoidancePoint()
