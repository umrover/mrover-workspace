#include "simpleAvoidance.hpp"

#include "stateMachine.hpp"
#include "utilities.hpp"

#include <cmath>
#include <utility>
#include <iostream>

// Constructs a SimpleAvoidance object with the input mStateMachine, rover, and roverConfig.
// SimpleAvoidance is abstacted from ObstacleAvoidanceStateMachine object so it creates an
// ObstacleAvoidanceStateMachine object with the mStateMachine, rover, and roverConfig.
// The SimpleAvoidance object will execute the logic for the simple avoidance algorithm
SimpleAvoidance::SimpleAvoidance(weak_ptr<StateMachine> roverStateMachine, shared_ptr<Rover> rover, const rapidjson::Document& roverConfig)
        : ObstacleAvoidanceStateMachine(move(roverStateMachine), move(rover), roverConfig) {}

// Destructs the SimpleAvoidance object.
SimpleAvoidance::~SimpleAvoidance() = default;

// Turn away from obstacle until it is no longer detected.
// If in search state and target is both detected and reachable, return NavState TurnToTarget.
// ASSUMPTION: There is no rock that is more than 8 meters (pathWidth * 2) in diameter
NavState SimpleAvoidance::executeTurnAroundObs(shared_ptr<Rover> rover, const rapidjson::Document& roverConfig) {
    shared_ptr<Environment> env = mStateMachine.lock()->getEnvironment();
    if (isTargetDetected() && isTargetReachable(rover, env, roverConfig)) {
        return NavState::TurnToTarget;
    }
    if (!isObstacleDetected(rover, env)) {
        double distanceAroundObs = mOriginalObstacleDistance /
                                   cos(fabs(degreeToRadian(mOriginalObstacleAngle)));
        mObstacleAvoidancePoint = createAvoidancePoint(rover, distanceAroundObs);
        if (rover->roverStatus().currentState() == NavState::TurnAroundObs) {
            return NavState::DriveAroundObs;
        }
        mJustDetectedObstacle = false;
        return NavState::SearchDriveAroundObs;
    }

    Obstacle const& obstacle = env->getObstacle();
    double obstacleBearing = (abs(obstacle.bearing) < abs(obstacle.rightBearing)) ? obstacle.bearing : obstacle.rightBearing;

    if (mJustDetectedObstacle &&
        (obstacleBearing < 0 ? mLastObstacleAngle >= 0 : mLastObstacleAngle < 0)) {
        obstacleBearing *= -1;
    }

    double desiredBearing = mod(rover->roverStatus().odometry().bearing_deg + obstacleBearing, 360);
    mJustDetectedObstacle = true;
    mLastObstacleAngle = obstacleBearing;
    rover->turn(desiredBearing);
    return rover->roverStatus().currentState();
} // executeTurnAroundObs()

// Drives to dummy waypoint. Once arrived, rover will drive to original waypoint
// ( original waypoint is the waypoint before obstacle avoidance was triggered )
NavState SimpleAvoidance::executeDriveAroundObs(shared_ptr<Rover> rover, const rapidjson::Document& roverConfig) {
    shared_ptr<Environment> env = mStateMachine.lock()->getEnvironment();
    if (isObstacleDetected(rover, env) && isObstacleInThreshold(rover, env, roverConfig)) {
        if (rover->roverStatus().currentState() == NavState::DriveAroundObs) {
            return NavState::TurnAroundObs;
        }
        return NavState::SearchTurnAroundObs;
    }

    DriveStatus driveStatus = rover->drive(mObstacleAvoidancePoint);
    if (driveStatus == DriveStatus::Arrived) {
        if (rover->roverStatus().currentState() == NavState::DriveAroundObs) {
            return NavState::Turn;
        }
        return NavState::SearchTurn;
    }
    if (driveStatus == DriveStatus::OnCourse) {
        return rover->roverStatus().currentState();
    }
    if (rover->roverStatus().currentState() == NavState::DriveAroundObs) {
        return NavState::TurnAroundObs;
    }
    return NavState::SearchTurnAroundObs;
} // executeDriveAroundObs()

// Create the odometry point used to drive around an obstacle
Odometry SimpleAvoidance::createAvoidancePoint(shared_ptr<Rover> rover, const double distance) {
    Odometry avoidancePoint = rover->roverStatus().odometry();
    double totalLatitudeMinutes = avoidancePoint.latitude_min +
                                  cos(degreeToRadian(avoidancePoint.bearing_deg)) * distance * LAT_METER_IN_MINUTES;
    double totalLongitudeMinutes = avoidancePoint.longitude_min +
                                   sin(degreeToRadian(avoidancePoint.bearing_deg)) * distance * rover->longMeterInMinutes();
    avoidancePoint.latitude_deg += totalLatitudeMinutes / 60;
    avoidancePoint.latitude_min = (totalLatitudeMinutes - (((int) totalLatitudeMinutes) / 60) * 60);
    avoidancePoint.longitude_deg += totalLongitudeMinutes / 60;
    avoidancePoint.longitude_min = (totalLongitudeMinutes - (((int) totalLongitudeMinutes) / 60) * 60);

    return avoidancePoint;

} // createAvoidancePoint()
