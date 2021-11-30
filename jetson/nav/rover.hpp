#ifndef ROVER_HPP
#define ROVER_HPP

#include <lcm/lcm-cpp.hpp>
#include <queue>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/Bearing.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/RepeaterDrop.hpp"
#include "rover_msgs/RadioSignalStrength.hpp"
#include "rover_msgs/TargetList.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "rapidjson/document.h"
#include "pid.hpp"
#include "gimbal.hpp"

using namespace rover_msgs;
using namespace std;

// This class is the representation of the navigation states.
enum class NavState
{
    // Base States
    Off = 0,
    Done = 1,

    // Simple Movement
    Turn = 10,
    Drive = 11,

    // Search States
    SearchFaceNorth = 20,
    SearchSpin = 21,
    SearchGimbal = 22,
    SearchWait = 24,
    SearchTurn = 25,
    SearchDrive = 26,
    ChangeSearchAlg = 27,

    // Target Found States
    TurnToTarget = 28,
    TurnedToTargetWait = 29,
    DriveToTarget = 30,

    // Obstacle Avoidance States
    TurnAroundObs = 31,
    DriveAroundObs = 32,
    SearchTurnAroundObs = 33,
    SearchDriveAroundObs = 34,

    // Gate Search States
    GateSpin = 40,
    GateWait = 41,
    GateTurn = 42,
    GateDrive = 43,
    GateTurnToCentPoint = 44,
    GateDriveToCentPoint = 45,
    GateFace = 46,
    GateDriveThrough = 47,
    GateTurnToFarPost = 48,
    GateDriveToFarPost = 49,
    GateTurnToGateCenter = 50,
    GateSearchGimbal = 51,

    // Radio Repeater States
    RadioRepeaterTurn = 60,
    RadioRepeaterDrive = 61,
    RepeaterDropWait = 62,

    // Unknown State
    Unknown = 255

}; // AutonState

// This class is the representation of the drive status.
enum class DriveStatus
{
    Arrived,
    OnCourse,
    OffCourse
}; // DriveStatus

// This class creates a Rover object which can perform operations that
// the real rover can perform.
class Rover
{
public:
    // This class holds all the status informatin of the rover.
    class RoverStatus
    {
    public:
        RoverStatus();

        RoverStatus(
            NavState navState,
            AutonState autonStateIn,
            Bearing bearingIn,
            Course courseIn,
            Obstacle obstacleIn,
            Odometry odometryIn,
            Target targetIn,
            Target target2In,
            RadioSignalStrength signalIn
            );

        NavState& currentState();

        AutonState& autonState();

        Course& course();

        deque<Waypoint>& path();

        Obstacle& obstacle();

        Odometry& odometry();

        Target& leftTarget();

        Target& rightTarget();

        RadioSignalStrength& radio();

        unsigned getPathTargets();

        RoverStatus& operator=( RoverStatus& newRoverStatus );

    private:
        // The rover's current navigation state.
        NavState mCurrentState;

        // The rover's current auton state.
        AutonState mAutonState;

        // The rover's overall course.
        Course mCourse;

        // The rover's current path. The path is initially the same as
        // the rover's course, however, as waypoints are visited, the
        // are removed from the path but not the course.
        deque<Waypoint> mPath;

        // The rover's current obstacle information from computer
        // vision.
        Obstacle mObstacle;

        // The rover's current odometry information.
        Odometry mOdometry;

        // The rover's current target information from computer
        // vision.
        Target mTarget1;

        Target mTarget2;

        // the rover's current signal strength to the base station
        RadioSignalStrength mSignal;

        // Total targets to seach for in the course
        unsigned mPathTargets;
    };

    Rover( const rapidjson::Document& config, lcm::LCM& lcm_in );

    DriveStatus drive( const Odometry& destination );

    DriveStatus drive( const double distance, const double bearing, const bool target = false );

    void drive( const int direction, const double bearing );

    bool turn( Odometry& destination );

    bool turn( double bearing );

    void stop();

    bool updateRover( RoverStatus newRoverStatus );

    RoverStatus& roverStatus();

    PidLoop& distancePid();

    PidLoop& bearingPid();

    const double longMeterInMinutes() const;

    void updateRepeater( RadioSignalStrength& signal);

    bool isTimeToDropRepeater();

    Gimbal& gimbal();

    void publishGimbal();

    bool sendGimbalSetpoint(double desired_yaw);

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishJoystick( const double forwardBack, const double leftRight, const bool kill );

    bool isEqual( const Obstacle& obstacle1, const Obstacle& obstacle2 ) const;

    bool isEqual( const Odometry& odometry1, const Odometry& odometry2 ) const;

    bool isEqual( const Target& target1, const Target& target2 ) const;

    bool isTurningAroundObstacle( const NavState currentState ) const;

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // The rover's current status.
    RoverStatus mRoverStatus;

    // A reference to the configuration file.
    const rapidjson::Document& mRoverConfig;

    // A reference to the lcm object that will be used for
    // communicating with the actual rover and the base station.
    lcm::LCM& mLcmObject;

    // The pid loop for driving.
    PidLoop mDistancePid;

    // The pid loop for turning.
    PidLoop mBearingPid;

    // If it is time to drop a radio repeater
    bool mTimeToDropRepeater;

    // The conversion factor from arcminutes to meters. This is based
    // on the rover's current latitude.
    double mLongMeterInMinutes;

    // Gimbal object.
    Gimbal mGimbal;
};

#endif // ROVER_HPP
