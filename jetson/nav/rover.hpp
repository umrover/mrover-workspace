#pragma once

#include <lcm/lcm-cpp.hpp>
#include <queue>
#include <memory>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/Bearing.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/TargetList.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "rapidjson/document.h"
#include "courseState.hpp"
#include "environment.hpp"
#include "pid.hpp"

using namespace rover_msgs;

// This class is the representation of the navigation states.
enum class NavState {
    // Base States
    Off = 0,
    Done = 1,

    // Simple Movement
    Turn = 10,
    Drive = 11,

    // Search States
    SearchFaceNorth = 20,
    SearchTurn = 21,
    SearchDrive = 22,
    ChangeSearchAlg = 23,

    // Target Found States
    TurnToTarget = 27,
    DriveToTarget = 28,

    // Obstacle Avoidance States
    TurnAroundObs = 30,
    DriveAroundObs = 31,
    SearchTurnAroundObs = 32,
    SearchDriveAroundObs = 33,

    // Gate Search States
    GateSpin = 40,
    GateSpinWait = 41,
    GateTurn = 42,
    GateDrive = 43,
    GateTurnToCentPoint = 44,
    GateDriveToCentPoint = 45,
    GateFace = 46,
    GateDriveThrough = 47,
    GateTurnToFarPost = 48,
    GateDriveToFarPost = 49,
    GateTurnToGateCenter = 50,

    // Unknown State
    Unknown = 255

}; // AutonState

// This class is the representation of the drive status.
enum class DriveStatus {
    Arrived,
    OnCourse,
    OffCourse
}; // DriveStatus

// This class creates a Rover object which can perform operations that
// the real rover can perform.
class Rover {
public:
    Rover(const rapidjson::Document& config, lcm::LCM& lcm_in);

    DriveStatus drive(const Odometry& destination);

    DriveStatus drive(double distance, double bearing, bool target = false);

    void drive(int direction, double bearing);

    bool turn(Odometry const& destination);

    bool turn(double bearing);

    void stop();

    PidLoop& distancePid();

    PidLoop& bearingPid();

    double longMeterInMinutes() const;

    NavState const& currentState() const;

    AutonState const& autonState() const;

    Odometry const& odometry() const;

    Target const& leftTarget() const;

    Target const& rightTarget() const;

    Target const& leftCacheTarget() const;

    Target const& rightCacheTarget() const;

    int getLeftMisses() const;

    int getRightMisses() const;

    int getLeftHits() const;

    int getRightHits() const;

    void setAutonState(AutonState state);

    void setOdometry(Odometry const& odometry);

    void updateTargets(shared_ptr<Environment> const& env, shared_ptr<CourseProgress> const& course);

    void setState(NavState state);

    void resetMisses();

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishJoystick(double forwardBack, double leftRight, bool kill);

    static bool isTurningAroundObstacle(NavState currentState);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // A reference to the configuration file.
    const rapidjson::Document& mRoverConfig;

    // A reference to the lcm object that will be used for
    // communicating with the actual rover and the base station.
    lcm::LCM& mLcmObject;

    // The pid loop for driving.
    PidLoop mDistancePid;

    // The pid loop for turning.
    PidLoop mBearingPid;

    // The conversion factor from arcminutes to meters. This is based
    // on the rover's current latitude.
    double mLongMeterInMinutes;

    // The rover's current navigation state.
    NavState mCurrentState{NavState::Off};

    // The rover's current auton state.
    AutonState mAutonState{};

    // The rover's current path. The path is initially the same as
    // the rover's course, however, as waypoints are visited, the
    // are removed from the path but not the course.

    // The rover's current odometry information.
    Odometry mOdometry{};

    // The rover's current target information from computer
    // vision.
    Target mTargetLeft{-1.0, 0.0, 0};

    Target mTargetRight{-1.0, 0.0, 0};

    // Cached Target information
    Target mCTargetLeft{-1.0, 0.0, 0};

    Target mCTargetRight{-1.0, 0.0, 0};

    // Count of misses with cache
    int mCountLeftMisses = 0;

    int mCountRightMisses = 0;

    // Count hits for avoiding FPs
    int mCountLeftHits = 0;

    int mCountRightHits = 0;
};
