#pragma once

#include <chrono>
#include <ctime>

#include <lcm/lcm-cpp.hpp>
#include <rapidjson/document.h>

#include "courseProgress.hpp"
#include "environment.hpp"
#include "pid.hpp"
#include "rover_msgs/AutonDriveControl.hpp"
#include "rover_msgs/Bearing.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Enable.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/ProjectedPoints.hpp"
#include "rover_msgs/TargetList.hpp"
#include "rover_msgs/Waypoint.hpp"

// TODO: replace with function
#define NOW std::chrono::high_resolution_clock::now()

using namespace rover_msgs;

enum class NavState {
    // Base States
    Off = 0,
    Done = 1,

    DriveWaypoints = 10,

    // Search States
    BeginSearch = 22,
    Search = 20,

    // Target Found States
    DriveToTarget = 28,

    // Obstacle Avoidance States
    TurnAroundObs = 30,
    DriveAroundObs = 31,
    SearchTurnAroundObs = 32,
    SearchDriveAroundObs = 33,

    // Gate Search
    BeginGateSearch = 40,
    GateTraverse = 41,

    // Unknown State
    Unknown = 255
};// AutonState

class Rover {
public:
    Rover(const rapidjson::Document& config, lcm::LCM& lcm_in);

    bool drive(const Odometry& destination, double stopDistance, double dt);

    bool drive(double distance, double bearing, double threshold, double dt);

    bool driveBackwards(const Odometry& destination, double stopDistance, double dt);

    bool driveBackwards(double distance, double bearing, double threshold, double dt);

    bool turn(Odometry const& destination, double dt);

    bool turn(double absoluteBearing, double dt);

    void stop();

    PidLoop& turningBearingPid();

    PidLoop& drivingBearingPid();

    [[nodiscard]] double longMeterInMinutes() const;

    [[nodiscard]] NavState const& currentState() const;

    [[nodiscard]] Enable const& autonState() const;

    [[nodiscard]] Odometry const& odometry() const;

    void setAutonState(Enable state);

    void setOdometry(Odometry const& odometry);

    void setState(NavState state);

    void setLongMeterInMinutes(double LongMeterInMinutes);

    bool isTurning();

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishAutonDriveCmd(double leftVel, double rightVel);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    const rapidjson::Document& mConfig;

    // Interface for communicating messages to and from the base station
    lcm::LCM& mLcmObject;

    // The pid loop for turning.
    PidLoop mTurningBearingPid;

    // The pid loop for turning while driving
    PidLoop mDriveBearingPid;

    // The conversion factor from arcminutes to meters. This is based n the rover's current latitude.
    double mLongMeterInMinutes;

    NavState mCurrentState{NavState::Off};

    Enable mAutonState{};

    Odometry mOdometry{};

    bool mTurning, mDriving;

    std::chrono::_V2::system_clock::time_point mTurnStartTime, mBackingUpStartTime;
};// Rover
