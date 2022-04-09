#pragma once

#include <queue>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/Bearing.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/TargetList.hpp"
#include "rover_msgs/Waypoint.hpp"
#include "rover_msgs/AutonDriveControl.hpp"
#include "rapidjson/document.h"
#include "courseProgress.hpp"
#include "environment.hpp"
#include "pid.hpp"


using namespace rover_msgs;

// This class is the representation of the navigation states.
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
    GateMakePath = 41,
    GateTraverse = 42,

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

    DriveStatus drive(const Odometry& destination, double dt, double stopDistance);

    DriveStatus drive(double distance, double bearing, double threshold, double dt);

    bool turn(Odometry const& destination, double dt);

    bool turn(double bearing, double dt);

    void stop();

    PidLoop& bearingPid();

    [[nodiscard]] double longMeterInMinutes() const;

    [[nodiscard]] NavState const& currentState() const;

    [[nodiscard]] AutonState const& autonState() const;

    [[nodiscard]] Odometry const& odometry() const;

    void setAutonState(AutonState state);

    void setOdometry(Odometry const& odometry);

    void setState(NavState state);

    void setLongMeterInMinutes(double LongMeterInMinutes);

private:
    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/
    void publishAutonDriveCmd(double leftVel, double rightVel);

    static bool isTurningAroundObstacle(NavState currentState);

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // A reference to the configuration file.
    const rapidjson::Document& mConfig;

    // A reference to the lcm object that will be used for
    // communicating with the actual rover and the base station.
    lcm::LCM& mLcmObject;

    // The pid loop for turning.
    PidLoop mBearingPid;

    // The conversion factor from arcminutes to meters. This is based
    // on the rover's current latitude.
    double mLongMeterInMinutes;

    // The rover's current navigation state.
    NavState mCurrentState{NavState::Off};

    // The rover's current auton state.
    AutonState mAutonState{};

    // The rover's current odometry information.
    Odometry mOdometry{};
};
