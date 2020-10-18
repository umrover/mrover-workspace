#ifndef ROVER_HPP
#define ROVER_HPP

#include <lcm/lcm-cpp.hpp>
#include <queue>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/Bearing.hpp"
#include "rover_msgs/Course.hpp"
#include "rover_msgs/Obstacle.hpp"
#include "rover_msgs/Odometry.hpp"
#include "rover_msgs/RepeaterDropInit.hpp"
#include "rover_msgs/RepeaterDropComplete.hpp"
#include "rover_msgs/RadioSignalStrength.hpp"
#include "rover_msgs/TargetList.hpp"
#include "..rover_msgs/Waypoint.hpp"
#include "rapidjson/document.h"
#include "pid.hpp"

using namespace rover_msgs;
using namespace std;

// This class is the representation of the auton arm states.
enum class AutonArmState
{
    // Base States
    Off = 0,
    Done = 1,

    // Nav-perception states
    WaitingForTag = 10,
    EvaluateTag = 11,
    RequestTag = 12,

    // Coordinates 
    RequestCoordinates = 20,
    WaitingForCoordinates = 21,
    
    // Nav-teleop
    SendCoordinates = 30,

    // Unknown State
    Unknown = 255
}; //AutomArmState

class Rover
{
public:
    // This class holds all the status information of the rover.
    class RoverStatus
    {
    public:
        RoverStatus();

        RoverStatus(
            AutonArmState navState,
            AutonState autonStateIn,
            Target targetIn,
            RadioSignalStrength signalIn
            );

        AutonArmState& currentState();

        AutonState& autonState();

        Target& target();

    private:
        // The rover's current navigation state.
        AutonArmState mCurrentState;

        // The rover's current auton state.
        AutonState mAutonState;

        // The rover's current target information from computer
        // vision.
        Target mTarget;

        // Total targets to seach for in the course
        unsigned mPathTargets;
    };

    Rover( const rapidjson::Document& config, lcm::LCM& lcm_in );

    void stop();

    bool updateRover( RoverStatus newRoverStatus );

    RoverStatus& roverStatus();

private:
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
    
};
#endif // ROVER_HPP