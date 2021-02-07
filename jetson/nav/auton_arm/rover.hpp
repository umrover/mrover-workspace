#ifndef ROVER_HPP
#define ROVER_HPP

#include <lcm/lcm-cpp.hpp>
#include <queue>

#include "rover_msgs/AutonState.hpp"
#include "rover_msgs/TargetPositionList.hpp"
#include "rapidjson/document.h"

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
    PauseTag = 12,
    
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
            TargetPositionList targetListIn
            );

        AutonArmState& currentState();

        AutonState& autonState();

        TargetPositionList& targetList();

    private:
        // The rover's current navigation state.
        AutonArmState mCurrentState;

        // The rover's current auton state.
        AutonState mAutonState;

        // The rover's current target information from computer
        // vision.
        TargetPositionList mTargetList;

        // Total targets to seach for in the course
        unsigned mPathTargets;
    };

    Rover( lcm::LCM& lcm_in );

    void stop();

    bool updateRover( RoverStatus newRoverStatus );

    RoverStatus& roverStatus();

private:

    /*************************************************************************/
    /* Private Member Functions */
    /*************************************************************************/

    bool isEqual( const TargetPositionList& target1, const TargetPositionList& targetList2 ) const;

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/

    // The rover's current status.
    RoverStatus mRoverStatus;

    // A reference to the lcm object that will be used for
    // communicating with the actual rover and the base station.
    lcm::LCM& mLcmObject;
    
};
#endif // ROVER_HPP