#ifndef AUTONARM_STATE_MACHINE_HPP
#define AUTONARM_STATE_MACHINE_HPP

#include <lcm/lcm-cpp.hpp>
#include "rapidjson/document.h"
#include "rover.hpp"

using namespace std;
using namespace rover_msgs;

class AutonArmStateMachine
{
public:
    AutonArmStateMachine( lcm::LCM& lcmObject );

    ~AutonArmStateMachine();

    void run(); //Start StateMachine

    void updateRoverStatus( AutonState autonState );

    void updateRoverStatus( TargetList targetList );
    
    bool isRoverReady() const;


private:
    bool isRoverReady() const;

    AutonArmState executeOff();

    AutonArmState executeDone();

    AutonArmState executeWaitingForTag();

    AutonArmState executeEvaluateTag();

    AutonArmState executeRequestTag();

    AutonArmState executeRequestCoordinates();

    AutonArmState executeSendCoordinates();

    /*************************************************************************/
    /* Private Member Variables */
    /*************************************************************************/
    // Rover object to do basic rover operations in the state machine.
    Rover* mPhoebe;

    // RoverStatus object for updating the rover's status.
    Rover::RoverStatus mNewRoverStatus;

    // Lcm object for sending and recieving messages.
    lcm::LCM& mLcmObject;

    // Configuration file for the rover.
    rapidjson::Document mRoverConfig;

    // Indicates if the state changed on a given iteration of run.
    bool mStateChanged;
}

#endif