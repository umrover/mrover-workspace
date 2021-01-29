#ifndef AUTONARM_STATE_MACHINE_HPP
#define AUTONARM_STATE_MACHINE_HPP

#include <lcm/lcm-cpp.hpp>
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

    void updateRoverStatus( TargetPositionList targetList );
    
    const string LCM_CHANNEL_NAME = "/autonomous_arm";


private:
    bool isRoverReady() const;

    void publishNavState() const;

    AutonArmState executeOff();

    AutonArmState executeDone();

    AutonArmState executeWaitingForTag();

    AutonArmState executeEvaluateTag();

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

    // Indicates if the state changed on a given iteration of run.
    bool mStateChanged;

    bool is_tag_received;

    bool is_coordinates_received;

    static const int32_t CORRECT_TAG_ID = -1;

};

#endif