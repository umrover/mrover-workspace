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

    void updateRoverStatus ( Pos position );
    
    bool isRoverReady() const;

    static const string LCM_CHANNEL_NAME = "/autonomous_arm";


private:
    bool isRoverReady() const;

    AutonArmState executeOff();

    AutonArmState executeDone();

    AutonArmState executeWaitingForTag();

    AutonArmState executeEvaluateTag();

    AutonArmState executeRequestTag();

    AutonArmState executeRequestCoordinates();

    AutonArmState executeWaitingForCoordinates();

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

    // Coordinates recieved from teleop after identifying correct tag
    Pos received_position;

    bool is_tag_received;

    bool is_coordinates_received;

    struct LCM_message {
        string message
        int packageNum
    };

    struct LCM_coordinates {
        Pos position,
        int packageNum;
    };

    static const int NAV_PACKAGE = 1;
    static const int TELEOP_PACKAGE = 2;
    static const int PERCEPTION_PACKAGE = 3;
}

#endif