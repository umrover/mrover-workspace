#ifndef AUTONARM_STATE_MACHINE_HPP
#define AUTONARM_STATE_MACHINE_HPP

#include <lcm/lcm-cpp.hpp>
#include <chrono>
#include <thread>
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
    
    const string LCM_CHANNEL_NAME = "/autonomous_arm"; //might not need


private:
    bool isEqual( const TargetPosition &a, const TargetPosition &b ) const;

    bool isRoverReady() const;

    void publishNavState() const;

    AutonArmState executeOff();

    AutonArmState executeDone();

    AutonArmState executeWaitingForTag();

    AutonArmState executeEvaluateTag();

    AutonArmState executePauseTag();

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

    // Holds position of last correct tag received
    TargetPosition currentPosition;

    // Tracks number of correct tags received in a row
    int correctTags;

    // Indicates if the state changed on a given iteration of run.
    bool mStateChanged;

    // Tracks whether or not target has been received 
    bool isTagReceived;

    // Start time for pause timer
    chrono::system_clock::time_point start; 

    // Number of correct tags needed in a row to proceed
    static const int CORRECT_TAGS_NEEDED = 3;

    // ID of correct tag 
    static const int32_t CORRECT_TAG_ID = -1;

    // Delay time between tag evaluations 
    chrono::duration<double, milli> DELAY = chrono::milliseconds(5000);
    //chrono::seconds::count x = 1;

    // Margin of error between each field (x,y,z) of 2 consecutive TargetPosition coordinates
    const double MARGIN_OF_ERROR = 5;
};

#endif