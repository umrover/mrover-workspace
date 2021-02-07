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
    bool isRoverReady() const;

    void publishNavState() const;

    AutonArmState executeOff();

    AutonArmState executeDone();

    AutonArmState executeWaitingForTag();

    AutonArmState executeEvaluateTag();

    AutonArmState executePauseTag();

    AutonArmState executeSendCoordinates();

    // Struct to store x, y, and z coordinates of most recent tag
    struct Position {
        const double MARGIN_OF_ERROR = 5;   //possibly put outside of struct

        double x;
        double y;
        double z;
        Position() : x(0), y(0), z(0) {}
        Position(double x, double y, double z) { this->x = x; this->y = y; this->z = z; }
        bool operator==(const Position &rhs);
        Position& operator=(const Position &rhs);
    };

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
    Position currentPosition;

    // Tracks number of correct tags received in a row
    int num_correct_tags;

    // Indicates if the state changed on a given iteration of run.
    bool mStateChanged;

    // Tracks whether or not target has been received 
    bool is_tag_received;

    // Number of correct tags needed in a row to proceed
    static const int CORRECT_TAGS_NEEDED = 3;

    // ID of correct tag 
    static const int32_t CORRECT_TAG_ID = -1;

    // Delay time between tag evaluations 
    chrono::duration<double, milli> DELAY = chrono::milliseconds(15000);
    //chrono::seconds::count x = 1;
};

#endif