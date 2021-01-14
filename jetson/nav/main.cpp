#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "stateMachine.hpp"

using namespace rover_msgs;
using namespace std;

// This class handles all incoming LCM messages for the autonomous
// navigation of the rover.
class LcmHandlers
{
public:
    // Constructs an LcmHandler with the given state machine to work
    // with.
    LcmHandlers( StateMachine* stateMachine )
        : mStateMachine( stateMachine )
    {}

    // Sends the auton state lcm message to the state machine.
    void autonState(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const AutonState* autonState
        )
    {
        mStateMachine->updateRoverStatus( *autonState );
    }

    // Sends the course lcm message to the state machine.
    void course(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const Course* course
        )
    {
        mStateMachine->updateRoverStatus( *course );
    }

    // Sends the obstacle lcm message to the state machine.
    void obstacle(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const Obstacle* obstacle
        )
    {
        mStateMachine->updateRoverStatus( *obstacle );
    }

    // Sends the odometry lcm message to the state machine.
    void odometry(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const Odometry* odometry
        )
    {
        mStateMachine->updateRoverStatus( *odometry );
    }

    // Sends the target lcm message to the state machine.
    void targetList(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const TargetList* targetListIn
        )
    {
        mStateMachine->updateRoverStatus( *targetListIn );
    }

    // Sends the radio lcm message to the state machine.
    void radioSignalStrength(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const RadioSignalStrength* signalIn
        )
    {
        mStateMachine->updateRoverStatus( *signalIn );
    }

    // Updates Radio Repeater bool in state machine.
    void repeaterDropComplete(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const RepeaterDrop* completeIn
        )
    {
        mStateMachine->updateRepeaterComplete( );
    }

private:
    // The state machine to send the lcm messages to.
    StateMachine* mStateMachine;
};

// Runs the autonomous navigation of the rover.
int main()
{
    lcm::LCM lcmObject;
    if( !lcmObject.good() )
    {
        cerr << "Error: cannot create LCM\n";
        return 1;
    }

    StateMachine roverStateMachine( lcmObject );
    LcmHandlers lcmHandlers( &roverStateMachine );

    lcmObject.subscribe( "/auton", &LcmHandlers::autonState, &lcmHandlers );
    lcmObject.subscribe( "/course", &LcmHandlers::course, &lcmHandlers );
    lcmObject.subscribe( "/obstacle", &LcmHandlers::obstacle, &lcmHandlers );
    lcmObject.subscribe( "/odometry", &LcmHandlers::odometry, &lcmHandlers );
    lcmObject.subscribe( "/radio", &LcmHandlers::radioSignalStrength, &lcmHandlers );
    lcmObject.subscribe( "/rr_drop_complete", &LcmHandlers::repeaterDropComplete, &lcmHandlers );
    lcmObject.subscribe( "/target_list", &LcmHandlers::targetList, &lcmHandlers );

    while( lcmObject.handle() == 0 )
    {
        roverStateMachine.run();
    }
    return 0;
} // main()
