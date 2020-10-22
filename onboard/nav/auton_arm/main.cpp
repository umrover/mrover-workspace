#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "auton_armStateMachine.hpp"

using namespace rover_msgs;
using namespace std;

class LcmHandlers{
public:
    void autonState(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const AutonState* autonState
        )
    {
        mStateMachine->updateRoverStatus( *autonState );
    }

    // Sends the target lcm message to the state machine.
    void targetList(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const TargetList* targetListIn
        )
    {
        statemachine->updateRoverStatus( *targetListIn );
    }

    void position(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const Pos* position
    )
    {
        statemachine->updateRoverStatus( *position );
    }
        
private:
    AutonArmStateMachine* statemachine = new AutonArmStateMachine();
};

int main(){

    lcm::LCM lcmObject;
        if( !lcmObject.good() )
        {
            cerr << "Error: cannot create LCM\n";
            return 1;
        }
        
        AutonArmStateMachine autonArmStateMachine;
        LcmHandlers lcmHandlers( &autonArmStateMachine );

        lcmObject.subscribe( "/auton", &LcmHandlers::autonState, &lcmHandlers );
        lcmObject.subscribe( "/target_list", &LcmHandlers::targetList, &lcmHandlers );
        lcmObject.subscribe( "/autonomous_arm", &LCMHandlers::position, &lcmHandlers );

        while( lcmObject.handle() == 0 )
        {
            autonArmStateMachine.run();
        }
        return 0;
} // main()