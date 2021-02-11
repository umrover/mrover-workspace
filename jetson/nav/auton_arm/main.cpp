#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "stateMachine.hpp"

using namespace rover_msgs;
using namespace std;

class LcmHandlers {
public:
    // Constructs an LcmHandler with the given state machine to work
    // with.
    LcmHandlers( AutonArmStateMachine* autonArmStateMachine )
        : statemachine( autonArmStateMachine )
    {}
    
    void autonState(
        const lcm::ReceiveBuffer* recieveBuffer,
        const string& channel,
        const AutonState* autonState
        )
    {
        statemachine->updateRoverStatus( *autonState );
    }

    // Sends the target lcm message to the state machine.
    void targetPositionList(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const TargetPositionList* targetIn
        )
    {
        statemachine->updateRoverStatus( *targetIn );
    }
        
private:
    AutonArmStateMachine* statemachine;
};

int main() {

    lcm::LCM lcmObject;
    if( !lcmObject.good() )
    {
        cerr << "Error: cannot create LCM\n";
        return 1;
    }
    
    AutonArmStateMachine autonArmStateMachine( lcmObject );
    LcmHandlers lcmHandlers( &autonArmStateMachine );

    lcmObject.subscribe( "/auton", &LcmHandlers::autonState, &lcmHandlers );
    lcmObject.subscribe( "/target_position_list", &LcmHandlers::targetPositionList, &lcmHandlers );

    cout << "Starting state machine" << endl;
    while( lcmObject.handle() == 0 )
    {
        autonArmStateMachine.run(); 
    }
    return 0;
} // main()