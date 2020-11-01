#include "auton_armStateMachine.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <map>
#include "rover_msgs/Pos.hpp"
#include "rover_msgs/Message.hpp"
#include "rover_msgs/Target.hpp"
//***include other stuff

// Constructs an AutonArmStateMachine object with the input lcm object.
// Reads the configuartion file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
AutonArmStateMachine::AutonArmStateMachine( lcm::LCM& lcmObject )
    : mPhoebe( nullptr )
    , mLcmObject( lcmObject )
    , mStateChanged( true )
    , is_tag_received ( false )
    , is_coordinates_received ( false )
{
    ifstream configFile;
    string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_nav/config.json";
    configFile.open( configPath );
    string config = "";
    string setting;
    while( configFile >> setting )
    {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );
    mPhoebe = new Rover( mRoverConfig, lcmObject );
} //AutonArmStateMachine Constructor

// Destructs the StateMachine object. Deallocates memory for the Rover
// object.
AutonArmStateMachine::~AutonArmStateMachine( )
{
    delete mPhoebe;
}

void AutonArmStateMachine::run() {
    publishNavState();
    if( isRoverReady() )
    {
        mStateChanged = false;
        AutonArmState nextState = AutonArmState::Unknown;

        if( !mPhoebe->roverStatus().autonState().is_auton )
        {
            nextState = AutonArmState::Off;
            mPhoebe->roverStatus().currentState() = executeOff(); // turn off immediately
            if( nextState != mPhoebe->roverStatus().currentState() )
            {
                mPhoebe->roverStatus().currentState() = nextState;
                mStateChanged = true;
            }
            return;
        }
        switch( mPhoebe->roverStatus().currentState() )
        {
            case AutonArmState::Off:
            {
                nextState = executeOff();
                break;
            }

            case AutonArmState::Done:
            {
                nextState = executeDone();
                break;
            }

            case AutonArmState::WaitingForTag:
            {
                nextState = executeWaitingForTag();
                break;
            }

            case AutonArmState::WaitingForCoordinates:
            {
                nextState = executeWaitingForCoordinates();
                break;  
            }

            case AutonArmState::EvaluateTag: 
            {
                nextState = executeEvaluateTag();
            }

            case AutonArmState::RequestTag:
            {
                nextState = executeRequestTag();
                break;
            }

            case AutonArmState::RequestCoordinates: 
            {
                nextState = executeRequestCoordinates();
                break;
            }

            case AutonArmState::SendCoordinates:
            {
                nextState = executeSendCoordinates();
                break;
            }

            case AutonArmState::Unknown:
            {
                cerr << "Entered unknown state.\n";
                exit(1);
            }
        } // switch

        if( nextState != mPhoebe->roverStatus().currentState() )
        {
            mStateChanged = true;
            mPhoebe->roverStatus().currentState() = nextState;
        }
        cerr << flush;
    } // if
} //run()

// Updates the auton state (on/off) of the rover's status.
void AutonArmStateMachine::updateRoverStatus(AutonState autonState) {
    mNewRoverStatus.autonState() = autonState;
} //updateRoverStatus(AutonState autonState)

// Updates the target information of the rover's status.
void AutonArmStateMachine::updateRoverStatus(TargetList targetList) {
    mNewRoverStatus.target() = targetList.targetList[0];
    is_tag_received = true;
} //updateRoverStatus(TargetList targetList)

void AutonArmStateMachine::updateRoverStatus(Pos position) {
    received_position = position;
    is_coordinates_received = true;
}

bool AutonArmStateMachine::isRoverReady() const {
    return mStateChanged || // internal data has changed
           mPhoebe->updateRover( mNewRoverStatus ); // external data ha s changed
} //isRoverReady()

void AutonArmStateMachine::publishNavState() const
{
    //TODO implement if we want 
} // publishNavState()

AutonArmState AutonArmStateMachine::executeOff() { 
    if( mPhoebe->roverStatus().autonState().is_auton )
    {
        //TODO add what we want here
    }
    return AutonArmState::Off;
}

AutonArmState AutonArmStateMachine::executeDone() {
    return AutonArmState::Done;
}

AutonArmState AutonArmStateMachine::executeWaitingForTag() {
    // If statement to check if tag recieved
    if(!is_tag_received)
        return AutonArmState::WaitingForTag;
    is_tag_received = false;
    return AutonArmState::EvaluateTag;
}

AutonArmState AutonArmStateMachine::executeEvaluateTag() {
    // Check if tag matches correct tag
    if(mNewRoverStatus.target().id == CORRECT_TAG_ID)
        return AutonArmState::RequestCoordinates;
    // Else request another tag
    return AutonArmState::RequestTag;
}

AutonArmState AutonArmStateMachine::executeRequestTag() {
    // Send request for tag
    Message message = {"request_tag", PERCEPTION_PACKAGE};
    mLcmObject.publish(LCM_CHANNEL_NAME, &message);
    return AutonArmState::WaitingForTag;
}

AutonArmState AutonArmStateMachine::executeRequestCoordinates() {
    // Send request for coordinates 
    Message message = {"request_coordinates", PERCEPTION_PACKAGE};
    mLcmObject.publish(LCM_CHANNEL_NAME, &message);
    return AutonArmState::WaitingForCoordinates;
}

AutonArmState AutonArmStateMachine::executeWaitingForCoordinates(){
    // If coordinates have not been recieved
    if(!is_coordinates_received)
        return AutonArmState::WaitingForCoordinates;
    // Else send coordinates
    is_coordinates_received = false;
    return AutonArmState::SendCoordinates;
}

AutonArmState AutonArmStateMachine::executeSendCoordinates() {
    // send coordinates
    received_position.target_package = TELEOP_PACKAGE;
    mLcmObject.publish(LCM_CHANNEL_NAME, &received_position);
    return AutonArmState::Done;
}
