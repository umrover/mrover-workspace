#include "auton_armStateMachine.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <map>
#include "rover_msgs/NavStatus.hpp"
#include "utilities.hpp"
//***include other stuff


// Constructs a StateMachine object with the input lcm object.
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

// Updates the target information of the rover's status.
void AutonArmStateMachine::updateRoverStatus( TargetList targetList )
{
    Target target1 = targetList.targetList[0];
    mNewRoverStatus.target() = target1;
} // updateRoverStatus( Target )

void AutonArmStateMachine::run() {
    publishNavState();
    if( isRoverReady() )
    {
        mStateChanged = false;
        NavState nextState = NavState::Unknown;

        if( !mPhoebe->roverStatus().autonState().is_auton )
        {
            nextState = NavState::Off;
            mPhoebe->roverStatus().currentState() = executeOff(); // turn off immediately
            clear( mPhoebe->roverStatus().path() );
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
    Target target1 = targetList.targetList[0];
    mNewRoverStatus.target() = target1;
    is_tag_received = true;
} //updateRoverStatus(TargetList targetList)

void AutonArmStateMachine::updateRoverStatus(Pos position){
    received_position = position;
        is_coordinates_received = true;
}

bool AutonArmStateMachine::isRoverReady() const {
    return mStateChanged || // internal data has changed
           mPhoebe->updateRover( mNewRoverStatus ); // external data has changed
} //isRoverReady()

AutonArmState AutonArmStateMachine::executeOff() { 
    if( mPhoebe->roverStatus().autonState().is_auton )
    {
        //TODO add what we want here
    }
    mPhoebe->stop();
    return AutonArmState::Off;
}

AutonArmState AutonArmStateMachine::executeDone() {
    mPhoebe->stop();
    return AutonArmState::Done;
}

AutonArmState AutonArmStateMachine::executeWaitingForTag() {
    // If statement to check if tag recieved
    if(!recieved_tag)
        return AutonArmState::WaitingForTag;
    is_tag_received = false;
    return AutonArmState::EvaluateTag;
}

AutonArmState AutonArmStateMachine::executeEvaluateTag() {
    //If statement to check if right tag is recieved //target.id
    return AutonArmState::RequestCoordinates;
    // Else request another tag
    return AutonArmState::RequestTag;
}

AutonArmState AutonArmStateMachine::executeRequestTag() {
    // Send request for tag
    LCM_message message = {"Request_Tag", PERCEPTION_PACKAGE};
    mLcmObject.publish(LCM_CHANNEL_NAME, &message);
    return AutonArmState::WaitingForTag;
}

AutonArmState AutonArmStateMachine::executeRequestCoordinates() {
    // Send request for coordinates 
    LCM_message message = {"Request_Coordinates", PERCEPTION_PACKAGE};
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

AutonArmState AutonArmStateMachine::executeSendCoordinates(TargetList targetList) {
    // send coordinates
    LCM_Coordinates coords = {received_position, TELEOP_PACKAGE};
    mLcmObject.publish(LCM_CHANNEL_NAME, &coords);
    return AutonArmState::Done;
}
