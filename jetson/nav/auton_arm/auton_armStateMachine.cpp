#include "auton_armStateMachine.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <map>
#include "rover_msgs/TargetPositionList.hpp"

// Constructs an AutonArmStateMachine object with the input lcm object.
// Reads the configuartion file and constructs a Rover objet with this
// and the lcmObject. Sets mStateChanged to true so that on the first
// iteration of run the rover is updated.
AutonArmStateMachine::AutonArmStateMachine( lcm::LCM& lcmObject )
    : mPhoebe( nullptr )
    , mLcmObject( lcmObject )
    , mStateChanged( true )
    , is_tag_received ( false )
{
    num_correct_tags = 0;
    mPhoebe = new Rover( lcmObject );
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

            case AutonArmState::EvaluateTag: 
            {
                nextState = executeEvaluateTag();
                break;
            }

            case AutonArmState::PauseTag:
            {
                nextState = executePauseTag();
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
void AutonArmStateMachine::updateRoverStatus(TargetPositionList targetList) {
    if(mPhoebe->roverStatus().currentState() != AutonArmState::WaitingForTag) return;
    cout << "Target received!" << endl;
    
    mNewRoverStatus.targetList() = targetList;
    is_tag_received = true;
} //updateRoverStatus(Target target)

bool AutonArmStateMachine::isRoverReady() const {
    return mStateChanged || // internal data has changed
           mPhoebe->updateRover( mNewRoverStatus ) || // external data has changed
           is_tag_received || // has a new tag been received 
           mPhoebe->roverStatus().currentState() == AutonArmState::PauseTag; // if timer is active
} //isRoverReady()

void AutonArmStateMachine::publishNavState() const
{
    //TODO implement if we want 
} // publishNavState()

AutonArmState AutonArmStateMachine::executeOff() { 
    if( mPhoebe->roverStatus().autonState().is_auton )
    {
        cout << "Reached executeOff" << endl;
        //TODO add what we want here
    }
    return AutonArmState::Off;
}

AutonArmState AutonArmStateMachine::executeDone() {
    cout << "reached executeDone" << endl;   
    return AutonArmState::Done;
}

AutonArmState AutonArmStateMachine::executeWaitingForTag() {
    // If statement to check if tag recieved
    if(!is_tag_received) {
        cout << "Waiting for tag" << endl;
        return AutonArmState::WaitingForTag;
    }

    cout << "Tag received" << endl;
    is_tag_received = false;
    return AutonArmState::EvaluateTag;
}

AutonArmState AutonArmStateMachine::executeEvaluateTag() {
    // Check if tag matches correct tag
    TargetPositionList &targetList = mNewRoverStatus.targetList();

    bool tagFound = false;
    for (int i = 0; i < targetList.num_targets; i++) {
        if (targetList.target_list[i].target_id == CORRECT_TAG_ID) { 
            TargetPosition newPosition = targetList.target_list[i];
            if(isEqual(currentPosition, newPosition)) {   
                num_correct_tags++;
                currentPosition = newPosition;
            }
            else {
                num_correct_tags = 1;
                currentPosition = newPosition;
            }
            tagFound = true;
            break;
        }
    }

    if(!tagFound) num_correct_tags = 0;
    else if(num_correct_tags >= CORRECT_TAGS_NEEDED) {
        return AutonArmState::SendCoordinates;
    }

    cout << "Number of correct tags is: " << num_correct_tags << endl;
    // Else wait for another tag list
    cout << "Waiting for another tag" << endl;
    
    start = chrono::system_clock::now();
    return AutonArmState::PauseTag;
}

AutonArmState AutonArmStateMachine::executePauseTag() {
    cout << "Waiting...\n";
    
    chrono::system_clock::time_point current = chrono::system_clock::now();
    if ( current-start >= DELAY ) {
        cout << "Done waiting\n";
        return AutonArmState::WaitingForTag;
    }
    return AutonArmState::PauseTag;    
}

AutonArmState AutonArmStateMachine::executeSendCoordinates() {
    // send coordinates
    cout << "executeSendCoordinates" << endl;

    mLcmObject.publish("/target_position", &currentPosition);
    cout << "Sent target position\n";
    
    return AutonArmState::Done;
}

bool AutonArmStateMachine::isEqual(const TargetPosition &a, const TargetPosition &b) const {
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2) + pow(a.z-b.z, 2)) < MARGIN_OF_ERROR;
}