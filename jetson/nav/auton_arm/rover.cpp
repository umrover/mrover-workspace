#include <iostream>
#include "rover.hpp"

// Constructs a rover object with the given configuration file and lcm
// object with which to use for communications.
Rover::Rover( lcm::LCM& lcmObject )
    : mLcmObject( lcmObject ) { }

Rover::RoverStatus::RoverStatus() : mCurrentState(AutonArmState::WaitingForTag) {
    mAutonState.is_auton = true;
}

AutonArmState& Rover::RoverStatus::currentState() {
    return mCurrentState;
} //currentState()

AutonState& Rover::RoverStatus::autonState() {
    return mAutonState;
} //autonState()

TargetPositionList& Rover::RoverStatus::targetList() {
    return mTargetList;
} //target()

bool Rover::updateRover(RoverStatus newRoverStatus) {
    // Rover currently on.
    if( mRoverStatus.autonState().is_auton )
    {
        // Rover turned off
        if( !newRoverStatus.autonState().is_auton )
        {
            mRoverStatus.autonState() = newRoverStatus.autonState();
            return true;
        }

        // If any data has changed, update all data
        if(!isEqual( mRoverStatus.targetList(), newRoverStatus.targetList()))
        {
            cout << "Target updated" << endl;
            mRoverStatus.targetList() = newRoverStatus.targetList();
            return true;
        }

        return false;
    }

    // Rover currently off.
    else
    {
        // Rover turned on.
        if( newRoverStatus.autonState().is_auton )
        {
            mRoverStatus = newRoverStatus;
            return true;
        }
        return false;
    }
    return false;
} //updateRover(newRoverStatus)

Rover::RoverStatus& Rover::roverStatus() {
    return mRoverStatus;
}

bool Rover::isEqual( const TargetPositionList& targetList1, const TargetPositionList& targetList2 ) const
{
    if (targetList1.num_targets != targetList2.num_targets) return false;

    for(int i = 0; i < targetList1.num_targets; i++)
    {
        if(
            targetList1.target_list[i].x != targetList2.target_list[i].x || 
            targetList1.target_list[i].y != targetList2.target_list[i].y ||  
            targetList1.target_list[i].z != targetList2.target_list[i].z || 
            targetList1.target_list[i].target_id != targetList2.target_list[i].target_id
        
        ) return false;
    }
    return true;

} // isEqual( Target )