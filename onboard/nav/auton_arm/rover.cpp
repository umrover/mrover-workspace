#include "rover.hpp"

Rover::RoverStatus::RoverStatus() : mCurrentState(NavState::Off){
    mAutonState.is_auton = false;
}

AutonArmState& Rover::RoverStatus::currentState() {
    return mCurrentState;
} //currentState()

AutonState& Rover::RoverStatus::autonState() {
    return mAutonState;
} //autonState()

Target& Rover::RoverStatus::target() {
    return mTarget;
} //target()

Rover::Rover(const rapidjson::Document& config, lcm::LCM& lcm_in) : mRoverConfig(config), mLcmObject(mLcmObject) {}

void Rover::stop() {
    publishJoystick( 0, 0, false ); //TODO Double Check
} //stop()

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
        if(!isEqual( mRoverStatus.target(), newRoverStatus.target()))
        {
            mRoverStatus.target() = newRoverStatus.target();
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

Rover::RoverStatus& Rover::roverStatus(){
    return mRoverStatus;
}