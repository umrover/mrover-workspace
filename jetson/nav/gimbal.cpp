#include "gimbal.hpp"
#include "iostream"
using namespace std;



//constructs gimbal object with tolerance
Gimbal::Gimbal(double TOLERANCE_IN): TOLERANCE(TOLERANCE_IN){}

//sets the target yaw variable and returns if the gimbal is at the target
bool Gimbal::setTargetYaw(double target){
    target_yaw = target;
    return (abs(this->target_yaw - this->cur_yaw) <= this->TOLERANCE);
}

//returns the current measured yaw of the gimbal
double Gimbal::getYaw() const {
    return this->cur_yaw;
}

//sets the current measured yaw of the gimbal
void Gimbal::setCurrentYaw(double yaw){
    this->cur_yaw = yaw;
}

//publishes the control signal LCM
void Gimbal::publishControlSignal(lcm::LCM & lcmObj, const rapidjson::Document& mRoverConfig) {
  
    string channel = mRoverConfig[ "lcmChannels" ][ "zedGimbalCommand" ].GetString();
    signal.angle = target_yaw;
    lcmObj.publish(channel, &signal);
}