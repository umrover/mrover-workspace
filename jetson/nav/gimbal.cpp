#include "gimbal.hpp"
#include "iostream"
using namespace std;

Gimbal::Gimbal(double MIN_YAW_IN, double MAX_YAW_IN, double TOLERANCE_IN):
    MAX_YAW(MAX_YAW_IN), MIN_YAW(MIN_YAW_IN), TOLERANCE(TOLERANCE_IN)
    {}


bool Gimbal::setTargetYaw(double target){
    target_yaw = target;
    // TODO: Update the command based on the current yaw
    return abs((this->target_yaw - this->cur_yaw) <= this->TOLERANCE);
}

double Gimbal::getYaw() const {
    return this->cur_yaw;
}

// TODO: Rename this struct
void Gimbal::setYaw(double yaw){
    this->cur_yaw = yaw;
}

//TODO: make sure gimbalChannel is defined in config.json
void Gimbal::publishControlSignal(lcm::LCM & lcmObj, const rapidjson::Document& mRoverConfig) {
    std::cout << "publishing gimbal signal" << std::endl;
    string channel = mRoverConfig[ "lcmChannels" ][ "zedGimbalCommand" ].GetString();
    signal.angle = target_yaw;
    lcmObj.publish(channel, &signal);
}