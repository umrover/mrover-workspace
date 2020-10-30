#include "gimbal.hpp"

using namespace std;

Gimbal::Gimbal(double MIN_YAW_IN, double MAX_YAW_IN, double TOLERANCE_IN, PidLoop pid_in):
    MAX_YAW(MAX_YAW_IN), MIN_YAW(MIN_YAW_IN), TOLERANCE(TOLERANCE_IN), pid(pid_in)
    {}


bool Gimbal::setTargetYaw(double target){
    target_yaw = target;
    pid.reset();
    return abs((this->target_yaw - this->cur_yaw) <= this->TOLERANCE);
}

double Gimbal::getYaw() const {
    return this->cur_yaw;
}

//TODO: make sure gimbalChannel is defined in config.json
void Gimbal::publishControlSignal(lcm::LCM & lcmObj, const rapidjson::Document& mRoverConfig) {
    string channel = mRoverConfig[ "lcmChannels" ][ "gimbalChannel" ].GetString();
    signal.yaw = pid.update(getYaw(), target_yaw);
    lcmObj.publish(channel, &signal);
}