#include <gimbal.hpp>

using namespace std;

Gimbal::Gimbal(const rapidjson::Document& mRoverConfig){
    this->MAX_YAW = mRoverConfig["gimbal"]["maxRange"].GetDouble();
    this->MIN_YAW = mRoverConfig["gimbal"]["minRange"].GetDouble();
    this->TOLERANCE = mRoverConfig["gimbal"]["tolerance"].GetDouble();
}


bool Gimbal::setTargetYaw(double target){
    this->target_yaw = target;
    return abs((this->target_yaw - this->cur_yaw) <= this->TOLERANCE);
}

double Gimbal::getYaw() const {
    return this->cur_yaw;
}

//TODO: make sure gimbalChannel is defined in config.json
void Gimbal::publishControlSignal(lcm::LCM & lcmObj, const rapidjson::Document& mRoverConfig) const {
    string channel = mRoverConfig[ "lcmChannels" ][ "gimbalChannel" ].GetString();
    lcmObj.publish(channel, &(this->signal));
}

