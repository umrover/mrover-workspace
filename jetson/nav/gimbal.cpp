#include <gimbal.hpp>

using namespace std;
void Gimbal::setTargetYaw(double target){
    this->target_yaw = target;
}

double Gimbal::getYaw() const {
    return this->cur_yaw;
}

//TODO: make sure gimbalChannel is defined in config.json
void Gimbal::publishControlSignal(lcm::LCM & lcmObj, const rapidjson::Document& mRoverConfig) const {
    string channel = mRoverConfig[ "lcmChannels" ][ "gimbalChannel" ].GetString();
    lcmObj.publish(channel, &(this->signal));
}

