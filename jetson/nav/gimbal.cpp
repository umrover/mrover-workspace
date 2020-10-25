#include <gimbal.hpp>

using namespace std;

Gimbal::Gimbal(const rapidjson::Document& mRoverConfig){
    this->MAX_YAW = mRoverConfig["gimbal"]["maxRange"].GetDouble();
    this->MIN_YAW = mRoverConfig["gimbal"]["minRange"].GetDouble();
    this->TOLERANCE = mRoverConfig["gimbal"]["tolerance"].GetDouble();
    this->pid = PidLoop(mRoverConfig["gimbalPid"]["Kp"].GetDouble(), mRoverConfig["gimbalPid"]["Ki"].GetDouble(), mRoverConfig["gimbalPid"]["Kd"].GetDouble());
}


bool Gimbal::setTargetYaw(double target){
    this->target_yaw = target;
    this->pid.reset();
    return abs((this->target_yaw - this->cur_yaw) <= this->TOLERANCE);
}

double Gimbal::getYaw() const {
    return this->cur_yaw;
}

//TODO: make sure gimbalChannel is defined in config.json
void Gimbal::publishControlSignal(lcm::LCM & lcmObj, const rapidjson::Document& mRoverConfig) const {
    string channel = mRoverConfig[ "lcmChannels" ][ "gimbalChannel" ].GetString();
    this->calculateControlSignal();
    lcmObj.publish(channel, &(this->signal));
}

void Gimbal::calculateControlSignal(){
    this->signal = this->pid.update(this->getYaw(), this->target_yaw);
}
