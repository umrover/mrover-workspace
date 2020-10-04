
#include "rover_msgs/GimbalCmd.lcm"
#include "pid.hpp"
#include "rapidjson/document.h"

#include <lcm/lcm-cpp.hpp>

class Gimbal{
    private:
        double cur_yaw;
        double target_yaw;
        double yaw_command;
        double MAX_YAW, MIN_YAW;
        GimbalCmd signal;
        double TOLERANCE;
    public:
        Gimbal(const rapidjson::Document& mRoverConfig);
        //sets the target yaw of the gimbal
        bool setTargetYaw(double target);

        //returns the current yaw of the gimbal
        double getYaw() const;

        //returns the LCM message to be sent. Takes in a reference to the LCM object that sends it
        void publishControlSignal(lcm::LCM &lcmObj, const rapidjson::Document& mRoverConfig) const;
};