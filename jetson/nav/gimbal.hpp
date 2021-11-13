#ifndef GIMBAL_HPP
#define GIMBAL_HPP


#include "rover_msgs/ZedGimbalPosition.hpp"
#include "pid.hpp"
#include "rapidjson/document.h"

#include <lcm/lcm-cpp.hpp>


class Gimbal{
    private:
        double cur_yaw;
        double desired_gimbal_yaw;
        double TOLERANCE;
        rover_msgs::ZedGimbalPosition signal;
    public:
        Gimbal(double tolerance_in );
        //sets the desired yaw of the gimbal
        bool setDesiredGimbalYaw( double desired_yaw );

        //returns the current yaw of the gimbal
        double getYaw() const;

        //sets the current yaw of the gimbal (from sensor)
        void setCurrentYaw( double yaw );

        //returns the LCM message to be sent. Takes in a reference to the LCM object that sends it
        void publishControlSignal( lcm::LCM &lcmObj, const rapidjson::Document& mRoverConfig );
};

#endif // gimbal.hpp