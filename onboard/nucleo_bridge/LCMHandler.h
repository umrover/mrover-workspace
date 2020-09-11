#ifndef LCMHANDLER_H
#define LCMHANDLER_H

//#define MULTITHREADING

#include "controller.h"
#include "hardware.h"

#include <unordered_map>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <chrono>

#include <rover_msgs/RAOpenLoopCmd.hpp>
#include <rover_msgs/SAOpenLoopCmd.hpp>
#include <rover_msgs/ArmPosition.hpp>
#include <rover_msgs/SAClosedLoopCmd.hpp>
#include <rover_msgs/RAConfigCmd.hpp>
#include <rover_msgs/SAConfigCmd.hpp>
#include <rover_msgs/HandCmd.hpp>
#include <rover_msgs/GimbalCmd.hpp>
#include <rover_msgs/RAPosData.hpp>
#include <rover_msgs/SAPosData.hpp>
//#include <rover_msgs/RAZeroTrigger.hpp>
#include <rover_msgs/SAZeroTrigger.hpp>
#include <rover_msgs/FootCmd.hpp>
#include <rover_msgs/ArmPosition.hpp>

#define LCM_INPUT const lcm::ReceiveBuffer *receiveBuffer, const std::string &channel
#define NOW std::chrono::high_resolution_clock::now()
using namespace rover_msgs;

class LCMHandler
{
private:
    std::unordered_map<std::string, Controller *> *controllers;
    std::chrono::high_resolution_clock::time_point lastTime = NOW;

    lcm::LCM *lcmBus;

    //Publish RA/SA position data as it arrives, or at least every 100 milliseconds
    void outgoing();

    //Handle new commands as they arrive
    void incoming();

public:
    LCMHandler(std::unordered_map<std::string, Controller *> *input) : controllers(input) {}

    //Entry point. Creates two threads, one to run incoming() and one to run outgoing()
    void run(lcm::LCM *in);

    //The following functions are handlers for the corresponding lcm messages
    void ra_closed_loop_cmd(LCM_INPUT, const ArmPosition *msg);

    void sa_closed_loop_cmd(LCM_INPUT, const SAClosedLoopCmd *msg);

    void ra_open_loop_cmd(LCM_INPUT, const RAOpenLoopCmd *msg);

    void sa_open_loop_cmd(LCM_INPUT, const SAOpenLoopCmd *msg);

    void ra_config_cmd(LCM_INPUT, const RAConfigCmd *msg);

    void sa_config_cmd(LCM_INPUT, const SAConfigCmd *msg);

    void hand_openloop_cmd(LCM_INPUT, const HandCmd *msg);

    void gimbal_cmd(LCM_INPUT, const GimbalCmd *msg);

    void refreshAngles();

    void ra_pos_data();

    void sa_pos_data();

    void sa_zero_trigger(LCM_INPUT, const SAZeroTrigger *msg);

    //void ra_zero_trigger(LCM_INPUT, const RAZeroTrigger *msg);
    void foot_openloop_cmd(LCM_INPUT, const FootCmd *msg);
};

#endif
