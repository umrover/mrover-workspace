#ifndef LCMHANDLER_H
#define LCMHANDLER_H

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

//This class handles all LCM related functions - subscribing, handling, and publishing to channels
class LCMHandler
{
private:
    inline static std::chrono::high_resolution_clock::time_point last_output_time = NOW;

    inline static std::unordered_map<std::string, Controller *> *controllers = nullptr;

    inline static lcm::LCM lcm_bus = 0;

    //Empty object to pass to lcm subscribe
    inline static LCMHandler *internal_object = nullptr;

public:
    //Initialize the lcm bus and subscribe to relevant channels with message handlers defined below
    static void init(std::unordered_map<std::string, Controller *> *input1);

    //Handles a single incoming lcm message
    static void handle_incoming();

    //Decide whether outgoing messages need to be sent, and if so, send them
    static void handle_outgoing();

    //Empty object to pass to lcm subscribe
    LCMHandler();

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
