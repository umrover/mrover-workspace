#ifndef LCMHANDLER_H
#define LCMHANDLER_H

#include "Controller.h"
#include "Hardware.h"

#include <unordered_map>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <chrono>

#include <rover_msgs/HandCmd.hpp>
#include <rover_msgs/FootCmd.hpp>
#include <rover_msgs/JointBCalibration.hpp>
#include <rover_msgs/MastGimbalCmd.hpp>
// #include <rover_msgs/RAConfigCmd.hpp>
#include <rover_msgs/RAOpenLoopCmd.hpp>
#include <rover_msgs/RAPosition.hpp>
// #include <rover_msgs/SAConfigCmd.hpp>
#include <rover_msgs/SAOpenLoopCmd.hpp>
#include <rover_msgs/SAPosition.hpp>
// #include <rover_msgs/SAZeroTrigger.hpp>
#include <rover_msgs/ScoopLimitSwitchEnable.hpp>
#include <rover_msgs/WristTurnCount.hpp>

#define LCM_INPUT const lcm::ReceiveBuffer *receiveBuffer, const std::string &channel
#define NOW std::chrono::high_resolution_clock::now()
using namespace rover_msgs;

/*
LCMHandler.h is responsible for handling incoming and outgoing lcm messages.
Incoming lcm messages will trigger functions which call the functions on the appropriate virtual Controllers. 
Outgoing lcm messages are triggered by a clock, which query the functions on the appropriate virtual Controllers for data.
*/
class LCMHandler
{
private:
    inline static std::chrono::high_resolution_clock::time_point last_output_time = NOW;

    inline static lcm::LCM *lcm_bus = nullptr;

    
    //Empty object to pass to lcm subscribe
    class InternalHandler
    {
    public: 
    	//The following functions are handlers for the corresponding lcm messages
        void foot_openloop_cmd(LCM_INPUT, const FootCmd *msg);

        void hand_openloop_cmd(LCM_INPUT, const HandCmd *msg);

        void mast_gimbal_cmd(LCM_INPUT, const MastGimbalCmd *msg);

        void ra_closed_loop_cmd(LCM_INPUT, const RAPosition *msg);

        void ra_open_loop_cmd(LCM_INPUT, const RAOpenLoopCmd *msg);

        void refresh_quad_angles();

        void refresh_calib_data();

        void refresh_turn_count_data();

        void sa_closed_loop_cmd(LCM_INPUT, const SAPosition *msg);

        void sa_open_loop_cmd(LCM_INPUT, const SAOpenLoopCmd *msg);

        void publish_calib_data();

        void publish_ra_pos_data();

        void publish_sa_pos_data();

        void publish_turn_count();

        void scoop_limit_switch_enable_cmd(LCM_INPUT, const ScoopLimitSwitchEnable *msg);

    };

    inline static InternalHandler *internal_object = nullptr;

public:
    //Initialize the lcm bus and subscribe to relevant channels with message handlers defined below
    static void init();

    //Handles a single incoming lcm message
    static void handle_incoming();

    //Decide whether outgoing messages need to be sent, and if so, send them
    static void handle_outgoing();
};

#endif
