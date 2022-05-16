#ifndef LCMHANDLER_H
#define LCMHANDLER_H

#include "Controller.h"
#include "Hardware.h"

#include <unordered_map>
#include <cmath>
#include <string>
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <chrono>

#include <rover_msgs/CarouselOpenLoopCmd.hpp>
#include <rover_msgs/CarouselPosition.hpp>
#include <rover_msgs/FootCmd.hpp>
#include <rover_msgs/HandCmd.hpp>
#include <rover_msgs/Calibrate.hpp>
#include <rover_msgs/MastGimbalCmd.hpp>
#include <rover_msgs/RAOpenLoopCmd.hpp>
#include <rover_msgs/RAPosition.hpp>
#include <rover_msgs/SAOpenLoopCmd.hpp>
#include <rover_msgs/SAPosition.hpp>
#include <rover_msgs/ScoopLimitSwitchEnable.hpp>
#include <rover_msgs/Signal.hpp>

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
    inline static std::chrono::high_resolution_clock::time_point last_heartbeat_output_time = NOW;
    inline static std::chrono::high_resolution_clock::time_point last_calib_data_output_time = NOW;

    inline static lcm::LCM *lcm_bus = nullptr;

    //Empty object to pass to lcm subscribe
    class InternalHandler
    {
    public: 
    	//The following functions are handlers for the corresponding lcm messages

        void carousel_closed_loop_cmd(LCM_INPUT, const CarouselPosition *msg);

        void carousel_open_loop_cmd(LCM_INPUT, const CarouselOpenLoopCmd *msg);

        void carousel_zero_cmd(LCM_INPUT, const Signal *msg);

        void foot_open_loop_cmd(LCM_INPUT, const FootCmd *msg);

        void hand_open_loop_cmd(LCM_INPUT, const HandCmd *msg);

        void mast_gimbal_cmd(LCM_INPUT, const MastGimbalCmd *msg);

        void ra_closed_loop_cmd(LCM_INPUT, const RAPosition *msg);

        void ra_open_loop_cmd(LCM_INPUT, const RAOpenLoopCmd *msg);

        void refresh_carousel_calib_data();

        void refresh_carousel_quad_angles();

        void refresh_ra_calib_data();

        void refresh_ra_quad_angles();

        void refresh_sa_calib_data();

        void refresh_sa_quad_angles();

        void sa_closed_loop_cmd(LCM_INPUT, const SAPosition *msg);

        void sa_open_loop_cmd(LCM_INPUT, const SAOpenLoopCmd *msg);

        void scoop_limit_switch_enable_cmd(LCM_INPUT, const ScoopLimitSwitchEnable *msg);

        void publish_carousel_calib_data();

        void publish_carousel_pos_data();

        void publish_ra_calib_data();

        void publish_ra_pos_data();

        void publish_sa_calib_data();

        void publish_sa_pos_data();

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
