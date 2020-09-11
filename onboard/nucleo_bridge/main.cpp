#include <string>
#include <unordered_map>
#include <iostream>
#include <thread>
#include <chrono>

#include "lcm/lcm-cpp.hpp"
#include "Controller.h"
#include "hardware.h"
#include "LCMHandler.h"
#include "I2C.h"

//Handles instantiation of Controller objects, FrontEnd, and BackEnd classes
int main()
{
    I2C::init();
    ControllerMap::init();

    //Named map of Controller objects
    //Configuration done here for visibility
    std::unordered_map<std::string, Controller *> controllers;

    controllers["HAND_FINGER_POS"] = new Controller(Hardware(HBridgePos));
    controllers["HAND_FINGER_NEG"] = new Controller(Hardware(HBridgeNeg));
    controllers["HAND_GRIP_POS"] = new Controller(Hardware(HBridgePos));
    controllers["HAND_GRIP_NEG"] = new Controller(Hardware(HBridgeNeg));
    controllers["FOOT_CLAW"] = new Controller(Hardware(Talon12V));
    controllers["FOOT_SENSOR"] = new Controller(Hardware(Talon12V));
    controllers["GIMBAL_PITCH_0_POS"] = new Controller(Hardware(HBridgePos));
    controllers["GIMBAL_PITCH_0_NEG"] = new Controller(Hardware(HBridgeNeg));
    controllers["GIMBAL_PITCH_1_POS"] = new Controller(Hardware(HBridgePos));
    controllers["GIMBAL_PITCH_1_NEG"] = new Controller(Hardware(HBridgeNeg));
    controllers["GIMBAL_YAW_0_POS"] = new Controller(Hardware(HBridgePos));
    controllers["GIMBAL_YAW_0_NEG"] = new Controller(Hardware(HBridgeNeg));
    controllers["GIMBAL_YAW_1_POS"] = new Controller(Hardware(HBridgePos));
    controllers["GIMBAL_YAW_1_NEG"] = new Controller(Hardware(HBridgeNeg));
    controllers["SA_0"] = new Controller(Hardware(Talon12V));
    controllers["SA_1"] = new Controller(Hardware(Talon24V));
    controllers["SA_2"] = new Controller(Hardware(Talon12V));
    controllers["RA_0"] = new Controller(Hardware(Talon24V));
    controllers["RA_1"] = new Controller(Hardware(Talon12V));
    controllers["RA_2"] = new Controller(Hardware(Talon12V));
    controllers["RA_3"] = new Controller(Hardware(Talon24V));
    controllers["RA_4"] = new Controller(Hardware(Talon24V));
    controllers["RA_5"] = new Controller(Hardware(Talon12V));

    controllers["SA_0"]->quadCPR = 464.64;
    controllers["SA_1"]->quadCPR = 23945.84;
    controllers["SA_2"]->quadCPR = 23945.84;

    controllers["RA_0"]->quadCPR = 100;
    controllers["RA_1"]->quadCPR = 1;
    controllers["RA_2"]->quadCPR = 342506.67;
    controllers["RA_3"]->quadCPR = 180266.67;
    controllers["RA_4"]->quadCPR = 180266.67;
    controllers["RA_5"]->quadCPR = 18144;

    controllers["RA_0"]->kP = 0;
    controllers["RA_1"]->kP = 0;
    controllers["RA_2"]->kP = 0.001;
    controllers["RA_3"]->kP = 0.001;
    controllers["RA_4"]->kP = -0.001;
    controllers["RA_5"]->kP = 0.005;

    //controllers["RA_1"]->kI = 0.0002;
    controllers["RA_2"]->kI = 0.00005;
    controllers["RA_3"]->kI = 0.00005;
    controllers["RA_4"]->kI = -0.00005;
    controllers["RA_5"]->kI = 0.00005;

    //Passes Controller to every controller
    for (auto element : controllers)
    {
        element.second->name = element.first;
    }

    //Passes Controller Map to LCMHandler
    LCMHandler lcm_handler = LCMHandler(&controllers);

    //Creation of lcm bus
    lcm::LCM lcmBus = lcm::LCM();
    if (!lcmBus.good())
    {
        printf("LCM Bus not created");
        exit(1);
    }

    //Subscription to lcm channels 
    lcmBus.subscribe("/ik_ra_control", &LCMHandler::ra_closed_loop_cmd, &lcm_handler);
    lcmBus.subscribe("/sa_closedloop_cmd", &LCMHandler::sa_closed_loop_cmd, &lcm_handler);

    lcmBus.subscribe("/ra_openloop_cmd", &LCMHandler::ra_open_loop_cmd, &lcm_handler);
    lcmBus.subscribe("/sa_openloop_cmd", &LCMHandler::sa_open_loop_cmd, &lcm_handler);

    //lcmBus.subscribe("/ra_config_cmd", &LCMHandler::ra_config_cmd, &lcm_handler);
    lcmBus.subscribe("/sa_config_cmd", &LCMHandler::sa_config_cmd, &lcm_handler);

    lcmBus.subscribe("/gimbal_openloop_cmd", &LCMHandler::gimbal_cmd, &lcm_handler);
    lcmBus.subscribe("/hand_openloop_cmd", &LCMHandler::hand_openloop_cmd, &lcm_handler);
    lcmBus.subscribe("/foot_openloop_cmd", &LCMHandler::foot_openloop_cmd, &lcm_handler);

    //lcmBus.subscribe("/ra_zero_trigger", &LCMHandler::ra_zero_trigger, &lcm_handler);
    lcmBus.subscribe("/sa_zero_trigger", &LCMHandler::sa_zero_trigger, &lcm_handler);

    //Pass flow control to LCMHandler
    lcm_handler.run(&lcmBus);
}
