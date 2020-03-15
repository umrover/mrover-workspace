#include <string>
#include <unordered_map>
#include <iostream>
#include <thread>
#include <chrono>

#include "lcm/lcm-cpp.hpp"
#include "controller.h"
#include "hardware.h"
#include "frontend.h"
#include "backend.h"

int main(){
    std::unordered_map<std::string, Controller *> controllers;
    
    BackEnd backEnd = BackEnd();

    controllers["HAND_FINGER_POS"] = new Controller(0, 2, Hardware(HBridgePos));
    controllers["HAND_FINGER_NEG"] = new Controller(0, 3, Hardware(HBridgeNeg));
    controllers["HAND_GRIP_POS"] = new Controller(0, 4, Hardware(HBridgePos));
    controllers["HAND_GRIP_NEG" ] = new Controller(0, 5, Hardware(HBridgeNeg));
    controllers["FOOT_CLAW"] = new Controller(2, 0, Hardware(Talon12V));
    controllers["FOOT_SENSOR"] = new Controller(1, 1, Hardware(Talon12V));
    controllers["GIMBAL_PITCH_0_POS"] = new Controller(1, 2, Hardware(HBridgePos));
    controllers["GIMBAL_PITCH_0_NEG"] = new Controller(1, 3, Hardware(HBridgeNeg));
    controllers["GIMBAL_PITCH_1_POS"] = new Controller(2, 2, Hardware(HBridgePos));
    controllers["GIMBAL_PITCH_1_NEG"] = new Controller(2, 3, Hardware(HBridgeNeg));
    controllers["GIMBAL_YAW_0_POS"] = new Controller(1, 4, Hardware(HBridgePos));
    controllers["GIMBAL_YAW_0_NEG"] = new Controller(1, 5, Hardware(HBridgeNeg));
    controllers["GIMBAL_YAW_1_POS"] = new Controller(2, 4, Hardware(HBridgePos));
    controllers["GIMBAL_YAW_1_NEG"] = new Controller(2, 5, Hardware(HBridgeNeg));
    controllers["SA_0"] = new Controller(0, 0, Hardware(Talon12V));
    controllers["SA_1"] = new Controller(0, 1, Hardware(Talon24V));
    controllers["SA_2"] = new Controller(1, 0, Hardware(Talon12V));
    controllers["RA_0"] = new Controller(0, 0, Hardware(Talon24V));
    controllers["RA_1"] = new Controller(0, 1, Hardware(Talon12V));
    controllers["RA_2"] = new Controller(1, 0, Hardware(Talon12V));
    controllers["RA_3"] = new Controller(1, 1, Hardware(Talon24V));
    controllers["RA_4"] = new Controller(2, 0, Hardware(Talon24V));
    controllers["RA_5"] = new Controller(2, 1, Hardware(Talon12V));

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

    for (auto element : controllers) {
        element.second->name = element.first;
        element.second->backEnd = &backEnd;
    }

    FrontEnd frontEnd = FrontEnd(&controllers);

    lcm::LCM lcmBus = lcm::LCM();

    if(!lcmBus.good()) {printf("LCM Bus not created"); exit(1);}

    lcmBus.subscribe("/ik_ra_control", &FrontEnd::ra_closed_loop_cmd, &frontEnd);
    lcmBus.subscribe("/sa_closedloop_cmd", &FrontEnd::sa_closed_loop_cmd, &frontEnd);

    lcmBus.subscribe("/ra_openloop_cmd", &FrontEnd::ra_open_loop_cmd, &frontEnd);
    lcmBus.subscribe("/sa_openloop_cmd", &FrontEnd::sa_open_loop_cmd, &frontEnd);

    //lcmBus.subscribe("/ra_config_cmd", &FrontEnd::ra_config_cmd, &frontEnd);
    lcmBus.subscribe("/sa_config_cmd", &FrontEnd::sa_config_cmd, &frontEnd);

    lcmBus.subscribe("/gimbal_openloop_cmd", &FrontEnd::gimbal_cmd, &frontEnd);
    lcmBus.subscribe("/hand_openloop_cmd", &FrontEnd::hand_openloop_cmd, &frontEnd);
    lcmBus.subscribe("/foot_openloop_cmd", &FrontEnd::foot_openloop_cmd, &frontEnd);

    //lcmBus.subscribe("/ra_zero_trigger", &FrontEnd::ra_zero_trigger, &frontEnd);
    lcmBus.subscribe("/sa_zero_trigger", &FrontEnd::sa_zero_trigger, &frontEnd);

    frontEnd.run(&lcmBus);
}
