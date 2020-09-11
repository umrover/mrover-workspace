#include "LCMHandler.h"

void LCMHandler::init(std::unordered_map<std::string, Controller *> *input1) {
    //Creation of lcm bus
    lcm_bus = lcm::LCM();
    if (!lcm_bus.good())
    {
        printf("LCM Bus not created");
        exit(1);
    }
    
    controllers = input1;
    
    //Subscription to lcm channels 
    lcm_bus.subscribe("/ik_ra_control", &LCMHandler::ra_closed_loop_cmd, internal_object);
    lcm_bus.subscribe("/sa_closedloop_cmd", &LCMHandler::sa_closed_loop_cmd, internal_object);

    lcm_bus.subscribe("/ra_openloop_cmd", &LCMHandler::ra_open_loop_cmd, internal_object);
    lcm_bus.subscribe("/sa_openloop_cmd", &LCMHandler::sa_open_loop_cmd, internal_object);

    //lcmBus->subscribe("/ra_config_cmd", &LCMHandler::ra_config_cmd, &internal_object);
    lcm_bus.subscribe("/sa_config_cmd", &LCMHandler::sa_config_cmd, internal_object);

    lcm_bus.subscribe("/gimbal_openloop_cmd", &LCMHandler::gimbal_cmd, internal_object);
    lcm_bus.subscribe("/hand_openloop_cmd", &LCMHandler::hand_openloop_cmd, internal_object);
    lcm_bus.subscribe("/foot_openloop_cmd", &LCMHandler::foot_openloop_cmd, internal_object);

    //lcm_bus.subscribe("/ra_zero_trigger", &LCMHandler::ra_zero_trigger, &internal_object);
    lcm_bus.subscribe("/sa_zero_trigger", &LCMHandler::sa_zero_trigger, internal_object);
}

//Handles a single incoming lcm message    
void LCMHandler::handle_incoming()
{
    lcm_bus.handle();
}

//Decide whether outgoing messages need to be sent, and if so, send them
void LCMHandler::handle_outgoing()
{
    //If the last time arm position messages were outputted was over 100 ms ago, get new data from Controllers to be sent
    std::chrono::duration deadTime = std::chrono::milliseconds(100);
    if (NOW - last_output_time > deadTime)
    {
        internal_object->refreshAngles();
        internal_object->sa_pos_data();
        internal_object->ra_pos_data();
    }
}

//Empty object to pass to lcm subscribe
LCMHandler::LCMHandler() {};

//The following functions are handlers for the corresponding lcm messages
void LCMHandler::ra_closed_loop_cmd(LCM_INPUT, const ArmPosition *msg)
{
    //(*controllers)["RA_0"]->closed_loop(msg->torque[0], msg->angle[0]);
    //(*controllers)["RA_1"]->closed_loop(msg->joint_b);
    (*controllers)["RA_2"]->closed_loop(0, msg->joint_c);
    (*controllers)["RA_3"]->closed_loop(0, msg->joint_d);
    (*controllers)["RA_4"]->closed_loop(0, msg->joint_e);
    (*controllers)["RA_5"]->closed_loop(0, msg->joint_f);
    ra_pos_data();
}

void LCMHandler::sa_closed_loop_cmd(LCM_INPUT, const SAClosedLoopCmd *msg)
{
    (*controllers)["SA_0"]->closed_loop(msg->torque[0], msg->angle[0]);
    (*controllers)["SA_1"]->closed_loop(msg->torque[1], msg->angle[1]);
    (*controllers)["SA_2"]->closed_loop(msg->torque[2], msg->angle[2]);
    sa_pos_data();
}

void LCMHandler::ra_open_loop_cmd(LCM_INPUT, const RAOpenLoopCmd *msg)
{
    (*controllers)["RA_0"]->open_loop(msg->throttle[0]);
    (*controllers)["RA_1"]->open_loop(msg->throttle[1]);
    (*controllers)["RA_2"]->open_loop(msg->throttle[2]);
    (*controllers)["RA_3"]->open_loop(msg->throttle[3]);
    (*controllers)["RA_4"]->open_loop(msg->throttle[4]);
    (*controllers)["RA_5"]->open_loop(msg->throttle[5]);
    ra_pos_data();
}

void LCMHandler::sa_open_loop_cmd(LCM_INPUT, const SAOpenLoopCmd *msg)
{
    (*controllers)["SA_0"]->open_loop(msg->throttle[0]);
    (*controllers)["SA_1"]->open_loop(msg->throttle[1]);
    (*controllers)["SA_2"]->open_loop(msg->throttle[2]);
    sa_pos_data();
}

void LCMHandler::ra_config_cmd(LCM_INPUT, const RAConfigCmd *msg)
{
    /*(*controllers)["RA_0"]->config(msg->Kp[0], msg->Ki[0], msg->Kd[0]);
	    (*controllers)["RA_1"]->config(msg->Kp[1], msg->Ki[1], msg->Kd[1]);
	    (*controllers)["RA_2"]->config(msg->Kp[2], msg->Ki[2], msg->Kd[2]);
	    (*controllers)["RA_3"]->config(msg->Kp[3], msg->Ki[3], msg->Kd[3]);
	    (*controllers)["RA_4"]->config(msg->Kp[4], msg->Ki[4], msg->Kd[4]);
	    (*controllers)["RA_5"]->config(msg->Kp[5], msg->Ki[5], msg->Kd[5]);*/
}

void LCMHandler::sa_config_cmd(LCM_INPUT, const SAConfigCmd *msg)
{
    (*controllers)["SA_0"]->config(msg->Kp[0], msg->Ki[0], msg->Kd[0]);
    (*controllers)["SA_1"]->config(msg->Kp[1], msg->Ki[1], msg->Kd[1]);
    (*controllers)["SA_2"]->config(msg->Kp[2], msg->Ki[2], msg->Kd[2]);
}

void LCMHandler::hand_openloop_cmd(LCM_INPUT, const HandCmd *msg)
{
    (*controllers)["HAND_FINGER_POS"]->open_loop(msg->finger);
    (*controllers)["HAND_FINGER_NEG"]->open_loop(msg->finger);
    (*controllers)["HAND_GRIP_POS"]->open_loop(msg->grip);
    (*controllers)["HAND_GRIP_NEG"]->open_loop(msg->grip);
}

void LCMHandler::gimbal_cmd(LCM_INPUT, const GimbalCmd *msg)
{
    (*controllers)["GIMBAL_PITCH_0_POS"]->open_loop(msg->pitch[0]);
    (*controllers)["GIMBAL_PITCH_0_NEG"]->open_loop(msg->pitch[0]);
    (*controllers)["GIMBAL_PITCH_1_POS"]->open_loop(msg->pitch[1]);
    (*controllers)["GIMBAL_PITCH_1_NEG"]->open_loop(msg->pitch[1]);
    (*controllers)["GIMBAL_YAW_0_POS"]->open_loop(msg->yaw[0]);
    (*controllers)["GIMBAL_YAW_0_NEG"]->open_loop(msg->yaw[0]);
    (*controllers)["GIMBAL_YAW_1_POS"]->open_loop(msg->yaw[1]);
    (*controllers)["GIMBAL_YAW_1_NEG"]->open_loop(msg->yaw[1]);
}

void LCMHandler::refreshAngles()
{
    (*controllers)["RA_0"]->angle();
    (*controllers)["RA_1"]->angle();
    (*controllers)["RA_2"]->angle();
    (*controllers)["RA_3"]->angle();
    (*controllers)["RA_4"]->angle();
    (*controllers)["RA_5"]->angle();
    (*controllers)["SA_0"]->angle();
    (*controllers)["SA_1"]->angle();
    (*controllers)["SA_2"]->angle();
}

void LCMHandler::ra_pos_data()
{
    ArmPosition msg;
    msg.joint_a = 0; //angle[0] = (*controllers)["RA_0"]->currentAngle;
    msg.joint_b = 0; // (*controllers)["RA_1"]->currentAngle;
    msg.joint_c = (*controllers)["RA_2"]->currentAngle;
    msg.joint_d = (*controllers)["RA_3"]->currentAngle;
    msg.joint_e = (*controllers)["RA_4"]->currentAngle;
    msg.joint_f = (*controllers)["RA_5"]->currentAngle;
    lcm_bus.publish("/arm_position", &msg);

    last_output_time = NOW;
}

void LCMHandler::sa_pos_data()
{
    SAPosData msg;
    msg.angle[0] = (*controllers)["SA_0"]->currentAngle;
    msg.angle[1] = (*controllers)["SA_1"]->currentAngle;
    msg.angle[2] = (*controllers)["SA_2"]->currentAngle;
    lcm_bus.publish("/sa_pos_data", &msg);

    last_output_time = NOW;
}

void LCMHandler::sa_zero_trigger(LCM_INPUT, const SAZeroTrigger *msg)
{
    (*controllers)["SA_0"]->zero();
    (*controllers)["SA_1"]->zero();
    (*controllers)["SA_2"]->zero();
}

/*	void LCMHandler::ra_zero_trigger(LCM_INPUT, const RAZeroTrigger *msg){
		(*controllers)["RA_0"]->zero();
		(*controllers)["RA_1"]->zero();
		(*controllers)["RA_2"]->zero();
		(*controllers)["RA_3"]->zero();
		(*controllers)["RA_4"]->zero();
		(*controllers)["RA_5"]->zero();
	}*/

void LCMHandler::foot_openloop_cmd(LCM_INPUT, const FootCmd *msg)
{
    (*controllers)["FOOT_CLAW"]->open_loop(msg->claw);
    (*controllers)["FOOT_SENSOR"]->open_loop(msg->sensor);
}