#include "LCMHandler.h"

//Initialize the lcm bus and subscribe to relevant channels with message handlers defined below
void LCMHandler::init()
{
    //Creation of lcm bus
    lcm_bus = new lcm::LCM();
    if (!lcm_bus->good())
    {
        printf("LCM Bus not created\n");
        exit(1);
    } else {
	printf("LCM Bus created\n");
    }
    
    internal_object = new InternalHandler();
    
    //Subscription to lcm channels 
    lcm_bus->subscribe("/ik_ra_control",        &LCMHandler::InternalHandler::ra_closed_loop_cmd,   internal_object);
    lcm_bus->subscribe("/sa_closedloop_cmd",    &LCMHandler::InternalHandler::sa_closed_loop_cmd,   internal_object);
    lcm_bus->subscribe("/ra_openloop_cmd",      &LCMHandler::InternalHandler::ra_open_loop_cmd,     internal_object);
    lcm_bus->subscribe("/sa_openloop_cmd",      &LCMHandler::InternalHandler::sa_open_loop_cmd,     internal_object);
    lcm_bus->subscribe("/gimbal_openloop_cmd",  &LCMHandler::InternalHandler::gimbal_cmd,           internal_object);
    lcm_bus->subscribe("/hand_openloop_cmd",    &LCMHandler::InternalHandler::hand_openloop_cmd,    internal_object);
    lcm_bus->subscribe("/foot_openloop_cmd",    &LCMHandler::InternalHandler::foot_openloop_cmd,    internal_object);
    /*
    The following functions may be reimplemented when IK is tested
    lcmBus->subscribe("/ra_config_cmd",         &LCMHandler::InternalHandler::ra_config_cmd,        internal_object);
    lcm_bus.subscribe("/sa_config_cmd",         &LCMHandler::InternalHandler::sa_config_cmd,        internal_object);
    lcm_bus.subscribe("/ra_zero_trigger",       &LCMHandler::InternalHandler::ra_zero_trigger,      internal_object);
    lcm_bus.subscribe("/sa_zero_trigger",       &LCMHandler::InternalHandler::sa_zero_trigger,      internal_object);
    */
    printf("LCM Bus channels subscribed\n");
}

//Handles a single incoming lcm message    
void LCMHandler::handle_incoming()
{
    lcm_bus->handle();
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

//The following functions are handlers for the corresponding lcm messages
void LCMHandler::InternalHandler::ra_closed_loop_cmd(LCM_INPUT, const ArmPosition *msg)
{
    ControllerMap::controllers["RA_0"]->closed_loop(0, msg->joint_a);
    ControllerMap::controllers["RA_1"]->closed_loop(0, msg->joint_b);
    ControllerMap::controllers["RA_2"]->closed_loop(0, msg->joint_c);
    ControllerMap::controllers["RA_3"]->closed_loop(0, msg->joint_d);
    ControllerMap::controllers["RA_4"]->closed_loop(0, msg->joint_e);
    ControllerMap::controllers["RA_5"]->closed_loop(0, msg->joint_f);
    ra_pos_data();
}

void LCMHandler::InternalHandler::sa_closed_loop_cmd(LCM_INPUT, const SAClosedLoopCmd *msg)
{
    ControllerMap::controllers["SA_0"]->closed_loop(msg->torque[0], msg->angle[0]);
    ControllerMap::controllers["SA_1"]->closed_loop(msg->torque[1], msg->angle[1]);
    ControllerMap::controllers["SA_2"]->closed_loop(msg->torque[2], msg->angle[2]);
    sa_pos_data();
}

void LCMHandler::InternalHandler::ra_open_loop_cmd(LCM_INPUT, const RAOpenLoopCmd *msg)
{
    ControllerMap::controllers["RA_0"]->open_loop(msg->throttle[0]);
    ControllerMap::controllers["RA_1"]->open_loop(msg->throttle[1]);
    ControllerMap::controllers["RA_2"]->open_loop(msg->throttle[2]);
    ControllerMap::controllers["RA_3"]->open_loop(msg->throttle[3]);
    ControllerMap::controllers["RA_4"]->open_loop(msg->throttle[4]);
    ControllerMap::controllers["RA_5"]->open_loop(msg->throttle[5]);
    ra_pos_data();
}

void LCMHandler::InternalHandler::sa_open_loop_cmd(LCM_INPUT, const SAOpenLoopCmd *msg)
{
    ControllerMap::controllers["SA_0"]->open_loop(msg->throttle[0]);
    ControllerMap::controllers["SA_1"]->open_loop(msg->throttle[1]);
    ControllerMap::controllers["SA_2"]->open_loop(msg->throttle[2]);
    sa_pos_data();
}

/*
The following functions may be reimplemented when IK is tested
void LCMHandler::InternalHandler::ra_config_cmd(LCM_INPUT, const RAConfigCmd *msg)
{
    ControllerMap::controllers["RA_0"]->config(msg->Kp[0], msg->Ki[0], msg->Kd[0]);
    ControllerMap::controllers["RA_1"]->config(msg->Kp[1], msg->Ki[1], msg->Kd[1]);
    ControllerMap::controllers["RA_2"]->config(msg->Kp[2], msg->Ki[2], msg->Kd[2]);
    ControllerMap::controllers["RA_3"]->config(msg->Kp[3], msg->Ki[3], msg->Kd[3]);
    ControllerMap::controllers["RA_4"]->config(msg->Kp[4], msg->Ki[4], msg->Kd[4]);
    ControllerMap::controllers["RA_5"]->config(msg->Kp[5], msg->Ki[5], msg->Kd[5]);
}

void LCMHandler::InternalHandler::sa_config_cmd(LCM_INPUT, const SAConfigCmd *msg)
{
    ControllerMap::controllers["SA_0"]->config(msg->Kp[0], msg->Ki[0], msg->Kd[0]);
    ControllerMap::controllers["SA_1"]->config(msg->Kp[1], msg->Ki[1], msg->Kd[1]);
    ControllerMap::controllers["SA_2"]->config(msg->Kp[2], msg->Ki[2], msg->Kd[2]);
}
*/

void LCMHandler::InternalHandler::hand_openloop_cmd(LCM_INPUT, const HandCmd *msg)
{
    ControllerMap::controllers["HAND_FINGER_POS"]->open_loop(msg->finger);
    ControllerMap::controllers["HAND_FINGER_NEG"]->open_loop(msg->finger);
    ControllerMap::controllers["HAND_GRIP_POS"]->open_loop(msg->grip);
    ControllerMap::controllers["HAND_GRIP_NEG"]->open_loop(msg->grip);
}

void LCMHandler::InternalHandler::gimbal_cmd(LCM_INPUT, const GimbalCmd *msg)
{
    ControllerMap::controllers["GIMBAL_PITCH_0_POS"]->open_loop(msg->pitch[0]);
    ControllerMap::controllers["GIMBAL_PITCH_0_NEG"]->open_loop(msg->pitch[0]);
    ControllerMap::controllers["GIMBAL_PITCH_1_POS"]->open_loop(msg->pitch[1]);
    ControllerMap::controllers["GIMBAL_PITCH_1_NEG"]->open_loop(msg->pitch[1]);
    ControllerMap::controllers["GIMBAL_YAW_0_POS"]->open_loop(msg->yaw[0]);
    ControllerMap::controllers["GIMBAL_YAW_0_NEG"]->open_loop(msg->yaw[0]);
    ControllerMap::controllers["GIMBAL_YAW_1_POS"]->open_loop(msg->yaw[1]);
    ControllerMap::controllers["GIMBAL_YAW_1_NEG"]->open_loop(msg->yaw[1]);
}

void LCMHandler::InternalHandler::refreshAngles()
{
    ControllerMap::controllers["RA_0"]->angle();
    ControllerMap::controllers["RA_1"]->angle();
    ControllerMap::controllers["RA_2"]->angle();
    ControllerMap::controllers["RA_3"]->angle();
    ControllerMap::controllers["RA_4"]->angle();
    ControllerMap::controllers["RA_5"]->angle();
    ControllerMap::controllers["SA_0"]->angle();
    ControllerMap::controllers["SA_1"]->angle();
    ControllerMap::controllers["SA_2"]->angle();
}

void LCMHandler::InternalHandler::ra_pos_data()
{
    ArmPosition msg;
    msg.joint_a = ControllerMap::controllers["RA_0"]->current_angle;
    msg.joint_b = ControllerMap::controllers["RA_1"]->current_angle;
    msg.joint_c = ControllerMap::controllers["RA_2"]->current_angle;
    msg.joint_d = ControllerMap::controllers["RA_3"]->current_angle;
    msg.joint_e = ControllerMap::controllers["RA_4"]->current_angle;
    msg.joint_f = ControllerMap::controllers["RA_5"]->current_angle;
    lcm_bus->publish("/arm_position", &msg);

    last_output_time = NOW;
}

void LCMHandler::InternalHandler::sa_pos_data()
{
    SAPosData msg;
    msg.angle[0] = ControllerMap::controllers["SA_0"]->current_angle;
    msg.angle[1] = ControllerMap::controllers["SA_1"]->current_angle;
    msg.angle[2] = ControllerMap::controllers["SA_2"]->current_angle;
    lcm_bus->publish("/sa_pos_data", &msg);

    last_output_time = NOW;
}


/*
The following functions may be reimplemented when IK is tested
void LCMHandler::InternalHandler::sa_zero_trigger(LCM_INPUT, const SAZeroTrigger *msg)
{
    ControllerMap::controllers["SA_0"]->zero();
    ControllerMap::controllers["SA_1"]->zero();
    ControllerMap::controllers["SA_2"]->zero();
}

void LCMHandler::ra_zero_trigger(LCM_INPUT, const RAZeroTrigger *msg)
{
    ControllerMap::controllers["RA_0"]->zero();
    ControllerMap::controllers["RA_1"]->zero();
 	ControllerMap::controllers["RA_2"]->zero();
 	ControllerMap::controllers["RA_3"]->zero();
 	ControllerMap::controllers["RA_4"]->zero();
 	ControllerMap::controllers["RA_5"]->zero();
}
*/

void LCMHandler::InternalHandler::foot_openloop_cmd(LCM_INPUT, const FootCmd *msg)
{
    ControllerMap::controllers["FOOT_CLAW"]->open_loop(msg->claw);
    ControllerMap::controllers["FOOT_SENSOR"]->open_loop(msg->sensor);
}
