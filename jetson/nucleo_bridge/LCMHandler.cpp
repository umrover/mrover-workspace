#include "LCMHandler.h"

// Initialize the lcm bus and subscribe to relevant channels with message handlers defined below
void LCMHandler::init()
{
    // Creation of lcm bus
    lcm_bus = new lcm::LCM();
    if (!lcm_bus->good())
    {
        printf("LCM Bus not created\n");
        exit(1);
    }
    else
    {
        printf("LCM Bus created\n");
    }

    internal_object = new InternalHandler();

    // Subscription to lcm channels
    lcm_bus->subscribe("/ra_ik_cmd", &LCMHandler::InternalHandler::ra_closed_loop_cmd, internal_object);
    lcm_bus->subscribe("/sa_ik_cmd", &LCMHandler::InternalHandler::sa_closed_loop_cmd, internal_object);
    lcm_bus->subscribe("/ra_openloop_cmd", &LCMHandler::InternalHandler::ra_open_loop_cmd, internal_object);
    lcm_bus->subscribe("/sa_openloop_cmd", &LCMHandler::InternalHandler::sa_open_loop_cmd, internal_object);
    lcm_bus->subscribe("/mast_gimbal_cmd", &LCMHandler::InternalHandler::gimbal_cmd, internal_object);
    lcm_bus->subscribe("/hand_openloop_cmd", &LCMHandler::InternalHandler::hand_openloop_cmd, internal_object);
    lcm_bus->subscribe("/foot_openloop_cmd", &LCMHandler::InternalHandler::foot_openloop_cmd, internal_object);
    lcm_bus->subscribe("/scoop_limit_switch_enable_cmd", &LCMHandler::InternalHandler::scoop_limit_switch_enable_cmd, internal_object);
    /*
    The following functions may be reimplemented when IK is tested
    lcmBus->subscribe("/ra_config_cmd",         &LCMHandler::InternalHandler::ra_config_cmd,        internal_object);
    lcm_bus.subscribe("/sa_config_cmd",         &LCMHandler::InternalHandler::sa_config_cmd,        internal_object);
    lcm_bus.subscribe("/ra_zero_trigger",       &LCMHandler::InternalHandler::ra_zero_trigger,      internal_object);
    lcm_bus.subscribe("/sa_zero_trigger",       &LCMHandler::InternalHandler::sa_zero_trigger,      internal_object);
    */
    printf("LCM Bus channels subscribed\n");
}

// Handles a single incoming lcm message
void LCMHandler::handle_incoming()
{
    lcm_bus->handle();
}

// Decide whether outgoing messages need to be sent, and if so, send them
void LCMHandler::handle_outgoing()
{
    // If the last time arm position messages were outputted was over 200 ms ago, get new data from Controllers to be sent
    std::chrono::duration deadTime = std::chrono::milliseconds(200);
    if (NOW - last_output_time > deadTime)
    {
        internal_object->refreshAngles();
        internal_object->refresh_calib_data();
        // internal_object->refresh_turn_count();
        internal_object->publish_sa_pos_data();
        internal_object->publish_ra_pos_data();
        internal_object->joint_b_calib_data();
        // internal_object->publish_turn_count();
    }
}

void LCMHandler::InternalHandler::foot_openloop_cmd(LCM_INPUT, const FootCmd *msg)
{
    ControllerMap::controllers["FOOT_SCOOP"]->open_loop(msg->microscope_triad);
    ControllerMap::controllers["FOOT_SENSOR"]->open_loop(msg->scoop);
}

void LCMHandler::InternalHandler::mast_gimbal_cmd(LCM_INPUT, const MastGimbalCmd *msg)
{
    ControllerMap::controllers["GIMBAL_PITCH"]->open_loop(msg->pitch[0]);
    ControllerMap::controllers["GIMBAL_YAW"]->open_loop(msg->yaw[0]);
}

void LCMHandler::InternalHandler::hand_openloop_cmd(LCM_INPUT, const HandCmd *msg)
{
    ControllerMap::controllers["HAND_FINGER"]->open_loop(msg->finger);
    ControllerMap::controllers["HAND_GRIP"]->open_loop(msg->grip);
}

void LCMHandler::InternalHandler::joint_b_calib_data()
{
    JointBCalibration msg;
    msg.calibrated = ControllerMap::controllers["RA_B"]->calibrated;
    lcm_bus->publish("/joint_b_calibration_data", &msg);

    last_output_time = NOW;
}

// The following functions are handlers for the corresponding lcm messages
void LCMHandler::InternalHandler::ra_closed_loop_cmd(LCM_INPUT, const RAPosition *msg)
{
    ControllerMap::controllers["RA_A"]->closed_loop(0, msg->joint_a);
    ControllerMap::controllers["RA_B"]->closed_loop(0, msg->joint_b);
    ControllerMap::controllers["RA_C"]->closed_loop(0, msg->joint_c);
    ControllerMap::controllers["RA_D"]->closed_loop(0, msg->joint_d);
    ControllerMap::controllers["RA_E"]->closed_loop(0, msg->joint_e);
    ControllerMap::controllers["RA_F"]->closed_loop(0, msg->joint_f);
    publish_ra_pos_data();
}

void LCMHandler::InternalHandler::ra_open_loop_cmd(LCM_INPUT, const RAOpenLoopCmd *msg)
{
    ControllerMap::controllers["RA_A"]->open_loop(msg->throttle[0]);
    ControllerMap::controllers["RA_B"]->open_loop(msg->throttle[1]);
    ControllerMap::controllers["RA_C"]->open_loop(msg->throttle[2]);
    ControllerMap::controllers["RA_D"]->open_loop(msg->throttle[3]);
    ControllerMap::controllers["RA_E"]->open_loop(msg->throttle[4]);
    ControllerMap::controllers["RA_F"]->open_loop(msg->throttle[5]);
    publish_ra_pos_data();
}


void LCMHandler::InternalHandler::ra_pos_data()
{
    RAPosition msg;
    msg.joint_a = ControllerMap::controllers["RA_A"]->current_angle;
    msg.joint_b = ControllerMap::controllers["RA_B"]->current_angle;
    msg.joint_c = ControllerMap::controllers["RA_C"]->current_angle;
    msg.joint_d = ControllerMap::controllers["RA_D"]->current_angle;
    msg.joint_e = ControllerMap::controllers["RA_E"]->current_angle;
    msg.joint_f = ControllerMap::controllers["RA_F"]->current_angle;
    lcm_bus->publish("/ra_position", &msg);

    last_output_time = NOW;
}

void LCMHandler::InternalHandler::refreshAngles()
{
    ControllerMap::controllers["RA_A"]->angle();
    ControllerMap::controllers["RA_B"]->angle();
    ControllerMap::controllers["RA_C"]->angle();
    ControllerMap::controllers["RA_D"]->angle();
    ControllerMap::controllers["RA_E"]->angle();
    ControllerMap::controllers["RA_F"]->angle();
    ControllerMap::controllers["SA_A"]->angle();
    ControllerMap::controllers["SA_B"]->angle();
    ControllerMap::controllers["SA_C"]->angle();
    ControllerMap::controllers["SA_E"]->angle();
}

void LCMHandler::InternalHandler::refresh_calib_data()
{
    ControllerMap::controllers["RA_B"]->calibration_data();
}

void LCMHandler::InternalHandler::refresh_turn_count_data()
{
    ControllerMap::controllers["RA_F"]->turn_count_data();
}

void LCMHandler::InternalHandler::sa_closed_loop_cmd(LCM_INPUT, const SAPosition *msg)
{
    ControllerMap::controllers["SA_A"]->closed_loop(0, msg->joint_a);
    ControllerMap::controllers["SA_B"]->closed_loop(0, msg->joint_b);
    ControllerMap::controllers["SA_C"]->closed_loop(0, msg->joint_c);
    ControllerMap::controllers["SA_E"]->closed_loop(0, msg->joint_e);
    publish_sa_pos_data();
}

void LCMHandler::InternalHandler::sa_open_loop_cmd(LCM_INPUT, const SAOpenLoopCmd *msg)
{
    ControllerMap::controllers["SA_A"]->open_loop(msg->throttle[0]);
    ControllerMap::controllers["SA_B"]->open_loop(msg->throttle[1]);
    ControllerMap::controllers["SA_C"]->open_loop(msg->throttle[2]);
    ControllerMap::controllers["SA_E"]->open_loop(msg->throttle[3]);
    publish_sa_pos_data();
}

void LCMHandler::InternalHandler::scoop_limit_switch_enable_cmd(LCM_INPUT, const ScoopLimitSwitchEnable *msg)
{
    ControllerMap::controllers["FOOT_SCOOP"]->limit_switch_enable(msg->enable);
}

/*
The following functions may be reimplemented when IK is tested
void LCMHandler::InternalHandler::ra_config_cmd(LCM_INPUT, const RAConfigCmd *msg)
{
    ControllerMap::controllers["RA_A"]->config(msg->Kp[0], msg->Ki[0], msg->Kd[0]);
    ControllerMap::controllers["RA_B"]->config(msg->Kp[1], msg->Ki[1], msg->Kd[1]);
    ControllerMap::controllers["RA_C"]->config(msg->Kp[2], msg->Ki[2], msg->Kd[2]);
    ControllerMap::controllers["RA_D"]->config(msg->Kp[3], msg->Ki[3], msg->Kd[3]);
    ControllerMap::controllers["RA_E"]->config(msg->Kp[4], msg->Ki[4], msg->Kd[4]);
    ControllerMap::controllers["RA_F"]->config(msg->Kp[5], msg->Ki[5], msg->Kd[5]);
}

void LCMHandler::InternalHandler::sa_config_cmd(LCM_INPUT, const SAConfigCmd *msg)
{
    ControllerMap::controllers["SA_A"]->config(msg->Kp[0], msg->Ki[0], msg->Kd[0]);
    ControllerMap::controllers["SA_B"]->config(msg->Kp[1], msg->Ki[1], msg->Kd[1]);
    ControllerMap::controllers["SA_C"]->config(msg->Kp[2], msg->Ki[2], msg->Kd[2]);
    ControllerMap::controllers["SA_E"]->config(msg->Kp[2], msg->Ki[2], msg->Kd[2]);
}
*/

void LCMHandler::InternalHandler::sa_pos_data()
{
    SAPosition msg;
    msg.joint_a = ControllerMap::controllers["SA_A"]->current_angle;
    msg.joint_b = ControllerMap::controllers["SA_B"]->current_angle;
    msg.joint_c = ControllerMap::controllers["SA_C"]->current_angle;
    msg.joint_e = ControllerMap::controllers["SA_E"]->current_angle;
    lcm_bus->publish("/sa_position", &msg);

    last_output_time = NOW;
}

void LCMHandler::InternalHandler::publish_turn_count()
{
    WristTurnCount msg;
    msg.turn_count = ControllerMap::controllers["RA_F"]->turn_count;
    lcm_bus->publish("/wrist_turn_count", &msg);

    last_output_time = NOW;
}

/*
The following functions may be reimplemented when IK is tested
void LCMHandler::InternalHandler::sa_zero_trigger(LCM_INPUT, const SAZeroTrigger *msg)
{
    ControllerMap::controllers["SA_A"]->zero();
    ControllerMap::controllers["SA_B"]->zero();
    ControllerMap::controllers["SA_C"]->zero();
    ControllerMap::controllers["SA_E"]->zero();
}

void LCMHandler::ra_zero_trigger(LCM_INPUT, const RAZeroTrigger *msg)
{
    ControllerMap::controllers["RA_A"]->zero();
    ControllerMap::controllers["RA_B"]->zero();
    ControllerMap::controllers["RA_C"]->zero();
    ControllerMap::controllers["RA_D"]->zero();
    ControllerMap::controllers["RA_E"]->zero();
    ControllerMap::controllers["RA_F"]->zero();
}
*/
