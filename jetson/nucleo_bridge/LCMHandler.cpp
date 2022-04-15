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
    lcm_bus->subscribe("/mast_gimbal_cmd", &LCMHandler::InternalHandler::mast_gimbal_cmd, internal_object);
    lcm_bus->subscribe("/hand_openloop_cmd", &LCMHandler::InternalHandler::hand_openloop_cmd, internal_object);
    lcm_bus->subscribe("/foot_openloop_cmd", &LCMHandler::InternalHandler::foot_openloop_cmd, internal_object);
    lcm_bus->subscribe("/scoop_limit_switch_enable_cmd", &LCMHandler::InternalHandler::scoop_limit_switch_enable_cmd, internal_object);
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
    std::chrono::duration heartbeat_dead_time = std::chrono::milliseconds(120);

    // This is used as a heart beat (to make sure that nucleos do not reset, but honestly that's too difficult)
    if (NOW - last_heartbeat_output_time > heartbeat_dead_time)
    {
        internal_object->publish_ra_pos_data();
        internal_object->publish_sa_pos_data();
    }


    // TODO - UNCOMMENT AS SOON AS JOINT B LIMIT SWITCH / CALIBRATION IS READY
    /*
    std::chrono::duration calib_data_output_dead_time = std::chrono::milliseconds(1000);
    // This is used as a heart beat (to make sure that nucleos do not reset)
    if (NOW - last_calib_data_output_time > calib_data_output_dead_time)
    {
        internal_object->refresh_calib_data();
        last_calib_data_output_time = NOW;

        internal_object->publish_calib_data();
    }
    */

    // TODO - UNCOMMENT AS SOON AS TURN COUNT IS READY
    /*
    std::chrono::duration turn_count_output_dead_time = std::chrono::milliseconds(1000);
    // This is used as a heart beat (to make sure that nucleos do not reset)
    if (NOW - last_turn_count_output_time > turn_count_output_dead_time)
    {
        internal_object->refresh_turn_count();
        last_turn_count_output_time = NOW;

        internal_object->publish_turn_count();
    }
    */
}

void LCMHandler::InternalHandler::foot_openloop_cmd(LCM_INPUT, const FootCmd *msg)
{
    ControllerMap::controllers["FOOT_SCOOP"]->open_loop(msg->microscope_triad);
    ControllerMap::controllers["FOOT_SENSOR"]->open_loop(msg->scoop);
}

void LCMHandler::InternalHandler::hand_openloop_cmd(LCM_INPUT, const HandCmd *msg)
{
    ControllerMap::controllers["HAND_FINGER"]->open_loop(msg->finger);
    ControllerMap::controllers["HAND_GRIP"]->open_loop(msg->grip);
}

void LCMHandler::InternalHandler::mast_gimbal_cmd(LCM_INPUT, const MastGimbalCmd *msg)
{
    ControllerMap::controllers["GIMBAL_PITCH"]->open_loop(msg->pitch[0]);
    ControllerMap::controllers["GIMBAL_YAW"]->open_loop(msg->yaw[0]);
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

    last_heartbeat_output_time = NOW;
}

void LCMHandler::InternalHandler::ra_open_loop_cmd(LCM_INPUT, const RAOpenLoopCmd *msg)
{
    ControllerMap::controllers["RA_A"]->open_loop(msg->throttle[0]);
    ControllerMap::controllers["RA_B"]->open_loop(msg->throttle[1]);
    ControllerMap::controllers["RA_C"]->open_loop(msg->throttle[2]);
    ControllerMap::controllers["RA_D"]->open_loop(msg->throttle[3]);
    ControllerMap::controllers["RA_E"]->open_loop(msg->throttle[4]);
    ControllerMap::controllers["RA_F"]->open_loop(msg->throttle[5]);
    last_heartbeat_output_time = NOW;

    publish_ra_pos_data();
}

void LCMHandler::InternalHandler::refresh_ra_quad_angles()
{
    ControllerMap::controllers["RA_A"]->quad_angle();
    ControllerMap::controllers["RA_B"]->quad_angle();
    ControllerMap::controllers["RA_C"]->quad_angle();
    ControllerMap::controllers["RA_D"]->quad_angle();
    ControllerMap::controllers["RA_E"]->quad_angle();
    ControllerMap::controllers["RA_F"]->quad_angle();
    //last_heartbeat_output_time = NOW;
}

void LCMHandler::InternalHandler::refresh_sa_quad_angles()
{
    ControllerMap::controllers["SA_A"]->quad_angle();
    ControllerMap::controllers["SA_B"]->quad_angle();
    ControllerMap::controllers["SA_C"]->quad_angle();
    ControllerMap::controllers["SA_E"]->quad_angle();
    //last_heartbeat_output_time = NOW;
}

void LCMHandler::InternalHandler::refresh_calib_data()
{
    ControllerMap::controllers["RA_B"]->refresh_calibration_data();

    last_heartbeat_output_time = NOW;
}

void LCMHandler::InternalHandler::refresh_turn_count()
{
    ControllerMap::controllers["RA_F"]->refresh_turn_count();
    last_heartbeat_output_time = NOW;
}

void LCMHandler::InternalHandler::sa_closed_loop_cmd(LCM_INPUT, const SAPosition *msg)
{
    ControllerMap::controllers["SA_A"]->closed_loop(0, msg->joint_a);
    ControllerMap::controllers["SA_B"]->closed_loop(0, msg->joint_b);
    ControllerMap::controllers["SA_C"]->closed_loop(0, msg->joint_c);
    ControllerMap::controllers["SA_E"]->closed_loop(0, msg->joint_e);
    last_heartbeat_output_time = NOW;

    publish_sa_pos_data();
}

void LCMHandler::InternalHandler::sa_open_loop_cmd(LCM_INPUT, const SAOpenLoopCmd *msg)
{
    ControllerMap::controllers["SA_A"]->open_loop(msg->throttle[0]);
    ControllerMap::controllers["SA_B"]->open_loop(msg->throttle[1]);
    ControllerMap::controllers["SA_C"]->open_loop(msg->throttle[2]);
    ControllerMap::controllers["SA_E"]->open_loop(msg->throttle[3]);
    last_heartbeat_output_time = NOW;

    publish_sa_pos_data();
}

void LCMHandler::InternalHandler::scoop_limit_switch_enable_cmd(LCM_INPUT, const ScoopLimitSwitchEnable *msg)
{
    ControllerMap::controllers["FOOT_SCOOP"]->limit_switch_enable(msg->enable);
}

void LCMHandler::InternalHandler::publish_calib_data()
{
    JointBCalibration msg;
    msg.calibrated = ControllerMap::controllers["RA_B"]->calibrated;
    lcm_bus->publish("/joint_b_refresh_calibration_data", &msg);
}

void LCMHandler::InternalHandler::publish_ra_pos_data()
{
    RAPosition msg;
    msg.joint_a = ControllerMap::controllers["RA_A"]->get_current_angle();
    msg.joint_b = ControllerMap::controllers["RA_B"]->get_current_angle();
    msg.joint_c = ControllerMap::controllers["RA_C"]->get_current_angle();
    msg.joint_d = ControllerMap::controllers["RA_D"]->get_current_angle();
    msg.joint_e = ControllerMap::controllers["RA_E"]->get_current_angle();
    msg.joint_f = ControllerMap::controllers["RA_F"]->get_current_angle();
    lcm_bus->publish("/ra_position", &msg);
}

void LCMHandler::InternalHandler::publish_sa_pos_data()
{
    SAPosition msg;
    msg.joint_a = ControllerMap::controllers["SA_A"]->get_current_angle();
    msg.joint_b = ControllerMap::controllers["SA_B"]->get_current_angle();
    msg.joint_c = ControllerMap::controllers["SA_C"]->get_current_angle();
    msg.joint_e = ControllerMap::controllers["SA_E"]->get_current_angle();
    lcm_bus->publish("/sa_position", &msg);
}

void LCMHandler::InternalHandler::publish_turn_count()
{
    WristTurnCount msg;
    msg.turn_count = ControllerMap::controllers["RA_F"]->turn_count;
    lcm_bus->publish("/wrist_turn_count", &msg);
}
