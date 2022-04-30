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
<<<<<<< HEAD
    lcm_bus->subscribe("/ra_ik_cmd", &LCMHandler::InternalHandler::ra_closed_loop_cmd, internal_object);
    lcm_bus->subscribe("/sa_ik_cmd", &LCMHandler::InternalHandler::sa_closed_loop_cmd, internal_object);
    lcm_bus->subscribe("/carousel_openloop_cmd", &LCMHandler::InternalHandler::carousel_openloop_cmd, internal_object);
=======
    lcm_bus->subscribe("/carousel_closedloop_cmd", &LCMHandler::InternalHandler::carousel_closedloop_cmd, internal_object);
    lcm_bus->subscribe("/carousel_openloop_cmd", &LCMHandler::InternalHandler::carousel_openloop_cmd, internal_object);
    lcm_bus->subscribe("/carousel_zero_cmd", &LCMHandler::InternalHandler::carousel_zero_cmd, internal_object);
    lcm_bus->subscribe("/foot_openloop_cmd", &LCMHandler::InternalHandler::foot_openloop_cmd, internal_object);
    lcm_bus->subscribe("/hand_openloop_cmd", &LCMHandler::InternalHandler::hand_openloop_cmd, internal_object);
    lcm_bus->subscribe("/mast_gimbal_cmd", &LCMHandler::InternalHandler::mast_gimbal_cmd, internal_object);
    lcm_bus->subscribe("/ra_ik_cmd", &LCMHandler::InternalHandler::ra_closed_loop_cmd, internal_object);
>>>>>>> upstream/travis-free
    lcm_bus->subscribe("/ra_openloop_cmd", &LCMHandler::InternalHandler::ra_open_loop_cmd, internal_object);
    lcm_bus->subscribe("/sa_ik_cmd", &LCMHandler::InternalHandler::sa_closed_loop_cmd, internal_object);
    lcm_bus->subscribe("/sa_openloop_cmd", &LCMHandler::InternalHandler::sa_open_loop_cmd, internal_object);
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
        internal_object->refresh_ra_quad_angles();
        internal_object->refresh_sa_quad_angles();
        internal_object->refresh_carousel_quad_angles();
    }


<<<<<<< HEAD
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
}
=======
    std::chrono::duration calib_data_output_dead_time = std::chrono::milliseconds(1000);
    // Refresh and post joint b calibration data every second
    if (NOW - last_calib_data_output_time > calib_data_output_dead_time)
    {
        internal_object->refresh_carousel_calib_data();
        internal_object->refresh_ra_calib_data();
        internal_object->refresh_sa_calib_data();
    }
}

void LCMHandler::InternalHandler::carousel_closedloop_cmd(LCM_INPUT, const CarouselPosition *msg)
{
    ControllerMap::controllers["CAROUSEL_MOTOR"]->closed_loop(0, msg->position);
}

>>>>>>> upstream/travis-free
void LCMHandler::InternalHandler::carousel_openloop_cmd(LCM_INPUT, const CarouselOpenLoopCmd *msg)
{
    ControllerMap::controllers["CAROUSEL_MOTOR"]->open_loop(msg->throttle);
}

<<<<<<< HEAD

void LCMHandler::InternalHandler::foot_openloop_cmd(LCM_INPUT, const FootCmd *msg)
{
    ControllerMap::controllers["FOOT_SCOOP"]->open_loop(msg->microscope_triad);
    ControllerMap::controllers["FOOT_SENSOR"]->open_loop(msg->scoop);
=======
void LCMHandler::InternalHandler::carousel_zero_cmd(LCM_INPUT, const Signal *msg)
{
    ControllerMap::controllers["CAROUSEL_MOTOR"]->zero();
}

void LCMHandler::InternalHandler::foot_openloop_cmd(LCM_INPUT, const FootCmd *msg)
{
    ControllerMap::controllers["FOOT_SENSOR"]->open_loop(msg->microscope_triad);
    ControllerMap::controllers["FOOT_SCOOP"]->open_loop(msg->scoop);
>>>>>>> upstream/travis-free
}

void LCMHandler::InternalHandler::hand_openloop_cmd(LCM_INPUT, const HandCmd *msg)
{
    ControllerMap::controllers["HAND_FINGER"]->open_loop(msg->finger);
    ControllerMap::controllers["HAND_GRIP"]->open_loop(msg->grip);
<<<<<<< HEAD
=======
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
}

void LCMHandler::InternalHandler::hand_openloop_cmd(LCM_INPUT, const HandCmd *msg)
{
    ControllerMap::controllers["RA_A"]->open_loop(msg->throttle[0]);
    ControllerMap::controllers["RA_B"]->open_loop(msg->throttle[1]);
    ControllerMap::controllers["RA_C"]->open_loop(msg->throttle[2]);
    ControllerMap::controllers["RA_D"]->open_loop(msg->throttle[3]);
    ControllerMap::controllers["RA_E"]->open_loop(msg->throttle[4]);
    ControllerMap::controllers["RA_F"]->open_loop(msg->throttle[5]);

    publish_ra_pos_data();
}

void LCMHandler::InternalHandler::refresh_carousel_calib_data()
{
    ControllerMap::controllers["CAROUSEL_MOTOR"]->refresh_calibration_data();
    publish_carousel_calib_data();
}

void LCMHandler::InternalHandler::refresh_carousel_quad_angles()
{
    ControllerMap::controllers["CAROUSEL_MOTOR"]->refresh_quad_angle();
    publish_carousel_pos_data();
}

void LCMHandler::InternalHandler::refresh_ra_calib_data()
{
    ControllerMap::controllers["RA_B"]->refresh_calibration_data();
    publish_ra_calib_data();
>>>>>>> upstream/travis-free
}

void LCMHandler::InternalHandler::refresh_ra_quad_angles()
{
    ControllerMap::controllers["RA_A"]->refresh_quad_angle();
    ControllerMap::controllers["RA_B"]->refresh_quad_angle();
    ControllerMap::controllers["RA_C"]->refresh_quad_angle();
    ControllerMap::controllers["RA_D"]->refresh_quad_angle();
    ControllerMap::controllers["RA_E"]->refresh_quad_angle();
    ControllerMap::controllers["RA_F"]->refresh_quad_angle();

    publish_ra_pos_data();
}


void LCMHandler::InternalHandler::refresh_sa_calib_data()
{
    ControllerMap::controllers["SA_B"]->refresh_calibration_data();
    publish_sa_calib_data();
}

void LCMHandler::InternalHandler::refresh_sa_quad_angles()
{
    ControllerMap::controllers["SA_A"]->refresh_quad_angle();
    ControllerMap::controllers["SA_B"]->refresh_quad_angle();
    ControllerMap::controllers["SA_C"]->refresh_quad_angle();
    ControllerMap::controllers["SA_E"]->refresh_quad_angle();

    publish_sa_pos_data();
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

void LCMHandler::InternalHandler::publish_carousel_calib_data()
{
    Calibrate msg;
    msg.calibrated = ControllerMap::controllers["CAROUSEL_MOTOR"]->calibrated;
    lcm_bus->publish("/carousel_calib_data", &msg);
    last_calib_data_output_time = NOW;
}

void LCMHandler::InternalHandler::publish_carousel_pos_data()
{
    CarouselPosition msg;
    float carousel_angle = ControllerMap::controllers["CAROUSEL_MOTOR"]->get_current_angle();
    msg.position = carousel_angle;
    lcm_bus->publish("/carousel_pos_data", &msg);
    last_calib_data_output_time = NOW;
}

void LCMHandler::InternalHandler::publish_ra_calib_data()
{
    Calibrate msg;
    msg.calibrated = ControllerMap::controllers["RA_B"]->calibrated;
    lcm_bus->publish("/ra_b_calib_data", &msg);
    last_calib_data_output_time = NOW;
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
    last_heartbeat_output_time = NOW;
}

<<<<<<< HEAD
=======
void LCMHandler::InternalHandler::publish_sa_calib_data()
{
    Calibrate msg;
    msg.calibrated = ControllerMap::controllers["SA_B"]->calibrated;
    lcm_bus->publish("/sa_b_calib_data", &msg);
    last_calib_data_output_time = NOW;
}

>>>>>>> upstream/travis-free
void LCMHandler::InternalHandler::publish_sa_pos_data()
{
    SAPosition msg;
    msg.joint_a = ControllerMap::controllers["SA_A"]->get_current_angle();
    msg.joint_b = ControllerMap::controllers["SA_B"]->get_current_angle();
    msg.joint_c = ControllerMap::controllers["SA_C"]->get_current_angle();
    msg.joint_e = ControllerMap::controllers["SA_E"]->get_current_angle();
    lcm_bus->publish("/sa_position", &msg);
<<<<<<< HEAD
=======
    last_heartbeat_output_time = NOW;
>>>>>>> upstream/travis-free
}
