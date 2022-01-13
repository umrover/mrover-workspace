#include "nlohmann/json.hpp"
#include "mrover_arm.hpp"
#include "utils.hpp"

#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <thread>
#include <iomanip>

using nlohmann::json;


class lcmHandlers {
public:
    lcmHandlers (MRoverArm* robot_arm) : arm(robot_arm) {}

    void lockJointsCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const LockJoints* lj_execute
        )
    {
        arm->lock_joints_callback( channel, *lj_execute );
    }

    void executeCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const TargetOrientation* m_execute
        )
    {
        arm->target_orientation_callback( channel, *m_execute );
    }
    
    void motionExecuteCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const MotionExecute* m_execute
    )
    {
        arm->motion_execute_callback( channel, *m_execute );
    }

    void executePresetCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const ArmPosition* p_execute
        )
    {
        arm->target_angles_callback( channel, *p_execute );
    }
      
    void armPositionCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const ArmPosition* arm_pos
    )
    {
        arm->arm_position_callback( channel, *arm_pos );
    }

    void ikEnabledCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const IkEnabled* enable
    )
    {
        arm->ik_enabled_callback( channel, *enable );
    }

    void simModeCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const SimulationMode* sim_mode
    )
    {
        arm->simulation_mode_callback( channel, *sim_mode );
    }
    
    void armControlCallback(
        const lcm::RecieveBuffer* receiveBuffer,
        const std::string& channel,
        const ArmControlState* arm_control
    )
    {
        arm->ra_control_callback( channel, *arm_control );
    }

private:
    MRoverArm* arm;
};


int main() {
    
    std::cout << std::fixed;
    std::cout << std::setprecision(4);
    std::cout << "INITIALIZING KINEMATICS FOR ROBOT ARM\n";

    json geom = read_json_from_file(get_mrover_arm_geom());

    lcm::LCM lcmObject;
    MRoverArm robot_arm(geom, lcmObject);

    lcmHandlers handler(&robot_arm);

    lcmObject.subscribe( "/arm_position", &lcmHandlers::armPositionCallback, &handler );
    lcmObject.subscribe( "/target_orientation" , &lcmHandlers::executeCallback, &handler );
    lcmObject.subscribe( "/preset_angles" , &lcmHandlers::executePresetCallback, &handler );
    lcmObject.subscribe( "/motion_execute", &lcmHandlers::motionExecuteCallback, &handler );
    lcmObject.subscribe( "/ik_enabled", &lcmHandlers::ikEnabledCallback, &handler );
    lcmObject.subscribe( "/simulation_mode", &lcmHandlers::simModeCallback, &handler );
    lcmObject.subscribe( "/locked_joints", &lcmHandlers::lockJointsCallback, &handler );
    lcmObject.subscribe( "/arm_control_state", &lcmHandlers::armControlCallback, &handler );
    
    std::thread execute_spline(&MRoverArm::execute_spline, &robot_arm);
    std::thread send_arm_position(&MRoverArm::encoder_angles_sender, &robot_arm);

    while( lcmObject.handle() == 0 ) {
        // run kinematics
    }

    execute_spline.join();
    send_arm_position.join();

    return 0;
}
