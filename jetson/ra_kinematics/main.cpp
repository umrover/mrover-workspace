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
      
    void raPositionCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const RAPosition* ra_pos)
    {
        arm->arm_position_callback( channel, *ra_pos );
    }

    void saPositionCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const SAPosition* sa_pos)
    {
        arm->arm_position_callback( channel, *sa_pos );
    }

    void simModeCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const SimulationMode* sim_mode)
    {
        arm->simulation_mode_callback( channel, *sim_mode );
    }
    
    void armControlCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const ArmControlState* arm_control)
    {
        arm->ra_control_callback( channel, *arm_control );
    }

    void useOrientationCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const UseOrientation* use_orientation)
    {
        arm->use_orientation_callback( channel, *use_orientation );
    }

    void lockJointsCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const LockJoints* lj_execute)
    {
        arm->lock_joints_callback( channel, *lj_execute );
    }

    void zeroPositionCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const Signal* zero_position)
    {
        arm->zero_position_callback( channel, *zero_position );
    }

    void armAdjustCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const ArmAdjustments* arm_adjustments)
    {
        arm->arm_adjust_callback( channel, *arm_adjustments );
    }

    void armPresetCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const ArmPreset* arm_preset)
    {
        arm->arm_preset_callback( channel, *arm_preset );
    }

    void pathPresetCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const ArmPresetPath* arm_preset_path)
        {
            arm->arm_preset_path_callback( channel, *arm_preset_path );
        }
    )

    void customPresetCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const CustomPreset* custom_preset)
    {
        arm->custom_preset_callback( channel, *custom_preset );
    }

    void wristTurnCountCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const std::string& channel,
        const WristTurnCount* wrist_turn_count)
    {
        arm->wristTurnCountCallback( channel, *wrist_turn_count );
    }

private:
    MRoverArm* arm;
};


int main() {
    
    std::cout << std::fixed;
    std::cout << std::setprecision(4);
    std::cout << "INITIALIZING KINEMATICS FOR ROBOT ARM\n";

    bool science = check_science();
    json geom = read_json_from_file(get_mrover_arm_geom(science));

    lcm::LCM lcmObject;

    if (science) {
        ScienceArm robot_arm(geom, lcmObject);

        lcmHandlers handler(&robot_arm);

        lcmObject.subscribe( "/sa_position", &lcmHandlers::saPositionCallback, &handler );
        lcmObject.subscribe( "/motion_execute", &lcmHandlers::motionExecuteCallback, &handler );
        lcmObject.subscribe( "/simulation_mode", &lcmHandlers::simModeCallback, &handler );
        lcmObject.subscribe( "/arm_control_state", &lcmHandlers::armControlCallback, &handler );
        lcmObject.subscribe( "/locked_joints", &lcmHandlers::lockJointsCallback, &handler );
        lcmObject.subscribe( "/zero_position", &lcmHandlers::zeroPositionCallback, &handler );
        lcmObject.subscribe( "/arm_adjustments", &lcmHandlers::armAdjustCallback, &handler );
        lcmObject.subscribe( "/arm_preset", &lcmHandlers::armPresetCallback, &handler );
        lcmObject.subscribe( "/arm_preset_path", &lcmHandlers::pathPresetCallback, &handler );


        lcmObject.subscribe( "/custom_preset", &lcmHandlers::customPresetCallback, &handler );

        Signal signal;
        lcmObject.publish("/ik_reset", &signal);

        std::thread execute_spline(&MRoverArm::execute_spline, &robot_arm);
        std::thread send_arm_position(&MRoverArm::encoder_angles_sender, &robot_arm);

        while( lcmObject.handle() == 0 ) {
            // run kinematics
        }

        execute_spline.join();
        send_arm_position.join();

        return 0;
    }
    else {
        StandardArm robot_arm(geom, lcmObject);

        lcmHandlers handler(&robot_arm);

        lcmObject.subscribe( "/ra_position", &lcmHandlers::raPositionCallback, &handler );
        lcmObject.subscribe( "/target_orientation" , &lcmHandlers::executeCallback, &handler );
        lcmObject.subscribe( "/motion_execute", &lcmHandlers::motionExecuteCallback, &handler );
        lcmObject.subscribe( "/simulation_mode", &lcmHandlers::simModeCallback, &handler );
        lcmObject.subscribe( "/arm_control_state", &lcmHandlers::armControlCallback, &handler );
        lcmObject.subscribe( "/use_orientation", &lcmHandlers::useOrientationCallback, &handler );
        lcmObject.subscribe( "/locked_joints", &lcmHandlers::lockJointsCallback, &handler );
        lcmObject.subscribe( "/zero_position", &lcmHandlers::zeroPositionCallback, &handler );
        lcmObject.subscribe( "/arm_adjustments", &lcmHandlers::armAdjustCallback, &handler );
        lcmObject.subscribe( "/arm_preset", &lcmHandlers::armPresetCallback, &handler );
        lcmObject.subscribe( "/custom_preset", &lcmHandlers::customPresetCallback, &handler );
        lcmObject.subscribe( "/wrist_turn_count", &lcmHandlers::wristTurnCountCallback, &handler );

        Signal signal;
        lcmObject.publish("/ik_reset", &signal);

        std::thread execute_spline(&MRoverArm::execute_spline, &robot_arm);
        std::thread send_arm_position(&MRoverArm::encoder_angles_sender, &robot_arm);

        while( lcmObject.handle() == 0 ) {
            // run kinematics
        }

        execute_spline.join();
        send_arm_position.join();

        return 0;
    }
}
