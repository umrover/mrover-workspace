#include "nlohmann/json.hpp"
#include "mrover_arm.hpp"
#include "utils.hpp"

#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <thread>

using nlohmann::json;
using namespace std;


class lcmHandlers {
public:
    lcmHandlers (MRoverArm* robot_arm) : arm(robot_arm) {}

    void executeCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const TargetOrientation* m_execute
        )
    {
        arm->target_orientation_callback( channel, *m_execute );
    }
    
    void motionExecuteCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const MotionExecute* m_execute
    )
    {
        arm->motion_execute_callback( channel, *m_execute );
    }

    void executePresetCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const ArmPosition* p_execute
        )
    {
        arm->target_angles_callback( channel, *p_execute );
    }
      
    void armPositionCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const ArmPosition* arm_pos
    )
    {
        arm->arm_position_callback( channel, *arm_pos );
    }

    void ikEnabledCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const IkEnabled* enable
    )
    {
        arm->ik_enabled_callback( channel, *enable );
    }

    void simModeCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const SimulationMode* sim_mode
    )
    {
        arm->simulation_mode_callback( channel, *sim_mode );
    }

private:
    MRoverArm* arm;
};



int main() {
    
    cout << "INITIALIZING KINEMATICS FOR ROBOT ARM\n";

    // TODO: don't use vagrant for pulling files
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";
    json geom = read_json_from_file(geom_file);

    cout << "INITIALIZING ROBOT ARM OBJECT\n";

    lcm::LCM lcmObject;
    MRoverArm robot_arm(geom, lcmObject);

    lcmHandlers handler(&robot_arm);

    lcmObject.subscribe( "/arm_position", &lcmHandlers::armPositionCallback, &handler );
    lcmObject.subscribe( "/target_orientation" , &lcmHandlers::executeCallback, &handler );
    lcmObject.subscribe( "/preset_angles" , &lcmHandlers::executePresetCallback, &handler );
    lcmObject.subscribe( "/motion_execute", &lcmHandlers::motionExecuteCallback, &handler );
    lcmObject.subscribe( "/ik_enabled", &lcmHandlers::ikEnabledCallback, &handler );
    lcmObject.subscribe( "/simulation_mode", &lcmHandlers::simModeCallback, &handler );
    
    thread execute_spline(&MRoverArm::execute_spline, &robot_arm);

    while( lcmObject.handle() == 0 ) {
        //run kinematics
        
    }

    execute_spline.join();

    return 0;
}