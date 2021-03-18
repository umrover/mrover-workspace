#include "json.hpp"
#include "arm_state.hpp"
#include "kinematics.hpp"
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

    void armPositionCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const ArmPosition* arm_pos
    )
    {
        arm->arm_position_callback( channel, *arm_pos );
    }

private:
    MRoverArm* arm;
};



int main() {
    
    cout << "INITIALIZING KINEMATICS FOR ROBOT ARM\n";
    // string config_path = getenv("MROVER_CONFIG");
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    lcm::LCM lcmObject;
    json geom = read_json_from_file(geom_file);

    cout << "INITIALIZING ROBOT ARM OBJECT\n";

    MRoverArm robot_arm(geom, lcmObject);
    KinematicsSolver solver = KinematicsSolver();

    lcmHandlers handler(&robot_arm);

    lcmObject.subscribe( "/target_orientation" , &lcmHandlers::executeCallback, &handler );
    lcmObject.subscribe( "/motion_execute", &lcmHandlers::motionExecuteCallback, &handler );
    lcmObject.subscribe( "/arm_position", &lcmHandlers::armPositionCallback, &handler );
    
    thread exe_spline(robot_arm.execute_spline);

    while( lcmObject.handle() == 0 ) {
        //run kinematics
        
    }

    exe_spline.join();

    // ArmState arm = ArmState(geom);

    // cout << "GETTING JOINT A\n";

    // arm.get_joint_mass("joint_a");


    return 0;
}