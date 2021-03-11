#include "json.hpp"
#include "arm_state.hpp"
#include "kinematics.hpp"
#include "mrover_arm.hpp"
#include "utils.hpp"


#include <lcm/lcm-cpp.hpp>
#include <iostream>

using nlohmann::json;
using namespace std;

class lcmHandlers {
public:
    lcmHandlers (MRoverArm* robot_arm) : arm(robot_arm) {}

    void executeCallback(
        const lcm::ReceiveBuffer* receiveBuffer,
        const string& channel,
        const MotionExecute* m_execute
        )
    {
        arm->motion_execute_callback( channel, *m_execute );
    }

private:
    MRoverArm* arm;
};



int main() 
{

    
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


    while( lcmObject.handle() == 0 ) {
        //run kinematics
        
    }



    // ArmState arm = ArmState(geom);

    // cout << "GETTING JOINT A\n";

    // arm.get_joint_mass("joint_a");


    return 0;
}