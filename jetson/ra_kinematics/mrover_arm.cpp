#include "mrover_arm.hpp"
#include "arm_state.hpp"
#include "motion_planner.hpp"
#include "kinematics.hpp"
#include "utils.hpp"

#include <chrono>
#include <thread>

using namespace std;
using namespace Eigen;
using nlohmann::json;

MRoverArm::MRoverArm(json &geom, lcm::LCM &lcm) :
    state(geom),
    motion_planner(state, solver),
    lcm_(lcm),
    done_previewing(false),
    enable_execute(false),
    sim_mode(true),
    ik_enabled(false)  { }

void MRoverArm::arm_position_callback(string channel, ArmPosition msg) {
    // if previewing, don't update state based on arm position
    if (ik_enabled && !enable_execute) {
        return;
    }

    if (channel == "/arm_position") {
        vector<double> angles{ msg.joint_a, msg.joint_b, msg.joint_c,
                               msg.joint_d, msg.joint_e, msg.joint_f };

        // update state
        state.set_joint_angles(angles);
        solver.FK(state);

        // update GUI
        publish_transforms(state);
    }
}

void MRoverArm::target_orientation_callback(string channel, TargetOrientation msg) {
    cout << "received target orientation, starting callback\n";

    enable_execute = false;

    // cout << "alpha beta gamma\n";
    // cout << msg.alpha << " , " << msg.beta << " , " << msg.gamma << "\n";

    bool use_orientation = msg.use_orientation;
    cout << "use orientation: " << use_orientation << "\n";

    Vector6d point;
    point(0) = (double) msg.x;
    point(1) = (double) msg.y;
    point(2) = (double) msg.z;
    point(3) = (double) msg.alpha;
    point(4) = (double) msg.beta;
    point(5) = (double) msg.gamma;

    // attempt to find ik_solution, starting at current position
    pair<Vector6d, bool> ik_solution = solver.IK(state, point, false, use_orientation);

    // attempt to find ik_solution, starting at up to 10 random positions
    for(int i = 0; i < 20; ++i) {
        if(ik_solution.second) {
            cout << "Solved IK with " << i << " random starting positions\n";
            break;
        }

        ik_solution = solver.IK(state, point, true, use_orientation);
    }

    // if no solution
    if(!ik_solution.second) {
        cout << "NO IK SOLUTION FOUND, please try a different configuration.\n";

        DebugMessage msg;
        msg.isError = false;
        msg.message = "No IK solution";
        
        // send popup message to GUI
        lcm_.publish("/debug_message", &msg);
        return;
    }

    // create path of the angles IK found
    plan_path(ik_solution.first);

    // view path on GUI without moving the physical arm
    preview();

    cout << "ended target orientation callback\n";
}

void MRoverArm::motion_execute_callback(string channel, MotionExecute msg) {

    // TODO do we ever need to preview at this stage? Isn't that done before we get here?
    // TODO if user cancels after preview, figure out how to send current position to GUI
    if (msg.preview) {
        // enable_execute = false;
        // preview();
    }
    else {
        // run loop inside execute_spline()
        cout << "Motion Executing!\n";
        enable_execute = true;
    }
}

void MRoverArm::execute_spline() { 
    double spline_t = 0.0;

    while (true) {
        if (enable_execute) {

            // get next set of angles in path
            vector<double> target_angles = motion_planner.get_spline_pos(spline_t);
            
            // if not in sim_mode, send physical arm a new target
            if (!sim_mode) {
                // TODO make publish function names more intuitive?
                publish_config(target_angles, "/ik_ra_control");
                spline_t += 0.003;
            }

            // TODO: make sure transition from not self.sim_mode
            //   to self.sim_mode is safe!!      previously commented

            // if in sim_mode, simulate that we have gotten a new current position
            else if (sim_mode) {
                publish_config(target_angles, "/arm_position");
                spline_t += 0.005;
            }

            // break out of loop if necessary
            if (spline_t > 1) {
                enable_execute = false;
                spline_t = 0.0;
                ik_enabled = false;
            }

            this_thread::sleep_for(chrono::milliseconds(10));
        }
        else {
            this_thread::sleep_for(chrono::milliseconds(200));
            spline_t = 0;
        }   
    }
}

void MRoverArm::publish_config(vector<double> &config, string channel) {
       ArmPosition arm_position;
       arm_position.joint_a = config[0];
       arm_position.joint_b = config[1];
       arm_position.joint_c = config[2];
       arm_position.joint_d = config[3];
       arm_position.joint_e = config[4];
       arm_position.joint_f = config[5];
       lcm_.publish(channel, &arm_position); //no matching call to publish should take in const msg type msg
}

void MRoverArm::matrix_helper(double arr[4][4], const Matrix4d &mat) {
   for (int i = 0; i < 4; ++i) {
       for (int j = 0; j < 4; ++j) {
           arr[i][j] = mat(i,j);
       }
   }
}
 
void MRoverArm::preview() {
    cout << "Previewing" << endl;
    ik_enabled = true;

    // backup angles
    vector<double> backup;
    map<string, double> start = state.get_joint_angles();
    for (string& joint_name : state.get_all_joints()) {
        backup.push_back(start[joint_name]);
    }

    double num_steps = 500.0;
    double t = 0.0;

    while (t <= 1) {

        // set state to next position in the path
        vector<double> target = motion_planner.get_spline_pos(t);
        state.set_joint_angles(target);

        // update transforms
        solver.FK(state); 

        // send transforms to GUI
        publish_transforms(state);

        t += 1.0 / num_steps;
        this_thread::sleep_for(chrono::milliseconds(2));
    }
    cout <<  "Preview Done" << endl;

    DebugMessage msg;
    msg.isError = false;
    msg.message = "Preview Done";
    
    // send popup message to GUI
    lcm_.publish("/debug_message", &msg);

    // recover from backup to previous arm position
    state.set_joint_angles(backup);

    // update state based on new angles
    solver.FK(state);
}

void MRoverArm::target_angles_callback(string channel, ArmPosition msg) {
    cout << "running target_angles_callback\n";

    enable_execute = false;

    // convert to Vector6d
    Vector6d target;
    target[0] = (double) msg.joint_a;
    target[1] = (double) msg.joint_b;
    target[2] = (double) msg.joint_c;
    target[3] = (double) msg.joint_d;
    target[4] = (double) msg.joint_e;
    target[5] = (double) msg.joint_f;
    
    cout << "requested angles: ";
    for (size_t i = 0; i < 6; ++i) {
        cout << target[i] << " ";
    }
    cout << "\n";

    plan_path(target);

    preview();

    cout << "finished target_angles_callback\n";
}

void MRoverArm::publish_transforms(const ArmState& pub_state) {
    FKTransform tm;
    matrix_helper(tm.transform_a, pub_state.get_joint_transform("joint_a"));
    matrix_helper(tm.transform_b, pub_state.get_joint_transform("joint_b"));
    matrix_helper(tm.transform_c, pub_state.get_joint_transform("joint_c"));
    matrix_helper(tm.transform_d, pub_state.get_joint_transform("joint_d"));
    matrix_helper(tm.transform_e, pub_state.get_joint_transform("joint_e"));
    matrix_helper(tm.transform_f, pub_state.get_joint_transform("joint_f"));

    lcm_.publish("/fk_transform", &tm);
}

void MRoverArm::lock_e_callback(string channel, LockJointE msg) {
    // cout << "joint e locked" << endl;
    // solver.lock_joint_e(msg.locked); //no function lock_joint_e in kinematics.cpp
}
void MRoverArm::ik_enabled_callback(string channel, IkEnabled msg) {  
    ik_enabled = msg.enabled;

    if (!ik_enabled) {
        enable_execute = false;
        publish_transforms(state);
    }
}       

void MRoverArm::plan_path(Vector6d goal) {
    bool path_found = motion_planner.rrt_connect(state, goal);
    if (path_found) {
        cout << "Planned path\n";
    }
    else {
        cout << "No path found\n";
    }
}

void MRoverArm::simulation_mode_callback(string channel, SimulationMode msg) {
    sim_mode = msg.sim_mode;
    // publish_transforms(state);
}

// void MRoverArm::cartesian_control_callback(string channel, IkArmControl msg) {
//    if(enable_execute) {
//        return;
//    }
 
//    IkArmControl cart_msg = msg;
//    double delta[3] = {cart_msg.deltaX, cart_msg.deltaY, cart_msg.deltaZ};
//    //idk if this line is right down here
//    pair<vector<double> joint_angles, bool is_safe> = solver.IK_delta(delta, 3); // IK_delta takes in a Vector6d. ik arm control only has 3 values.
  
//    if(is_safe) {
//        ArmPosition arm_position = ArmPosition();
//        map<string,double> gja = state.get_joint_angles();
//        arm_position.joint_a = gja["joint_a"];
//        arm_position.joint_b = gja["joint_b"];
//        arm_position.joint_c = gja["joint_c"];
//        arm_position.joint_d = gja["joint_d"];
//        arm_position.joint_e = gja["joint_e"];
//        arm_position.joint_f = gja["joint_f"];
//        vector<double> angles = {gja["joint_a"], gja["joint_b"], gja["joint_c"], gja["joint_d"], gja["joint_e"], gja["joint_f"]};
//        state.set_joint_angles(angles);
//        solver.FK(state);  
//        publish_transforms(state);
//        //again, running into the issue of encode(), should we even have it there
//        if(sim_mode) {
//            cout << "Printing sim_mode" << endl;
//            lcm_.publish("/arm_position", arm_position.encode());
//        }
//        else{
//            cout << "Printing" << endl;
//            lcm_.publish("/ik_ra_control", arm_position.encode());
//        }
//    }
 
 
// }
 