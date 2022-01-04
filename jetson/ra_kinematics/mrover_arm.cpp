#include "mrover_arm.hpp"
#include "arm_state.hpp"
#include "motion_planner.hpp"
#include "kinematics.hpp"
#include "utils.hpp"

#include <chrono>
#include <thread>

using namespace Eigen;
using nlohmann::json;

MRoverArm::MRoverArm(json &geom, lcm::LCM &lcm) :
    state(geom),
    solver(),
    motion_planner(state, solver),
    lcm_(lcm),
    enable_execute(false),
    sim_mode(true),
    ik_enabled(false),
    previewing(false)  {

        prev_angles.clear();
        prev_angles.resize(6);
        faulty_encoders.resize(6);

        for (size_t joint = 0; joint < faulty_encoders.size(); ++joint) {
            faulty_encoders[joint] = false;
        }
    }

void MRoverArm::arm_position_callback(std::string channel, ArmPosition msg) {

    std::vector<double> angles{ msg.joint_a, msg.joint_b, msg.joint_c,
                            msg.joint_d, msg.joint_e, msg.joint_f };
    
    // Adjust for encoders not being properly zeroed.
    if (!sim_mode) {
        for (size_t i = 0; i < 6; ++i) {
            angles[i] -= state.get_joint_encoder_offset(i);
            angles[i] *= state.get_joint_encoder_multiplier(i);
        }
    }

    encoder_error = false;
    encoder_error_message = "Encoder Error in encoder(s) (joint A = 0, F = 5): ";

    // if (check_zero_encoder(angles)) {
    //     return;
    // }

    // if (check_joint_limits(angles)) {
    //     return;
    // }

    // If we have less than 5 previous angles to compare to
    if (prev_angles[0].size() < MAX_NUM_PREV_ANGLES) {

        // For each joint
        for (size_t joint = 0; joint < 6; ++joint) {
            
            // For each previous angle we have to compare to
            for (size_t i = 0; i < prev_angles[joint].size(); ++i) {
                double diff = abs(angles[joint] - prev_angles[joint][i]);

                if ((!sim_mode && diff < ZERO_ENCODER_EPSILON) ||
                            diff > ENCODER_ERROR_THRESHOLD * (i + 1)) {
                    faulty_encoders[joint] = true;
                    encoder_error_message += ", " + std::to_string(joint);
                    encoder_error = true;
                    break;
                }

                if (i == prev_angles[joint].size() - 1) {
                    faulty_encoders[joint] = false;
                }
            }                
        }
    }
    else {
        // For each joint
        for (size_t joint = 0; joint < 6; ++joint) {

            size_t num_fishy_vals = 0;
            
            // For each previous angle we have to compare to
            for (size_t i = 0; i < MAX_NUM_PREV_ANGLES; ++i) {
                double diff = abs(angles[joint] - prev_angles[joint][i]);

                if ((!sim_mode && diff < ZERO_ENCODER_EPSILON) ||
                            diff > ENCODER_ERROR_THRESHOLD * (i + 1)) {
                    ++num_fishy_vals;
                }
            }

            if (num_fishy_vals > MAX_FISHY_VALS) {
                faulty_encoders[joint] = true;
                encoder_error = true;
                encoder_error_message += ", " + std::to_string(joint);
            }
            else {
                faulty_encoders[joint] = false;
            }
        }
    }

    // Give each angle to prev_angles (stores up to 5 latest values)
    for (size_t joint = 0; joint < 6; ++joint) {
        if (prev_angles[joint].size() >= MAX_NUM_PREV_ANGLES) {
            prev_angles[joint].pop_back();
        }

        prev_angles[joint].push_front(angles[joint]);
        if (faulty_encoders[joint]) {
            angles[joint] = state.get_joint_angle(joint);
        }
    }

    // if previewing, don't update state based on arm position
    if (!previewing) {
        // update state
        state.set_joint_angles(angles);
        solver.FK(state);

        // update GUI
        publish_transforms(state);
    }
}

void MRoverArm::target_orientation_callback(std::string channel, TargetOrientation msg) {
    std::cout << "Received target!\n";
    std::cout << "Target position: " << msg.x << "\t" << msg.y << "\t" << msg.z << "\n";
    if (msg.use_orientation) {
        std::cout << "Target orientation: " << msg.alpha << "\t" << msg.beta << "\t" << msg.gamma << "\n";
    }
    std::cout << "Initial joint angles: ";
    for (double ang : state.get_joint_angles()) {
        std::cout << ang << "\t"; 
    }
    std::cout << "\n";

    enable_execute = false;

    bool use_orientation = msg.use_orientation;

    Vector6d point;
    point(0) = (double) msg.x;
    point(1) = (double) msg.y;
    point(2) = (double) msg.z;
    point(3) = (double) msg.alpha;
    point(4) = (double) msg.beta;
    point(5) = (double) msg.gamma;

    // attempt to find ik_solution, starting at current position
    std::pair<Vector6d, bool> ik_solution = solver.IK(state, point, false, use_orientation);

    // attempt to find ik_solution, starting at up to 10 random positions
    for(int i = 0; i < 25; ++i) {
        if(ik_solution.second) {
            std::cout << "Solved IK with " << i << " random starting positions\n";
            break;
        }

        ik_solution = solver.IK(state, point, true, use_orientation);
    }

    // if no solution
    if(!ik_solution.second) {
        std::cout << "NO IK SOLUTION FOUND, please try a different configuration.\n";

        DebugMessage msg;
        msg.isError = false;
        msg.message = "No IK solution";
        
        // send popup message to GUI
        lcm_.publish("/debug_message", &msg);
        return;
    }

    std::cout << "Final joint angles: ";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << ik_solution.first[i] << "\t"; 
    }
    std::cout << "\n";

    // create path of the angles IK found and preview on GUI
    plan_path(ik_solution.first);
}

void MRoverArm::motion_execute_callback(std::string channel, MotionExecute msg) {

    // TODO do we ever need to preview at this stage? Isn't that done before we get here?
    // TODO if user cancels after preview, figure out how to send current position to GUI
    if (msg.preview) {
        // enable_execute = false;
        // preview();
    }
    else {
        // run loop inside execute_spline()
        std::cout << "Motion Executing!\n";
        enable_execute = true;
    }
}

void MRoverArm::execute_spline() { 
    double spline_t = 0.0;
    double spline_t_iterator = 0.001;


    while (true) {
        if (enable_execute) {

            if (encoder_error) {
                enable_execute = false;
                spline_t = 0.0;
                ik_enabled = false;

                DebugMessage msg;
                msg.isError = true;
                msg.message = encoder_error_message;

                // send popup message to GUI
                lcm_.publish("/debug_message", &msg);

                if (sim_mode) {
                    for (size_t i = 0; i < MAX_NUM_PREV_ANGLES; ++i) {
                        publish_config(state.get_joint_angles(), "/arm_position");
                    }
                }
            }
            else {
                        
                //find arm's current angles
                std::vector<double> init_angles = state.get_joint_angles(); 
                //find angles D_SPLINE_T (%) further down the spline path
                std::vector<double> final_angles = motion_planner.get_spline_pos(spline_t + D_SPLINE_T);

                double max_time = -1; //in ms

                // Get max time to travel for joints a through e
                for (int i = 0; i < 5; ++i) {
                    //in ms, time needed to move D_SPLINE_T (%)
                    double joint_time = abs(final_angles[i] - init_angles[i]) 
                        / (state.get_joint_max_speed(i) / 1000.0); 
                    //sets max_time to greater value
                    max_time = max_time < joint_time ? joint_time : max_time;
                }

                //determines size of iteration by dividing number of iterations by distance
                spline_t_iterator = D_SPLINE_T / (max_time / SPLINE_WAIT_TIME);
                spline_t += spline_t_iterator;

                // get next set of angles in path
                std::vector<double> target_angles = motion_planner.get_spline_pos(spline_t);

                for (size_t i = 0; i < 6; ++i) {
                    if (target_angles[i] < state.get_joint_limits(i)[0]) {
                        target_angles[i] = state.get_joint_limits(i)[0];
                    }
                    else if (target_angles[i] > state.get_joint_limits(i)[1]) {
                        target_angles[i] = state.get_joint_limits(i)[1];
                    }
                }

                // if not in sim_mode, send physical arm a new target
                if (!sim_mode) {
                    // TODO make publish function names more intuitive?
                    publish_config(target_angles, "/ik_ra_control");
                }

                // TODO: make sure transition from not self.sim_mode
                //   to self.sim_mode is safe!!      previously commented

                // if in sim_mode, simulate that we have gotten a new current position
                else if (sim_mode) {
                    publish_config(target_angles, "/arm_position");
                }

                // break out of loop if necessary
                if (spline_t > 1) {
                    enable_execute = false;
                    spline_t = 0.0;
                    ik_enabled = false;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds((int) SPLINE_WAIT_TIME));
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            spline_t = 0;
            spline_t_iterator = 0.001;
        }   
    }
}

void MRoverArm::publish_config(const std::vector<double> &config, std::string channel) {
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
    std::cout << "Previewing...\n";
    ik_enabled = true;
    previewing = true;

    // backup angles
    std::vector<double> backup = state.get_joint_angles();

    double num_steps = 100.0;
    double t = 0.0;

    while (t <= 1) {

        // set state to next position in the path
        std::vector<double> target = motion_planner.get_spline_pos(t);
        state.set_joint_angles(target);

        // update transforms
        solver.FK(state); 

        // send transforms to GUI
        publish_transforms(state);

        t += 1.0 / num_steps;
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    std::cout <<  "Preview Done\n";

    DebugMessage msg;
    msg.isError = false;
    msg.message = "Preview Done";
    
    // send popup message to GUI
    lcm_.publish("/debug_message", &msg);

    // recover from backup to previous arm position
    state.set_joint_angles(backup);

    // update state based on new angles
    solver.FK(state);

    previewing = false;
}

void MRoverArm::target_angles_callback(std::string channel, ArmPosition msg) {
    std::cout << "Received target angles\n";

    enable_execute = false;

    // convert to Vector6d
    Vector6d target;
    target[0] = (double) msg.joint_a;
    target[1] = (double) msg.joint_b;
    target[2] = (double) msg.joint_c;
    target[3] = (double) msg.joint_d;
    target[4] = (double) msg.joint_e;
    target[5] = (double) msg.joint_f;
    
    std::cout << "Requested angles: ";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << target[i] << " ";
    }
    std::cout << "\n";

    plan_path(target);
}

void MRoverArm::publish_transforms(const ArmState& pub_state) {
    FKTransform tm;
    matrix_helper(tm.transform_a, pub_state.get_joint_transform(0));
    matrix_helper(tm.transform_b, pub_state.get_joint_transform(1));
    matrix_helper(tm.transform_c, pub_state.get_joint_transform(2));
    matrix_helper(tm.transform_d, pub_state.get_joint_transform(3));
    matrix_helper(tm.transform_e, pub_state.get_joint_transform(4));
    matrix_helper(tm.transform_f, pub_state.get_joint_transform(5));

    lcm_.publish("/fk_transform", &tm);
}

void MRoverArm::ik_enabled_callback(std::string channel, IkEnabled msg) {  
    ik_enabled = msg.enabled;

    if (!ik_enabled) {
        enable_execute = false;
        publish_transforms(state);
    }
}       

void MRoverArm::plan_path(Vector6d goal) {
    bool path_found = motion_planner.rrt_connect(state, goal);

    if (path_found) {
        preview();
    }
    else {
        DebugMessage msg;
        msg.isError = false;
        msg.message = "Unable to plan path!";
        
        // send popup message to GUI
        lcm_.publish("/debug_message", &msg);
    }
}

void MRoverArm::simulation_mode_callback(std::string channel, SimulationMode msg) {
    std::cout << "Received Simulation Mode value: " << msg.sim_mode;

    if (previewing)
    sim_mode = msg.sim_mode;
    // publish_transforms(state);
}

void MRoverArm::lock_joints_callback(std::string channel, LockJoints msg) {
    std::cout << "Running lock_joints_callback: ";

    state.set_joint_locked(0, (bool)msg.jointa);
    state.set_joint_locked(1, (bool)msg.jointb);
    state.set_joint_locked(2, (bool)msg.jointc);
    state.set_joint_locked(3, (bool)msg.jointd);
    state.set_joint_locked(4, (bool)msg.jointe);
    state.set_joint_locked(5, (bool)msg.jointf);

    std::cout << "\n";
}

bool MRoverArm::check_zero_encoder(const std::vector<double> &angles) const {
    size_t num_faulty = 0;
    for (size_t i = 0; i < angles.size(); ++i) {
        if (abs(angles[i] - ZERO_ENCODER_VALUE) < ZERO_ENCODER_EPSILON) {
            ++num_faulty;
        }
    }

    return num_faulty > angles.size()/2;
}

bool MRoverArm::check_joint_limits(const std::vector<double> &angles) {
    for (size_t i = 0; i < angles.size(); ++i) {
        std::vector<double> limits = state.get_joint_limits(i);
        if (angles[i] < limits[0] || angles[i] > limits[1]) {
            encoder_error = true;
            encoder_error_message = "Encoder Error: " + std::to_string(angles[i]) + " beyond joint " + std::to_string(i) + " limits (joint A = 0, F = 5)"; 
            return true;
        }
    }
    return false;
}

// void MRoverArm::cartesian_control_callback(std::string channel, IkArmControl msg) {
//    if(enable_execute) {
//        return;
//    }
 
//    IkArmControl cart_msg = msg;
//    double delta[3] = {cart_msg.deltaX, cart_msg.deltaY, cart_msg.deltaZ};
//    //idk if this line is right down here
//    std::pair<std::vector<double> joint_angles, bool is_safe> = solver.IK_delta(delta, 3); // IK_delta takes in a Vector6d. ik arm control only has 3 values.
  
//    if(is_safe) {
//        ArmPosition arm_position = ArmPosition();
//        map<std::string,double> gja = state.get_joint_angles();
//        arm_position.joint_a = gja["joint_a"];
//        arm_position.joint_b = gja["joint_b"];
//        arm_position.joint_c = gja["joint_c"];
//        arm_position.joint_d = gja["joint_d"];
//        arm_position.joint_e = gja["joint_e"];
//        arm_position.joint_f = gja["joint_f"];
//        std::vector<double> angles = {gja["joint_a"], gja["joint_b"], gja["joint_c"], gja["joint_d"], gja["joint_e"], gja["joint_f"]};
//        state.set_joint_angles(angles);
//        solver.FK(state);  
//        publish_transforms(state);
//        //again, running into the issue of encode(), should we even have it there
//        if(sim_mode) {
//            std::cout << "Printing sim_mode\n";
//            lcm_.publish("/arm_position", arm_position.encode());
//        }
//        else{
//            std::cout << "Printing\n";
//            lcm_.publish("/ik_ra_control", arm_position.encode());
//        }
//    }
 
 
// }
 