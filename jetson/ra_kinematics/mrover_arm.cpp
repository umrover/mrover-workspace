#include "mrover_arm.hpp"
#include "arm_state.hpp"
#include "motion_planner.hpp"
#include "kinematics.hpp"
#include "utils.hpp"

#include <chrono>
#include <thread>
#include <limits>
#include <cmath>

using namespace Eigen;
using nlohmann::json;

MRoverArm::MRoverArm(json &geom, lcm::LCM &lcm) :
    arm_state(geom),
    solver(),
    motion_planner(arm_state, solver),
    lcm_(lcm),
    control_state(ControlState::OFF),
    sim_mode(true),
    prev_angle_b(std::numeric_limits<double>::quiet_NaN())
{
    prev_angles.clear();
    prev_angles.resize(6);
    faulty_encoders.resize(6);

    for (size_t joint = 0; joint < faulty_encoders.size(); ++joint) {
        faulty_encoders[joint] = false;
    }

    DUD_ENCODER_VALUES.push_back(0.0);
}

void MRoverArm::ra_control_callback(std::string channel, ArmControlState msg) {
    std::string new_state = msg.state;
    
    std::cout << "Received Arm Control State: " << new_state << "\n";

    if (new_state == "closed-loop") {
        control_state = ControlState::WAITING_FOR_TARGET;
    }
    else {
        control_state = ControlState::OFF;
    }
}

void MRoverArm::arm_position_callback(std::string channel, ArmPosition msg) {

    std::vector<double> angles{ msg.joint_a, msg.joint_b, msg.joint_c,
                            msg.joint_d, msg.joint_e, msg.joint_f };
    
    // Adjust for encoders not being properly zeroed.
    if (!sim_mode) {
        for (size_t i = 0; i < 6; ++i) {
            angles[i] -= arm_state.get_joint_encoder_offset(i);
            angles[i] *= arm_state.get_joint_encoder_multiplier(i);
        }
    }

    // Adjust for shaky joint B values
    angles[1] = joint_b_stabilizer(angles[1]);

    encoder_error = false;
    encoder_error_message = "Encoder Error in encoder(s) (joint A = 0, F = 5): ";

    check_dud_encoder(angles);
    check_joint_limits(angles);
    
    // If we have less than 5 previous angles to compare to
    if (prev_angles[0].size() < MAX_NUM_PREV_ANGLES) {

        // For each joint
        for (size_t joint = 0; joint < 6; ++joint) {
            
            // For each previous angle we have to compare to
            for (size_t i = 0; i < prev_angles[joint].size(); ++i) {
                double diff = std::abs(angles[joint] - prev_angles[joint][i]);

                if (diff > ENCODER_ERROR_THRESHOLD * (i + 1)) {
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
                double diff = std::abs(angles[joint] - prev_angles[joint][i]);

                if (diff > ENCODER_ERROR_THRESHOLD * (i + 1)) {
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
            angles[joint] = arm_state.get_joint_angle(joint);
        }
    }

    // update arm_state
    arm_state.set_joint_angles(angles);

    // if previewing or finished previewing, don't update GUI based on arm position
    if (control_state != ControlState::PREVIEWING && control_state != ControlState::READY_TO_EXECUTE) {
        // Calculate transforms and update GUI
        solver.FK(arm_state);
        publish_transforms(arm_state);
    }
}

void MRoverArm::target_orientation_callback(std::string channel, TargetOrientation msg) {
    if (control_state != ControlState::WAITING_FOR_TARGET) {
        std::cout << "Received target but not currently waiting for target.\n";
        return;
    }
    control_state = ControlState::CALCULATING;

    std::cout << "Received target!\n";
    std::cout << "Target position: " << msg.x << "\t" << msg.y << "\t" << msg.z << "\n";
    if (msg.use_orientation) {
        std::cout << "Target orientation: " << msg.alpha << "\t" << msg.beta << "\t" << msg.gamma << "\n";
    }

    std::cout << "Initial joint angles: ";
    for (double ang : arm_state.get_joint_angles()) {
        std::cout << ang << "\t"; 
    }
    std::cout << "\n";

    if (!solver.is_safe(arm_state)) {
        std::cout << "STARTING POSITION NOT SAFE, please adjust arm in Open Loop.\n";

        DebugMessage msg;
        msg.isError = false;
        msg.message = "Unsafe Starting Position";
        
        // send popup message to GUI
        lcm_.publish("/debug_message", &msg);

        control_state = ControlState::WAITING_FOR_TARGET;
        return;
    }

    bool use_orientation = msg.use_orientation;

    Vector6d point;
    point(0) = (double) msg.x;
    point(1) = (double) msg.y;
    point(2) = (double) msg.z;
    point(3) = (double) msg.alpha;
    point(4) = (double) msg.beta;
    point(5) = (double) msg.gamma;

    ArmState hypo_state = arm_state;

    // attempt to find ik_solution, starting at current position
    std::pair<Vector6d, bool> ik_solution = solver.IK(hypo_state, point, false, use_orientation);

    // attempt to find ik_solution, starting at up to 25 random positions
    for(int i = 0; i < 25; ++i) {
        if (ik_solution.second) {
            std::cout << "Solved IK with " << i << " random starting positions\n";
            break;
        }

        if (control_state != ControlState::CALCULATING) {
            std::cout << "IK calculations canceled\n";
            break;
        }

        ik_solution = solver.IK(hypo_state, point, true, use_orientation);
    }

    // if no solution
    if(!ik_solution.second) {
        std::cout << "NO IK SOLUTION FOUND, please try a different configuration.\n";

        DebugMessage msg;
        msg.isError = false;
        msg.message = "No IK solution";
        
        // send popup message to GUI
        lcm_.publish("/debug_message", &msg);

        control_state = ControlState::WAITING_FOR_TARGET;
        return;
    }

    std::cout << "Final joint angles: ";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << ik_solution.first[i] << "\t"; 
    }
    std::cout << "\n";

    // create path of the angles IK found and preview on GUI
    plan_path(hypo_state, ik_solution.first);
}

void MRoverArm::target_angles_callback(std::string channel, ArmPosition msg) {
    if (control_state != ControlState::WAITING_FOR_TARGET) {
        std::cout << "Received target but not in closed-loop waiting state.\n";
        return;
    }
    control_state = ControlState::CALCULATING;

    // convert to Vector6d
    Vector6d target;
    target[0] = (double) msg.joint_a;
    target[1] = (double) msg.joint_b;
    target[2] = (double) msg.joint_c;
    target[3] = (double) msg.joint_d;
    target[4] = (double) msg.joint_e;
    target[5] = (double) msg.joint_f;

    std::cout << "Received target angles:  ";
    for (size_t i = 0; i < 6; ++i) {
        std::cout << target[i] << "  ";
    }
    std::cout << "\n";

    std::cout << "Initial joint angles:  ";
    for (double ang : arm_state.get_joint_angles()) {
        std::cout << ang << "  "; 
    }
    std::cout << "\n";

    if (!solver.is_safe(arm_state)) {
        std::cout << "STARTING POSITION NOT SAFE, please adjust arm in Open Loop.\n";

        DebugMessage msg;
        msg.isError = false;
        msg.message = "Unsafe Starting Position";
        
        // send popup message to GUI
        lcm_.publish("/debug_message", &msg);

        control_state = ControlState::WAITING_FOR_TARGET;
        return;
    }

    // TODO check if target is safe.

    ArmState hypo_state = arm_state;

    plan_path(hypo_state, target);
}

void MRoverArm::plan_path(ArmState& hypo_state, Vector6d goal) {
    bool path_found = motion_planner.rrt_connect(hypo_state, goal);

    if (path_found) {
        preview(hypo_state);
    }
    else {
        control_state = ControlState::WAITING_FOR_TARGET;

        DebugMessage msg;
        msg.isError = false;
        msg.message = "Unable to plan path!";
        
        // send popup message to GUI
        lcm_.publish("/debug_message", &msg);
    }
}

void MRoverArm::preview(ArmState& hypo_state) {
    std::cout << "Previewing...\n";
    control_state = ControlState::PREVIEWING;

    double num_steps = 20.0;
    double t = 0.0;

    while (t <= 1 && control_state == ControlState::PREVIEWING) {

        // set hypo_state to next position in the path
        std::vector<double> target = motion_planner.get_spline_pos(t);
        hypo_state.set_joint_angles(target);

        // update transforms
        solver.FK(hypo_state); 

        // send transforms to GUI
        publish_transforms(hypo_state);

        t += 1.0 / num_steps;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // If previewing was canceled early
    if (control_state != ControlState::PREVIEWING) {
        std::cout << "Preview interrupted.\n";

        if (control_state != ControlState::OFF) {
            std::cout << "UNEXPECTED BEHAVIOR: exited preview but did not leave closed-loop! Setting state to off.\n";
            control_state = ControlState::OFF;
        }
        return;
    }

    std::cout <<  "Preview Done\n";

    DebugMessage msg;
    msg.isError = false;
    msg.message = "Preview Done";

    // send popup message to GUI
    lcm_.publish("/debug_message", &msg);

    // Change state to ready to execute
    control_state = ControlState::READY_TO_EXECUTE;
}

void MRoverArm::motion_execute_callback(std::string channel, MotionExecute msg) {
    bool execute = msg.execute;
    std::cout << "Received execute message: " << execute << "\n";

    if (control_state != ControlState::READY_TO_EXECUTE) {
        std::cout << "Received execute message but not ready to execute!\n";

        if (control_state != ControlState::OFF) {
            std::cout << "UNEXPECTED BEHAVIOR: canceled execution but did not leave closed-loop! Setting state to off.\n";
            control_state = ControlState::OFF;
        }
        return;
    }

    if (execute) {
        // run loop inside execute_spline()
        control_state = ControlState::EXECUTING;
    }
    else {
        control_state = ControlState::WAITING_FOR_TARGET;
    }
}

void MRoverArm::execute_spline() { 
    double spline_t = 0.0;
    double spline_t_iterator;

    while (true) {
        if (control_state == ControlState::EXECUTING) {

            if (encoder_error) {
                spline_t = 0.0;

                DebugMessage msg;
                msg.isError = true;
                msg.message = encoder_error_message;

                // send popup message to GUI
                lcm_.publish("/debug_message", &msg);

                if (sim_mode) {
                    for (size_t i = 0; i < MAX_NUM_PREV_ANGLES; ++i) {
                        publish_config(arm_state.get_joint_angles(), "/arm_position");
                    }
                }

                control_state = ControlState::WAITING_FOR_TARGET;
                continue;
            }

            //find arm's current angles
            std::vector<double> init_angles = arm_state.get_joint_angles(); 
            //find angles D_SPLINE_T (%) further down the spline path
            std::vector<double> final_angles = motion_planner.get_spline_pos(spline_t + D_SPLINE_T);

            double max_time = -1; //in ms

            // Get max time to travel for joints a through e
            for (int i = 0; i < 5; ++i) {
                double max_speed = arm_state.get_joint_max_speed(i);

                //in ms, time needed to move D_SPLINE_T (%)
                double joint_time = std::abs(final_angles[i] - init_angles[i])
                    / (max_speed / 1000.0); // convert max_speed to rad/ms
                
                //sets max_time to greater value
                max_time = max_time < joint_time ? joint_time : max_time;
            }

            //determines size of iteration by dividing number of iterations by distance
            spline_t_iterator = D_SPLINE_T / (max_time / SPLINE_WAIT_TIME);
            spline_t += spline_t_iterator;

            // break out of loop if necessary
            if (spline_t > 1.0) {
                std::cout << "Finished executing!\n";
                spline_t = 0.999999999999;
                control_state = ControlState::WAITING_FOR_TARGET;
            }

            // get next set of angles in path
            std::vector<double> target_angles = motion_planner.get_spline_pos(spline_t);

            for (size_t i = 0; i < 6; ++i) {
                if (target_angles[i] < arm_state.get_joint_limits(i)[0]) {
                    target_angles[i] = arm_state.get_joint_limits(i)[0];
                }
                else if (target_angles[i] > arm_state.get_joint_limits(i)[1]) {
                    target_angles[i] = arm_state.get_joint_limits(i)[1];
                }
            }

            // std::cout << "joint b current position: " << arm_state.get_joint_angle(1) << "\n";
            // std::cout << "joint b target:           " << target_angles[1] << "\n\n";

            // if not in sim_mode, send physical arm a new target
            if (!sim_mode) {
                // TODO make publish function names more intuitive?

                // Adjust for encoders not being properly zeroed.
                for (size_t i = 0; i < 6; ++i) {
                    target_angles[i] *= arm_state.get_joint_encoder_multiplier(i);
                    target_angles[i] += arm_state.get_joint_encoder_offset(i);
                }

                publish_config(target_angles, "/ik_ra_control");
            }

            // if in sim_mode, simulate that we have gotten a new current position
            else if (sim_mode) {
                arm_state.set_joint_angles(target_angles);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds((int) SPLINE_WAIT_TIME));
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            spline_t = 0;
        }   
    }
}

void MRoverArm::simulation_mode_callback(std::string channel, SimulationMode msg) {
    sim_mode = msg.sim_mode;
    std::cout << "Received Simulation Mode value: " << sim_mode << "\n";
}

void MRoverArm::lock_joints_callback(std::string channel, LockJoints msg) {
    std::cout << "Running lock_joints_callback:   ";

    arm_state.set_joint_locked(0, (bool) msg.jointa);
    arm_state.set_joint_locked(1, (bool) msg.jointb);
    arm_state.set_joint_locked(2, (bool) msg.jointc);
    arm_state.set_joint_locked(3, (bool) msg.jointd);
    arm_state.set_joint_locked(4, (bool) msg.jointe);
    arm_state.set_joint_locked(5, (bool) msg.jointf);

    std::cout << "\n";
}

void MRoverArm::encoder_angles_sender() {
    // Continuously send mock values if in sim mode
    while (true) {
        if (sim_mode) {
            encoder_angles_sender_mtx.lock();

            ArmPosition arm_position;
            arm_position.joint_a = arm_state.get_joint_angle(0);
            arm_position.joint_b = arm_state.get_joint_angle(1);
            arm_position.joint_c = arm_state.get_joint_angle(2);
            arm_position.joint_d = arm_state.get_joint_angle(3);
            arm_position.joint_e = arm_state.get_joint_angle(4);
            arm_position.joint_f = arm_state.get_joint_angle(5);
            lcm_.publish("/arm_position", &arm_position);

            encoder_angles_sender_mtx.unlock();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(SPLINE_WAIT_TIME));
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

void MRoverArm::publish_transforms(const ArmState& arbitrary_state) {
    FKTransform tm;
    matrix_helper(tm.transform_a, arbitrary_state.get_joint_transform(0));
    matrix_helper(tm.transform_b, arbitrary_state.get_joint_transform(1));
    matrix_helper(tm.transform_c, arbitrary_state.get_joint_transform(2));
    matrix_helper(tm.transform_d, arbitrary_state.get_joint_transform(3));
    matrix_helper(tm.transform_e, arbitrary_state.get_joint_transform(4));
    matrix_helper(tm.transform_f, arbitrary_state.get_joint_transform(5));

    lcm_.publish("/fk_transform", &tm);
}      

void MRoverArm::matrix_helper(double arr[4][4], const Matrix4d &mat) {
   for (int i = 0; i < 4; ++i) {
       for (int j = 0; j < 4; ++j) {
           arr[i][j] = mat(i,j);
       }
   }
}

void MRoverArm::check_dud_encoder(std::vector<double> &angles) const {
    // For each angle
    for (size_t i = 0; i < angles.size(); ++i) {
        // For each dud value
        for (size_t j = 0; j < DUD_ENCODER_VALUES.size(); ++j) {

            // If angle is very nearly dud value
            if (std::abs(angles[i] - DUD_ENCODER_VALUES[j]) < DUD_ENCODER_EPSILON) {
                angles[i] = arm_state.get_joint_angle(i);
            }
        }
    }
}

void MRoverArm::check_joint_limits(std::vector<double> &angles) {
    // For each angle
    for (size_t i = 0; i < angles.size(); ++i) {
        std::vector<double> limits = arm_state.get_joint_limits(i);

        // If angle is only just past lower limit
        if (angles[i] < limits[0] && std::abs(angles[i] - limits[0]) < ACCEPTABLE_BEYOND_LIMIT) {
            angles[i] = limits[0];
        }

        // If angle is only just past upper limit
        else if (angles[i] > limits[1] && std::abs(angles[i] - limits[1]) < ACCEPTABLE_BEYOND_LIMIT) {
            angles[i] = limits[1];
        }

        // If angle is far outside limits
        else if (angles[i] < limits[0] || angles[i] > limits[1]) {
            encoder_error = true;
            encoder_error_message = "Encoder Error: " + std::to_string(angles[i]) + " beyond joint " + std::to_string(i) + " limits (joint A = 0, F = 5)";
        }
    }
}

double MRoverArm::joint_b_stabilizer(double angle) {
    // If prev_angle hasn't been set yet
    if (std::isnan(prev_angle_b)) {
        prev_angle_b = angle;
        return angle;
    }

    // If angle seems bad, ignore it
    if (std::isnan(angle) || std::abs(angle - prev_angle_b) > 0.3) {
        return prev_angle_b;
    }

    // Update prev angle to current angle
    prev_angle_b = JOINT_B_STABILIZE_MULTIPLIER * prev_angle_b + (1 - JOINT_B_STABILIZE_MULTIPLIER) * angle;
    return prev_angle_b;
}
