#include "unit_test_framework.h"
#include "nlohmann/json.hpp"
#include "../arm_state.hpp"
#include "../kinematics.hpp"
#include "../utils.hpp"

using namespace std;

#define CREATE_FILE false
#define IK_TEST true
#define ANGLE_INCREMENT 0.45

#include <string>
#include <map>

#include <iostream>
#include <fstream>

class ConfigSpaceTest {
private:

    size_t num_valid_points;
    size_t num_joints;
    vector< map<string, double> > limits;

public:

    void write_valid_configs(ArmState &arm, KinematicsSolver &solver, string filename) {
        ofstream file(filename);

        num_joints = (size_t) arm.num_joints();
        num_valid_points = 0;

        vector<double> curr_angles;

        vector<string> joint_names = arm.get_all_joints();
        for (const string& joint : joint_names) {
            limits.push_back(arm.get_joint_limits(joint));
            curr_angles.push_back(arm.get_joint_limits(joint)["lower"]);
        }

        increment_loop(arm, solver, file, 0, curr_angles);

        std::cout << "created " << num_valid_points << " points\n";

        file.close();
    }
    
    void test_IK(ArmState &arm, KinematicsSolver &solver, string input_filename) {
        ifstream file(input_filename);
        string line, cell;

        size_t tot_attempts = 0;
        size_t tot_success = 0;

        num_joints = (size_t) arm.num_joints();

        vector<double> target_angs;
        target_angs.resize(num_joints);

        while (getline(file, line)) {
            stringstream ss(line);

            size_t i = 0;
            
            while (getline(ss, cell, ',')) {
                target_angs[i] = atof(cell.c_str());

                if (i == 5) {
                    break;
                }
                ++i;
            }

            arm.set_joint_angles(target_angs);
            solver.FK(arm);

            Vector6d target;
            target.head(3) = arm.get_ef_pos_world();
            target.tail(3) = arm.get_ef_ang_world();

            // attempt to find ik_solution with random starting position
            pair<Vector6d, bool> ik_solution = solver.IK(arm, target, true, false);

            // attempt to find ik_solution, starting at up to 10 random positions
            for(int i = 0; i < 10; ++i) {
                if(ik_solution.second) {
                    break;
                }
                ik_solution = solver.IK(arm, target, true, false);
            }

            if (ik_solution.second) {
                ++tot_success;
            }
            
            ++tot_attempts;

            if (tot_attempts % 20 == 0) {
                cout << "total success: " << tot_success << " out of " << tot_attempts << " attempts.\n";
                // if (ik_solution.second) {

                //     vector<double> sol_angs;
                //     sol_angs.reserve(6);
                //     for (size_t i = 0; i < 6; ++i) {
                //         sol_angs.push_back(ik_solution.first[i]);
                //     }

                //     cout << "transferred angles";

                //     arm.set_joint_angles(sol_angs);
                //     solver.FK(arm);

                //     Vector6d sol;
                //     sol.head(3) = arm.get_ef_pos_world();
                //     sol.tail(3) = arm.get_ef_ang_world();

                //     cout << "target point: " << "\n" << target[0] << "\n" << target[1] << "\n" << target[2] << "\n";
                //     cout << "solution point: " << "\n" << sol[0] << "\n" << sol[1] << "\n" << sol[2] << "\n";

                //     cout << "target orientation: " << "\n" << target[3] << "\n" << target[4] << "\n" << target[5] << "\n";
                //     cout << "solution orientation: " << "\n" << sol[3] << "\n" << sol[4] << "\n" << sol[5] << "\n\n";
                // }
                // else {
                //     cout << "(previous attempt failed)\n\n";
                // }
            }
        }

        std::cout << "Number of successes: " << tot_success << "\n";
        std::cout << "Number of attempts: " << tot_attempts << "\n";
        std::cout << "Success rate: " << ((double) tot_success) / ((double) tot_attempts) << "\n";

        file.close();
    }

private:

    void increment_loop(ArmState &arm, KinematicsSolver &solver, ofstream &file,
                                size_t current_joint, vector<double> &curr_angles) {
        
        if (current_joint == num_joints - 1) {
            if (solver.is_safe(arm, curr_angles)) {
                ++num_valid_points;
                
                for (double ang : curr_angles) {
                    file << ang << ",";
                }
                file << "\n";
            }

            return;
        }

        while (curr_angles[current_joint] <= limits[current_joint]["upper"]) {
            increment_loop(arm, solver, file, current_joint + 1, curr_angles);
            curr_angles[current_joint] += ANGLE_INCREMENT;
        }

        curr_angles[current_joint] = limits[current_joint]["lower"];
    }

};

TEST(config_space) {
    json geom = read_json_from_file(get_mrover_arm_geom());

    ArmState arm(geom);
    KinematicsSolver solver;

    ConfigSpaceTest tester;

    string filename = "jetson/ra_kinematics/test/valid_configurations_2021.csv";

    if (CREATE_FILE) {
        tester.write_valid_configs(arm, solver, filename);
    }
    if (IK_TEST) {
        tester.test_IK(arm, solver, filename);
    }
}


TEST_MAIN()
