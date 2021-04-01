#include "unit_test_framework.h"
#include "../json.hpp"
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

    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";
    json geom = read_json_from_file(geom_file);

    ArmState arm(geom);
    KinematicsSolver solver;

    ConfigSpaceTest tester;

    string filename = "valid_configurations_2021.csv";

    if (CREATE_FILE) {
        tester.write_valid_configs(arm, solver, filename);
    }
    if (IK_TEST) {
        tester.test_IK(arm, solver, filename);
    }
}







TEST_MAIN()