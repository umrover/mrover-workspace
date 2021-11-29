#include "unit_test_framework.h"
#include "nlohmann/json.hpp"
#include "../arm_state.hpp"
#include "../kinematics.hpp"
#include "../utils.hpp"

using namespace std;

#define CREATE_FILE true
#define IK_TEST true

#include <string>
#include <map>

#include <iostream>
#include <fstream>
#include <time.h>

class ConfigSpaceTest {
private:

    size_t num_valid_points;
    vector< vector<double> > limits;

public:

    void write_valid_configs(ArmState &arm, KinematicsSolver &solver, string filename) {
        ofstream file(filename);

        num_valid_points = 0;

        vector<double> rand_angs;
        rand_angs.resize(6);

        Vector6d pos;

        while (num_valid_points < 500) {
            rand_angs[0] = ((double) rand() / (RAND_MAX) - 0.5) * 4;  // -2   ... 2
            rand_angs[1] = ((double) rand() / (RAND_MAX)) + 0.25;     // 0.25 ... 1.25
            rand_angs[2] = ((double) rand() / (RAND_MAX) - 0.5) * 3;  // -1.5 ... 1.5
            rand_angs[3] = ((double) rand() / (RAND_MAX) - 0.5) * 6;  // -3   ... 3
            rand_angs[4] = ((double) rand() / (RAND_MAX) - 0.75) * 2; // -1.5 ... 0.5
            rand_angs[5] = ((double) rand() / (RAND_MAX) - 0.5) * 6;  // -3   ... 3

            if (solver.is_safe(arm, rand_angs)) {
                arm.set_joint_angles(rand_angs);
                solver.FK(arm);

                pos.head(3) = arm.get_ef_pos_world();
                pos.tail(3) = arm.get_ef_ang_world();

                for (size_t i = 0; i < 6; ++i) {
                    file << pos[i] << ",";
                }
                file << "\n";

                ++num_valid_points;
            }
        }

        std::cout << "created " << num_valid_points << " points\n";
        file.close();
    }
    
    void test_IK(ArmState &arm, KinematicsSolver &solver, string input_filename) {
        ifstream file(input_filename);
        string line, cell;

        size_t tot_attempts = 0;
        size_t tot_success = 0;

        Vector6d target_pos;

        while (getline(file, line)) {
            stringstream ss(line);

            size_t i = 0;
            
            while (getline(ss, cell, ',')) {
                target_pos[i] = atof(cell.c_str());

                if (i == 5) {
                    break;
                }
                ++i;
            }

            pair<Vector6d, bool> ik_solution;

            // attempt to find ik_solution, starting at up to 25 random positions
            for(int i = 0; i < 25; ++i) {
                ik_solution = solver.IK(arm, target_pos, true, true);

                if (ik_solution.second) {
                    ++tot_success;
                    break;
                }
            }
            
            ++tot_attempts;

            if (tot_attempts % 10 == 0) {
                cout << "total success: " << tot_success << " out of " << tot_attempts << " attempts.\n";
            }
        }

        std::cout << "Number of successes: " << tot_success << "\n";
        std::cout << "Number of attempts: " << tot_attempts << "\n";
        std::cout << "Success rate: " << ((double) tot_success) / ((double) tot_attempts) << "\n";

        file.close();
    }
};

TEST(config_space) {
    json geom = read_json_from_file(get_mrover_arm_geom());

    ArmState arm(geom);
    KinematicsSolver solver;

    ConfigSpaceTest tester;

    string filename = "jetson/ra_kinematics/test/valid_configurations_2022.csv";

    srand (time(NULL));

    if (CREATE_FILE) {
        tester.write_valid_configs(arm, solver, filename);
    }
    if (IK_TEST) {
        tester.test_IK(arm, solver, filename);
    }
}


TEST_MAIN()
