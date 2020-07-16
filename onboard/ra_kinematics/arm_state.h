#ifndef ARM_STATE_H
#define ARM_STATE_H

#include <fstream>
#include "json.hpp"
#include <vector>

using namespace nlohmann;
using namespace std;

class ArmState{

private:
    struct Joint {
        
        Joint(string name_in) : name(name_in) {}

        string name;
    };

public:
    ArmState();

    // vector<string> get_all_joints();

    // vector<string> get_all_links();

    // get_collision_mat();

    double get_joint_com(string joint);

    double get_joint_mass(string joint);

    double get_joint_axis(string joint);



};

#endif