#include "arm_state.hpp"
#include "json.hpp"
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace nlohmann;
using namespace std;

ArmState::ArmState(json &geom) : ef_pos_world(Vector3d::Zero()), ef_xform(Matrix4d::Identity()) {
    json joints_json = geom["joints"];
    for (json::iterator it = joints_json.begin(); it != joints_json.end(); it++) {
        add_joint(it.key(), it.value());
    }
}

ArmState::~ArmState() {
    delete_joints();
}

void ArmState::add_joint(string name, json &joint_geom) {
    joints[name] = new Joint(name, joint_geom);
}

void ArmState::delete_joints() {
    map<string, Joint *>::iterator it;
    for (it = joints.begin(); it != joints.end(); it++) {
        delete it->second;
    }
}

Vector3d ArmState::get_joint_pos_local(string joint) {
    return joints[joint]->pos_local;
}

Vector3d ArmState::get_joint_axis(string joint) {
    return joints[joint]->rot_axis;
}

Vector3d ArmState::get_joint_com(string joint) {
    return joints[joint]->local_center_of_mass;
}

double ArmState::get_joint_mass(string joint) {
    cout << "joint mass for joint: " << joint << "\n";
    cout << "end effector position: \n" << ef_pos_world << "\n";
    return 0;
}

map<string, double> ArmState::get_joint_limits(string joint) {
    return joints[joint]->joint_limits;
}

void ArmState::set_joint_angles(vector<double> angles) {
    // TODO
}


