#include "arm_state.hpp"
#include "json.hpp"
#include "utils.hpp"
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace nlohmann;
using namespace std;

// TODO: Change this value as necessary
static const int num_collision_parts = 23;

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
    for (auto it = joints.begin(); it != joints.end(); it++) {
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
    // Iterate through all angles and joints adding the angles to each corresponding joint
    auto angle_it = angles.begin();
    for (auto it = joints.begin(); it != joints.end() && angle_it != angles.end(); ++it) {
        it->second->angle = *angle_it;
        ++angle_it;
    }
}

vector<string> ArmState::get_all_joints() {
    // Returns a vector containing all of the joint names
    vector<string> joint_names;
    for (auto it = joints.begin(); it != joints.end(); ++it) {
        joint_names.push_back(it->second->name);
    }
}

// vector<string> get_all_links();

Matrix<double, num_collision_parts, num_collision_parts> ArmState::get_collision_mat() {
    return collision_mat;
}

Vector3d ArmState::get_joint_axis_world(string joint) {
    return joints[joint]->joint_axis_world;
}

Matrix4d ArmState::get_joint_transform(string joint) {
    return joints[joint]->global_transform;
}

void ArmState::set_joint_transform(string joint, Matrix4d xform) {
    joints[joint]->global_transform = xform;
}

Matrix4d ArmState::get_ef_transform() {
    return ef_xform;
}

void ArmState::set_ef_transform(Matrix4d xform) {
    ef_xform = xform;
}

void ArmState::set_joint_pos_world(string joint, Vector3d position) {
    joints[joint]->pos_world = position;
}

Vector3d ArmState::get_joint_pos_world(string joint) {
    return joints[joint]->pos_world;
}

Vector3d ArmState::get_ef_pos_world() {
    return ef_pos_world;
}

Vector6d ArmState::get_ef_pos_and_euler_angles() {}

map<string, double> ArmState::get_angles() {
    map<string, double> angles;
    for (auto it = joints.begin(); it != joints.end(); ++it) {
        angles[it->first] = it->second->angle;
    }
    return angles;
}

bool ArmState::link_link_check(map<string, Joint*>::iterator it, map<string, Joint*>::iterator jt) {
    double closest_dist;
    if (it->second->type == "capsule" && jt->second->type == "capsule") {
        Vector3d b1 = it->second->points[0];
        Vector3d b2 = jt->second->points[0];
        Vector3d e1 = it->second->points[1];
        Vector3d e2 = jt->second->points[1];
        closest_dist = closest_dist_bet_lines(b1, e1, b2, e2);
    }
    else if (it->second->type == "capsule" || jt->second->type == "capsule") {
        if (it->second->type == "capsule") {
            Vector3d b1 = it->second->points[0];
            Vector3d e1 = it->second->points[1];
            closest_dist = point_line_distance(b1, e1, jt->second->center);
        }
        else {
            Vector3d b2 = jt->second->points[0];
            Vector3d e2 = jt->second->points[1];
            closest_dist = point_line_distance(b2, e2, it->second->center);
        }
    }
    else {
        closest_dist = (it->second->center - jt->second->center).norm();
    }
    return closest_dist < (it->second->radius + jt->second->radius);
}

bool ArmState::obstacle_free() {
    auto it = joints.begin();
    for (size_t i = 0; i < joints.size(); ++i) {
        auto jt = it;
        for (size_t j = i + 1; j < joints.size(); ++j) {
            ++jt;
            if (collision_mat(i, j)) {
                // Perform link-link check
                if (link_link_check(it, jt)) {
                    return false;
                }
            }
        }
        ++it;
    }
    return true;
}

