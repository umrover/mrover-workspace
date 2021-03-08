#include "arm_state.hpp"
#include "json.hpp"
#include "utils.hpp"
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

using namespace Eigen;
using namespace nlohmann;
using namespace std;

// Tested in joint creation test
ArmState::ArmState(json &geom) : ef_pos_world(Vector3d::Zero()), ef_xform(Matrix4d::Identity()) {

    // Create all Joint instances from mrover_arm json
    set_ef_xyz(geom["endeffector"]["origin"]["xyz"]);

    json joints_json = geom["joints"];
    for (json::iterator it = joints_json.begin(); it != joints_json.end(); it++) {
        add_joint(it.key(), it.value());
    }

    // Create all Avoidance_Link instances from mrover_arm json (for collision avoidance)
    json links_json = geom["links"];
    for (json::iterator it = links_json.begin(); it != links_json.end(); it++) {
        // Create the link object:
        links[it.key()].name = it.key();
        links[it.key()].global_transform = Matrix4d::Identity();

        string joint_origin = it.value()["visual"]["origin"]["joint_origin"];
        vector<size_t> collisions = it.value()["collisions"];
        
        json link_shapes = it.value()["link_shapes"];
        for (json::iterator jt = link_shapes.begin(); jt != link_shapes.end(); jt++) {
            try {
                collision_avoidance_links.emplace_back(joint_origin, jt.value(), collisions);
            }
            catch (json::exception& e) {
                cout << "Error creating avoidance link: " << e.what() << "\n"
                     << "exception id: " << e.id << std::endl;
            }
        }
    }

    // Sort links by link_num, since they may not be in order from the json
    Link_Comp comparator;
    sort(collision_avoidance_links.begin(), collision_avoidance_links.end(), comparator);
}

bool ArmState::Link_Comp::operator()(const Avoidance_Link& a, const Avoidance_Link& b) {
    return a.link_num < b.link_num;
}

// Tested in joint_creation_test
void ArmState::add_joint(string name, json &joint_geom) {
    // Add joint with given configuration to map of joints - used during initialization
    // Joint(name, joint_geom);
    joints.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(name, joint_geom));
    joint_names.push_back(name);
}

Vector3d ArmState::get_joint_pos_local(string joint) {
    // Return joint position relatice to local frame (position relative to previous joint)

    // TODO write function to handle possible exceptions with call to at()
    return joints.at(joint).pos_local;
}

Vector3d ArmState::get_joint_axis(string joint) {
    // Return local axis of rotation
    return joints.at(joint).rot_axis;
}

Vector3d ArmState::get_joint_com(string joint) {
    // Return center of mass of specific link relative to the joint origin
    Matrix4d transform = get_joint_transform(joint);
    Matrix4d translation = Matrix4d::Identity();
    translation.block(0,3,3,1) = joints.find(joint)->second.local_center_of_mass;
    Matrix4d output = transform*translation;
    Vector3d out_vec(output(0,3), output(1,3), output(2,3));
    return out_vec;
}

double ArmState::get_joint_mass(string joint) {
    // TODO: Consider adding mass to the joint struct?
    return joints.at(joint).mass;
}

map<string, double> ArmState::get_joint_limits(string joint) const {
    // Returns a map of the joint rotation limits in radians.
    // Map should have a "lower" value and "upper" value.
    return joints.at(joint).joint_limits;
}
// Tested in set_joint_angles_test
void ArmState::set_joint_angles(const vector<double>& angles) {
    // TODO consider clipping invalid angles or throwing error

    // Iterate through all angles and joints adding the angles to each corresponding joint.
    // angles vector should be same size as internal joints map.
    int i = 0;
    for (auto it = joints.begin(); it != joints.end(); ++it) {
        it->second.angle = angles[i];
        ++i;
    }
}

vector<string> ArmState::get_all_joints() const {
    // Return a vector containing all of the joint names
    return joint_names;
}

Vector3d ArmState::get_joint_axis_world(string joint) {
    return joints.at(joint).joint_axis_world;
}

Matrix4d ArmState::get_joint_transform(string joint) {
    return joints.at(joint).global_transform;
}

void ArmState::set_joint_transform(string joint, Matrix4d xform) {
    joints.at(joint).global_transform = xform;
}

void ArmState::set_link_transform(string link, Matrix4d xform) {
    links[link].global_transform = xform;
}

Matrix4d ArmState::get_ef_transform() {
    return ef_xform;
}

void ArmState::set_ef_transform(Matrix4d xform) {
    ef_xform = xform;
}

Vector3d ArmState::get_joint_pos_world(string joint) {
    Matrix4d joint_xform = get_joint_transform(joint);
    Vector3d joint_pos(joint_xform(0,3), joint_xform(1,3), joint_xform(2,3));
    return joint_pos;
}

Vector3d ArmState::get_ef_pos_world() {
    Matrix4d trans_mat = get_ef_transform();
    Vector3d ret_vec(trans_mat(0,3), trans_mat(1,3), trans_mat(2,3));
    return ret_vec;
}

Vector3d ArmState::get_ef_ang_world() {
    Matrix3d ang_xform = ef_xform.block(0,0,3,3);
    return compute_euler_angles(ang_xform);
}

// TODO: Do we need to implement this function?
vector<double> ArmState::get_ef_pos_and_euler_angles() {
    Vector3d ef_pos_vec = get_ef_pos_world();
    Vector3d ef_ang_world = get_ef_ang_world();
    vector<double> ef_pos_and_angles;
    for (int i = 0; i < 3; ++i) {
        ef_pos_and_angles.push_back(ef_pos_vec(i));
    }
    for (int i = 3; i < 6; ++i) {
        ef_pos_and_angles.push_back(ef_ang_world(i-3));
    }
    return ef_pos_and_angles;
}

// Tested in set_joint_angles_test
map<string, double> ArmState::get_joint_angles() {
    map<string, double> angles;
    for (auto it = joints.begin(); it != joints.end(); ++it) {
        angles[it->first] = it->second.angle;
    }
    return angles;
}

void ArmState::transform_avoidance_links() {
    for (size_t i = 0; i < collision_avoidance_links.size(); ++i) {
        Avoidance_Link& link = collision_avoidance_links.at(i);

        const Matrix4d &joint_xform = get_joint_transform(link.joint_origin);
        
        for (size_t j = 0; j < link.points.size(); ++j) {
            link.points[j] = apply_transformation(joint_xform, link.points[j]);
        }
    }
}

// Tested in link_link_check_test
bool ArmState::link_link_check(size_t index_1, size_t index_2) {
    double closest_dist;
    Avoidance_Link& link_1 = collision_avoidance_links.at(index_1);
    Avoidance_Link& link_2 = collision_avoidance_links.at(index_2);

    if (link_1.type == "capsule" && link_2.type == "capsule") {
        Vector3d b1 = link_1.points[0];
        Vector3d b2 = link_2.points[0];
        Vector3d e1 = link_1.points[1];
        Vector3d e2 = link_2.points[1];
        closest_dist = closest_dist_bet_lines(b1, e1, b2, e2);
    }
    else if (link_1.type == "capsule") {
        Vector3d b1 = link_1.points[0];
        Vector3d e1 = link_1.points[1];
        Vector3d center = link_2.points[0];
        closest_dist = point_line_distance(b1, e1, center);
    }
    else if (link_2.type == "capsule") {
        Vector3d b2 = link_2.points[0];
        Vector3d e2 = link_2.points[1];
        Vector3d center = link_1.points[0];
        closest_dist = point_line_distance(b2, e2, center);
    }
    else {
        closest_dist = (link_1.points[0] - link_2.points[0]).norm();
    }

    return closest_dist < (link_1.radius + link_2.radius);
}

// Tested in link_link_check_test
bool ArmState::obstacle_free() {
    transform_avoidance_links();
    
    for (size_t i = 1; i < collision_avoidance_links.size(); ++i) {
        for (size_t possible_collision : collision_avoidance_links[i].collisions) {
            if (link_link_check(i, possible_collision)) {
                return false;
            }
        }
    }
    return true;
}

// Used for testing ArmState functions
int ArmState::num_joints() {
    return joints.size();
}

string ArmState::get_child_link(string joint){
    return joints.at(joint).child_link;
}

Vector3d ArmState::get_joint_torque(string joint){
    return joints.at(joint).torque;
}

void ArmState::set_joint_torque(string joint, Vector3d torque){
    joints.at(joint).torque = torque;
}

Vector3d ArmState::get_link_point_world(string link){
    Vector3d link_point_world;
    link_point_world(0) = links[link].global_transform(0,3);
    link_point_world(1) = links[link].global_transform(1,3);
    link_point_world(2) = links[link].global_transform(2,3);
    return link_point_world;
}

Matrix4d ArmState::get_link_xform(string link) {
    return links[link].global_transform;
}

Vector3d ArmState::get_ef_xyz() {
    return ef_xyz;
}

void ArmState::set_ef_xyz(vector<double> ef_xyz_vec){
    ef_xyz(0) = ef_xyz_vec[0];
    ef_xyz(1) = ef_xyz_vec[1];
    ef_xyz(2) = ef_xyz_vec[2];
}
