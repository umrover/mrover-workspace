#ifndef ARM_STATE_H
#define ARM_STATE_H

#include <fstream>
#include "json.hpp"
#include <vector>
#include <Eigen/Dense>

using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Matrix;
using namespace nlohmann;
using namespace std;

class ArmState{

private:
    struct Joint {
        
        Joint(string name_in, json joint_geom)
            : name(name_in), angle(0), pos_world(Vector3d::Zero(3)), 
              global_transform(Matrix4d::Identity()), torque(Vector3d::Zero(3)) {}

        string name;
        double angle;
        Vector3d pos_world;
        Matrix4d global_transform;
        Vector3d local_center_of_mass;
        Vector3d torque;
        Vector3d rot_axis;
    };

    vector<Joint> joints;
    static const int num_total_collision_parts = 23;
    Vector3d ef_pos_world;
    Matrix4d ef_xform;
    Matrix<double, num_total_collision_parts, num_total_collision_parts> collision_mat;


public:
    ArmState(json &geom);

    // vector<string> get_all_joints();

    // vector<string> get_all_links();

    // get_collision_mat();

    double get_joint_com(string joint);

    double get_joint_mass(string joint);

    double get_joint_axis(string joint);

    void set_joint_angles(vector<double> angles);



};

#endif