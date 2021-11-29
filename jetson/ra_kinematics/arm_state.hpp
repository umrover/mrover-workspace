#ifndef ARM_STATE_H
#define ARM_STATE_H

#include <fstream>
#include <vector>
#include <iostream>

#include "nlohmann/json.hpp"
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace nlohmann;
using namespace std;

typedef Matrix<double, 6, 1> Vector6d;

/**
 * Represent MRover's 6 DOF arm
 * */
class ArmState{
private:

    struct Link {
        string name;
        Matrix4d global_transform;
    };

    struct Joint {
        
        /**
         * Construct joint from json input
         * */
        Joint(string name_in, const json &joint_geom)
            : name(name_in), angle(0), pos_world(Vector3d::Zero(3)), 
              global_transform(Matrix4d::Identity()), torque(Vector3d::Zero(3)) 
        {
            pos_local << joint_geom["origin"]["xyz"][0], joint_geom["origin"]["xyz"][1], joint_geom["origin"]["xyz"][2];
            local_center_of_mass << joint_geom["mass_data"]["com"]["x"], joint_geom["mass_data"]["com"]["y"], joint_geom["mass_data"]["com"]["z"];

            rot_axis << joint_geom["axis"][0], joint_geom["axis"][1], joint_geom["axis"][2];

            // TODO check with RA whether joint limits are correct
            joint_limits.push_back(joint_geom["limit"]["lower"]);
            joint_limits.push_back(joint_geom["limit"]["upper"]);

            child_link = joint_geom["child"];

            max_speed = joint_geom["max_speed"];
            // TODO check if mass is necessary. If so, initialize here

            // joints b and c get an angle of 0, the rest should get 0   
            if (name_in == "joint_b") {
                angle = 1.0;
            }
            else if (name_in == "joint_c") {
                angle = 0.5;
            }
            else {
                angle = 0;
            }

            continuous_range = (joint_limits[0] < -3.1399 && joint_limits[1] > 3.1399);

            locked = false;
        }

        string name;
        double angle;
        double mass;
        Vector3d pos_world;
        Matrix4d global_transform;
        string child_link;
        Vector3d torque;
        Vector3d pos_local;
        Vector3d local_center_of_mass;
        Vector3d rot_axis;
        Vector3d joint_axis_world;
        vector<double> joint_limits;
        double max_speed; // radians/s
        bool locked;
        bool continuous_range;
    };

    struct Avoidance_Link {

        Avoidance_Link(size_t joint_origin_index, json &link_json, vector<size_t> collisions) : 
                           joint_origin(joint_origin_index), collisions(collisions) {

            type = link_json["type"];
            radius = link_json["radius"];
            link_num = link_json["link_num"];

            if (type == "sphere") {
                Vector3d p1(link_json["center"]["x1"], link_json["center"]["x1"], link_json["center"]["x1"]);
                points.push_back(p1);
            }
            else if (type == "capsule") {
                Vector3d p1(link_json["point_1"]["x1"], link_json["point_1"]["y1"], link_json["point_1"]["z1"]);
                Vector3d p2(link_json["point_2"]["x2"], link_json["point_2"]["y2"], link_json["point_2"]["z2"]);
                points.push_back(p1);
                points.push_back(p2);
            }
        }

        int link_num;
        size_t joint_origin;
        string type;
        vector<Vector3d> points;
        double radius;
        vector<size_t> collisions;
    };

    vector<string> joint_names;
    vector<string> link_names;

    vector<Joint> joints;
    vector<Link> links;

    vector<Avoidance_Link> collision_avoidance_links; // could make this an array
    // TODO: Change num_collision_parts value as necessary
    static const int num_collision_parts = 23;
    Vector3d ef_pos_world;
    Matrix4d ef_xform;
    Vector3d ef_xyz;


    void add_joint(string joint, json &joint_geom);

    void transform_parts();

    struct Link_Comp {
        bool operator()(const Avoidance_Link &a, const Avoidance_Link &b);
    };

    void set_ef_xyz(vector<double> ef_xyz_vec);

public:
    ArmState(json &geom);

    vector<string> get_all_joints() const;

    Vector3d get_joint_com(size_t joint_index) const;

    double get_joint_mass(size_t joint_index) const;

    Vector3d get_joint_axis(size_t joint_index) const;

    Vector3d get_joint_axis_world(size_t joint_index) const;

    vector<double> get_joint_limits(size_t joint_index) const;

    Matrix4d get_joint_transform(size_t joint_index) const;

    void set_joint_transform(size_t joint_index, const Matrix4d &xform);

    void set_link_transform(size_t link_index, const Matrix4d &xform);

    Matrix4d get_ef_transform() const;

    void set_ef_transform(const Matrix4d &xform);

    Vector3d get_joint_pos_world(size_t joint_index) const;

    Vector3d get_joint_pos_local(size_t joint_index) const;

    Vector3d get_ef_pos_world() const;

    Vector3d get_ef_ang_world() const;

    vector<double> get_ef_pos_and_euler_angles() const;

    vector<double> get_joint_angles() const;

    void set_joint_angles(const vector<double> &angles);

    void transform_avoidance_links();

    /**
     * @param index1 index for collision_avoidance_links
     * @param index2 index for collision_avoidance_links
     * 
     * Returns true if there is a collision between the links
     * */
    bool link_link_check(size_t index_1, size_t index_2) const;

    bool obstacle_free();

    int num_joints() const;

    string get_child_link(size_t joint_index) const;

    Vector3d get_joint_torque(size_t joint_index) const;

    void set_joint_torque(size_t joint_index, const Vector3d &torque);

    Vector3d get_link_point_world(size_t link_index);

    Matrix4d get_link_xform(size_t link_index);

    Vector3d get_ef_xyz() const;

    double get_joint_max_speed(size_t joint_index) const;

    bool get_joint_locked(size_t joint_index) const;

    void set_joint_locked(size_t joint_index, bool is_locked);

    double get_joint_angle(size_t joint_index) const;

    void set_joint_angle(size_t joint_index, double angle);

    bool is_continuous(size_t joint_index);
};

#endif
