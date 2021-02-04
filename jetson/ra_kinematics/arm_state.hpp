#ifndef ARM_STATE_H
#define ARM_STATE_H

#include <fstream>
#include "json.hpp"
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace nlohmann;
using namespace std;

typedef Matrix<double, 6, 1> Vector6d;

class ArmState{

private:
    struct Link {
        string name;
        Matrix4d global_transform;
    };

    struct Joint {
        
        Joint(string name_in, const json& joint_geom)
            : name(name_in), angle(0), pos_world(Vector3d::Zero(3)), 
              global_transform(Matrix4d::Identity()), torque(Vector3d::Zero(3)) 
              {
                  pos_local << joint_geom["origin"]["xyz"][0], joint_geom["origin"]["xyz"][1], joint_geom["origin"]["xyz"][2];
                  local_center_of_mass << joint_geom["mass_data"]["com"]["x"], joint_geom["mass_data"]["com"]["y"], joint_geom["mass_data"]["com"]["z"];

                  rot_axis << joint_geom["axis"][0], joint_geom["axis"][1], joint_geom["axis"][2];

                  joint_limits["lower"] = joint_geom["limit"]["lower"];
                  joint_limits["upper"] = joint_geom["limit"]["upper"];
                  child_link = joint_geom["child"];
                  // joints b and c get an angle of 0, the rest should get 0   
                  if (name_in == "joint_b" || name_in == "joint_c") {
                      angle = 1.0;
                  }
                  else{
                      angle = 0;
                  } 
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
        map<string, double> joint_limits;
    };

    struct Avoidance_Link {

        Avoidance_Link(int link_num_in, string joint_origin_name, json &link_json) : 
                           link_num(link_num_in), joint_origin(joint_origin_name) {

            type = link_json["type"];
            radius = link_json["radius"];

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
        string joint_origin;
        string type;
        vector<Vector3d> points;
        double radius;
    };

    vector<string> joint_names;
    vector<string> link_names;

    vector<Avoidance_Link> collision_avoidance_links; // could make this an array
    // TODO: Change num_collision_parts value as necessary
    static const int num_collision_parts = 23;
    Vector3d ef_pos_world;
    Matrix4d ef_xform;
    Matrix<double, num_collision_parts, num_collision_parts> collision_mat;

    void add_joint(string joint, json &joint_geom);

    void add_avoidance_link(int link_num, string joint_origin_name, json &link_json);

    void transform_parts();

    void delete_joints();

public:
    json joints_json;
    json links_json;
    map<string, Joint> joints;
    map<string, Link> links;
    ArmState(json &geom);
    
    // ~ArmState();

    vector<string> get_all_joints() const;

    vector<string> get_all_links();

    Vector3d get_joint_com(string joint);

    double get_joint_mass(string joint);

    Vector3d get_joint_axis(string joint);

    Vector3d get_joint_axis_world(string joint);

    map<string, double> get_joint_limits(string joint) const;

    Matrix4d get_joint_transform(string joint);

    void set_joint_transform(string joint, Matrix4d xform);

    void set_link_transform(string link, Matrix4d xform);

    Matrix4d get_ef_transform();

    void set_ef_transform(Matrix4d xform);

    void set_joint_pos_world(string joint, Vector3d position);

    Vector3d get_joint_pos_world(string joint);

    Vector3d get_joint_pos_local(string joint);

    Vector3d get_ef_pos_world();

    Vector3d get_ef_ang_world();

    void set_ef_pos_world(Vector3d ef_pos);

    vector<double> get_ef_pos_and_euler_angles();

    map<string, double> get_joint_angles();

    void set_joint_angles(const vector<double>& angles);

    void transform_avoidance_links();

    bool link_link_check(vector<Avoidance_Link>::iterator it, vector<Avoidance_Link>::iterator jt);

    bool obstacle_free();

    int num_joints();

    string get_child_link(string joint);

    Vector3d get_joint_torque(string joint);

    void set_joint_torque(string joint, Vector3d torque);

    Vector3d get_link_point_world(string link);

};

#endif