#ifndef KINEMATICS_UTILS
#define KINEMATICS_UTILS

#include "nlohmann/json.hpp"
#include <eigen3/Eigen/Dense>

using namespace nlohmann;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;


std::string get_mrover_arm_geom();

json read_json_from_file(const std::string &filepath);

double point_line_distance(const Vector3d &end1, const Vector3d &end2, const Vector3d &point);

double closest_dist_bet_lines(const Vector3d &a0, const Vector3d &a1, const Vector3d &b0, const Vector3d &b1);

Vector3d compute_euler_angles(const Matrix3d &xform_mat);

double degrees_to_radians(double degrees);

double radians_to_degrees(double radians);

Vector3d calculate_midpoint(const Vector3d &point1, const Vector3d &point2);

Vector3d calculate_center_of_mass(const Vector3d &joint1, const Vector3d &joint2, double percent_length);

Vector3d calculate_torque(const Vector3d &r, double mass, const Vector3d &rot_axis);

Vector3d apply_transformation(const Matrix4d &transform, const Vector3d &point);

Vector6d vecTo6d(const std::vector<double> &inVec);

std::vector<double> vector6dToVec(const Vector6d &inVector6d);

#endif
