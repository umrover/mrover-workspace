#ifndef KINEMATICS_UTILS
#define KINEMATICS_UTILS

#include "json.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace nlohmann;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;


json read_json_from_file(string filepath);

double point_line_distance(Vector3d end1, Vector3d end2, Vector3d point);

double closest_dist_bet_lines(Vector3d a0, Vector3d a1, Vector3d b0, Vector3d b1);

Vector3d compute_euler_angles(Matrix3d xform_mat);

double degrees_to_radians(double degrees);

double radians_to_degrees(double radians);

Vector3d calculate_midpoint(Vector3d point1, Vector3d point2);

Vector3d calculate_center_of_mass(Vector3d joint1, Vector3d joint2, double percent_length);

Vector3d calculate_torque(Vector3d r, double mass, Vector3d rot_axis);

Vector3d apply_transformation(const Matrix4d &transform, const Vector3d &point);

Vector6d vecTo6d(vector<double> inVec);

#endif