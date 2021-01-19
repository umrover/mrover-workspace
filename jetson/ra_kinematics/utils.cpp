#include "json.hpp"
#include "utils.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

using namespace nlohmann;
using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

json read_json_from_file(string filepath) {
    ifstream file(filepath);

    json config;

    file >> config;
    return config;
}

double point_line_distance(Vector3d end1, Vector3d end2, Vector3d point) {
    double t = 0;
    for (int i = 0; i < 3; ++i){
        t += (point[i] - end1[i]) * (end2[i] - end1[i]);
    }
    if (t < 0) {
        t = 0;
    }
    else {
        t = min(t, 1.0);
    }
    Vector3d closest_point (0, 0, 0);
    for (int i = 0; i < 3; ++i){
        closest_point[i] = end1[i] + t * (end2[i] - end1[i]);
    }
    return (closest_point - point).norm();
}

double closest_dist_bet_lines(Vector3d a0, Vector3d a1, Vector3d b0, Vector3d b1) {
    float SMALL_NUM = 0.00000001;
    Vector3d u = a1 - a0;
    Vector3d v = b1 - b0;
    Vector3d w = a0 - b0;
    float a = u.dot(u);         // always >= 0
    float b = u.dot(v);
    float c = v.dot(v);         // always >= 0
    float d = u.dot(w);
    float e = v.dot(w);
    float D = a*c - b*b;        // always >= 0
    float sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    float tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;         // force using point P0 on segment S1
        sD = 1.0;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    // get the difference of the two closest points
    Vector3d dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

    return dP.norm();
}

Vector3d compute_euler_angles(Matrix3d xform_mat) {
    double alpha = atan2((double)xform_mat(0,2), -(double)xform_mat(1,2));
    double beta = acos((double)xform_mat(2,2));
    double gamma = atan2((double)xform_mat(2,0), (double)xform_mat(2,1));

    Vector3d angles(alpha, beta, gamma);
    return angles;
}

double degrees_to_radians(double degrees) {
    return degrees * 2 * acos(0.0) / 180;
}

double radians_to_degrees(double radians) {
    return radians * 180 / (2 * acos(0.0));
}

Vector3d calculate_midpoint(Vector3d point1, Vector3d point2){
    return (point2 + point1)/2;
}

Vector3d calculate_center_of_mass(Vector3d joint1, Vector3d joint2, double percent_length) {
    return percent_length * (joint2 - joint1) + joint1;
}

Vector3d calculate_torque(Vector3d r, double mass, Vector3d rot_axis){
    double g = -9.807;
    Vector3d force = {0, 0, g * mass};
    return r.cross(force);
}

Vector3d apply_transformation(const Matrix4d &transform, const Vector3d &point) {
    Vector4d four_d(point[0], point[1], point[2], 1);
    Vector4d transformed = transform * four_d;
    Vector3d vector_transformed(transformed(0), transform(1), transform(2));
    return vector_transformed;
}

Vector6d vecTo6d(vector<double> inVec) {
    Vector6d retVec;
    for (int i = 0; i < 6; ++i) {
        retVec(i) = inVec[i];
    }
    return retVec;
}