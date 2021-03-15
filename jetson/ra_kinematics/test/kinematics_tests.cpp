#include "unit_test_framework.h"
#include "../json.hpp"
#include "../arm_state.hpp"
#include "../utils.hpp"
#include "../kinematics.hpp"

#include <iostream>
#include <string>
#include <iomanip>
#include <chrono>

using namespace nlohmann;
using namespace std;
using namespace std::chrono;

typedef Eigen::Matrix<double, -1, -1> MatrixXd;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/* 
Here are some varius tests we can use to test kinematics.cpp once it is written (using 2019 configuration):

NOTE: The algorithm should be able to approach the target point but will not
necessarily pass the safety check!

NOTE: These points were reached by setting random_angles = true and e_locked = false

Point 1: [0.3975877299536904,-0.595437721432015,0.05701726525706301,-2.888598515425053,0.5056772404722869,-0.2702381850292698]
            NOTE: This point returns and passes the safety check.

Point 2: [0.41735297576788866,-0.6301581425659601,-0.09231003891980295,-1.302881860149742,0.13410241193690717,-1.8221069366052942]
            NOTE: This point returns and passes the safety check.

Point 3: [0.43561896709482717,-0.5849202310118653,-0.23898894427981895,-0.19091988273941293,0.5705597354134033,-2.8999168062356357]
            Note: This point returns and passes the safety check

NOTE: For more testing points, see valid_configurations_2.csv located towards the end of the mrover directory.


*/

bool transMatAlmostEqual(Matrix4d a, Matrix4d b, double epsilon) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (abs(a(i, j) - b(i, j)) > epsilon) {
                return false;
            }
        }
    }
    return true;
}

bool vec3dAlmostEqual(Vector3d a, Vector3d b, double epsilon) {
    for (int i = 0; i < 3; ++i) {
        if (abs(a(i)-b(i)) > epsilon) {
            return false;
        }
    }
    return true;
}

TEST(kinematics_initialization) {
    cout << setprecision(8);
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";
    
    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    KinematicsSolver solver();
}

TEST(fk_test) {
    cout << setprecision(8);
    // Create the arm to be tested on:
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    vector<double> angles;
    angles.push_back(0);
    angles.push_back(1);
    angles.push_back(1);
    angles.push_back(0);
    angles.push_back(0);
    angles.push_back(0);

    arm.set_joint_angles(angles);

    cout << "ef_pos:\n";
    cout << arm.get_ef_pos_world() << "\n";
    solver.FK(arm);
    cout << "ef_pos after FK:\n";
    cout << arm.get_ef_pos_world() << "\n";

    Matrix4d a_b;
    double epsilon = 0.001;
    a_b << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Matrix4d b_c;
    b_c <<  1,          0,           0,           0,        
            0,          0.54030231, -0.84147098,  0.03429,
            0,          0.84147098,  0.54030231,  0.02858,
            0,          0,           0,           1;
    Matrix4d c_d;
    c_d <<  1,          0,           0,          -0.0584,
            0,         -0.41614684, -0.90929743,  0.33986104,
            0,          0.90929743, -0.41614684,  0.52210772,
            0,          0,           0,           1;
    Matrix4d d_e;
    d_e << 1,          0,           0,          -0.102723,
           0,         -0.41614684, -0.90929743,  0.31556032,
           0,          0.90929743, -0.41614684,  0.57520578,
           0,          0,           0,           1;
    Matrix4d e_f;
    e_f << 1,          0,           0,           -0.0443284,
           0,         -0.41614684, -0.90929743,  0.13792929,
           0,          0.90929743, -0.41614684,  0.96333666,
           0,          0,           0,            1;
    Matrix4d hand;
    hand << 1,          0,           0,           -0.0236782,
            0,         -0.41614684, -0.90929743,   0.0953546,
            0,          0.90929743, -0.41614684,   1.1250297,
            0,          0,           0,            1;
    Matrix4d ef;
    ef <<   1,          0,           0,          -0.05197634,
            0,         -0.41614684, -0.90929743,  0.039639447,
            0,          0.90929743, -0.41614684,  1.2467695,
            0,          0,           0,           1;

    // Assert links are being created accurately
    ASSERT_TRUE(transMatAlmostEqual(a_b, arm.get_link_xform("a-b"), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(b_c, arm.get_link_xform("b-c"), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(c_d, arm.get_link_xform("c-d"), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(d_e, arm.get_link_xform("d-e"), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(e_f, arm.get_link_xform("e-f"), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(hand, arm.get_link_xform("hand"), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(ef, arm.get_ef_transform(), epsilon));

    cout << "a-b\n" << arm.get_link_xform("a-b") << "\n\n";
    cout << "b-c\n" << arm.get_link_xform("b-c") << "\n\n";
    cout << "c-d\n" << arm.get_link_xform("c-d") << "\n\n";
    cout << "d-e\n" << arm.get_link_xform("d-e") << "\n\n";
    cout << "e-f\n" << arm.get_link_xform("e-f") << "\n\n";
    cout << "hand\n" << arm.get_link_xform("hand") << "\n\n";
    cout << "ef\n" << arm.get_ef_transform() << "\n\n";

    // Assert joints are being created accurately:
    Vector3d joint_a(0, 0, 0);
    Vector3d joint_b(0,           0.03429,     0.02858);
    Vector3d joint_c(-0.0584,     0.33986104,  0.52210772);
    Vector3d joint_d(-0.102723,   0.31556032,  0.57520578);
    Vector3d joint_e(-0.0443284,  0.13792929,  0.96333666);
    Vector3d joint_f(-0.0236782,  0.0953546,   1.1250297);

    cout << "a\n" << arm.get_joint_pos_world("joint_a") << "\n\n";
    cout << "b\n" << arm.get_joint_pos_world("joint_b") << "\n\n";
    cout << "c\n" << arm.get_joint_pos_world("joint_c") << "\n\n";
    cout << "d\n" << arm.get_joint_pos_world("joint_d") << "\n\n";
    cout << "e\n" << arm.get_joint_pos_world("joint_e") << "\n\n";
    cout << "f\n" << arm.get_joint_pos_world("joint_f") << "\n\n";

    
    ASSERT_TRUE(vec3dAlmostEqual(joint_a, arm.get_joint_pos_world("joint_a"), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_b, arm.get_joint_pos_world("joint_b"), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_c, arm.get_joint_pos_world("joint_c"), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_d, arm.get_joint_pos_world("joint_d"), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_e, arm.get_joint_pos_world("joint_e"), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_f, arm.get_joint_pos_world("joint_f"), epsilon));
    // FK working , yay!!!
}

TEST(ik_test1) {
    // cout << setprecision(8);
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();
    // Create target point vector:
    Vector6d target;
    target <<   0.43561896709482717, -0.5849202310118653, -0.23898894427981895,
                -0.19091988273941293, 0.5705597354134033, -2.8999168062356357;
    solver.FK(arm);
    bool success = false;
    auto start = high_resolution_clock::now();
    for (size_t i = 0; i < 10; ++i) {
        if (solver.IK(arm, target, true, false).second) {
            success = true;
            break;
        } 
    }

    ASSERT_TRUE(success);
    
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop-start);
    cout << "Time to run FK: " << duration.count() << " milliseconds!\n";
}

TEST(ik_test2) {
    cout << setprecision(9);
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    // Create target point vector:
    Vector6d target;
    target << 0.242980428, -0.0244739898, 0.117660569, -0.426478891, 0.839306191, -1.09910019;
    //solver.FK(arm);

    bool success = false;
    for (size_t i = 0; i < 10; ++i) {
        if (solver.IK(arm, target, true, false).second) {
            success = true;
            break;
        } 
    }

    ASSERT_TRUE(success);
}

TEST(ik_test3) {
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    // Create target point vector:
    Vector6d target;
    //target << 0.242980428, -0.0244739898, 0.117660569, -0.426478891, 0.839306191, -1.09910019;
    target << 0.542980428, 0.1244739898, 0.017660569, -0.426478891, 0.839306191, -1.09910019;
    //solver.FK(arm);

    bool success = false;
    for (size_t i = 0; i < 10; ++i) {
        if (solver.IK(arm, target, true, false).second) {
            success = true;
            break;
        } 
    }

    ASSERT_TRUE(success);
}

TEST(ik_test4) {
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    // Create target point vector:
    Vector6d target;
    target << 0.016696540990824446, -0.303096070950721, -0.21998941217186194,
            -2.35009466480546, 0.4985607719497288, -2.8692554925434313;
    //solver.FK(arm);

    bool success = false;
    for (size_t i = 0; i < 10; ++i) {
        if (solver.IK(arm, target, true, false).second) {
            success = true;
            break;
        } 
    }

    ASSERT_TRUE(success);
}

TEST(ik_test_short) {
    string geom_file = "/vagrant/config/kinematics/mrover_arm_geom.json";

    json geom = read_json_from_file(geom_file);
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    arm.set_joint_angles({0, 1, -1, 0, 0, 0});
    solver.FK(arm);
    cout << "ef_pos after FK:\n";
    cout << arm.get_ef_pos_world() << "\n";

    // Create target point vector:
    Vector6d target;
    target << 0, 0.95, 0.35, 0.0, 0.0, 0.0;

    bool success = false;
    for (size_t i = 0; i < 1; ++i) {
        if (solver.IK(arm, target, false, false).second) {
            success = true;
            break;
        } 
    }

    ASSERT_TRUE(success);
}

TEST_MAIN()