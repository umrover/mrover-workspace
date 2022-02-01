#include "unit_test_framework.h"
#include "nlohmann/json.hpp"
#include "../arm_state.hpp"
#include "../utils.hpp"
#include "../kinematics.hpp"

#include <iostream>
#include <string>
#include <iomanip>
#include <chrono>

using namespace nlohmann;

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
            if (std::abs(a(i, j) - b(i, j)) > epsilon) {
                return false;
            }
        }
    }
    return true;
}

bool vec3dAlmostEqual(Vector3d a, Vector3d b, double epsilon) {
    for (int i = 0; i < 3; ++i) {
        if (std::abs(a(i)-b(i)) > epsilon) {
            return false;
        }
    }
    return true;
}

TEST(kinematics_initialization) {
    std::cout << std::setprecision(8);
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
    KinematicsSolver solver();
}

TEST(fk_test) {
    std::cout << std::setprecision(8);
    // Create the arm to be tested on:
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    std::vector<double> angles;
    angles.push_back(0);
    angles.push_back(1);
    angles.push_back(1);
    angles.push_back(0);
    angles.push_back(0);
    angles.push_back(0);

    arm.set_joint_angles(angles);

    std::cout << "ef_pos:\n";
    std::cout << arm.get_ef_pos_world() << "\n";
    solver.FK(arm);
    std::cout << "ef_pos after FK:\n";
    std::cout << arm.get_ef_pos_world() << "\n";
    std::cout << "ef_ang after FK:\n";
    std::cout << arm.get_ef_ang_world() << "\n";

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
    hand <<   1,          0,           0,          -0.02299899,
            0,         -0.41614684, -0.90929743,  0.069306726,
            0,          0.90929743, -0.41614684,  1.1132747,
            0,          0,           0,           1;
    Matrix4d ef;
    ef <<   1,          0,           0,          -0.05129713,
            0,         -0.41614684, -0.90929743,  0.013591573,
            0,          0.90929743, -0.41614684,  1.2350145,
            0,          0,           0,           1;

    // Assert links are being created accurately
    ASSERT_TRUE(transMatAlmostEqual(a_b, arm.get_link_xform(1), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(b_c, arm.get_link_xform(2), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(c_d, arm.get_link_xform(3), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(d_e, arm.get_link_xform(4), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(e_f, arm.get_link_xform(5), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(hand, arm.get_link_xform(6), epsilon));
    ASSERT_TRUE(transMatAlmostEqual(ef, arm.get_ef_transform(), epsilon));

    // Assert joints are being created accurately:
    Vector3d joint_a(0, 0, 0);
    Vector3d joint_b(0,           0.03429,     0.02858);
    Vector3d joint_c(-0.0584,     0.33986104,  0.52210772);
    Vector3d joint_d(-0.102723,   0.31556032,  0.57520578);
    Vector3d joint_e(-0.0443284,  0.13792929,  0.96333666);
    Vector3d joint_f(-0.0236782,  0.0953546,   1.1250297);
    
    ASSERT_TRUE(vec3dAlmostEqual(joint_a, arm.get_joint_pos_world(0), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_b, arm.get_joint_pos_world(1), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_c, arm.get_joint_pos_world(2), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_d, arm.get_joint_pos_world(3), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_e, arm.get_joint_pos_world(4), epsilon));
    ASSERT_TRUE(vec3dAlmostEqual(joint_f, arm.get_joint_pos_world(5), epsilon));
    // FK working , yay!!!
}

TEST(is_safe_test) {
    // Create the arm to be tested on:
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    std::string presets_file = "/vagrant/base_station/kineval_stencil/dist/mrover_arm_presets.json";

    json presets = read_json_from_file(presets_file);
    for (json::iterator it = presets.begin(); it != presets.end(); it++) {
        ASSERT_TRUE(solver.is_safe(arm, it.value()));
    }
}

TEST(ik_test1) {
    // std::cout << std::setprecision(8);
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();
    // Create target point vector:
    Vector6d target;
    target <<   0.43561896709482717, -0.5849202310118653, -0.23898894427981895,
                -0.19091988273941293, 0.5705597354134033, -2.8999168062356357;
    solver.FK(arm);
    bool success = false;
    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < 10; ++i) {
        if (solver.IK(arm, target, true, false).second) {
            success = true;
            break;
        } 
    }

    ASSERT_TRUE(success);
    
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
    std::cout << "Time to run FK: " << duration.count() << " milliseconds!\n";
}

TEST(ik_test2) {
    std::cout << std::setprecision(9);
    json geom = read_json_from_file(get_mrover_arm_geom());
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
    json geom = read_json_from_file(get_mrover_arm_geom());
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
    json geom = read_json_from_file(get_mrover_arm_geom());
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
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    arm.set_joint_angles({0, 1, -1, 0, 0, 0});
    solver.FK(arm);
    std::cout << "ef_pos after FK:\n";
    std::cout << arm.get_ef_pos_world() << "\n";

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

TEST(ik_local_min) {
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    arm.set_joint_angles({0, 1, -1, 0, 0, 0});
    solver.FK(arm);
    std::cout << "ef_pos after FK:\n";
    std::cout << arm.get_ef_pos_world() << "\n";

    // Create target point vector:
    Vector6d target;
    target << 38, 400, 14, 0.0, 0.0, 0.0;

    solver.IK(arm, target, false, false);

    std::cout << "num iterations: " << solver.get_num_iterations() << "\n";
    ASSERT_FALSE(solver.get_num_iterations() > 500);
}

// Test that IK respects the locking of joints
TEST(ik_test_lock) {
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    // Starting angles.
    std::vector<double> start = {0, 1, -1, 0, 0, 0};

    // Angles to find a target position.
    std::vector<double> target = {0.1, 0.9, -1.1, 0.1, -0.1, 0};

    ASSERT_TRUE(solver.is_safe(arm, start));
    ASSERT_TRUE(solver.is_safe(arm, target));

    // Generate the target position.
    arm.set_joint_angles(target);
    solver.FK(arm);

    Vector6d target_pos;
    target_pos.head(3) = arm.get_ef_pos_world();
    target_pos.tail(3) = arm.get_ef_ang_world();

    // Lock joint c
    arm.set_joint_locked(2, true);

    // Reset arm to starting position.
    arm.set_joint_angles(start);
    solver.FK(arm);

    // Run IK
    std::pair<Vector6d, bool> result = solver.IK(arm, target_pos, false, false);

    // Check that joint c did not move from the start position.
    ASSERT_ALMOST_EQUAL(-1, result.first[2], 0.0000001);

    // Check that IK found a solution.
    ASSERT_TRUE(result.second);
}

// Test that IK respects the locking of joints
TEST(ik_test_orientation_experiment) {
    json geom = read_json_from_file(get_mrover_arm_geom());
    ArmState arm = ArmState(geom);
    KinematicsSolver solver = KinematicsSolver();

    // Starting angles.
    std::vector<double> start = {0, 1, -1, 0, 0, 0};

    // Angles to find a target position.
    std::vector<double> target = {0.1, 0.9, -1, 0, 0, 0};

    ASSERT_TRUE(solver.is_safe(arm, start));
    ASSERT_TRUE(solver.is_safe(arm, target));

    // Generate the target position.
    arm.set_joint_angles(target);
    solver.FK(arm);

    Vector6d target_pos;
    target_pos.head(3) = arm.get_ef_pos_world();
    target_pos.tail(3) = arm.get_ef_ang_world();

    // Reset arm to starting position.
    arm.set_joint_angles(start);
    solver.FK(arm);

    // Run IK
    std::pair<Vector6d, bool> result = solver.IK(arm, target_pos, false, true);

    // Check that IK found a solution.
    ASSERT_TRUE(result.second);
}

TEST_MAIN()
