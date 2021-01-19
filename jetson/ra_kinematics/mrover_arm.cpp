#include "mrover_arm.hpp"
#include "arm_state.hpp"
#include "motion_planner.hpp"
#include "kinematics.hpp"

using namespace Eigen;

MRoverArm::MRoverArm(json config, lcm::LCM lcm) : config(config), lcm_(lcm) {
    json geom = read_geometry_from_JSON();
    ArmState state = ArmState(geom);
    KinematicSolver solver = KinematicsSolver(state, lcm);
    MotionPlanner motion_planner = MotionPlanner(state, lcm, solver);
    // current_spline = [];  // list
    int spline_t = 0;
    bool done_previewing = False;
    bool enable_execute = False;
    bool sim_mode = True;
    bool ik_enabled = False;
}

void MRoverArm::arm_position_callback(string channel, ArmPosition msg){
    /*
        Handler for angles
        Triggers forward kinematics
    */  
    if (ik_enabled && !enable_execute){
        return;
    }

    ArmPosition arm_position = ArmPosition.decode(msg)
    if (channel == "/arm_position") {
        state.set_angles(arm_position);
        solver.FK(state);
        publish_transforms(state);
    }
}

void MRoverArm::publish_config(Vector6d config, string channel){
        ArmPosition arm_position = ArmPosition();
        arm_position.joint_a = config[0];
        arm_position.joint_b = config[1];
        arm_position.joint_c = config[2];
        arm_position.joint_d = config[3];
        arm_position.joint_e = config[4];
        arm_position.joint_f = config[5];

        lcm_.publish(channel, arm_position.encode());
}

void MRoverArm::publish_transforms(ArmState state){
        tm = FKTransform();
        tm.transform_a = state.get_joint_transform('joint_a');
        tm.transform_b = state.get_joint_transform('joint_b');
        tm.transform_c = state.get_joint_transform('joint_c');
        tm.transform_d = state.get_joint_transform('joint_d');;
        tm.transform_e = state.get_joint_transform('joint_e');
        tm.transform_f = state.get_joint_transform('joint_f');
        lcm_.publish('/fk_transform', tm.encode());
}
        