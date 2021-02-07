#include "mrover_arm.hpp"
#include "arm_state.hpp"
#include "motion_planner.hpp"
#include "kinematics.hpp"
#include "spline.h"
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/ArmPosition.hpp"
#include "rover_msgs/MotionExecute.hpp"
#include "rover_msgs/FKTransform.hpp"
 
using namespace std;
using namespace Eigen;
 
MRoverArm::MRoverArm(json &geom, lcm::LCM &lcm) : geom(geom), state(ArmState(geom)), solver(KinematicsSolver()), motion_planner(MotionPlanner(state, solver)), lcm_(lcm), done_previewing(false), enable_execute(false), sim_mode(true), ik_enabled(false)  {}
 
void MRoverArm::arm_position_callback(string channel, ArmPosition msg){
   /*
       Handler for angles
       Triggers forward kinematics
   */ 
   if (ik_enabled && !enable_execute){
       return;
   }
 
   if (channel == "/arm_position") {
       msg.joint_a = 0.0;
       msg.joint_b = 0.0;
       msg.joint_c = 0.0;
       msg.joint_d = 0.0;
       msg.joint_e = 0.0;
       msg.joint_f = 0.0;
       vector<double> angles{msg.joint_a, msg.joint_b, msg.joint_c, msg.joint_d, msg.joint_e, msg.joint_f};
       state.set_joint_angles(angles);
       solver.FK(state);
       publish_transforms(state);
   }
}
 
void MRoverArm::publish_config(vector<double> &config, string channel){
       ArmPosition arm_position;
       arm_position.joint_a = config[0];
       arm_position.joint_b = config[1];
       arm_position.joint_c = config[2];
       arm_position.joint_d = config[3];
       arm_position.joint_e = config[4];
       arm_position.joint_f = config[5];
       lcm_.publish(channel, &arm_position); //no matching call to publish should take in const msg type msg
 
}
 
void MRoverArm::matrix_helper(double arr[4][4], const Matrix4d &mat)
{
   for (int i = 0; i < 4; ++i)
   {
       for (int j = 0; j < 4; ++j)
       {
           arr[i][j] = mat(i,j);
       }
   }
} 
 
void MRoverArm::publish_transforms(ArmState state){
       FKTransform tm;  
       matrix_helper(tm.transform_a, state.get_joint_transform("joint_a"));
       matrix_helper(tm.transform_b, state.get_joint_transform("joint_b"));
       matrix_helper(tm.transform_c, state.get_joint_transform("joint_c"));
       matrix_helper(tm.transform_d, state.get_joint_transform("joint_d"));
       matrix_helper(tm.transform_e, state.get_joint_transform("joint_e"));
       matrix_helper(tm.transform_f, state.get_joint_transform("joint_f"));
       // tm.transform_b = state.get_joint_transform("joint_b");
       // tm.transform_c = state.get_joint_transform("joint_c");
       // tm.transform_d = state.get_joint_transform("joint_d");;
       // tm.transform_e = state.get_joint_transform("joint_e");
       // tm.transform_f = state.get_joint_transform("joint_f");
}
 
void MRoverArm::motion_execute_callback(string channel, MotionExecute msg){
       if (msg.preview) {
           enable_execute = false;
           preview();
       }
       else
       {
           execute_spline();
       }
}
      
 
void MRoverArm::execute_spline(){ 
       cout << "Executing path on arm" << endl;  
       double spline_t = 0;
       // run spline
       while(enable_execute){
           cout << "spline time: " << spline_t << endl; 
           vector<double> target_angles = motion_planner.get_spline_pos(spline_t);
           target_angles[4] *= -1;
           if (!sim_mode){ 
               publish_config(target_angles, "/ik_ra_control");
               spline_t += 0.01;
           }
           // TODO: make sure transition from not self.sim_mode
           //   to self.sim_mode is safe!!      previously commented
           else if (sim_mode){
               publish_config(target_angles, "/arm_position");
               spline_t += 0.01;
           }
           if (spline_t >= 1){
               enable_execute = false;
           }
       }
  
}
 
void MRoverArm::preview(){
       cout << "Previewing" << endl;      
       ArmState preview_robot = state; 
       int num_steps = 500;
       double t = 0.0;
       while (t < 1){
           vector<double> target = motion_planner.get_spline_pos(t); 
           ArmPosition target_pos;
           target_pos.joint_a = target[0];
           target_pos.joint_b = target[1];
           target_pos.joint_c = target[2];
           target_pos.joint_d = target[3];
           target_pos.joint_e = -target[4];
           target_pos.joint_f = target[5];
           vector<double> targ_pos = {target[0], target[1], target[2], target[3], target[4], target[5]};
           preview_robot.set_joint_angles(targ_pos);

           solver.FK(state); 
 
           publish_transforms(preview_robot);
           t += 1/num_steps;
       } 
       cout <<  "Preview Done" << endl;
}
void MRoverArm::lock_e_callback(string channel, LockJointE msg){
       cout << "joint e locked" << endl;
    //    solver.lock_joint_e(msg.locked); //no function lock_joint_e in kinematics.cpp
}
void MRoverArm::k_enabled_callback(string channel, IkEnabled msg){  
       ik_enabled = msg.enabled;
}
 
void MRoverArm::target_orientation_callback(string channel, TargetOrientation msg){
   cout << "target orientation callback";
   TargetOrientation point_msg = msg;
   enable_execute = false;
   //this statement replaced logger
   cout << "Got a target point." << endl;
 
   cout << "alpha beta gamma" << endl << endl;
   cout << point_msg.alpha << " , " << point_msg.beta << " , " << point_msg.gamma << endl << endl;
   bool use_orientation = point_msg.use_orientation;
   cout << use_orientation << endl;
 
   Vector6d point;
   point(0) = (double)point_msg.x;
   point(1) = (double)point_msg.y;
   point(2) = (double)point_msg.z;
   point(3) = (double)point_msg.alpha;
   point(4) = (double)point_msg.beta;
   point(5) = (double)point_msg.gamma;
 
   bool success = false;
   pair<Vector6d, bool> ik_solution = solver.IK(state, point, false, use_orientation);
  
   for(int i = 0; i<5; ++i){
       if(ik_solution.second){
           cout << "Solved IK" << endl;
           break;
       }
       cout << "Attempting new IK solution..." << endl << i << endl;
       ik_solution = solver.IK(state, point, true, use_orientation);
   }
   if(!ik_solution.second){
       cout << "NO IK SOLUTION FOUND, please try a different configuration." << endl;
       return;
   }
 
   publish_transforms(state);
   map<string, double> getjoint = state.get_joint_angles(); 
   Vector6d goal;
   goal(0) = getjoint["joint_a"];
   goal(1) = getjoint["joint_b"];
   goal(2) = getjoint["joint_c"];
   goal(3) = getjoint["joint_d"];
   goal(4) = getjoint["joint_e"];
   goal(5) = getjoint["joint_f"];
   plan_path(goal);
}
 
void MRoverArm::target_angles_callback(string channel, TargetAngles msg){
   enable_execute = false;
   TargetAngles target_angles = msg;  
 
   Vector6d goal;
   goal(0) = (double)target_angles.joint_a;
   goal(1) = (double)target_angles.joint_b;
   goal(2) = (double)target_angles.joint_c;
   goal(3) = (double)target_angles.joint_d;
   goal(4) = (double)target_angles.joint_e;
   goal(5) = (double)target_angles.joint_f;

   for(int i = 0; i<6; i++){
       cout << goal[i] << "\t";
   }
   cout << endl;
   plan_path(goal);
}
 
void MRoverArm::plan_path(Vector6d goal){
   cout << "goal" << endl;
   for(int i = 0; i<goal.size(); i++){
       cout << goal[i] << "\t";
   }
   cout << endl << "start" << endl;
   map<string, double> joint_angles = state.get_joint_angles();
//    for(int i = 0; i<joint_angles.size(); i++){ // cant index into map
//        cout << joint_angles[i] << "\t";
//    }
   cout << endl;  
 
   //idk this data type
//    vector<tk::spline> path_spline =   this was the beginning of the line below but rrtconnect is void
   bool path = motion_planner.rrt_connect(state, goal); //is rrt_connect a void function
   if(path){
       cout << "planned path" << endl;
   }
   else{
       cout << "No path found" << endl;
   }
}
 
void MRoverArm::simulation_mode_callback(string channel, SimulationMode msg){
   SimulationMode simulation_mode_msg = msg;
 
   bool sim_mode = simulation_mode_msg.sim_mode;
   publish_transforms(state);
}
 
// void MRoverArm::cartesian_control_callback(string channel, IkArmControl msg){
//    if(enable_execute){
//        return;
//    }
 
//    IkArmControl cart_msg = msg;
//    double delta[3] = {cart_msg.deltaX, cart_msg.deltaY, cart_msg.deltaZ};
//    //idk if this line is right down here
//    pair<vector<double> joint_angles, bool is_safe> = solver.IK_delta(delta, 3); // IK_delta takes in a Vector6d. ik arm control only has 3 values.
  
//    if(is_safe){
//        ArmPosition arm_position = ArmPosition();
//        map<string,double> gja = state.get_joint_angles();
//        arm_position.joint_a = gja["joint_a"];
//        arm_position.joint_b = gja["joint_b"];
//        arm_position.joint_c = gja["joint_c"];
//        arm_position.joint_d = gja["joint_d"];
//        arm_position.joint_e = gja["joint_e"];
//        arm_position.joint_f = gja["joint_f"];
//        vector<double> angles = {gja["joint_a"], gja["joint_b"], gja["joint_c"], gja["joint_d"], gja["joint_e"], gja["joint_f"]};
//        state.set_joint_angles(angles);
//        solver.FK(state);  
//        publish_transforms(state);
//        //again, running into the issue of encode(), should we even have it there
//        if(sim_mode){
//            cout << "Printing sim_mode" << endl;
//            lcm_.publish("/arm_position", arm_position.encode());
//        }
//        else{
//            cout << "Printing" << endl;
//            lcm_.publish("/ik_ra_control", arm_position.encode());
//        }
//    }
 
 
// }
 
