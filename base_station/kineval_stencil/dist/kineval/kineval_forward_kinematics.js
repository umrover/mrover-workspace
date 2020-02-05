
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded Reasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () {
    if (typeof kineval.buildFKTransforms === 'undefined') {
        return;
    }

    // STENCIL: implement  
    //var robot_heading = [];
    //var robot_lateral = [];
    kineval.buildFKTransforms();

}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));

    kineval.buildFKTransforms = function buildFKTransforms () {
        

        
        traverseFKBase();
        var lengthBaseChildren = robot.links[robot.base].children.length;// mustexist
        for (var  indexBaseChild = 0; indexBaseChild < lengthBaseChildren; indexBaseChild++){
            traverseFKJoint(robot.links[robot.base].children[indexBaseChild]);
        }
        
    }
    
    function traverseFKBase() {
        var mR = generate_rotation_matrix(robot.origin.rpy[0],robot.origin.rpy[1],robot.origin.rpy[2]);
        var mT = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
        var heading_vector = [[0],[0],[1],[1]];
        robot.links[robot.base].xform = matrix_multiply(mT,mR);
        robot_heading = matrix_multiply(robot.links[robot.base].xform,heading_vector);
        var lateral_vector = [[1],[0],[0],[1]];
        robot_lateral = matrix_multiply(robot.links[robot.base].xform,lateral_vector);
        if (robot.links_geom_imported) {
            robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2)));
        }
    }
    
    function traverseFKJoint(curJoint) {
        var mR = generate_rotation_matrix(robot.joints[curJoint].origin.rpy[0],robot.joints[curJoint].origin.rpy[1],robot.joints[curJoint].origin.rpy[2]);
        var mT = generate_translation_matrix(robot.joints[curJoint].origin.xyz[0], robot.joints[curJoint].origin.xyz[1], robot.joints[curJoint].origin.xyz[2]);
        var mJ;
        if (robot.links_geom_imported){
            if (robot.joints[curJoint].type === "prismatic"){
                var temp = [ robot.joints[curJoint].angle, robot.joints[curJoint].angle, robot.joints[curJoint].angle];
                temp = vector_dot(temp, robot.joints[curJoint].axis);
                mJ = generate_translation_matrix(temp[0],temp[1],temp[2]);
            }else if ((robot.joints[curJoint].type === "continuous")|(robot.joints[curJoint].type === "revolute")){
                mJ = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[curJoint].angle,robot.joints[curJoint].axis)));
            }else{
                mJ = generate_identity(4);
            }
        }
        else{
            mJ = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[curJoint].angle,robot.joints[curJoint].axis))); 
        }
        var mTrans = matrix_multiply(robot.links[robot.joints[curJoint].parent].xform,matrix_multiply(mT,mR));
        robot.joints[curJoint].xform = matrix_multiply(mTrans, mJ);
        traverseFKLink(robot.joints[curJoint].child);
    }
    
    function traverseFKLink(curLink) {
        robot.links[curLink].xform = robot.joints[robot.links[curLink].parent].xform;
        if (typeof robot.links[curLink].children !== 'undefined'){
            for (var indexLinkChild = 0; indexLinkChild < robot.links[curLink].children.length; indexLinkChild++){
                traverseFKJoint(robot.links[curLink].children[indexLinkChild]);
            }
        }else {
        return;
        }
    }
