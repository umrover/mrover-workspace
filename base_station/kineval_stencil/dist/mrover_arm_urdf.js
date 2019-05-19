//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

var scale_factor = 0.0254;
var scale_factor_neg = -1*scale_factor;

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "mrover_arm";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0.1,0], rpy:[0,0,0]};

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "chassis-a";

// specify and create data objects for the links of the robot
// units in inches
robot.links = {
    "chassis-a": {
        visual : {
            origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "chassis-a.stl" } },
            material : { color : { rgba : [0.356, 0.361, 0.376, 1] } }
        },
        link_shape : {
            type: 'sphere',
            center: { x1: 0, y1: 0, z1: -13*scale_factor },
            radius: 15*scale_factor
        }
    },
    "a-b": {
        visual : {
            origin : { xyz: [0.0,0.0,0.2721*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "a-b.stl" } },
            material : { color : { rgba : [0.356, 0.361, 0.376, 1] } }
        },
        link_shape : {
            type: 'capsule',
            point_1: {
                x1: 0.75*scale_factor, y1: 0.82225*scale_factor, z1: 1.4563*scale_factor
            },
            point_2: {
                x2: -2.25*scale_factor, y2: 0.82225*scale_factor, z2: 1.4563*scale_factor
            },
            radius: 2.5*scale_factor,
        },
        actuator_shape: {
            type: 'capsule',
            point_1: {
                x1: -2.125*scale_factor, y1: -2.8619*scale_factor, z1: 1.6*scale_factor
            },
            point_2: {
                x2: -2.125*scale_factor, y2: -6.8619*scale_factor, z2: 1.6*scale_factor
            },
            radius: 1*scale_factor
        }
    },
    "b-c": {
        visual : {
            origin : { xyz: [0.000,-1.4234*scale_factor_neg,1.8571*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "b-c.stl" } },
            material : { color : { rgba : [0.356, 0.361, 0.376, 1] } }
        },
        link_shape : {
            type: 'capsule',
            point_1: {
                x1: 0, y1: -0.75*scale_factor, z1: -0.125*scale_factor
            },
            point_2: {
                x2: 0, y2: -0.75*scale_factor, z2: -0.75*scale_factor
            },
            radius: 1.5*scale_factor
        },
        actuator_shape: {
            type: 'capsule',
            point_1: {
                x1: 0, y1: -1.7453*scale_factor, z1: 11.125*scale_factor
            },
            point_2: {
                x2: 0, y2: -8.2453*scale_factor, z2: 11.125*scale_factor
            },
            radius: 1.5*scale_factor
        }
    },
    "c-d": {
        visual : {
            origin : { xyz: [0.000,-0.5393*scale_factor_neg,12.9821*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "c-d.stl" } },
            material : { color : { rgba : [0.356, 0.361, 0.376, 1] } }
        },
        link_shape : {
            type: 'capsule',
            point_1: {
                x1: 0, y1: 0.75*scale_factor, z1: -0.125*scale_factor
            },
            point_2: {
                x2: 0, y2: 0.75*scale_factor, z2: 9.125*scale_factor
            },
            radius: 1.5*scale_factor,
        },
        actuator_shape: {
            type: 'capsule',
            point_1: {
                x1: 0, y1: -0.394377*scale_factor, z1: 9.125*scale_factor
            },
            point_2: {
                x2: 0, y2: -6.8944*scale_factor, z2: 9.125*scale_factor
            },
            radius: 1.5*scale_factor
        }
    },
    "d-e": {
        visual : {
            origin : { xyz: [0.000,1.7249*scale_factor_neg,22.1071*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "d-e.stl" } },
            material : { color : { rgba : [0.356, 0.361, 0.376, 1] } }
        },
        link_shape : {
            type: 'capsule',
            point_1: {
                x1: 0, y1: 1*scale_factor, z1: 0.248*scale_factor
            },
            point_2: {
                x2: 0, y2: 1*scale_factor, z2: 1.498*scale_factor
            },
            radius: 1.5*scale_factor
        }
    },
    "e-f": {
        visual : {
            origin : { xyz: [-1.002*scale_factor_neg,2.7240*scale_factor_neg,22.6051*scale_factor_neg], rpy:[0,0,0] },
            origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "e-f.stl" } },
            material : { color : { rgba : [0.356, 0.361, 0.376, 1] } }
        },
        link_shape : {
            type: 'capsule',
            point_1: {
                x1: -1.5141*scale_factor, y1: 0, z1: -0.125*scale_factor
            },
            point_2: {
                x2: -1.5141*scale_factor, y2: 0, z2: 10.375*scale_factor
            },
            radius: 1.5*scale_factor
        },
        actuator_shape : {
            type: 'capsule',
            point_1: {
                x1: -3.1585*scale_factor, y1: 0, z1: 0
            },
            point_2: {
                x2: -9.1585*scale_factor, y2: 0, z2: 0
            },
            radius: 1.5*scale_factor
        }
    },
    "hand": {
        visual: {
            origin : { xyz: [-2.5166*scale_factor_neg,2.7226*scale_factor_neg,34.3566*scale_factor_neg], rpy:[0,0,0] },
            //origin : { xyz: [0.0,0.0,0.0], rpy:[0,0,0] },
            geometry : { mesh : { filename : "hand.stl" } },
            material : { color : { rgba : [0.356, 0.361, 0.376, 1] } }
        },
        link_shape: {
            type: 'sphere',
            center: { x1: -1.5141*scale_factor, y1: 0, z1: 14.7835*scale_factor },
            radius: 5*scale_factor
        }
    }
};

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]};
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0];
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

// units converted from inches to meters

robot.joints.joint_a = {parent:"chassis-a", child:"a-b"};
robot.joints.joint_a.origin = {xyz: [0.0,0.0,0.2721*scale_factor], rpy:[0,0,0]};
robot.joints.joint_a.axis = [0.0,0.0,-1.0];
robot.joints.joint_a.type = "revolute";
robot.joints.joint_a.limit = {lower:0, upper:3.05};

robot.joints.joint_b = {parent:"a-b", child:"b-c"};
robot.joints.joint_b.origin = {xyz: [0.0,-1.4234*scale_factor,1.585*scale_factor], rpy:[0,0,0]};
robot.joints.joint_b.axis = [0.0,1.0,0];
robot.joints.joint_b.type = "revolute";
robot.joints.joint_b.limit = {lower:0, upper:1.57};

robot.joints.joint_c = {parent:"b-c", child:"c-d"};
robot.joints.joint_c.origin = {xyz: [0.0,0.8841*scale_factor,11.125*scale_factor], rpy:[0,0,0]};
robot.joints.joint_c.axis = [0.0,1.0,0];
robot.joints.joint_c.type = "revolute";
robot.joints.joint_c.limit = {lower:-2.0, upper:2.0};

robot.joints.joint_d = {parent:"c-d", child:"d-e"};
robot.joints.joint_d.origin = {xyz: [0.0,2.2642*scale_factor,9.125*scale_factor], rpy:[0,0,0]};
robot.joints.joint_d.axis = [0.0,1.0,0];
robot.joints.joint_d.type = "revolute";
robot.joints.joint_d.limit = {lower:-2.0, upper:2.0};

robot.joints.joint_e = {parent:"d-e", child:"e-f"};
robot.joints.joint_e.origin = {xyz: [-1.002*scale_factor,0.9991*scale_factor,1.498*scale_factor], rpy:[0,0,0]};
robot.joints.joint_e.axis = [1.0,0.0,0];
robot.joints.joint_e.type = "revolute";
robot.joints.joint_e.limit = {lower:-2.0, upper:2.0};

robot.joints.joint_f = {parent:"e-f", child:"hand"};
robot.joints.joint_f.origin = {xyz: [-1.5146*scale_factor,-0.0014*scale_factor,10.7515*scale_factor], rpy:[0,0,0]};
robot.joints.joint_f.axis = [0.0,0.0,1.0];
robot.joints.joint_f.type = "revolute";
robot.joints.joint_f.limit = {lower:-2.0, upper:2.0};

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "joint_f";
robot.endeffector.position = [[0],[0],[0],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

robot.links_geom_imported = true;

module.exports = robot