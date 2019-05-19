var fname = "mrover_arm_geom.json";
var fs = require("fs");

// import robot from 'robots/mrover_arm_urdf/.js'
var robot = require("./mrover_arm_urdf")
var robot_geom = JSON.stringify(robot, null, 4);
fs.writeFile(fname, robot_geom, 'utf8', function(x) {console.log(x)});
