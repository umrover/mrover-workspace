
    // set rectangular boundary of robot's world as min and max locations
    // collision only checked in x-z plane
    robot_boundary = [[-5,0,-5],[5,0,5]];

    // set spherical obstacles in robot's world
    // with locations specified in homogeneous coordinates as 2D array
    robot_obstacles = []; 

    robot_obstacles[0] = {}; 
    robot_obstacles[0].location = [[0],[0.5],[2],[1]]; 
    robot_obstacles[0].radius = 0.5; 
    robot_obstacles[1] = {}; 
    robot_obstacles[1].location = [[2],[0.5],[0],[1]]; 
    robot_obstacles[1].radius = 0.5;
    robot_obstacles[2] = {}; 
    robot_obstacles[2].location = [[-2],[0.5],[0],[1]];
    robot_obstacles[2].radius = 0.5;

