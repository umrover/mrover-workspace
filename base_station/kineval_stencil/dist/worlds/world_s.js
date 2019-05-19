
    // set rectangular boundary of robot's world as min and max locations
    // collision only checked in x-z plane
    robot_boundary = [[-10,0,-10],[2,0,2]];

    // set spherical obstacles in robot's world
    // with locations specified in homogeneous coordinates as 2D array
    robot_obstacles = []; 

    robot_obstacles[0] = {location:[[-2],[0.5],[0],[1]], radius:0.5}; 
    robot_obstacles[1] = {location:[[-2],[0.5],[-1],[1]], radius:0.5}; 
    robot_obstacles[2] = {location:[[-2],[0.5],[-2],[1]], radius:0.5}; 
    robot_obstacles[3] = {location:[[-2],[0.5],[-3],[1]], radius:0.5}; 
    robot_obstacles[4] = {location:[[-2],[0.5],[-4],[1]], radius:0.5}; 
    robot_obstacles[5] = {location:[[-2],[0.5],[-5],[1]], radius:0.5}; 
    robot_obstacles[6] = {location:[[-2],[0.5],[-6],[1]], radius:0.5}; 
    robot_obstacles[7] = {location:[[-2],[0.5],[-7],[1]], radius:0.5}; 
    robot_obstacles[8] = {location:[[-2],[0.5],[-8],[1]], radius:0.5}; 
    robot_obstacles[9] = {location:[[-2],[0.5],[1],[1]], radius:0.5}; 
    robot_obstacles[10] = {location:[[-2],[0.5],[2],[1]], radius:0.5}; 

    robot_obstacles[10] = {location:[[-6],[0.5],[0],[1]], radius:0.5}; 
    robot_obstacles[11] = {location:[[-6],[0.5],[-1],[1]], radius:0.5}; 
    robot_obstacles[12] = {location:[[-6],[0.5],[-2],[1]], radius:0.5}; 
    robot_obstacles[13] = {location:[[-6],[0.5],[-3],[1]], radius:0.5}; 
    robot_obstacles[14] = {location:[[-6],[0.5],[-4],[1]], radius:0.5}; 
    robot_obstacles[15] = {location:[[-6],[0.5],[-5],[1]], radius:0.5}; 
    robot_obstacles[16] = {location:[[-6],[0.5],[-6],[1]], radius:0.5}; 
    robot_obstacles[17] = {location:[[-6],[0.5],[-7],[1]], radius:0.5}; 
    robot_obstacles[18] = {location:[[-6],[0.5],[-8],[1]], radius:0.5}; 
    robot_obstacles[19] = {location:[[-6],[0.5],[-9],[1]], radius:0.5}; 
    robot_obstacles[20] = {location:[[-6],[0.5],[-10],[1]], radius:0.5}; 

    robot_obstacles[21] = {location:[[-2],[0.5],[2],[1]], radius:0.5}; 
