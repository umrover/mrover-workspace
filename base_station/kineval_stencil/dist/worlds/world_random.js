
    // set rectangular boundary of robot's world as min and max locations
    // collision only checked in x-z plane
    robot_boundary = [[-50,0,-50],[2,0,2]];

    // set spherical obstacles in robot's world
    // with locations specified in homogeneous coordinates as 2D array
    robot_obstacles = []; 

    var i;
    for (i=0;i<100;i++) {
        robot_obstacles[i] = { 
            location: [
                [Math.random()*robot_boundary[0][0]],
                [(Math.random()-0.2)*3],
                [Math.random()*robot_boundary[0][2]],
                [1]
            ],
            radius: Math.random()*3.0
        };
    }

