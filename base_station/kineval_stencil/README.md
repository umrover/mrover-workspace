
KinEval
=======

The Kinematic Evaluator (KinEval) is a package containing a collection of HTML5/Javascript implementations for teaching robot kinematics, control, decision making, and dynamics.

To see kineval in action, open home.html in a web browser.  Firefox 29.0 through 41.0 works for sure.  Chrome and Opera will throw security errors when loading local files from JavaScript (which feels like forcing people into the cloud).  One method around this issue is to serve home.html from a web server.  If python is available on your system, this can be done by running python's SimpleHTTPServer: 

python -m SimpleHTTPServer

and loading the file from the following URL:

http://localhost:8000/home.html

or, alternatively, NodeJS's (https://nodejs.org/) simple-server from your working directory:

npm install simple-server
node simple-server

and loading the file from the following URL:

http://localhost:3000/

## Controls (approximate description):

- W/A/S/D - move forward/backward and turn right/left
- Q/R - strafe left right
- U/I - control current joint to rotate forward/backward
- H/J/K/L - switch control to parent/child/sibling joint
- O - servo all joints based on seconds of the system clock
- P - perform inverse kinematics iteration (hold down for continual motion)
- R/F - move inverse kinematics target up/down
- M - execute RRT planner
- N/B - set robot configuration to next/previous configuration in generated RRT plan

## Robots and Worlds:

The subdirectories "robots" and "worlds" contains descriptions of various robots and worlds as js files that can be used with KinEval.  These robots and worlds can be included by including the appropriate js files in home.html.  For example:

<script src="robots/robot_br2.js"></script> 
<script src="worlds/world_local_minima.js"></script> 

## Sub-Projects

This project also includes sub-projects for:

### JavaScript/HTML5 examples

Each of the files in the tutorial_js subdirectory can be opened in a browser to demonstrate some simple examples of HTML documents with 2D drawing and animation.  These are meant to provide examples for tutorial purposes.  This tutorial assumes the reader is familiar with modern programming langugages, such as C.

### JavaScript/HTML5 heapsort tutorial

The file heapsort.html in the tutorial_heapsort subdirectory provides tutorial-by-example code stencil for implementing a heap sort routine in JavaScript/HTML5.  This tutorial expects the reader to be familiar the concepts for implementing a min binary heap.  This heap will be the foundation of the priority queue for the path planner.

### 2D Path planner

The file project_pathplan/search_canvas.html is a code stencil for implementing various search algorithms for path planning.  The planner will execute automatically upon page load of the file into the browser.  Other worlds can be used through custom URL commands (described below) , or uncomment/commenting lines in the initPlanner() function.

### Usage

The planner will start upon browser loading this file, with execution parameters specifing:

      search_alg: string specifying choice of search algorithm, as one of:
        "depth-first","breadth-first","greedy-best-first","A-star",
        "RRT","RRT-connect","RRT-star"
      planning_scene: string specifying choice of 2D world, as one of:
        "empty","misc","narrow1","narrow2","three_sections"
      q_init: 2-element 1D array with coordinates of start location
      q_goal: 2-element 1D array with coordinates of goal location
      eps: "epsilon" number for search spatial resolution of graph 
        as well as RRT step length

These parameters can be specified in the URL as parameter assignments separated by question marks, as in the following example: 

      search_canvas.html?search_alg=A-star?planning_scene=misc?q_init=[0,0]?q_goal=[4,4]?eps=0.1


### 1 DOF pendulum simulation with servo control

The file project_pendularm/pendularm1.html is a code stencil for implementing physical simulation and motion control of a simple frictionless pendulum.  Pendulum will servo to a set desired angle by default.

#### Controls:

- S - let the pendulum swing
- Q/E - move desired angle forward/backward
- A/D - apply a forward/backward force


