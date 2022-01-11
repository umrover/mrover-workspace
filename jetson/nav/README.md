# <img src="../../simulators/nav/src/static/mrover.png" alt="MRover Logo" width="30"/> MRover Navigation Codebase

## Table of Contents
[Project Overview](#project-overview)<br/>
[Top-Level Code](#Top-Level-Code)<br/>
[Gate Search (`gate_search/`)](#Gate-Search)<br/>
[Obstacle Avoidance (`obstacle_avoidance/`)](#obstacle-avoidance)</br>
[Search (`search/`)](#search)<br/>
[Variables and Utilities](#vars-and-utils)<br/>


**Created:** Ana Warner, Oct 3, 2021
**Updated:** Zachary Goldston, December 7, 2021

---

<!----------------------------- Project Overview ----------------------------->
## Project Overview
The nav codebase contains logic for commanding the rover during the Autonomous Traversal task. The task consists of several legs that increase in difficulty. During a leg, the operator inputs the destination GPS waypoint and end behavior for the leg (search and/or gate), then starts the Auton system from the base station. The rover will first drive autonomously to the destination waypoint without responding to target data from Perception. It will avoid obstacles as they appear. Once the waypoint is reached, the rover will start a search pattern if the leg was a search leg. The search algorithm first attempts a spiral out search pattern and approaches the tag once it is found. Finally, if the operator indicated the leg is a gate leg, there is another post to find, so the gate search and traversal algorithm will start.

---

<!----------------------------- Top-Level Code ----------------------------->
## Top-Level Code

#### `main.cpp`
The `main.cpp` file contains the `main()` function. In the main function, we create an instance of the state machine, create the LCM handlers class, and subscribe to the LCM channels that we will read messages from. (For more about LCM’s, see below.) Then we call the outermost function of the state machine, `run()`, which begins executing the state machine logic.

#### `stateMachine.hpp`
This is an example of a header file, commonly used in C and C++. The header file for a class (an object) contains the class declaration. A class declaration lists the class’s member variables and declares the member functions, which are then implemented (“defined”) in the .cpp file. The `stateMachine.hpp` file contains the state machine variables, including pointers to the search state machine and obstacle avoidance state machine, which are derived classes from the regular state machine.

#### `stateMachine.cpp`
This file contains implementations of the stateMachine object’s member functions, including the `run()` function, which executes the logic for switching between navigation states and calling the functions to run in each state.

#### `rover.cpp`
This file defines the rover and rover status objects. The rover object is used throughout the codebase to interact with real-life capabilities of the rover. Notably, the object contains functions like `drive()` and `turn()`. The rover status object/class is nested in the rover class, and it contains information about the current state of the rover and relevant features like targets and obstacles. Most variables in the rover status are populated from LCM messages.

---

<!----------------------------- Gate Search ----------------------------->
## Gate Search (`gate_search/` folder)

#### `diamondGateSearch.cpp`
This file creates the search waypoints in the shape of a diamond for completing a search for the second gate post.

#### `gateStateMachine.cpp`
Defines gate search/traversal states and functions.


---

<!----------------------------- Obstacle Avoidance ----------------------------->
## Obstacle Avoidance

#### `obstacleAvoidanceStateMachine.cpp`
Defines an obstacle avoidance state machine with minimal functionality, intended to be a parent class for different types of obstacle avoidance strategies

#### `simpleAvoidance.cpp`
This contains what is currently our only implementation of obstacle avoidance behavior. Inherited from the obstacle state machine, it is a very simple algorithm and just drops a waypoint at the front of the queue, with a position at a safe location away from the obstacle, for the rover to drive to before continuing to its previous destination. If we developed another behavior algorithm, we would make another obstacle state machine child class like simpleAvoidance.


---

<!----------------------------- Search ----------------------------->
## Search
Similar to the `gate_search/` folder, this folder for search logic contains a `searchStateMachine` object and files to define the waypoints for different types of searches. First we follow a square spiral outwards with points generated in spiralOutSearch.cpp, then if the search completes and the target is not found, we will move onto trying the lawnmower search and the spiral in search.


---

<!----------------------------- Vars and Utils ----------------------------->
## Variables and Utilities

#### Nav State
The nav state specifies what state of the state machine we are in. It is implemented as an enum (a C++ type) where named states are associated with a number behind the scenes.

#### Auton State
This is just a boolean hidden as a type called AutonState and it tells us if Auton is on or off. It is read from LCMs published by the GUI or simulator.

##### `utilities.cpp`
Contains functions used commonly throughout auton code. 


---

<!----------------------------- LCMs ----------------------------->
## LCM Channels Publishing/Subscribed To 
**Target [subscriber]** \
Messages: [ AutonState.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/AutonState.lcm) “/auton_state” \
Publishers: jetson/teleop \
Subscribers: jetson/nav

**Course [subscriber]** \
Messages: [ Course.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Course.lcm) “/course” \
Publishers: simulators/nav, base_station/gui \
Subscribers: jetson/nav

**Obstacle [subscriber]** \
Messages: [ Obstacle.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Obstacle.lcm) “/obstacle” \
Publishers: jetson/percep \
Subscribers: jetson/nav

**Odometry [subscriber]** \
Messages: [ Odometry.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Odometry.lcm) “/odometry” \
Publishers: jetson/filter \
Subscribers: jetson/nav

**Target List [subscriber]** \
Messages: [ TargetList.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/TargetList.lcm) “/target_list” \
Publishers: jetson/percep \
Subscribers: jetson/nav

**ZED Gimbal Data [subscriber]** \
Messages: [ ZedGimablPosition.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/TargetList.lcm) “/zed_gimbal_data” \
Publishers: simulators/nav, raspi/zed_gimbal, jetson/nav (TODO) \
Subscribers: jetson/nav 

**Joystick [publisher]** \
Messages: [ Joystick.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Joystick.lcm) “/autonomous” \
Publishers: jetson/nav \
Subscribers: jetson/teleop, simulators/nav, base_station/gui

**NavStatus [publisher]** \
Messages: [ NavStatus.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/NavStatus.lcm) “/nav_status” \
Publishers: jetson/nav \
Subscribers: simulators/nav, base_station/gui, jetson/science_bridge


---

<!----------------------------- Simulator Usage ----------------------------->
## Simulator Usage
To run the Navigation Simulator, you will need to open at least 3 terminals from the `mrover-workspace` directory (at the top level). Run `vagrant up` in one of the terminals, and once that completes, run `vagrant ssh` in all three of them. You will then follow these commands in order, one set of commands for each terminal. Once the three sets of commands are run and in use, go to the [ Navigation Simulator Website ](http://localhost:8010/). Note, this will not show anything until the last set of commands are run.

### Terminal 1 (`jetson/nav` code)
Run the command `$./jarvis build jetson/nav`, this command will compile all the navigation directory code. If it compiles succecssful, no errors will be returned. Then, run `$./jarvis exec jetson/nav` to start the navigation code.

### Terminal 2 (LCMs)
Run the command `$./jarvis build lcm_bridge/server`, this command will build the LCM messages. If it compiles successful, no errors will be returned. Then, run `$./jarvis exec lcm_bridge/server` to run the LCM messages.

### Terminal 3 (Navigation Simulator)
Run the command `$./jarvis build simulators/nav`, this command will build the Navigation Simulator. If it compiles successful, no errors will be returned. Then, run `$./jarvis exec simulators/nav` to run the Simulator.

If desired, you can run a fourth terminal for debugging purposes via LCM messages. To do so, make sure you have another terminal, and starting in the `mrover-workspace` directory, run `vagrant ssh`. Once we are ssh'ed into the virtual machine, run `$./jarvis build lcm_tools/echo` to build the echo tool for LCMs. This will return the messages that are being communicated between publishers and subscribers. To run, enter the command `$./jarvis exec lcm_tools/echo TYPE_NAME CHANNEL` to echo the specified LCM and channel. (These are described in our LCM section and ICDs on the Drive)


---

<!----------------------------- Rover Testing ----------------------------->
## Rover Testing (in-person)
To run the rover at testing, all that needs to be executing is `$./jarvis build jetson/nav` and `$./jarvis exec jetson/nav` to run our code. This should be done AFTER the Perception and GPS executables and necessary programs are running. To start, use the Base Station GUI to create waypoints via dropping. (dropping will place a waypoint at the current GPS location of the rover) Additionally, the GUI has an option to create a waypoint by directly inputting the GPS information Then, drag the newly created waypoint to the course.  Repeat this process until all desired waypoints are added. Finally, to start the Navigation code, hit the "Autonomy" button on the GUI.