# <img src="../../simulators/nav/src/static/mrover.png" alt="MRover Logo" width="30"/> MRover Navigation Codebase

## Table of Contents

[Project Overview](#project-overview) \
[Top-Level Code](#top-level-code) \
[Gate Search (`gate_search/`)](#gate-search) \
[Obstacle Avoidance (`obstacle_avoidance/`)](#obstacle-avoidance) \
[Search (`search/`)](#search) \
[Variables and Utilities](#variables-and-utilities)

**Created:** Ana Warner, Oct 3, 2021 \
**Updated:** Zachary Goldston, December 7, 2021 \
**Updated:** Quintin Dwight, May 26, 2022

---

<!----------------------------- Project Overview ----------------------------->

## Project Overview

The nav codebase contains logic for commanding the rover during the Autonomous Traversal task. The task consists of
several legs that increase in difficulty. During a leg, the operator inputs the destination GPS waypoint and end
behavior for the leg (search and/or gate), then starts the Auton system from the base station. The rover will first
drive autonomously to the destination waypoint without responding to target data from Perception. It will avoid
obstacles as they appear. Once the waypoint is reached, the rover will start a search pattern if the leg was a search
leg. The search algorithm first attempts a spiral out search pattern and approaches the tag once it is found. Finally,
if the operator indicated the leg is a gate leg, there is another post to find, so the gate search and traversal
algorithm will start.

---

<!----------------------------- Top-Level Code ----------------------------->

## Top Level Code

#### `main.cpp`

Contains the `main()` entrypoint. Here [LCM](https://github.com/lcm-proj/lcm) callbacks are set up to receive commands
and status from the base station.
The rover class, environment class (physical surroundings), course progress, and configuration are all initialized.
These are passed to the state machine construction.
Then the state machine is run every time we receive a new LCM message from the network.

#### [`stateMachine.cpp`](./stateMachine.cpp)

Main state machine logic, defines how states transition to one another.
Waypoint traversal logic is implemented here since it is simple.
For search (single tag) and gate search states, sub state machines are invoked to keep code properly encapsulated.
Note that the actual state variable is held by the rover class.

#### [`rover.cpp`](./rover.cpp)

Information about the rover and functionality for controlling the drivetrain.
Contains the current state, odometry (position + rotation), the PID controllers for bearing (turning) and distance.
Has behavior for driving to a waypoint which handles turning and driving to within a threshold.

#### [`environment.cpp`](./environment.cpp)

Holds processed information about the physical surroundings.
Raw and unreliable data is received from perception and passed through several caches and filters.
The caches ensure that briefly missing data or briefly seeing false positives does not have a large effect on the
overall state.
The filters combine multiple readings into a more accurate one via chopping out extreme values and then taking the
average of the remaining.
Gate location is calculated as global positioning by combing camera and odometry information.
Often times we do not have the targets in view, so this makes them persistent in space.

---

<!----------------------------- Search ----------------------------->

## Search

#### [`searchStateMachine.cpp`](./search/searchStateMachine.cpp)
#### [`searchFromPathFile.cpp`](./search/searchFromPathFile.cpp)

Sub state machines for driving to a single post/tag.
First we need to find the tag by driving in a spiral.
The spiral is pre-generated and loaded from a file.
The idea is that eventually we will spot the tag, then transition to the other sub state machine.
Now we can handle the logic of discerning which of the left/right tag (in screen space) from the environment is the
correct ID.

---

<!----------------------------- Gate Search ----------------------------->

## Gate Search

[`gateStateMachine.cpp`](./gate_search/gateStateMachine.cpp)

Sub state machine for gate search and traversal.
Checking the source code is highly recommended as this has the most vector arithmetic.
Basically we first line up with the gate, so we can cleanly drive straight through it.
Sometimes this involves an extra step if both gate posts are in front of us (worst case).

---

<!----------------------------- Obstacle Avoidance ----------------------------->

## Obstacle Avoidance

Not yet implemented.

---

## State Diagram

![image](https://user-images.githubusercontent.com/20666629/170621044-2055089d-cba9-4164-8472-e669dfc1ee66.png)

---

<!----------------------------- LCMs ----------------------------->

## LCM Channels Publishing/Subscribed To

**Target [subscriber]** \
Messages: [Enable.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/Enable.lcm)
“/auton_enabled” \
Publishers: jetson/teleop \
Subscribers: jetson/nav

**Course [subscriber]** \
Messages: [Course.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Course.lcm) “/course” \
Publishers: simulators/nav, base_station/gui \
Subscribers: jetson/nav

**Obstacle [subscriber]** \
Messages: [Obstacle.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Obstacle.lcm)
“/obstacle” \
Publishers: jetson/percep \
Subscribers: jetson/nav

**Odometry [subscriber]** \
Messages: [Odometry.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Odometry.lcm)
“/odometry” \
Publishers: jetson/filter \
Subscribers: jetson/nav

**Target List [subscriber]** \
Messages: [TargetList.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/TargetList.lcm)
“/target_list” \
Publishers: jetson/percep \
Subscribers: jetson/nav

**Joystick [publisher]** \
Messages: [Joystick.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Joystick.lcm)
“/autonomous” \
Publishers: jetson/nav \
Subscribers: jetson/teleop, simulators/nav, base_station/gui

**NavStatus [publisher]** \
Messages: [NavStatus.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/NavStatus.lcm)
“/nav_status” \
Publishers: jetson/nav \
Subscribers: simulators/nav, base_station/gui, jetson/science_bridge

**NavStatus [publisher]** \
Messages: [ProjectedPoints.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ProjectedPoints.lcm)
“/projected_points” \
Publishers: jetson/nav

---

<!----------------------------- Simulator Usage ----------------------------->

## Simulator Usage

To run the Navigation Simulator, you will need to open at least 3 terminals from the `mrover-workspace` directory (at
the top level). Run `vagrant up` in one of the terminals, and once that completes, run `vagrant ssh` in all three of
them. You will then follow these commands in order, one set of commands for each terminal. Once the three sets of
commands are run and in use, go to the [Navigation Simulator Website](http://localhost:8010/). Note, this will not
show anything until the last set of commands are run.

### Terminal 1 (`jetson/nav` code)

Run the command `./jarvis build jetson/nav`, this command will compile all the navigation directory code. If it
compiles successful, no errors will be returned. Then, run `./jarvis exec jetson/nav` to start the navigation code.

### Terminal 2 (LCMs)

Run the command `./jarvis build lcm_bridge/server`, this command will build the LCM messages. If it compiles
successful, no errors will be returned. Then, run `./jarvis exec lcm_bridge/server` to run the LCM messages.

### Terminal 3 (Navigation Simulator)

Run the command `./jarvis build simulators/nav`, this command will build the Navigation Simulator. If it compiles
successful, no errors will be returned. Then, run `./jarvis exec simulators/nav` to run the Simulator.

If desired, you can run a fourth terminal for debugging purposes via LCM messages. To do so, make sure you have another
terminal, and starting in the `mrover-workspace` directory, run `vagrant ssh`. Once we are ssh'ed into the virtual
machine, run `./jarvis build lcm_tools/echo` to build the echo tool for LCMs. This will return the messages that are
being communicated between publishers and subscribers. To run, enter the
command `./jarvis exec lcm_tools/echo TYPE_NAME CHANNEL` to echo the specified LCM and channel. (These are described in
our LCM section and ICDs on the Drive)

---

<!----------------------------- Rover Testing ----------------------------->

## Rover Testing (in-person)

To run the rover at testing, all that needs to be executing is `./jarvis build jetson/nav`
and `./jarvis exec jetson/nav` to run our code. This should be done AFTER the Perception and GPS executables and
necessary programs are running. To start, use the Base Station GUI to create waypoints via dropping. (dropping will
place a waypoint at the current GPS location of the rover) Additionally, the GUI has an option to create a waypoint by
directly inputting the GPS information Then, drag the newly created waypoint to the course. Repeat this process until
all desired waypoints are added. Finally, to start the Navigation code, hit the "Autonomy" button on the GUI.
