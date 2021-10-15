# <img src="../simulators/nav/src/static/mrover.png" alt="MRover Logo" width="30"/> MRover Navigation Codebase

## Table of Contents
[Project Overview](#project-overview)<br/>
[Top-Level Code](#Top-Level-Code)<br/>
[Gate Search (gate_search/ folder)](#Gate-Search)<br/>
[Obstacle Avoidance (obstacle_avoidance/)](#obstacle-avoidance)</br>
[Search (search/)](#search)<br/>
[Variables and Utilities](#vars-and-utils)<br/>


**Created:** Ana Warner, Oct 3, 2021
**Updated:** A.Warner, 10/14/2021

---

<!----------------------------- Project Overview ----------------------------->
## Project Overview
The nav codebase contains logic for commanding the rover during the Autonomous Traversal task. The task consists of several legs that increase in difficulty. During a leg, the operator starts inputs the destination GPS waypoint and end behavior for the leg (search and/or gate), then starts the Auton system from the base station. The rover will first drive autonomously to the destination waypoint without responding to target data from Perception. It will avoid obstacles as they appear. Once the waypoint is reached, the rover will start a search pattern if the leg was a search leg. The search algorithm first attempts a spiral out search pattern and approaches the tag once it is found. Finally, if there is a second tag composing the gate, the gate search and traversal algorithm will start.

---

<!----------------------------- Top-Level Code ----------------------------->
## Top-Level Code

#### `main.cpp`
The `main.cpp` file contains the `main()` function. In the main function, we create an instance of the state machine, create the LCM handlers class, and subscribe to the LCM channels that we will read messages from. (For more about LCM’s, see below.) Then we call the outermost function of the state machine, `run()`, which begins executing the state machine logic.

#### `stateMachine.hpp`
This is an example of a header file, commonly used in C and C++. The header file for a class (an object) contains the class declaration. A class declaration lists the class’s member variables and declares the member functions, which are then implemented (“defined”) in the .cpp file. The `stateMachine.hpp` file contains the state machine variables, including, notably, pointers to the search state machine and obstacle avoidance state machine, which are derived classes from the regular state machine.

#### `stateMachine.cpp`
This file contains implementations of the stateMachine object’s member functions, including the `run()` function, which executes the logic for switching between navigation states and calling the functions to run in each state.

#### `rover.cpp`
This file defines the rover and rover status objects. The rover object is used throughout the codebase to interact with real-life capabilities of the rover. Notably, the object contains functions like `drive()` and `turn()`. The rover status object/class is nested in the rover class, and it contains information about the current state of the rover and relevant features like targets and obstacles. Most variables in the rover status are populated from LCM messages.

---

!----------------------------- Gate Search ----------------------------->
## Gate Search (`gate_search/` folder)

#### `diamondGateSearch.cpp`
This file creates the search waypoints in the shape of a diamond for completing a search for the second gate post.

#### `gateStateMachine.cpp`
Defines gate search/traversal states and functions.


---

!----------------------------- Obstacle Avoidance ----------------------------->
## Obstacle Avoidance

#### `obstacleAvoidanceStateMachine.cpp`
Defines an obstacle avoidance state machine with minimal functionality, intended to be a parent class for different types of obstacle avoidance strategies

#### `simpleAvoidance.cpp`
This contains our current only implementation of obstacle avoidance behavior. Inherited from the obstacle state machine, it is a very simple algorithm and just drops a waypoint at the front of the queue, with a position at a safe location away from the obstacle, for the rover to drive to before continuing to its previous destination.


---

!----------------------------- Search ----------------------------->
## Search
Similar to the `gate_search/` folder, this folder for search logic contains a `searchStateMachine` object and files to define the waypoints for different types of searches. First we follow a square spiral outwards with points generated in spiralOutSearch.cpp, then if the search completes and the target is not found, we will move onto trying the lawnmower search and the spiral in search.


---

!----------------------------- Vars and Utils ----------------------------->
## Variables and Utilities

#### Nav State
The nav state specifies what state of the state machine we are in. It is implemented as an enum (a C++ type) where named states are associated with a number behind the scenes.

#### Auton State
This is just a boolean hidden as a type called AutonState and it tells us if Auton is on or off. It is read from LCMs published by the GUI or simulator.

##### `utilities.cpp`
Contains functions used commonly throughout auton code. 



