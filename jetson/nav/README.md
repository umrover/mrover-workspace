# <img src="src/static/mrover.png" alt="MRover Logo" width="30"/> MRover Navigation Codebase

## Table of Contents
[Project Overview](#project-overview)<br/>
[main.cpp](#main.cpp)<br/>
[stateMachine.hpp](#stateMachine.hpp)<br/>
[stateMachine.cpp](#stateMachine.cpp)<br/>
[rover.cpp](#rover.cpp)<br/>
[gate_search/](#gate_search)<br/>

**Created:** Ana Warner, Oct 3, 2021
**Updated:** A.Warner, 10/3/2021

---

<!----------------------------- Project Overview ----------------------------->
## Project Overview
The nav codebase contains logic for commanding the rover during the Autonomous Traversal task. The task consists of several legs that increase in difficulty. During a leg, the operator starts inputs the destination GPS waypoint and end behavior for the leg (search and/or gate), then starts the Auton system from the base station. The rover will first drive autonomously to the destination waypoint without responding to target data from Perception. It will avoid obstacles as they appear. Once the waypoint is reached, the rover will start a search pattern if the leg was a search leg. The search algorithm first attempts a spiral out search pattern and approaches the tag once it is found. Finally, if there is a second tag composing the gate, the gate search and traversal algorithm will start.

---

!----------------------------- main.cpp ----------------------------->
## main.cpp
The main.cpp file contains the main() function. In the main function, we create an instance of the state machine, create the LCM handlers class, and subscribe to the LCM channels that we will read messages from. (For more about LCM’s, see below.) Then we call the outermost function of the state machine, run(), which begins executing the state machine logic.

---
