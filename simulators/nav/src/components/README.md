# <img src="../static/mrover.png" alt="MRover Logo" width="30"/> MRover Navigation Simulator

## Table of Contents
[Directory Overview](#directory-overview)<br/>
[Code Overview](#code-overview)<br/>
[Related Documents](#related-documents)<br/>

---

<!---------------------------- Directory Overview ---------------------------->
## Directory Overview
The `components` directory contains all the Vue components that make up the simulator as well as the back-end logic that specifically makes each of those components work.

---

<!------------------------------- Code Overview ------------------------------>
## Code Overview
This sections gives descriptions of the various directories and files in the `components` directory. This continues the code overview started in the the [src `README`](../README.md).

#### `common`
The `common` directory contains components that can be reused throughout the project (i.e. they are not specialized for a certain functionality).

##### `BinaryIndicator.vue`
The `BinaryIndicator` component displays if the input variable is on or off.

##### `Button.vue`
The `Button` component is a simple button. An example of a `Button` is the "Reset Rover" button in the `DebugTools` component.

##### `Checkbox.vue`
Th `Checkbox` component is a simple checkbox. An example of a `Checkbox` is the "Play/Pause" button in the Debug Tools component.

##### `NumberInput.vue`
The `NumberInput` component like an HTML `input` tag where the value is enforced to be a number.

##### `RadioSelector.vue`
The `RadioSelector` component is grouping of radio buttons. It is templated so options can be of any type (but must be uniform). An example is the Odom Format options in the SimSettings component.

#

#### `control_panel`
The `control_panel` directory contains the components that make up the `ControlPanel` component which contains the majority of the controls for the simulator.

##### `ControlPanel.vue`
The `ControlPanel` component is the container for the SimSettings, DrawModule, and DebugTools components.

##### `DebugTools.vue`
The `DebugTools` component contains IDE features (e.g. pause) and rover optons (e.g. field of view and speed).

##### `DrawModule.vue`
The `DrawModule` component controls options for placing items on the field/canvas.

##### `SimSettings.vue`
The `SimSettings` component includes the odom format selector, the simulate localization checkbox, etc.

#

#### `field`
The `field` directory contains the components that make up the `Field` component which handles all of the drawing on the field/canvas.

##### `ar_tags.ts`
The `CanvasArTags` class for drawing ar tags and gates on the canvas.

##### `Field.vue`
The `Field` component includes uploading/downloading test cases as well as drawing items on the canvas.

##### `obstacles.ts`
The `CanvasObstacles` class for drawing obstacles on the canvas.

##### `repeater.ts`
The `CanvasRepeater` class for drawing the radio repeater on the canvas.

##### `rover.ts`
The `CanvasRover` class for drawing the rover on the canvas.

##### `waypoints.ts`
The `CanvasWaypoints` class for drawing waypoints on the canvas.

#

#### `field_items`
The `field_items` directory contains the components that make up the `FieldItems` component which lists and details all of the items on the field/canvas.

##### `ArTagItem.vue`
The `ArTagItem` component displays an AR tag item in the field items component.

##### `FieldItemInfo.vue`
The `FieldItemInfo` component displays latitude, longitude, distance from rover, and bearing from rover in the various types of field item components.

##### `FieldItems.vue`
The `FieldItems` component is the container for the lists of AR tags, gates, obstacles, radio repeater, and waypoints.

##### `GateItem.vue`
The `GateItem` component displays a gate item in the field items component.

##### `ObstacleItem.vue`
The `ObstacleItem` component displays an obstacle item in the field items component.

##### `RadioRepeaterItem.vue`
The `RadioRepeaterItem` component displays the radio repeater item in the field items component.

##### `WaypointItem.vue`
The `WaypointItem` component displays a waypoint item in the field items component.

#

#### `header`
The `header` directory contains the components that make up the `Header` component.

##### `Header.vue`
The `Header` component contains the logo, title, and the connection status indicators.

#

#### `lcm_center`
The `lcm_center` directory contains the components that make up the `LCMCenter` component which handles displaying LCM messages (including the on/off button).

##### `AutonStateLCM.vue`
The `AutonStateLCM` component displays the auton state lcm as well as acts as the on/off button for autonomy.

##### `JoystickLCM.vue`
The `JoystickLCM` component displays the joystick lcm.

##### `LCMCenter.vue`
The `LCMCenter` component contains the components for displaying the various LCM messages (e.g. nav status and radio signal strength). Note that this includes turning on and off autonomy the auton state LCM.

##### `NavStatusLCM.vue`
The `NavStatusLCM` component displays the nav status lcm.

##### `ObstacleLCM.vue`
The `ObstacleLCM` component which displays the obstacle lcm.

##### `OdometryLCM.vue`
The `OdometryLCM` component which displays the odometry lcm.

##### `RadioRepeaterLCM.vue`
The `RadioRepeaterLCM` component which displays the repeater strength lcm as well as allows for updating the simulated radio signal strength.

##### `TargetListLCM.vue`
The `TargetListLCM` component which displays the target list lcm.

#

#### `perception`
The `perception` directory contains the components that make up the `Perception` component which handles all the logic for the simulated perception system. None of the components in this directory are visible.

##### `obstacle_detector.ts`
The `ObstacleDetector` class contains the logic for detecting obstacles.

##### `open_interval_heap.ts`
The `OpenIntervalHeap` class which handles the logic for finding open intervals in a given range. This is used for tasks such as finding an open area to drive through in obstacle detection.

##### `Perception.vue`
The `Perception` component is an abstract component. It simulates all the logic that is performed by the rover's perception program.

##### `taret_detector.ts`
The `TargetDetector` class contains the logic for detecting targets.

#

#### `NavSimulator.vue`
The `NavSimulator` component contains all of the simulator's major components (e.g. Header, Perception, Field, etc.). This is the main body of the simulator.

#### `HotKeys.vue`
The `HotKeys` component contains logic for enabling hot keys and renders nothing
to the screen. See the
[Hotkeys Guide](/simulators/nav/src/components/hotkeys.md) for details on what
hotkeys there are and what they do.

---

<!----------------------------- Related Documents ---------------------------->
## Related Documents
* [project `README`](../../README.md)
* [`src` `README`](../README.md)
