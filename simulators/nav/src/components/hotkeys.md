# <img src="../static/mrover.png" alt="MRover Logo" width="30"/> MRover Navigation Simulator Hotkeys Guide

| Hotkey | Description | File |
| :----- | :---------- | :--- |
|| <div align="center">_LCM Messages_</div> ||
| `shift+enter` | Turn the rover on/off. | `HotKeys.vue` |
|| <div align="center">_Debug Tools_</div> ||
| `shift+space` | Play/pause the simulator. | `HotKeys.vue` |
| `shift+alt+space` | When the rover is paused, take a "step" by applying a single joystick command then pausing again. | `HotKeys.vue` |
| `shift+up` | Apply a single drive forward joystick command. | `HotKeys.vue` |
| `shift+down` | Apply a single drive backward joystick command. | `HotKeys.vue` |
| `shift+left` | Apply a single turn left joystick command. | `HotKeys.vue` |
| `shift+right` | Apply a single turn right joystick command. | `HotKeys.vue` |
| `shift+alt+left` | Apply a single ZED gimbal command for 1 degree to the left. | `HotKeys.vue` |
| `shift+alt+right` | Apply a single ZED gimbal command for 1 degree to the right. | `HotKeys.vue` |
| `shift+r` | Turn off the rover and move it to the starting location. | `DebugTools.vue` |
|| <div align="center">_Draw Module_</div> ||
| `tab` | Switch the the next draw mode. | `HotKeys.vue` |
| `shift+tab` | Switch the the previous draw mode. | `HotKeys.vue` |
| `shift+1` | Switch to waypoint draw mode. | `HotKeys.vue` |
| `shift+2` | Switch to AR tag draw mode. | `HotKeys.vue` |
| `shift+3` | Switch to gate draw mode. | `HotKeys.vue` |
| `shift+4` | Switch to obstacle draw mode. | `HotKeys.vue` |
| `shift+5` | Switch to reference point draw mode. | `HotKeys.vue` |
| `shift+s` | Toggle the search point button in the draw module. | `DrawModule.vue` |
| `shift+g` | Toggle the gate search point button in the draw module | `DrawModule.vue` |
|| <div align="center">_Field Controls_</div> ||
| `shift+backspace` | Clear all field items and the rover's path. | `FieldItems.vue` |
| `shift+c keydown` | Turn on "Move Rover Mode" which allows you to click on the field to move the rover. | `HotKeys.vue` |
| `shift+c keyup` | Turn off "Move Rover Mode" (see `shift+c keydown`). | `HotKeys.vue` |
| `shift+alt+c` | Make the current rover location the starting location. | `HotKeys.vue` |
| `shift+ctrl+c` | Reset the starting location to the original location. | `HotKeys.vue` |
|| <div align="center">_Simulator Settings_</div> ||
| `shift+l` | Turn on/off simulation of localization. | `HotKeys.vue` |
| `shift+p` | Turn on/off simulation of perception. | `HotKeys.vue` |
| `shift+alt+d` | Switch odometry readings to degrees. | `HotKeys.vue` |
| `shift+alt+m` | Switch odometry readings to degrees and minutes. | `HotKeys.vue` |
| `shift+alt+s` | Switch odometry readings to degrees, minutes and seconds. | `HotKeys.vue` |
