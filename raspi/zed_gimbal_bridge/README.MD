Code to control the zed gimbal, using pi3hat and moteus
===============================================
### About

zed_gimbal_bridge is responsible for connecting the rover network's LCM interface with the zed gimbal system's motor controller, the moteus.

### LCM Channels
#### Zed Gimbal Position \[Publisher\] "/zed_gimbal_data"
Message: [ZedGimbalPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ZedGimbalPosition.lcm) \
Publisher: raspi/zed_gimbal_bridge \
Subscribers: jetson/nav and base_station/gui

#### Zed Gimbal Position \[Subscriber\] "/zed_gimbal_cmd"
Message: [ZedGimbalPosition.lcm](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/ZedGimbalPosition.lcm) \
Publisher: jetson/nav \
Subscriber: raspi/zed_gimbal_bridge

### Usage

To build zed_gimbal_bridge use `$./jarvis build raspi/zed_gimbal_bridge/ ` from the mrover-workspace directory.

To run zed_gimbal_bridge use `$./jarvis exec raspi/zed_gimbal_bridge/ `
from the mrover-workspace directory.

### Common Errors

None known so far

### Notes

Reference.md for moteus is [here](https://github.com/mjbots/moteus/blob/main/docs/reference.md).
