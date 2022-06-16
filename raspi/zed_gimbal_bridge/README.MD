Code to control the zed gimbal, using pi3hat and moteus
===============================================
### About

zed_gimbal_bridge is responsible for connecting the rover network's LCM interface with the zed gimbal system's motor controller, the moteus. Note that this code has never actually been used on the rover.

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

SSH into the raspberrypi.

Enable multicast by running `sudo ifconfig eth0 multicast` and `sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0`.
Replace eth0 with whatever the connection is if that does not work.

To build zed_gimbal_bridge use `sudo ./jarvis build raspi/zed_gimbal_bridge/` from the mrover-workspace directory.

To run zed_gimbal_bridge use `sudo LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec raspi/zed_gimbal_bridge/`
from the mrover-workspace directory.

If you want to listen to channels, on the basestation use `LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_echo ZedGimbalPosition "/zed_gimbal_data"`.

To command it to go specific angles, one can use `LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_send "/zed_gimbal_cmd" "{'type' : 'ZedGimbalPosition', 'angle' : 0}"` where the angle is in degrees.


### Common Errors

When first integrating the zed gimbal motor with the moteus controller, there were issues with wobbling and jittering. There is a thread on the MJBots discord server called "2022-01-22 new motor configuration" that goes over some of these problems and its solutions. The following will summarize a few of these errors.

For the 2022 zed gimbal motor controller, voltage_mode_control had to be set to 1 (changing it from current mode control to voltage mode control) because our motor had a range of currents of 0-1A while the moteus sense resistors were trying to sense currents between 0-100A.

Also, the moteus should be setup so that its PID values are tuned. The simplest way to do this is through tview. One way is to forward the gui onto the computer (one can use VcXsrv or Xming). Otherwise, the easiest way is to connect the raspberry pi to a monitor with an HDMI cable.


### Notes

Reference.md for moteus is [here](https://github.com/mjbots/moteus/blob/main/docs/reference.md).

## TODO
- [ ] Finish this readme
- [ ] Change from left-handed coordinate system to right
- [ ] Figure how to ssh into pi through linux system
- [ ] Enable multicast be default