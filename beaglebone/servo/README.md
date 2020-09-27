Code for Servos
 ---
### About

This code is intended to run the servos in order to lower the beaker for science testing. The beaker should stay in place until an empty LCM struct is sent where it will lower the beaker.

### LCM Channels Publishing/Subscribed To
**ServoCMD**[subscriber] \
Messages:  [ServoCmd.lcm](https://github.com/jjtom34/mrover-workspace/blob/AmmoniaServo/rover_msgs/ServoCmd.lcm) "/servo_cmd" \
Publishers: tbd
Subscribers: tbd


### Usage
#### Require Electronic Components:
- 1 Servo
- 1 Beaglebone Black board or similar board


#### Setup
When the servo is set up with the beaker, the program should be run as follows
`$ ~/cd mrover-workspace`
`$ /jarvis build beaglebone/servo` \
`$ /jarvis exec beaglebone_servo` \


#### LCM Commands
When the test wants to be run the following command should send an empty lcm message triggering the drop.
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_send “/servocmd” “{‘type’: ‘ServoCMD’, ‘id’: ‘<id>’, ‘degrees’: 90}”`
Replace `<id>` with the appropriate servo, `'left'` or `'right'` .  

This triggers the callback and should set the angle to 0 and then 90 degrees after a short delay.  

### To Do
-   [x] Base Code

-   [ ] Find out Specific Beaglebone pin and max/min dc for the servo
    
-   [ ] Test LCM implementation
-   [ ] Testing on the Rover

### Notes

--

  

