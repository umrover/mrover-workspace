Code for Servos for Ammonia Test
 ---
### About

This code is intended to run the servos for the ammonia test at certain angles in order to drop the ammonia ??? idk what this is for exactly

### LCM Channels Publishing/Subscribed To
**Ammonia Servo**[subscriber] \
Messages:  [AmmoniaServo.lcm](https://github.com/jjtom34/mrover-workspace/blob/AmmoniaServo/rover_msgs/AmmoniaServo.lcm) "/ammoniaservo" \
Publishers: tbd
Subscribers: tbd


### Usage
#### Require Electronic Components:
- 1 Servo
- 1 Beaglebone Black board or similar board


#### Running
When the servo is set up, the program should be set up as follows
`$ ~/cd mrover-workspace`
`$ /jarvis build beaglebone/ammoniaservo` \
`$ /jarvis exec beaglebone/ammoniaservo` \


#### LCM Commands
When the test wants to be run the following command should send an empty lcm message triggering the drop.
`$ LCM_DEFAULT_URL="udpm://239.255.76.67:7667?ttl=255" ./jarvis exec lcm_tools_send “/ammoniaservo” “{}”`
I'm guessing this is how an empty lcm message is formatted  

This should set the servo angle to 0 and then 90 degrees after a short delay, effectively dropping the material.  


### To Do
-   [x] Base Code
    
-   [ ] Tested LCM implementation
-   [ ] Testing on the Rover

### Notes

--

  

