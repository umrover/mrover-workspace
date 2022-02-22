Code to control the USB Cameras
----

### About
For the 2022 Mars Rover there are 8 USB cameras. This is the program \
responsible for handling LCM messages recieved from the base station. \


#### LCM Channels Publishing/Subscribed To 
**Cameras Command [subscriber]** \
Messages: [ Cameras.lcm ](https://github.com/amszuch/mrover-workspace/blob/cameras/rover_msgs/Cameras.lcm) “/cameras_cmd” \
Publishers: base_station/gui \
Subscribers: jetson/cameras

### Usage 
On the base station, run `python3 base_station/cameras/run.sh`. On the jetson, run `./jarvis exec jetson/cameras`

### Issues
The cameras randomly disconnect. The operator with the base station gui may have to turn off and turn on each camera.

### ToDo 

- [ ] Fix disconnect issues
- [ ]
