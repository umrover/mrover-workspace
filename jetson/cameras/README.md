Code to control the USB Cameras
----

### About
For the 2022 Mars Rover there are 8 USB cameras. This is the program \
responsible for handling LCM messages received from the base station. \

The LCM currently takes two integers representing the device number that the user wants on each of the ports, where the streams can be accessed at 10.0.0.1:5000 and 10.0.0.1:5001. Extra streams can also be accessed at 10.0.0.2:5000 and 10.0.0.2:5001 during the science mission, accessible on a the other science laptop. -1 means no device, and 0-7 represent what the jetson recognizes at /dev/video0 to /dev/video7. The program does not crash if the video device does not exist.

The program relies on jetson-utils to operate. Its python functions are called in order to get feed from the camera and render them to a stream. Two pipelines (two streams) are constantly capturing images and rendering them to the output. 

run.py is an extra file that can be used to test streaming through the terminal with python subprocess. It is not advised to use subprocess and instead use the jetson-utils python functions because with subprocess, the feeds sometimes secretly crash and does not automatically recover. 


#### LCM Channels Publishing/Subscribed To 
**Cameras Command [subscriber]** \
Messages: [ Cameras.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Cameras.lcm) “/cameras_cmd” \
Publishers: jetson/teleop \
Subscribers: jetson/cameras

**Mission [subscriber]** \
Messages: [ Mission.lcm ](https://github.com/umrover/mrover-workspace/blob/master/rover_msgs/Mission.lcm) “/cameras_mission \
Publishers: jetson/teleop \
Subscribers: jetson/cameras

### Usage 
The ansible scripts rover-base_station_gstreamer.service and rover-base_station_gstreamer.service_2 should be running in the background. On the jetson, run the ansible script rover-jetson-cameras.service which should run `./jarvis exec jetson/cameras`.

To run the ansible scripts, on the base station try doing ```sudo systemctl restart rover-base_station_gstreamer.service``` and ```sudo systemctl restart rover-base_station_gstreamer_2.service```. On the jetson, try doing ```sudo systemctl restart rover-jetson-cameras.service```. Or run the ansible playbook.

The user can edit the bitrate and the remote ip and ports by editing the jetson/cameras/src/\_\_main\_\_.py file.

### Setup

In a docker container, do the equivalent of the following:

Clone [jetson-utils](https://github.com/dusty-nv/jetson-utils) on the Jetson outside of mrover-workspace, make build folder, cd into that and run cmake .. then make. Without the jetson program, you can just run ```./aarch64/bin/video-viewer /dev/video0 rtp://10.0.0.1:5000 --headless``` on the jetson.

```
git clone https://github.com/dusty-nv/jetson-utils
cd jetson-utils
mkdir build
cd build
cmake ..
make
```


Depending on your version of python, you may need to change the CMakeLists.txt inside the jetson-utils/python/ folder and include your version for the PYTHON_BINDING_VERSIONS. In addition to this, you should edit the jetson/cameras/src/\_\_main\_\_.py file and replace "/usr/lib/python3.6/dist-packages" with your version of python.  

### Issues
Depending on which Jetson you use, you may not be able to stream as many cameras as you'd like at the same time. We use the Jetson Xavier NX, which can only stream up to 4 cameras via USB at a time.

Still need to figure out a way to get camera streams onto the gui.

Get jetson-utils in travis.

### ToDo 

- [X] Verify that ansible scripts work on the jetson
- [ ] Map physical ports to video so cameras are known by default
- [X] Fix up issue with microscope camera
- [ ] Camera program freezes if USB is disconnected while streaming
- [ ] Low priority - Get camera streams onto the gui
- [ ] Look into 4 camera streams during other missions (prob change LCM w/ Teleop) + ansible scripts
