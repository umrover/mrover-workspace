Code to control the USB Cameras
----

### About
For the 2022 Mars Rover there are 8 USB cameras. This is the program \
responsible for playing the base station camera streams. \

### Usage 
On the base station, run `python3 base_station/cameras/run.py`. On the jetson, run `./jarvis exec jetson/cameras`.

### Issues
Still need to figure out a way to get camera streams onto the gui. Also, it would be preferred to use the ansible script instead of this.

### ToDo 

- [X] Fix disconnect issues
- [ ] Get camera streams onto the gui
- [ ] Verify that the ansible scripts work so that this program doesn't need to be used.
