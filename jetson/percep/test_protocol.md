## Compile Test
- [ ] Offline Debug: ` ./jarvis build jetson/percep -o with_zed=false ar_detection=true obs_detection=true perception_debug=true `
  - [ ] No compiler errors or warnings
- [ ] Offline Silent: ` ./jarvis build jetson/percep -o with_zed=false ar_detection=true obs_detection=true perception_debug=false `
  - [ ] No compiler errors or warnings
- [ ] ZED Debug: ` ./jarvis build jetson/percep -o with_zed=true ar_detection=true obs_detection=true perception_debug=true `
  - [ ] No compiler errors, only ZED warnings
- [ ] ZED Silent: ` ./jarvis build jetson/percep -o with_zed=true ar_detection=true obs_detection=true perception_debug=false `
   - [ ] No compiler errors, only ZED warnings

## Offline Debug Test
- [ ] `./jarvis build jetson/percep -o with_zed=false ar_detection=true obs_detection=true perception_debug=true`
- [ ] Run code against test1 image set located in perception-files repository
- Check for:
   - [ ]  Left and right path are being calculated
   - [ ]  Calculated bearing is reasonable given viewer output
   - [ ]  AR Tag ID is 2
   - [ ]  Obstacles with large width are detected
   - [ ]  Path is projected around not through large obstacles
   - [ ]  Short obstacles are detected
   - [ ]  Steep inclines are detected as obstacles
- [ ] Run code against test2 image set located in perception-files repository
- Check for:
   - [ ]  Left and right path are being calculated
   - [ ]  Calculated bearing is reasonable given viewer output
   - [ ]  AR Tag ID is 6
   - [ ]  2 AR Tags detected simultaneously
   - [ ]  Obstacles with large width are detected
   - [ ]  Path is projected around not through large obstacles
   - [ ]  Short obstacles are detected
   - [ ]  Steep inclines are detected as obstacles

## Offline Silent Test
- [ ] `./jarvis build jetson/percep -o with_zed=false ar_detection=true obs_detection=true perception_debug=false`
- [ ] Run `time ./jarvis exec jetson/percep` against test_case1 image set
- [ ] Run `time ./jarvis exec jetson/percep` against test_case2 image set
- [ ] User value is less than or equal to \<TODO Determine Value\>

## Unit Test
- [ ] ` ./jarvis build jetson/percep -o with_zed=true ar_detection=true obs_detection=true perception_debug=true `
- [ ] `./jarvis exec lcm_tools_echo Obstacle "/obstacle"`
- [ ] `./jarvis exec lcm_tools_echo TargetList "/target_list"`
- Test environment contains:
  - [ ] 1 large obstacle ~ over 40cm tall
  - [ ] 2 small obstacles ~  25cm-30cm tall
  - [ ] 1 post
  - [ ] 1 hill
- Test for:
  - [ ] Detected large obstacle 6m away
  - [ ] Detected small obstacle 2m away
  - [ ] Detected clear path when 1.2m gap between 1 large and 1 small obstacle
  - [ ] Left and right path are being calculated
  - [ ] Calculated bearing is reasonable given viewer output of path
  - [ ] AR Tag ID is the correct ID
  - [ ] Obstacles with large width are detected
  - [ ] Short obstacles are detected
  - [ ] FPS values don't exceed \<TODO find this value in Issue #552\>
  - [ ] Obstacle.lcm in /obstacle is populated correctly
  - [ ] TargetList.lcm in /target_list is populated correctly

## Integration Test
- [ ] Create new branch based off of main
- [ ] Checkout new branch
- [ ] Cherry-Pick all PRs that need testing onto new branch
- [ ] Compile all Software Packages
- [ ] Run all code simultaneously
- [ ] Obstacle.lcm in /obstacle is populated correctly
- [ ] TargetList.lcm in /target_list is populated correctly
- [ ] Sensors are Active
  - [ ] Stereo Camera (ZED)
  - [ ] Inertial Measurement Unit (IMU)
  - [ ] Global Positioning System (GPS)
