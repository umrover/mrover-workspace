## Build
    ./jarvis build jetson/percep

## Build with Options
    ./jarvis build jetson/percep -o [options]

## Execute
    ./jarvis exec jetson/percep

## Configuration Options:
    with_zed
    perception_debug
    obs_detection
    ar_detection
    ar_record
    vm_config
    write_frame
    data_folder

## Option Descriptions:

### with_zed
    [true] will grab images from zed
    [false] will grab images from folder

### perception_debug
    [true] will print debug output
    [false] will run in silent mode

### obs_detection
    [true] will run obstacle detection
    [false] will not run obstacle detection

### ar_detection
    [true] will run ar detection
    [false] won't run ar detection

### ar_record
    [true] will create video of ar detection output
    [false] will not create video

### vm_config
    [true] will run obstacle detection with VTK 6.3
    [false] will run obstacle detection with VTK 8.2

### write_frame
    [true] will write input frames to a file
    [false] will not write frames to file

### data_folder
    ['<path to folder>'] takes path to folder to write images in

## Handy Configurations:

### Record Data from ZED
    ./jarvis build jetson/percep -o with_zed=true write_frame=true data_folder='/home/<username>/folder/'

### Obstacle Detection Only
    ./jarvis build jetson/percep -o with_zed=false ar_detection=false obs_detection=true

### AR Detection Only
    ./jarvis build jetson/percep -o with_zed=false ar_detection=true obs_detection=false

### VirtualBox
    ./jarvis build jetson/percep -o with_zed=false ar_detection=true obs_detection=true vm_config=true