## To build with meson options.
    jarvis build onboard/cv -o [options]

For example:

    jarvis build onboard/cv -o with_zed=true perception_debug=true write_frame=true data_folder='/home/jessica/auton_data/'

## To run with the ZED:
    with_zed=true

## To run with data (if you don't have the ZED or nvidia):
    with_zed=false

## To run in competition (no output):
    with_zed=true
    perception_debug=false

## To not run ar
    ar_detection=false

## To not run obstacle
    obs_detection=false
    
## To record ar video
    ar_record=true

## To record obs video
    obs_record=true

## To collect data:
    with_zed=true
    write_frame=true
    data_folder='/home/jessica/auton_data/' (replace with path where you want the data saved)

