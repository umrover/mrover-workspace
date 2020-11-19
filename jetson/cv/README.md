## To build with meson options.
    jarvis build jetson/cv -o [options]

For example:

    jarvis build jetson/cv -o with_zed=true perception_debug=true write_frame=true data_folder='/home/jessica/auton_data/'

## To run with the ZED:
    with_zed=true

## To run with data (if you don't have the ZED or nvidia):
    with_zed=false

## To run in competition (no output):
    with_zed=true
    perception_debug=false

## To collect data:
    with_zed=true
    write_frame=true
    data_folder='/home/jessica/auton_data/' (replace with path where you want the data saved)

