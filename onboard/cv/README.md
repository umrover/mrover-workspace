## To run with the ZED:
    perception.hpp ->       define ZED_SDK_PRESENT true 
    meson.build ->          set with_zed to true

## To run with data (if you don't have the ZED or nvidia):
    perception.hpp ->       define ZED_SDK_PRESENT false 
    meson.build ->          set with_zed to false    

## To run in competition (no output):
    perception.hpp ->       define ZED_SDK_PRESENT true 
                            define PERCEPTION_DEBUG false 
    meson.build ->          set with_zed to true

## To collect data:
    perception.hpp ->       define ZED_SDK_PRESENT true 
                            set WRITE_CURR_FRAME_TO_DISK = true
                            replace DEFAULT_ONLINE_DATA_FOLDER("/home/jessica/auton_data/") with the path where you want the data saved
    meson.build ->          set with_zed to true
