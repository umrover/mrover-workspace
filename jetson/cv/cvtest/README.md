# MRover CV Testing Suite

## Setup

Images taken outside in the GBBL parking lot and labeled with center coordinates and depth data are contained in cv_test_images. Any additional images must be named in the format: imgnumber_xcoordinate_ycoordinate_depth_.jpg.

## Building and Executing

Navigate to mrover-workspace and run ./jarvis build jetson/cv/cvtest to build the test. Then, run ./jarvis exec jetson_cv_cvtest to execute it

## Output

For each image, the test number, predicted center, real center, and difference between the predicted x and y coordinates and the real ones is printed via cout. It also says at the top whether or not a given test 'passed', with passing being defined as both the x and y predictions being within 10 pixels of the real values.
