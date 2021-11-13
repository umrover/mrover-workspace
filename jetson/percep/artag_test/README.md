# AR Tag Test Suite
This folder houses the necessary images, labels, and code to test the TagDetector classes capability to accurately detect and classify AR Tags.
## File Structure
|Filename or Folder Name  |Description  |
|--|--|
|artag_test_errors.[hc]pp  |Definition and implementation of the different errors that could be thrown when running the test suite.|
|artag_test_framework.[hc]pp|Definition and implementation of the Test and TestSuite classes.|
|main.cpp|Driver that runs the test suite.|
|images/|Stores all testing images.|
|labels/|Stores a label (AR Tag id and corners) that correspond to each image.|
## Usage
### Building and Running the Code
This can be accomplished by running two commands from the mrover_workspace directory.
```
./jarvis build jetson/percep -o artag_tests=true perception_debug=false
./jarvis exec jetson_percep_artag
```
### Adding  a Testcase
The Test Suite will automatically run all test cases in the images/ folder. To add a testcase here are the steps we need to follow. First pick an image and place it in the images folder and give it a corresponding label in the labels folder. The naming convention that I have been using is:
 image1.jpg -> image1.tag, image2.jpg ->image2.tag. 

However feel free to name the image whatever you want so long as you follow this convention:
{image_name}.jpg -> {image_name}.tag

The format of the label file is as follows:
```
{ARTAG1_ID}
{TAG1_CORNER1_X} {TAG1_CORNER1_Y}...{TAG1_CORNER4_X} {TAG1_CORNER4_Y}
{ARTAG2_ID}
{TAG2_CORNER1_X} {TAG2_CORNER1_Y}...{TAG2_CORNER4_X} {TAG2_CORNER4_Y}
```
If there is only 1 tag in the image only include the first 2 rows. Additionally list the tags in the left to right order they appear in the image. If the image has 3 tags, only include the leftmost and rightmost tags in the image.



