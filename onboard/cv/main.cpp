#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <unistd.h>
#include <thread>
#include <ctime>

using namespace cv;
using namespace std;

int calcFocalWidth(){   //mm
    return tan(fieldofView/2) * focalLength;
}

int calcRoverPix(float dist, float pixWidth){   //pix
    float roverWidthSensor = realWidth * 1.2  * focalLength/(dist * 1000);
    return roverWidthSensor*(pixWidth/2)/calcFocalWidth();
}

float getGroundDist(float angleOffset){  // the expected distance if no obstacles
    return zedHeight/sin(angleOffset);
}

double getAngle(float xPixel, float wPixel){
    return atan((xPixel - wPixel/2)/(wPixel/2)* tan(fieldofView/2))* 180.0 /PI;
}

float getObstacleMin(float expected){
    return expected - obstacleThreshold/sin(angleOffset);
}

bool cam_grab_succeed(Camera &cam, int & counter_fail) {
  while (!cam.grab()) {
    counter_fail++;
    usleep(1000);
    if (counter_fail > 1000000) {
      cerr<<"camera failed\n";
      return false;
    }
  }
  counter_fail = 0;
  return true;
}

static string rgb_foldername, depth_foldername;
void disk_record_init() {
  #if WRITE_CURR_FRAME_TO_DISK
    // write images colleted to the folder
    // absolute path
    rgb_foldername = DEFAULT_ONLINE_DATA_FOLDER "rgb/";
    depth_foldername = DEFAULT_ONLINE_DATA_FOLDER "depth/";
    string mkdir_rgb =  std::string("mkdir -p ") + rgb_foldername;
    string mkdir_depth =  std::string("mkdir -p ") + depth_foldername;
    int dir_err_rgb = system( mkdir_rgb.c_str() );
    int dir_err_depth = system(mkdir_depth.c_str());
    if (-1 == dir_err_rgb || -1 == dir_err_depth) {
      exit(1);
    }
  #endif
}

void write_curr_frame_to_disk(Mat rgb, Mat depth, int counter) {
    string fileName = to_string(counter / FRAME_WRITE_INTERVAL);
    while(fileName.length() < 4){
      fileName = '0'+fileName;
    }
    cv::imwrite(rgb_foldername +  fileName + std::string(".jpg"), rgb );
    cv::imwrite(depth_foldername +  fileName + std::string(".exr"), depth );
}

int main() {
  /*initialize camera*/
  Camera cam;
  cam.grab();
  int j = 0;
  int counter_fail = 0;
  #if PERCEPTION_DEBUG
    namedWindow("Obstacle");
    namedWindow("depth", 2);
  #endif
  disk_record_init();

  //Video Stuff
  TagDetector d1;
  pair<Tag, Tag> tp;

  //Create time value
  time_t now = time(0);
  char* ltm = ctime(&now);
  string timeStamp(ltm);
  Mat rgb;
  Mat src = cam.image();
  

  #if AR_RECORD
  //initializing ar tag videostream object
  
  Mat depth_img = cam.depth();
  tp = d1.findARTags(src, depth_img, rgb);
  Size fsize = rgb.size();
  
  string s = "artag_" + timeStamp + ".avi";

  VideoWriter vidWrite(s, VideoWriter::fourcc('M','J','P','G'),10,fsize,true);

  if(vidWrite.isOpened() == false)
  {
	  cout << "didn't open";
	  exit(1);
  }
  #endif

  #if OBS_RECORD
 //initializing obstacle detection
  src = cam.image();
  Mat bigD = cam.depth();
  float pw = src.cols;
  int roverpw = calcRoverPix(distThreshold, pw);

  avoid_obstacle_sliding_window(bigD, src, num_sliding_windows, roverpw);
  Size fs = src.size();
  string name = "obs_" + timeStamp + ".avi";

  VideoWriter vidWriteObs(name, VideoWriter::fourcc('M','J','P','G'),10,fs,true);

  if(vidWriteObs.isOpened() == false)
  {
	  cout << "didn't open";
	  exit(1);
  }
  #endif

  /*initialize lcm messages*/
  lcm::LCM lcm_;
  rover_msgs::TargetList arTagsMessage;
  rover_msgs::Target* arTags = arTagsMessage.targetList;
  rover_msgs::Obstacle obstacleMessage;
  arTags[0].distance = -1;
  arTags[1].distance = -1;
  obstacleMessage.detected = false;

  //tag detection stuff
  TagDetector detector;
  pair<Tag, Tag> tagPair;
  int left_tag_buffer = 0;
  int right_tag_buffer = 0;

  #if PERCEPTION_DEBUG
    //Make trackbars
    int thresh1 = 300000;
    int thresh2 = 70000;
    createTrackbar("Main Window", "Obstacle", &thresh1, 500000);
    createTrackbar("Sub Window", "Obstacle", &thresh2, 120000);
  #endif
  
  while (true) {
    if (!cam_grab_succeed(cam, counter_fail)) break;

    Mat rgb;
    Mat src = cam.image();
    Mat depth_img = cam.depth();

    // write to disk if permitted
    #if WRITE_CURR_FRAME_TO_DISK
      if (j % FRAME_WRITE_INTERVAL == 0) {
        Mat rgb_copy = src.clone(), depth_copy = depth_img.clone();
        thread write_thread(write_curr_frame_to_disk, rgb_copy, depth_copy, j);
        write_thread.detach();
      }
    #endif

    /* AR Tag Detection*/
    arTags[0].distance = -1;
    arTags[1].distance = -1;
    #if AR_DETECTION
      tagPair = detector.findARTags(src, depth_img, rgb);
      #if AR_RECORD
      vidWrite.write(rgb);
      #endif
      //update both tags in LCM message
      //first tag
      if(tagPair.first.id == -1){//no tag found
        if(left_tag_buffer <= 20){//send the buffered tag
          ++left_tag_buffer;
        } else {//we probably actually lost the tag
          arTags[0].distance = -1;
          arTags[0].bearing = -1;
          arTags[0].id = -1;
        }
      } else { //one tag found
        if(!isnan(depth_img.at<float>(tagPair.first.loc.y, tagPair.first.loc.x)))
          arTags[0].distance = depth_img.at<float>(tagPair.first.loc.y, tagPair.first.loc.x);
        arTags[0].bearing = getAngle((int)tagPair.first.loc.x, src.cols);
        arTags[0].id = tagPair.first.id;
        left_tag_buffer = 0;
      }
      
      //second tag
      if(tagPair.second.id == -1){//no tag found
        if(right_tag_buffer <= 20){//send the buffered tag
          ++right_tag_buffer;
        } else {//we probably actually lost the tag
          arTags[1].distance = -1;
          arTags[1].bearing = -1;
          arTags[1].id = -1;
        }
      } else { //one tag found
        if(!isnan(depth_img.at<float>(tagPair.second.loc.y, tagPair.second.loc.x)))
          arTags[1].distance = depth_img.at<float>(tagPair.second.loc.y, tagPair.second.loc.x);
        arTags[1].bearing = getAngle((int)tagPair.second.loc.x, src.cols);
        arTags[1].id = tagPair.second.id;
        right_tag_buffer = 0;
      }


    #endif



    /*initialize obstacle detection*/
    obstacleMessage.detected = false;
    #if OBSTACLE_DETECTION
      float pixelWidth = src.cols;
      int roverPixWidth = calcRoverPix(distThreshold, pixelWidth);
      
      /* obstacle detection */
      obstacle_return obstacle_detection =  avoid_obstacle_sliding_window(depth_img, src,  num_sliding_windows , roverPixWidth);
      #if OBS_RECORD
      vidWriteObs.write(src);
      #endif

      if(obstacle_detection.bearing > 0.05 || obstacle_detection.bearing < -0.05) {
        // cout<< "bearing not zero!\n";
        obstacleMessage.detected = true;    //if an obstacle is detected in front
        obstacleMessage.distance = obstacle_detection.center_distance; //update LCM distance field
      }
      obstacleMessage.bearing = obstacle_detection.bearing;

    #endif

    lcm_.publish("/target_list", &arTagsMessage);
    lcm_.publish("/obstacle", &obstacleMessage);

    #if PERCEPTION_DEBUG
      imshow("depth", depth_img);
      imshow("Obstacle", src);
      updateThresholds(thresh1,thresh2);
      waitKey(FRAME_WAITKEY);
    #endif

    j++;
  }
  #if AR_RECORD
  vidWrite.release();
  #endif
  #if OBS_RECORD
  vidWriteObs.release();
  #endif
  return 0;
}
