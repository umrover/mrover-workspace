#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/TargetPosition.hpp"
#include "rover_msgs/TargetPositionList.hpp"
//#include "rover_msgs/TargetList.hpp"
//#include "rover_msgs/TargetOrientation.hpp"
#include <unistd.h>
#include "perception.hpp"

using namespace std;

int main() {
    cout<<"Hello world"<<endl;

  /* --- Camera Initializations --- */
  Camera cam;
  int iterations = 0;
  cam.grab();

  #if PERCEPTION_DEBUG
    namedWindow("depth", 2);
  #endif
  
  #if AR_DETECTION
  Mat rgb;
  Mat src = cam.image();
  #endif

  /*#if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
    cam.disk_record_init();
  #endif */

  /* -- LCM Messages Initializations -- */
  lcm::LCM lcm_;
  rover_msgs::TargetPositionList arTagsMessage;
  rover_msgs::TargetPosition* arTags = arTagsMessage.target_list;
  arTags[0].z = -1;
  arTags[1].z = -1;

  /* --- AR Tag Initializations --- */
  TagDetector detector;
  pair<Tag, Tag> tagPair;
  
   /* --- AR Recording Initializations and Implementation--- */ 
  
  time_t now = time(0);
  char* ltm = ctime(&now);
  string timeStamp(ltm);

  #if AR_RECORD
  //initializing ar tag videostream object
  cam.record_ar_init();
  #endif

/* --- Main Processing Stuff --- */
  while (true) {
    //Check to see if we were able to grab the frame
    if (!cam.grab()) break;

    #if AR_DETECTION
    //Grab initial images from cameras
    Mat rgb;
    Mat src = cam.image();
    Mat depth_img = cam.depth();
    #endif

   /* #if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
      if (iterations % FRAME_WRITE_INTERVAL == 0) {
        Mat rgb_copy = src.clone(), depth_copy = depth_img.clone();
        cerr << "Copied correctly" << endl;
        cam.write_curr_frame_to_disk(rgb_copy, depth_copy, pointcloud.pt_cloud_ptr, iterations);
      }
    #endif*/

/* --- AR Tag Processing --- */
    arTags[0].z = -1;
    arTags[1].z = -1;
    #if AR_DETECTION
    cerr<<"Finding AR Tags"<<endl;
      tagPair = detector.findARTags(src, depth_img, rgb);
      #if AR_RECORD
        cam.record_ar(rgb);
      #endif

      detector.updateDetectedTagInfo(arTags, tagPair, depth_img, src, rgb);
      //cerr<<arTags[0].x<<" "<<arTags[0].y<<" "<<arTags[0].z<<" "<<arTags[0].target_id<<endl;

    #if PERCEPTION_DEBUG && AR_DETECTION
      imshow("depth", src);
      waitKey(1);  
    #endif

    #endif

/* --- Publish LCMs --- */
  lcm_.publish("/target_position_list", &arTagsMessage);

    #if !ZED_SDK_PRESENT
    std::this_thread::sleep_for(0.2s); // Iteration speed control not needed when using camera 
    #endif
    cerr<<"LCM sent"<<endl;
  }
  cerr<<"process completed"<<endl;
/* --- Wrap Things Up --- */
  #if AR_RECORD
    cam.record_ar_finish();
  #endif

 return 0;

}