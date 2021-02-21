#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <unistd.h>
#include <deque>
#include <thread>
#include <fstream>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

enum viewerType {
    newView, //set to 0 -or false- to be passed into updateViewer later
    originalView //set to 1 -or true- to be passed into updateViewer later
};
#if OBSTACLE_DETECTION
// PCL Thread function
void PCLProcessing(PCL &pointCloudIn, deque<bool> &outliersIn, obstacle_return &lastObstacleIn, 
                    rover_msgs::Obstacle &obstacleMessageIn, deque <bool> &checkTrueIn, deque <bool> &checkFalseIn) {
  
  #if OBSTACLE_DETECTION && !WRITE_CURR_FRAME_TO_DISK
    
  #if PERCEPTION_DEBUG
      //Update Original 3D Viewer
      pointCloudIn.updateViewer(originalView);
      cout<<"Original W: " <<pointCloudIn.pt_cloud_ptr->width<<" Original H: "<<pointCloudIn.pt_cloud_ptr->height<<endl;
  #endif

    //Run Obstacle Detection
    pointCloudIn.pcl_obstacle_detection();  
    obstacle_return obstacle_detection (pointCloudIn.bearing, pointCloudIn.distance);

    //Outlier Detection Processing
    outliersIn.pop_back(); //Remove outdated outlier value

    if(pointCloudIn.bearing > 0.05 || pointCloudIn.bearing < -0.05) //Check left bearing
        outliersIn.push_front(true);//if an obstacle is detected in front
    else 
        outliersIn.push_front(false); //obstacle is not detected

    if(outliersIn == checkTrueIn) //If past iterations see obstacles
      lastObstacleIn = obstacle_detection;
    else if (outliersIn == checkFalseIn) // If our iterations see no obstacles after seeing obstacles
      lastObstacleIn = obstacle_detection;

    obstacleMessageIn.distance = lastObstacleIn.distance; //update LCM distance field
    obstacleMessageIn.bearing = lastObstacleIn.bearing; //update LCM bearing field
    #if PERCEPTION_DEBUG
        cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Path Sent: " << obstacleMessageIn.bearing << "\n";
        cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Distance Sent: " << obstacleMessageIn.distance << "\n";
    #endif
  #if PERCEPTION_DEBUG
      //Update Processed 3D Viewer
      pointCloudIn.updateViewer(newView);
      cerr<<"Downsampled W: " <<pointCloudIn.pt_cloud_ptr->width<<" Downsampled H: "<<pointCloudIn.pt_cloud_ptr->height<<endl;
  #endif
    
  #endif
}
#endif
int main() {

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

  #if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
    cam.disk_record_init();
  #endif

  /* -- LCM Messages Initializations -- */
  lcm::LCM lcm_;
  rover_msgs::TargetList arTagsMessage;
  rover_msgs::Target* arTags = arTagsMessage.targetList;
  rover_msgs::Obstacle obstacleMessage;
  arTags[0].distance = -1;
  arTags[1].distance = -1;

  /* --- AR Tag Initializations --- */
  TagDetector detector;
  pair<Tag, Tag> tagPair;
  
  /* --- Point Cloud Initializations --- */
  #if OBSTACLE_DETECTION

  PCL pointcloud;
  
  /* --- Outlier Detection --- */
  int numChecks = 3;
  deque <bool> outliers;
  outliers.resize(numChecks, true); //initializes outliers vector
  deque <bool> checkTrue(numChecks, true); //true deque to check our outliers deque against
  deque <bool> checkFalse(numChecks, false); //false deque to check our outliers deque against
  obstacle_return lastObstacle;

  #endif

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

    #if PERCEPTION_DEBUG
        namedWindow("depth", 2);
    #endif
    
    #if AR_DETECTION
    Mat rgb;
    Mat src = cam.image();
    Mat depth_img = cam.depth();
    #endif

    #if OBSTACLE_DETECTION
        //Update Point Cloud
        pointcloud.update();
        cam.getDataCloud(pointcloud.pt_cloud_ptr);
    #endif

    #if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
            if (iterations % FRAME_WRITE_INTERVAL == 0) {
                Mat rgb_copy = src.clone(), depth_copy = depth_img.clone();
                #if PERCEPTION_DEBUG
                    cout << "Copied correctly" << endl;
                #endif
                cam.write_curr_frame_to_disk(rgb_copy, depth_copy, pointcloud.pt_cloud_ptr, iterations);
              }
    #endif

  #if OBSTACLE_DETECTION
  // Launch PCL thread
  thread PCLThread(PCLProcessing, ref(pointcloud), ref(outliers), ref(lastObstacle), ref(obstacleMessage), 
                    ref(checkTrue), ref(checkFalse));
  #endif
 
/* --- AR Tag Processing --- */
  arTags[0].distance = -1;
  arTags[1].distance = -1;
  #if AR_DETECTION
      tagPair = detector.findARTags(src, depth_img, rgb);
      #if AR_RECORD
          cam.record_ar(rgb);
      #endif

      detector.updateDetectedTagInfo(arTags, tagPair, depth_img, src);

  #if PERCEPTION_DEBUG && AR_DETECTION
      imshow("depth", src);
      waitKey(1);  
  #endif

  #endif

  #if OBSTACLE_DETECTION
  // Wait for PCL thread to finish
  PCLThread.join();
  #endif
  
  /* --- Publish LCMs --- */
    lcm_.publish("/target_list", &arTagsMessage);
    lcm_.publish("/obstacle", &obstacleMessage);

    #if !ZED_SDK_PRESENT
            std::this_thread::sleep_for(0.2s); // Iteration speed control not needed when using camera 
    #endif

    ++iterations;
  }

/* --- Wrap Things Up --- */
  #if OBSTACLE_DETECTION && PERCEPTION_DEBUG
    pointcloud.~PCL();
  #endif
  
  #if AR_RECORD
        cam.record_ar_finish();
  #endif
  
    return 0;
}

