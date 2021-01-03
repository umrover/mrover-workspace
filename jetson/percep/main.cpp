#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <unistd.h>
#include <deque>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

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

  #if PERCEPTION_DEBUG
    /* --- Create PCL Visualizer --- */
    shared_ptr<pcl::visualization::PCLVisualizer> viewer = pointcloud.createRGBVisualizer(); //This is a smart pointer so no need to worry ab deleteing it
    shared_ptr<pcl::visualization::PCLVisualizer> viewer_original = pointcloud.createRGBVisualizer();
  #endif

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

    #if AR_DETECTION
    //Grab initial images from cameras
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
        cerr << "Copied correctly" << endl;
        cam.write_curr_frame_to_disk(rgb_copy, depth_copy, pointcloud.pt_cloud_ptr, iterations);
      }
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

/* --- Point Cloud Processing --- */
    #if OBSTACLE_DETECTION && !WRITE_CURR_FRAME_TO_DISK
    
    #if PERCEPTION_DEBUG
    //Update Original 3D Viewer
    viewer_original->updatePointCloud(pointcloud.pt_cloud_ptr);
    viewer_original->spinOnce(10);
    cerr<<"Original W: " <<pointcloud.pt_cloud_ptr->width<<" Original H: "<<pointcloud.pt_cloud_ptr->height<<endl;
    #endif

    //Run Obstacle Detection
    pointcloud.pcl_obstacle_detection(viewer);  
    obstacle_return obstacle_detection (pointcloud.bearing, pointcloud.distance);

    //Outlier Detection Processing
    outliers.pop_back(); //Remove outdated outlier value

    if(pointcloud.bearing > 0.05 || pointcloud.bearing < -0.05)
        outliers.push_front(true);//if an obstacle is detected in front
    else 
        outliers.push_front(false); //obstacle is not detected

    if(outliers == checkTrue) //If past iterations see obstacles
      lastObstacle = obstacle_detection;
    else if (outliers == checkFalse) // If our iterations see no obstacles after seeing obstacles
      lastObstacle = obstacle_detection;

     //Update LCM 
    obstacleMessage.bearing = lastObstacle.bearing; //update LCM bearing field
    if(lastObstacle.distance <= obstacle_detection.distance)
      obstacleMessage.distance = (lastObstacle.distance/1000); //update LCM distance field
    else
      obstacleMessage.distance = (obstacle_detection.distance/1000); //update LCM distance field
    cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Path Sent: " << obstacleMessage.bearing << "\n";

    #if PERCEPTION_DEBUG
      //Update Processed 3D Viewer
      viewer->updatePointCloud(pointcloud.pt_cloud_ptr);
      viewer->spinOnce(20);
      cerr<<"Downsampled W: " <<pointcloud.pt_cloud_ptr->width<<" Downsampled H: "<<pointcloud.pt_cloud_ptr->height<<endl;
    #endif
    
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
    viewer->close();
  #endif
  
  #if AR_RECORD
    cam.record_ar_finish();
  #endif
  
  return 0;
}

