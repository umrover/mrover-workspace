#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <unistd.h>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

double getAngle(float xPixel, float wPixel){
    return atan((xPixel - wPixel/2)/(wPixel/2)* tan(fieldofView/2))* 180.0 /PI;
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

int main() {
  /* --- Initialize Camera --- */
  Camera cam;
  cam.grab();
  int iterations = 0;
  int counter_fail = 0;
  #if PERCEPTION_DEBUG
    namedWindow("depth", 2);
  #endif
  cam.disk_record_init();
  //Video Stuff
  TagDetector d1;
  pair<Tag, Tag> tp;

  //Create time value
  time_t now = time(0);
  char* ltm = ctime(&now);
  string timeStamp(ltm);
  
  #if AR_DETECTION
  Mat rgb;
  Mat src = cam.image();
  #endif

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

  /* - Initialize LCM Messages - */
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

  #if OBSTACLE_DETECTION

  //Generate point cloud and put in point cloud object
  //This is a smart pointer so no need to worry ab deleteing it
  PCL pointcloud;
  
  #if PERCEPTION_DEBUG
    /* --- Create PCL Visualizer --- */
    shared_ptr<pcl::visualization::PCLVisualizer> viewer = pointcloud.createRGBVisualizer(); //This is a smart pointer so no need to worry ab deleteing it
    shared_ptr<pcl::visualization::PCLVisualizer> viewer_original = pointcloud.createRGBVisualizer();
  #endif

  #endif
  
  while (true) {
    if (!cam_grab_succeed(cam, counter_fail)) break;

    #if AR_DETECTION
    //Grab initial images from cameras
    Mat rgb;
    Mat src = cam.image();
    Mat depth_img = cam.depth();
    #endif
    

    // write to disk if permitted
    #if WRITE_CURR_FRAME_TO_DISK
      if (iterations % FRAME_WRITE_INTERVAL == 0) {
        Mat rgb_copy = src.clone(), depth_copy = depth_img.clone();
        cerr << "Copied correctly" << endl;
        cam.write_curr_frame_to_disk(rgb_copy, depth_copy, iterations);
      }
    #endif

    /* --- AR Tag Detection --- */
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

    /* --- Run PCL Stuff --- */
    #if OBSTACLE_DETECTION

    //Update Point Cloud
    pointcloud.update();
    cam.getDataCloud(pointcloud.pt_cloud_ptr);

    #if PERCEPTION_DEBUG
    //Update Original 3D Viewer
    viewer_original->updatePointCloud(pointcloud.pt_cloud_ptr);
    viewer_original->spinOnce(10);
    cerr<<"Original W: " <<pointcloud.pt_cloud_ptr->width<<" Original H: "<<pointcloud.pt_cloud_ptr->height<<endl;
    #endif

    //Run Obstacle Detection
    pointcloud.pcl_obstacle_detection(viewer);

    if(pointcloud.bearing > 0.05 || pointcloud.bearing < -0.05) {
        obstacleMessage.detected = true;    //if an obstacle is detected in front
        obstacleMessage.distance = pointcloud.distance; //update LCM distance field
    }
    else 
      obstacleMessage.detected = false;

    obstacleMessage.bearing = pointcloud.bearing;

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

    #if PERCEPTION_DEBUG && AR_DETECTION
      imshow("depth", src);
      waitKey(1);
      std::this_thread::sleep_for(0.2s);   
    #endif
    ++iterations;
  }
  #if OBSTACLE_DETECTION && PERCEPTION_DEBUG
  viewer->close();
  #endif
  
  #if AR_RECORD
  vidWrite.release();
  #endif
  return 0;
}

