#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <unistd.h>
#include <thread>

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

//Creates a PCL Visualizer
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer")); //This is a smart pointer so no need to worry ab deleteing it
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0,0,-800,0,-1,0);
    return (viewer);
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

  /* --- Dynamically Allocate Point Cloud --- */
  sl::Resolution cloud_res = sl::Resolution(PT_CLOUD_WIDTH, PT_CLOUD_HEIGHT);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //This is a smart pointer so no need to worry ab deleteing it

  #if PERCEPTION_DEBUG
    //Create PCL Visualizer
    shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(point_cloud_ptr); //This is a smart pointer so no need to worry ab deleteing it
    shared_ptr<pcl::visualization::PCLVisualizer> viewer_original = createRGBVisualizer(point_cloud_ptr);
  #endif
  
  while (true) {
    if (!cam_grab_succeed(cam, counter_fail)) break;

    //Grab initial images from cameras
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

    /* -- Run PCL Stuff --- */
    #if OBSTACLE_DETECTION

    //Update Point Cloud
    point_cloud_ptr->clear();
    point_cloud_ptr->points.resize(cloud_res.area());
    point_cloud_ptr->width = PT_CLOUD_WIDTH;
    point_cloud_ptr->height = PT_CLOUD_HEIGHT;
    

    cam.getDataCloud(point_cloud_ptr);
    /*
    string name = ("pcl" + to_string(j) + ".pcd");
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name, *point_cloud_ptr) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  
    #if OBSTACLE_RECORD
    string name = ("pcl" + to_string(j) + ".pcd");
    cerr<<name<<"\n";
    pcl::io::savePCDFileASCII (name, *point_cloud_ptr);
    std::cerr << "Saved " << point_cloud_ptr->size () << " data points to test_pcd.pcd." << std::endl;
    #endif
*/
    #if PERCEPTION_DEBUG
    //Update Original 3D Viewer
    viewer_original->updatePointCloud(point_cloud_ptr);
    viewer_original->spinOnce(10);
    cerr<<"Original W: " <<point_cloud_ptr->width<<" Original H: "<<point_cloud_ptr->height<<endl;
    #endif

    //Run Obstacle Detection
    obstacle_return obstacle_detection = pcl_obstacle_detection(point_cloud_ptr, viewer);  
    if(obstacle_detection.bearing > 0.05 || obstacle_detection.bearing < -0.05) {
        obstacleMessage.detected = true;    //if an obstacle is detected in front
        obstacleMessage.distance = obstacle_detection.distance; //update LCM distance field
      }
    obstacleMessage.bearing = obstacle_detection.bearing;

    #if PERCEPTION_DEBUG
    //Update Processed 3D Viewer
    viewer->updatePointCloud(point_cloud_ptr);
    viewer->spinOnce(20);
    cerr<<"Downsampled W: " <<point_cloud_ptr->width<<" Downsampled H: "<<point_cloud_ptr->height<<endl;
    #endif
    
    #endif
    
    /* --- Publish LCMs --- */
    lcm_.publish("/target_list", &arTagsMessage);
    lcm_.publish("/obstacle", &obstacleMessage);

    #if PERCEPTION_DEBUG
      imshow("depth", depth_img);
      waitKey(FRAME_WAITKEY);
    #endif

    j++;
  }
  #if PERCEPTION_DEBUG
  viewer->close();
  #endif
  #if AR_RECORD
  vidWrite.release();
  #endif
  return 0;
}

