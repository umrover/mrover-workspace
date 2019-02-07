#include "perception.hpp"
#include <unistd.h>

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
    //cerr << "grab failed once\n"; 
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
  if (WRITE_CURR_FRAME_TO_DISK) {
    // write images colleted to the folder
    // absolute path
    rgb_foldername = DEFAULT_ONLINE_DATA_FOLDER + "rgb/"; 
    depth_foldername = DEFAULT_ONLINE_DATA_FOLDER + "depth/" ;
    string mkdir_rgb =  std::string("mkdir -p ") + rgb_foldername;
    string mkdir_depth =  std::string("mkdir -p ") + depth_foldername;
    int dir_err_rgb = system( mkdir_rgb.c_str() );
    int dir_err_depth = system(mkdir_depth.c_str());
    if (-1 == dir_err_rgb || -1 == dir_err_depth) {
      exit(1);
    }
  }

}

void write_curr_frame_to_disk(Mat &rgb, Mat & depth, int counter ) {
    cv::imwrite(rgb_foldername +  std::to_string(counter) + std::string(".jpg"), rgb );
    //std::string file_str = std::string("depth_") + std::to_string(counter);// + std::string(".jpg");
    
    cv::imwrite(depth_foldername +  std::to_string(counter) + std::string(".exr"), depth );
}

int main() {

  /*initialize camera*/
  Camera cam;
  int j = 0;
  double frame_time = 0;
  int counter_fail = 0;
  #ifdef PERCEPTION_DEBUG
    namedWindow("image",1);
    namedWindow("depth",2);
  #endif
  disk_record_init();

  /*initialize lcm messages*/
  lcm::LCM lcm_;
  rover_msgs::TennisBall tennisMessage;
  rover_msgs::Obstacle obstacleMessage;
  tennisMessage.found = false;
  obstacleMessage.detected = false;

  int tennisBuffer = 0;

  
  while (true) {
    if (!cam_grab_succeed(cam, counter_fail)) break;
  
    auto start = chrono::high_resolution_clock::now();
    Mat src = cam.image();
    
    #ifdef PERCEPTION_DEBUG
          imshow("image", src);
    #endif
          Mat depth_img = cam.depth();

    // write to disk if permitted
    if(WRITE_CURR_FRAME_TO_DISK){
      write_curr_frame_to_disk(src, depth_img, j );
    }

    /*initialize obstacle detection*/
    float pixelWidth = src.cols;
    //float pixelHeight = src.rows;
    int roverPixWidth = calcRoverPix(distThreshold, pixelWidth);

    /* obstacle detection */
    obstacle_return obstacle_detection =  avoid_obstacle_sliding_window(depth_img, src,  num_sliding_windows , roverPixWidth);
    if(obstacle_detection.bearing > 0.05 || obstacle_detection.bearing < -0.05) {
      cout<< "bearing not zero!\n";
      obstacleMessage.detected = true;    //if an obstacle is detected in front
    } else {
      cout<<"bearing zero\n";
      obstacleMessage.detected = false;
    }
    obstacleMessage.bearing = obstacle_detection.bearing;

    #ifdef PERCEPTION_DEBUG
    cout << "Turn " << obstacleMessage.bearing << ", detected " << (bool)obstacleMessage.detected<< endl;
    #endif

    /* Tennis ball detection*/
    vector<Point2f> centers = findTennisBall(src, depth_img);
    if(centers.size() != 0){
      float dist = depth_img.at<float>(centers[0].y, centers[0].x);
      if (dist < BALL_DETECTION_MAX_DIST) {
        tennisMessage.distance = dist;
        tennisMessage.bearing = getAngle((int)centers[0].x, src.cols);

        tennisMessage.found = true;
        tennisBuffer = 0;

        #ifdef PERCEPTION_DEBUG
        cout << centers.size() << " tennis ball(s) detected: " << tennisMessage.distance 
                                                        << "m, " << tennisMessage.bearing << "degrees\n";
        #endif

      }else if(tennisBuffer < 5){   //give 5 frames to recover if tennisball lost due to noise
        tennisBuffer++;
      }else
        tennisMessage.found = false;
    }

    lcm_.publish("/tennis_ball", &tennisMessage);
    lcm_.publish("/obstacle", &obstacleMessage);

    #ifdef PERCEPTION_DEBUG
      imshow("depth", depth_img);
      imshow("image", src);
      waitKey(FRAME_WAITKEY);
    #endif
    auto end = chrono::high_resolution_clock::now();

    auto delta = chrono::duration_cast<chrono::duration<double>>(end - start);
    frame_time += delta.count();
    #ifdef PERCEPTION_DEBUG
        if(j % 100 == 0){
            cout << "framerate: " << 1.0f/(frame_time/j) << endl;
        }
    #endif
    j++;
  }

  return 0;
}
