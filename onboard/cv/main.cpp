#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera.hpp"
#include "rover_msgs/TennisBall.hpp"
#include "rover_msgs/Obstacle.hpp"
#include <chrono>
#include <cmath>
#include <lcm/lcm-cpp.hpp>

using namespace cv;
using namespace std;

#define PI 3.14159265
const float fieldofView = 110 * PI/180;
const float inf = -std::numeric_limits<float>::infinity();

int THRESHOLD_NO_WAY = 300000;//500000; //how will we calibrate if the rover width changes
int center_point_height = 360;  //not sure
int THRESHOLD_NO_OBSTACLE_CENTER = 600000;

const float zedHeight = 17 * 0.0254; //inches to meters off the ground
const float realWidth = 46 * 25.4; //rover width , rover is 46 inches wide TODO make a little longer to be safe
const float angleOffset = 10 * PI/180;    //angle offset of the rover

const float focalLength = 2.8; //zed focal length in mm

const float distThreshold = 4;    //meters, used to calculate rover pixels
const float obstacleThreshold = 5 * 0.0254; //inches to meters

float minDepth = 1; //need to set
float pixelWidth = 1; //need to set
float pixelHeight = 1;
int roverPixWidth = 1; //initialize

struct obstacle_return {
  float center_distance; // distance to anything in front of the center of the camera                                                                 
  float bearing; // [-50 degree, 50 degree]                                                                                       
};

obstacle_return avoid_obstacle_sliding_window(Mat &depth_img, Mat &rgb_img, int num_windows ) {

  // filter out nan values                                                                                                        
  depth_img = max(depth_img, 0.7);
  depth_img = min(depth_img, 20.0);

  Size size = depth_img.size();
  int type = depth_img.type();
  Mat diff_img = Mat::zeros(size, type);
  //int height = size.height; not used currently
  int width = size.width;


  float center_point_depth = (float) depth_img.at<float>(  center_point_height, 640);

  Mat mean_row_vec = Mat::zeros(1, width, CV_32F);
  reduce(depth_img, mean_row_vec, 0, CV_REDUCE_SUM, CV_32F);

  // 720p, sw stands for "sliding window"                                                                                         
  int step_size = (width-roverPixWidth)/(num_windows-1);
  //int num_sw =(int)( width *2 / WIDTH_ROVER ) -1;                                                                               
  float max_sum_sw = FLT_MIN;
  int final_start_col = -1;
  float left_sum =0, right_sum = 0;

  //check middle first
  obstacle_return noTurn;
  noTurn.center_distance = center_point_depth;
  noTurn.bearing = 0;
  int curr_col = (width - roverPixWidth)/2;
  Point startScan(pixelWidth/2 - roverPixWidth/2, pixelHeight/2), endScan(pixelWidth/2 + roverPixWidth/2, pixelHeight/2);
  //line(rgb_img, startScan, endScan, Scalar(0, 0, 255), 2, 8, 0);
  const Mat sub_col =  mean_row_vec.colRange(curr_col, curr_col+roverPixWidth-1 );
  float window_sum = sum( sub_col )[0]; 
  if(window_sum > THRESHOLD_NO_OBSTACLE_CENTER){
    rectangle(rgb_img, Point( startScan.x, 0), Point( endScan.x, 720), Scalar(0, 0, 255), 3);
    cout<<"middle: window sub_col sum is "<<window_sum<<endl;
    return noTurn;
  }
  
  for (int i = 1; i < num_windows-1; i++) {
    int curr_col = i * step_size;  //i *  (int)WIDTH_ROVER/2;                                                                     
    const Mat sub_col =  mean_row_vec.colRange(curr_col, curr_col+roverPixWidth-1 );
    float window_sum = sum( sub_col )[0];
    if (i == 0) left_sum = window_sum;
    if (i == num_windows - 1) right_sum = window_sum;
    cout<<"[col "<<curr_col<<"], window sub_col sum is "<<window_sum<<endl;
    if (window_sum > max_sum_sw) {
      max_sum_sw = window_sum;
      final_start_col = curr_col;
    }
  }

  obstacle_return rt_val;
  rt_val.center_distance = center_point_depth;
  cout<<"max_sum_sw "<<max_sum_sw<<endl;
  if (max_sum_sw > THRESHOLD_NO_WAY) {
    cout<<"final_start_col: "<<final_start_col<<endl;
    rectangle(depth_img, Point( final_start_col, 0), Point( final_start_col+roverPixWidth, 720), Scalar(0, 0, 255),\
3);
    rectangle(rgb_img, Point( final_start_col, 0), Point( final_start_col+roverPixWidth, 720), Scalar(0, 0, 255), 3);
    float direction_center_diff =  ((float)(final_start_col + roverPixWidth / 2) - (float)width/2 ) ;
    rt_val.bearing = direction_center_diff / (float)(width/2) * (50.0);

  } else {
    cout<<"Big obstacle in the front. Need to escape from one side!\n";
    rt_val.bearing =  (left_sum > right_sum)? (-45.0): (45.0);
  }
  return rt_val;

}

int calcFocalWidth(){   //mm
    return tan(fieldofView/2) * focalLength;
}

int calcRoverPix(float dist){   //pix
    float roverWidthSensor = realWidth * focalLength/(dist * 1000);
    return roverWidthSensor*(pixelWidth/2)/calcFocalWidth();
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

bool scanMiddle(Mat &depth_img, Mat &src, float scanRow){ //scanRow is a fraction
    scanRow*=pixelHeight;
    Scalar color = Scalar(0, 0, 255);
    Point startScan(pixelWidth/2 - roverPixWidth/2, scanRow), endScan(pixelWidth/2 + roverPixWidth/2, scanRow);
    line(src, startScan, endScan, color, 2, 8, 0);
    for(int i = startScan.x; i < endScan.x; i++){   // loop through the pointcloud center
        float value = depth_img.at<float>(i, scanRow);
        if((value < minDepth) && value !=0 && value != inf){  // if a depth is too close or too far imediately return true
            return true;
        }
    }
    return false;
}


Mat greenFilter(const Mat& src){
    assert(src.type() == CV_8UC3);

    Mat greenOnly;
    Scalar lowerb = Scalar(71, 210, 90);
    Scalar upperb = Scalar(74, 255, 147);
    inRange(src, lowerb, upperb, greenOnly);

    return greenOnly;
}


vector<Point2f> findTennisBall(Mat &src){

    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);

    Mat mask = greenFilter(hsv);

    imshow("mask", mask);

    medianBlur(mask, mask, 11);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( unsigned i = 0; i < contours.size(); i++ ){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

  /// Draw polygonal contour + bonding rects + circles
        //Mat drawing = Mat::zeros( mask.size(), CV_8UC3);
    for( unsigned i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar(0, 0, 255);
        drawContours( src, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        circle( src, center[i], (int)radius[i], color, 2, 8, 0 );
    }

    //src = mask;
    return center;
}


int main() {
    /*initialize values*/

    /*initialize camera*/
    Camera cam;
    int j = 0;
    double frame_time = 0;
    int counter_fail = 0;
    while (true) {
        while (!cam.grab()) {
            cerr << "grab failed.\n";
            counter_fail++;
            if (counter_fail > 1000000) break;
        }

        if (counter_fail  ) {

            cerr<< "camera fails\n"; 
            break;
        }
        else counter_fail = 0;

        auto start = chrono::high_resolution_clock::now();
        Mat src = cam.image();
        imshow("orginal", src);
        Mat depth_img = cam.depth();

        /*initialize lcm messages*/
        lcm::LCM lcm_;
        rover_msgs::TennisBall tennisMessage;
        rover_msgs::Obstacle obstacleMessage;
        tennisMessage.found = false;
        obstacleMessage.detected = false;

        /*initialize obstacle detection*/
        pixelWidth = src.cols;
        pixelHeight = src.rows;
        roverPixWidth = calcRoverPix(distThreshold);
        float expectedDist = getGroundDist(angleOffset);
        minDepth = getObstacleMin(expectedDist);


        obstacle_return obstacle_detection =  avoid_obstacle_sliding_window(depth_img, src,  10 );
        if(obstacle_detection.bearing != 0) obstacleMessage.detected = true;    //if an obstacle is detected in front
        obstacleMessage.bearing = obstacle_detection.bearing;
        cout << "Turn " << obstacle_detection.bearing << endl;

        vector<Point2f> centers = findTennisBall(src);
        if(centers.size() != 0){
            tennisMessage.found = true;
            tennisMessage.distance = depth_img.at<float>(centers[0].x, centers[0].y);
            tennisMessage.bearing = getAngle((int)centers[0].x, pixelWidth);
            cout << centers.size() << " tennis ball(s) detected: " << tennisMessage.distance 
                                                        << "m, " << tennisMessage.bearing << "degrees\n";
        }else{
            //cout << "tennis ball not detected.\n";
        }
        lcm_.publish("/tennis_ball", &tennisMessage);
        lcm_.publish("/obstacle", &obstacleMessage);

    	imshow("depth", depth_img);
        imshow("camera", src);
        auto end = chrono::high_resolution_clock::now();

        auto delta = chrono::duration_cast<chrono::duration<double>>(end - start);
        frame_time += delta.count();
        if(j % 100 == 0){
            cout << "framerate: " << 1.0f/(frame_time/j) << endl;
        }   
        j++;
        waitKey(0);
    }

    //cam.deleteZed();
    return 0;
}
