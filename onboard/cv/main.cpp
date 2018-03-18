#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera.hpp"
#include <chrono>

using namespace cv;
using namespace std;

Mat greenFilter(const Mat& src){
    assert(src.type() == CV_8UC3);

    Mat greenOnly;
    Scalar lowerb = Scalar(26, 50, 150);
    Scalar upperb = Scalar(60, 210, 255);
    inRange(src, lowerb, upperb, greenOnly);

    return greenOnly;
}

struct obstacle_return {
  float center_distance; // distance to the center of the camera                                                                  
  float bearing; // [-50 degree, 50 degree]                                                                                       
};

obstacle_return avoid_obstacle_sliding_window(Mat &depth_img , int num_windows);


int  WIDTH_ROVER = 500;
int THRESHOLD_NO_WAY = 500000;
int center_point_height = 360;

obstacle_return avoid_obstacle_sliding_window(Mat &depth_img, int num_windows ) {

  // filter out nan values                                                                                                        
  depth_img = max(depth_img, 0.7);
  depth_img = min(depth_img, 20.0);

  Size size = depth_img.size();
  int type = depth_img.type();
  Mat diff_img = Mat::zeros(size, type);
  int height = size.height;
  int width = size.width;


  float center_point_depth = (float) depth_img.at<float>(  center_point_height, 640);

  Mat mean_row_vec = Mat::zeros(1, width, type);
  reduce(depth_img, mean_row_vec, 0, CV_REDUCE_SUM);

  // 720p, sw stands for "sliding window"                                                                                         
  int step_size = (width-WIDTH_ROVER)/(num_windows-1);
  //int num_sw =(int)( width *2 / WIDTH_ROVER ) -1;                                                                               
  float max_sum_sw = FLT_MIN ;
  int final_start_col = -1;
  float left_sum =0, right_sum = 0;
  for (int i = 0; i!= num_windows; i++) {
    int curr_col = i * step_size;  //i *  (int)WIDTH_ROVER/2;                                                                     
    const Mat sub_col =  mean_row_vec.colRange(curr_col, curr_col+WIDTH_ROVER-1 );
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

  if (max_sum_sw > THRESHOLD_NO_WAY) {
    cout<<"final_start_col: "<<final_start_col<<endl;
    rectangle(depth_img, Point( final_start_col, 0), Point( final_start_col+WIDTH_ROVER, 720), Scalar(0, 0, 255),\
3);
    float direction_center_diff =  ((float)(final_start_col + WIDTH_ROVER / 2) - (float)width/2 ) ;
    rt_val.bearing = direction_center_diff / (float)(width/2) * (50.0);

  } else {
    cout<<"Big obstacle in the front. Need to escape from one side!\n";
    rt_val.bearing =  (left_sum > right_sum)? (-45.0): (45.0);
  }
  return rt_val;

}


bool findTennisBall(Mat &src){
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);

    Mat mask = greenFilter(hsv);

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
    if(contours.size() != 0){
        return true;
    }return false;
}


int main() {
    Camera cam;
    int j = 0;
    double frame_time = 0;
    while (true) {
        if (!cam.grab()) {
            cerr << "camera failed.\n";
            break;
        }

        auto start = chrono::high_resolution_clock::now();
        Mat src = cam.image();

	// assume rover width to be 500 . TO TEST
	// TODO: write to LCM
	obstacle_return obstacle_detection =  avoid_obstacle_sliding_window(src, 10 );

        if(findTennisBall(src)){
            cout<<"tennis ball detected.\n";
        }else{
            cout << "tennis ball not detected.\n";
        }
        
        imshow("camera", src);
        auto end = chrono::high_resolution_clock::now();

        auto delta = chrono::duration_cast<chrono::duration<double>>(end - start);
        frame_time += delta.count();
        if(j % 100 == 0){
            cout << "framerate: " << 1.0f/(frame_time/j) << endl;
        }   
        j++;
        waitKey(5);
    }

    return 0;
}
