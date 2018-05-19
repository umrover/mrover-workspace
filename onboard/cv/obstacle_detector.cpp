#include "perception.hpp"
using namespace cv;
using namespace std;


// try to go straigh as much as possible
// mean_row_vec: obtained from avoid_obstacle_sliding_window, which is computed from depth_img
obstacle_return scan_middle(Mat & rgb_img, float center_point_depth,  int rover_width, Mat & mean_row_vec ) {

  obstacle_return noTurn;
  noTurn.center_distance = center_point_depth;
  noTurn.bearing = -1;

  // center col
  Size img_shape = rgb_img.size();
  int center_start_col = (img_shape.width - rover_width )/2;
  Mat sub_col =  mean_row_vec.colRange(center_start_col, center_start_col+rover_width-1 );
  float window_sum = sum( sub_col )[0]; 
  if(window_sum > THRESHOLD_NO_OBSTACLE_CENTER){
    #ifdef PERCEPTION_DEBUG
    rectangle(rgb_img, Point( center_start_col, 0), Point( center_start_col+rover_width-1, RESOLUTION_HEIGHT), Scalar(0, 0, 255), 3);
    cout<<"No turn: center window sub_col sum is "<<window_sum<<endl;
    #endif
    noTurn.bearing = 0;
  }
  return noTurn;
}

obstacle_return avoid_obstacle_sliding_window(Mat &depth_img_src, Mat &rgb_img, int num_windows, int rover_width ) {
  // filter out nan values. 0.7 and 20 are ZED stero's limits
  Mat depth_img = depth_img_src.clone();
  depth_img = max(depth_img, 0.7);
  depth_img = min(depth_img, 20.0);
  depth_img = depth_img(Rect( 0, 300,  1280, 400));
  Size size = depth_img.size();
  float center_point_depth = (float) depth_img.at<float>(  size.height/2, size.width/2);

  Mat mean_row_vec = Mat::zeros(1, size.width, CV_32F);
  reduce(depth_img, mean_row_vec, 0, CV_REDUCE_SUM, CV_32F);

  int step_size = (size.width-rover_width)/(num_windows-1);
  float max_sum_sw = FLT_MIN;
  int final_start_col = -1;
  float left_sum =0, right_sum = 0;

  // check middel col first. If there is no close obstacle in the middle, go straight
  obstacle_return rt_val = scan_middle(rgb_img, center_point_depth, rover_width, mean_row_vec);
  rt_val.center_distance = center_point_depth;
  if (rt_val.bearing == 0) return rt_val;

  // linear search for the col with max distance
  Mat sub_col;
  for (int i = 1; i < num_windows; i++) {
    int curr_col = i * step_size;
    sub_col =  mean_row_vec.colRange(curr_col,curr_col+rover_width-1 );
    float window_sum = sum( sub_col )[0];
    if (i == 0) left_sum = window_sum;
    if (i == num_windows - 1) right_sum = window_sum;
    #ifdef PERCEPTION_DEBUG
    cout<<"[col "<<curr_col<<"], window sub_col sum is "<<window_sum<<endl;
    #endif
    if (window_sum > max_sum_sw) {
      max_sum_sw = window_sum;
      final_start_col = curr_col;
    }
  }

  if (max_sum_sw > THRESHOLD_NO_WAY) {
    #ifdef PERCEPTION_DEBUG
    cout<<"max_sum_sw "<<max_sum_sw<<", col start at "<<final_start_col<<endl;
    // rectangle(depth_img, Point( final_start_col, 0), Point( final_start_col+roverPixWidth, 720), Scalar(0, 0, 255), 3);
    rectangle(rgb_img, Point( final_start_col, 0), Point( final_start_col+rover_width, RESOLUTION_HEIGHT), Scalar(0, 0, 255), 3);
    #endif

    // compute bearing
    if (size.width /2 > final_start_col && size.width/2 < final_start_col + rover_width-1) {
      rt_val.bearing = 0;
    } else {
      float direction_center_diff =  ((float)(final_start_col + rover_width / 2) - (float)size.width/2 ) ;
      rt_val.bearing = direction_center_diff / (float)(size.width/2) * (50.0);
    }
  } else {
    #ifdef PERCEPTION_DEBUG
    cout<<"Big obstacle in the front. Need to escape from one side!\n";
    #endif
    rt_val.bearing =  (left_sum > right_sum)? (-45.0): (45.0);
  }
  return rt_val;

}
