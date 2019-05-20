#include "perception.hpp"
using namespace cv;
using namespace std;

// record the last direction
static int last_center;
Rect cropped = Rect( 20, SKY_START_ROW, 1240, 300 ); // (x, y, width, height) 

bool check_divided_window(Mat & rgb_img, int num_splits, Mat & mean_row_vec, int start_col, int end_col){
  int split_size = (end_col - start_col)/num_splits;
  for(int i = 0; i < num_splits; i++){  //check each sub window
    Mat sub_col = mean_row_vec.colRange(start_col, start_col + split_size);
    float window_sum = sum(sub_col)[0];
    #if PERCEPTION_DEBUG
      //cout << "Sub[" <<i << "] sum = " << window_sum <<endl;
    #endif
    if(window_sum < THRESHOLD_NO_SUBWINDOW){
      #if PERCEPTION_DEBUG
        putText(rgb_img, "Obstacle Detected", Point( start_col, SKY_START_ROW), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(50, 50, 255), 2);
        rectangle(rgb_img, Point( start_col, SKY_START_ROW), Point( start_col+split_size, RESOLUTION_HEIGHT), Scalar(50, 50, 255), 3);
      #endif
        return false;
    }
    start_col+=split_size;  //update the start col
  }
  return true; //all good
}

// Goal: if ahead is safe zone, keep going straight
// try to go straigh as much as possible
// mean_row_vec: obtained from avoid_obstacle_sliding_window, which is computed from depth_img
obstacle_return scan_middle(Mat & rgb_img, float center_point_depth,  int rover_width, Mat & mean_row_vec, float & middle_sum) {

  obstacle_return noTurn;
  noTurn.center_distance = center_point_depth;
  noTurn.bearing = -1;

  // center col
  Size img_shape = rgb_img.size();
  int center_start_col = (img_shape.width - rover_width )/2;
  Mat sub_col =  mean_row_vec.colRange(center_start_col, center_start_col+rover_width-1 );
  middle_sum = sum( sub_col )[0];

  if(middle_sum > THRESHOLD_NO_OBSTACLE_CENTER){
    if(check_divided_window(rgb_img, 4, mean_row_vec, center_start_col, center_start_col+rover_width-1)){
      #if PERCEPTION_DEBUG
      putText(rgb_img, "Path Clear", Point( center_start_col+5, SKY_START_ROW+50), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
      rectangle(rgb_img, Point( center_start_col, SKY_START_ROW), Point( center_start_col+rover_width-1, RESOLUTION_HEIGHT), Scalar(0, 255, 0), 3);
      //cout<<"No turn: center window sub_col sum is "<<middle_sum<<endl;
      #endif
      noTurn.bearing = 0;
    }
  }else{
    putText(rgb_img, "Center Path Obstructed", Point( center_start_col+5, SKY_START_ROW+50), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    rectangle(rgb_img, Point( center_start_col, SKY_START_ROW), Point( center_start_col+rover_width-1, RESOLUTION_HEIGHT), Scalar(0, 0, 255), 3);
  }


  return noTurn;
}


bool compare_second(pair<int, float> p1, pair<int, float> p2)
{
    return (p1.second > p2.second);
}

pair<int, float> get_final_col(vector<pair<int, float> > & sorted_sums, float middle_sum ) {
  float max_sum_threshold = sorted_sums[0].second - SIMILARITY_THRESHOLD;
  // go straight if possible
  #if PERCEPTION_DEBUG
    //cout<<"middle col sum is "<<middle_sum<<endl;
  #endif

  if (max_sum_threshold < middle_sum) {
    return make_pair(-1, middle_sum);
  }

  vector<pair<int, float> >::iterator final = lower_bound( sorted_sums.begin(), sorted_sums.end(), make_pair(0, max_sum_threshold), compare_second );
  #if PERCEPTION_DEBUG
    for (vector<pair<int, float> >::iterator it=sorted_sums.begin(); it!=final; it++) //cout<<"("<<it->first<<", "<<it->second<<")";
    //cout<<endl;
  #endif

  if (final == sorted_sums.end()) {
    // go straight if each direction has similar value
    return make_pair(-1, middle_sum);

  } else {
    // otherwise, return the one that is cloesest with last time
    int smallest_diff = abs(last_center - (sorted_sums[0].first));
    pair<int, float> smallest_dist_pair = sorted_sums[0];
    for (vector<pair<int, float> >::iterator it=sorted_sums.begin(); it!= final; it++ ) {
      int curr_diff = abs(it->first - last_center);
      if ( curr_diff < smallest_diff ) {
        smallest_diff = curr_diff;
        smallest_dist_pair = *it;
      }
    }
    return smallest_dist_pair;
  }

}

// check whether there exists a big obstacle in the front that blocks all directons
obstacle_return refine_rt(obstacle_return rt_val, pair<int, float> candidate, Size size, int rover_width, Mat & rgb_img, float left_sum, float right_sum) {

  float max_sum_sw = candidate.second;
  int final_start_col = candidate.first;

  if (max_sum_sw > THRESHOLD_NO_WAY) {
    #if PERCEPTION_DEBUG
      //cout<<"max_sum_sw "<<max_sum_sw<<", col start at "<<final_start_col<<endl;
      putText(rgb_img, "New Clear Path", Point( final_start_col, SKY_START_ROW-60), CV_FONT_HERSHEY_SIMPLEX, 1, cvScalar(255, 0, 0), 2);
      rectangle(rgb_img, Point( final_start_col, SKY_START_ROW), Point( final_start_col+rover_width, RESOLUTION_HEIGHT), Scalar(255, 0, 0), 3);
    #endif

    // compute bearing
    if (size.width /2 > final_start_col  + (rover_width * 2 / 5) &&
          size.width /2 < final_start_col  + (rover_width* 3 / 5 )) {
      last_center = RESOLUTION_WIDTH / 2;
      rt_val.bearing = 0;
    } else {
      float direction_center_diff =  ((float)(final_start_col + rover_width / 2) - (float)size.width/2 ) ;
      rt_val.bearing = direction_center_diff / (float)(size.width/2) * (50.0);
      last_center = final_start_col + rover_width /2;
    }
  } else {
    #if PERCEPTION_DEBUG
      //cout<<"Big obstacle in the front. Need to escape from one side!\n";
    #endif
    last_center = (left_sum>right_sum)? 0:RESOLUTION_WIDTH;
    rt_val.bearing =  (left_sum > right_sum)? (-45.0): (45.0);
  }

  return rt_val;
}

obstacle_return avoid_obstacle_sliding_window(Mat &depth_img_src, Mat &rgb_img, int num_windows, int rover_width ) {
  // filter out nan values. 0.7 and 20 are ZED stero's limits
  Mat depth_img = depth_img_src.clone();
  patchNaNs(depth_img, 0.0);
  depth_img = max(depth_img, 0.7);
  depth_img = min(depth_img, 20.0);
  depth_img = depth_img(cropped);
  //depth_img = depth_img(Rect( 0, 450,  1280, 250));

  blur( depth_img, depth_img, Size( 7, 7 ), Point(-1,-1) );
  Size size = depth_img.size();
  float center_point_depth = (float) depth_img.at<float>(  size.height/2, size.width/2);

  Mat mean_row_vec = Mat::zeros(1, size.width, CV_32F);
  reduce(depth_img, mean_row_vec, 0, CV_REDUCE_SUM, CV_32F);

  #if PERCEPTION_DEBUG
    //cout<<"last center "<<last_center<<endl;
  #endif

  // check middel col first. If there is no close obstacle in the middle, go straight
  float middle_sum = 0;
  obstacle_return rt_val = scan_middle(rgb_img, center_point_depth, rover_width, mean_row_vec, middle_sum);
  rt_val.center_distance = center_point_depth;
  if (rt_val.bearing == 0) {
    last_center = RESOLUTION_WIDTH / 2;
    return rt_val;
  }

  // line search for the col with max distance
  Mat sub_col;
  int step_size = (size.width-rover_width)/(num_windows-1);
  float left_sum =0, right_sum = 0;
  vector<pair<int,float> > sums(num_windows);
  for (int i = 1; i < num_windows; i++) {
    int curr_col = i * step_size;
    sub_col =  mean_row_vec.colRange(curr_col,curr_col+rover_width-1 );
    float window_sum = sum( sub_col )[0];
    if (i == 1) left_sum = window_sum;
    if (i == num_windows - 1) right_sum = window_sum;
    #if PERCEPTION_DEBUG
      //cout<<"[col "<<curr_col<<"], window sub_col sum is "<<window_sum<<endl;
    #endif
    sums[i] = (make_pair(curr_col,window_sum));
  }
  sort(sums.begin(), sums.end(), compare_second );

  // try to reduce noise
  // 0 for middle
  pair<int, float> final_window = get_final_col(sums, middle_sum); //may add split window check
  if (final_window.first == -1) {
    #if PERCEPTION_DEBUG
      //cout<<"max_sum_sw "<<final_window.second<<" at center\n";
      putText(rgb_img, "No Clear Path", Point( size.width / 2 - rover_width/2, SKY_START_ROW-60), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
      rectangle(rgb_img, Point( size.width / 2 - rover_width/2, SKY_START_ROW), Point( size.width/2 + rover_width/2, RESOLUTION_HEIGHT), Scalar(0, 0, 255), 3);
    #endif

    last_center = RESOLUTION_WIDTH/2;
    rt_val.bearing = 0;
    return rt_val;
  }

  // check whether there exists a big obstacle that blocks even the chosen direction
  rt_val = refine_rt(rt_val, final_window, size, rover_width, rgb_img, left_sum, right_sum);
  return rt_val;

}
