#include "perception.hpp"


using namespace std;
using namespace cv;

#ifdef PERCEPTION_DEBUG
struct mouse_data {
  Mat * hsv;
  Mat * depth;
};
static mouse_data m_data;
static Mat hsv;
/* For debug use: print the HSV values at mouseclick locations */
void onMouse(int event, int x, int y, int flags, void* param)
{
  if (event != EVENT_LBUTTONDOWN) return;
  
  //char text[100];
  mouse_data * m_d =  (mouse_data *) param;
  float d = m_d->depth->at<float>(y,x);
  Vec3b p = m_d->hsv->at<Vec3b>(y,x);
  
  printf("Get mouse click at (%d, %d), HSV value is H: %d, S: %d, V:%d, depth is %.2f meters \n", y, x,
	 p.val[0], p.val[1], p.val[2], d);

  //sprintf(text, "Depth=%.2f meters at (%d,%d)", p, y, x);

  //putText(img2, text, Point(10,20), FONT_HERSHEY_PLAIN, 2.0, CV_RGB(0,255,0));
}
#endif



Mat greenFilter(const Mat& src){
    assert(src.type() == CV_8UC3);

    Mat greenOnly;
    // 36 170 80
    Scalar lowerb = Scalar(36, 170, 80);
    Scalar upperb = Scalar(43, 226, 196);
    inRange(src, lowerb, upperb, greenOnly);

    return greenOnly;
}


vector<Point2f> findTennisBall(Mat &src, Mat & depth_src){
  
    #ifndef PERCEPTION_DEBUG
        Mat hsv;
    #endif

    cvtColor(src, hsv, COLOR_BGR2HSV);

    Mat mask = greenFilter(hsv);

    #ifdef PERCEPTION_DEBUG
    mouse_data * m_d = & m_data;
    m_d->hsv = &hsv;
    m_d->depth = &depth_src;
    imshow("hsv", hsv);
    setMouseCallback("image", onMouse, (void *)m_d);    
    imshow("mask", mask);
    #endif

    // smoothing
    //medianBlur(mask, mask, 11);
    Size ksize(5,5);
    GaussianBlur(mask, mask, ksize, 1, 1, BORDER_DEFAULT );

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

    
    #ifdef PERCEPTION_DEBUG
    /// Draw polygonal contour + bonding rects + circles
    //Mat drawing = Mat::zeros( mask.size(), CV_8UC3);
    for( unsigned i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar(0, 0, 255);
        drawContours( src, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        circle( src, center[i], (int)radius[i], color, 2, 8, 0 );
    }
    #endif

    return center;
}
