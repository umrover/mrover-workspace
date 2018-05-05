#include "perception.hpp"
using namespace std;
using namespace cv;

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

    #ifdef PERCEPTION_DEBUG
    imshow("mask", mask);
    #endif
    
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
