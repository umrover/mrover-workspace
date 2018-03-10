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
