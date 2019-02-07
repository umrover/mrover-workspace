#ifndef automated_test
#define automated_test
#include <cstdlib>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <boost/lexical_cast.hpp>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
//#include "tennis_ball_detection.h"


using namespace cv;
using namespace std;

struct TestCase {
    unsigned x, y, num;
    double depth;
    bool hasTennisBall;
    Mat img;
};

Mat greenFilter(const Mat& src){
    
    Mat greenOnly;
    Scalar lowerb = Scalar(36, 170, 80);
    Scalar upperb = Scalar(43, 226, 196);
    inRange(src, lowerb, upperb, greenOnly);
    return greenOnly;
    
}

vector<Point2f> findTennisBall(Mat &src) { //}, Mat & depth_src) {
    
    Mat hsv;
    
    cvtColor(src, hsv, COLOR_BGR2HSV);
    
    Mat mask = greenFilter(hsv);
    
    // smoothing
    // medianBlur(mask, mask, 11);
    Size ksize(5,5);
    GaussianBlur(mask, mask, ksize, 1, 1, BORDER_DEFAULT );
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    /// Find contours
    findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    
    vector<vector<Point> > contours_poly( contours.size());
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );
    
    for( unsigned i = 0; i < contours.size(); i++ ){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        Scalar color = Scalar(0, 0, 255);
        drawContours(src, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
        circle(src, center[i], (int)radius[i], color, 2, 8, 0);
    }
    
    imshow("Image", mask);
    waitKey(0);
    
    return center;
}

vector<TestCase> parse_file(vector<string> & fileList) {
    
    vector<TestCase> testCases;
    for (unsigned i = 0; i < fileList.size(); ++i) {
        size_t position = 0;
        
        if (fileList[i][0] != '.') {
          //  cout << "File name: " << fileList[i] << "\n";
            TestCase t;
            string name("test/" + fileList[i]);
            t.img = imread(name, CV_LOAD_IMAGE_COLOR);
            
            int count = 0;
            
            while ((position = fileList[i].find("_")) != string::npos) {
                switch (count) {
                    case 0:
                     //   t.hasTennisBall = boost::lexical_cast<bool>(fileList[i].substr(0, position));
                        t.num = (fileList[i].substr(0, position));
                        break;
                    case 1:
                      //  cout << "x:" << fileList[i].substr(0, position) << "\n";
                        t.x = (unsigned)stoll(fileList[i].substr(0, position));
                        break;
                    case 2:
                     //   cout << "y:" << fileList[i].substr(0, position) << "\n";
                        t.y = (unsigned)stoll(fileList[i].substr(0, position));
                        break;
                   // case 3:
                     //   t.depth = stod(fileList[i].substr(0, position));
                       // break;
                    default:
                        break;
                }
                fileList[i].erase(0, position + 1);
                ++count;
            }
		if (fileList[i] == "none")
		{
			t.hasTennisBall = false;
			t.x = 0;
			t.y = 0;
			t.depth = 0;
			
		} else {
			t.hasTennisBall = true;
			position = fileList[i].find(".");
			t.depth = stod(fileList[i].substr(0, position);
		}
            testCases.push_back(t);
        }
    }
    
    return testCases;
}

int main(int argc, char * argv[]) {
    
    DIR           *d;
    struct dirent *dir;
    vector<string> fileList;
    if (argc != 2) {
        cerr << "\nTesting Error: Must Provide a Directory\n" << endl;
        exit(0);
    }
    string directory_name = argv[1];
    
    int i = 0;
    
    d = opendir(directory_name.c_str());
    
    if (d) {

        while ((dir = readdir(d)) != NULL) {
            ++i;
            fileList.push_back(dir->d_name);
        }
        
        vector<TestCase> testCases = parse_file(fileList);
        vector<string> successRates;

        for (unsigned i = 0; i < testCases.size(); i++) {
            vector<Point2f> ball = findTennisBall(testCases[i].img);
	    string result = "test number: " + testCases[i].num + "-- ";
	    if (ball.size() == 0)
	    {
		result += "predicted: no tennis ball, real: "
		if (testCases[i].hasTennisBall == true)
		{
			result += "ball found";
		} else
		{
			result += "no ball found";
		}
	    } else 
		{	
	    string result = "predicted: x =  " + ball[0] + ", y =  " + ball[1] + ", real: x = " + testCases[i].x + ", y = " + testCases[i].y;
	        }
  		 successRates.push_back(result);
           // imshow("Test " + to_string(i + 1), testCases[i].img);
           // waitKey(0);
        }
     
        closedir(d);
        
        for (unsigned i = 0; i < successRates.size(); ++i) {
            cout << successRates[i] << endl;
        }
        
        // if more than one tennis ball detected, return false
        // improve validation
        // create file names
    
    }
    else {
        cerr << "\nTesting Error: Invalid Directory\n" << endl;
        exit(0);
    }
    
    
    
    return 0;
}
#endif
