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
    
    imshow("Image", src);
    waitKey(0);
    
    return center;
}

vector<TestCase> parse_file(vector<string> & fileList) {
    
    vector<TestCase> testCases;
    for (unsigned i = 2; i < fileList.size(); ++i) {
        size_t position = 0;
        
       // if (fileList[i][0] != '.') {
          //  cout << "File name: " << fileList[i] << "\n";
            TestCase t;
		string s = "stuff/" + fileList[i];
	//	cout << s << endl;
            string name(s);
            t.img = imread(name, CV_LOAD_IMAGE_COLOR);       
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

                        t.num = (unsigned)stoll(fileList[i].substr(0, position));
          //             cout << t.num << endl; //DELETE LATER
			 break;
                    case 1:
                      //  cout << "x:" << fileList[i].substr(0, position) << "\n";
                        t.x = (unsigned)stoll(fileList[i].substr(0, position));
            //           cout << t.x << endl; //DELETE LATER
			 break;
                    case 2:
                     //   cout << "y:" << fileList[i].substr(0, position) << "\n";
                        t.y = (unsigned)stoll(fileList[i].substr(0, position));
              //         cout << t.y << endl; //DELETE LATER
			 break;
                   case 3:
                   t.depth = stod(fileList[i].substr(0, position));
                //   cout << t.depth << endl;
			 break;
                   default:
                   break;
                
                }
                fileList[i].erase(0, position + 1);
                ++count;
            }
		if (fileList[i].find("none.jpg") != string::npos || fileList[i].find("none.jpeg") != string::npos) 
		{
			t.hasTennisBall = false;
			t.x = 0;
			t.y = 0;
			t.depth = 0;
}
            testCases.push_back(t);
        }
    }
>>>>>>> 43cba54670228d90faed93168f048e4dd243ef0b
    
    return testCases;
}

int main(int argc, char * argv[]) {

    cout << endl;
    DIR           *d;
    struct dirent *dir;
    vector<string> fileList;
   // if (argc != 2) {
     //   cerr << "\nTesting Error: Must Provide a Directory\n" << endl;
       // exit(0);
   // }
    //string directory_name = argv[3];
    string directory_name = "stuff";
    int i = 0;
    
    //d = opendir(directory_name.c_str());
    d= opendir(directory_name.c_str());
   // cout << directory_name << endl;    
if (d) {
    
    int i = 0;

        while ((dir = readdir(d)) != NULL) {
            ++i;
            fileList.push_back(dir->d_name);
        }
        
        vector<TestCase> testCases = parse_file(fileList);
        vector<string> successRates;

        for (unsigned i = 0; i < testCases.size(); i++) {

//	o = imread("none.jpg", CV_LOAD_IMAGE_COLOR);
  //      vector<Point2f> willitwork = findTennisBall(poo);
//	cout << "ass" << endl;
	    vector<Point2f> ball = findTennisBall(testCases[i].img);
	//	cout << ball[0] << endl;
	//	cout << ball[1] << endl;
	    ostringstream result;
	    result << "test number: " << testCases[i].num <<  "----- ";
	    if (ball.size() == 0)
	    {
		result << "predicted: no tennis ball, real: ";
		if (testCases[i].hasTennisBall == true)
		{
			result << "ball found";
		} else
		{
			result << "no ball found";
		}
	    } else 
		{
		int count = 1;
		while (ball.size() != 0)
		{
			result << "center " << count << ": x = " << ball[0].x << ", y = " << ball[0].y << " ... ";
ball.erase(ball.begin());
++count;  
		}	
	    // result << "predicted: x =  " << ball[0] << ", y =  " << ball[1] << ", real: x = " << testCases[i].x << ", y = " << testCases[i].y;
	result << count << " CENTERS FOUND//////  " << "real: x = " << testCases[i].x << ", y = " << testCases[i].y;        
	}
		string s(result.str());
  		 successRates.push_back(s);
	}
}
     
        closedir(d);
        
        for (unsigned i = 0; i < successRates.size(); ++i) {
            cout << successRates[i] << endl << endl;
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
