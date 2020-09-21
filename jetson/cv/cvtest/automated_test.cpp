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
#include <map>
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

   //displaying image
   //imshow("Image", src);
   //waitKey(0);

    return center;
}

vector<TestCase> parse_file(vector<string> & fileList) {

    vector<TestCase> testCases;
    for (unsigned i = 0; i < fileList.size(); ++i) {
        size_t position = 0;

        if (fileList[i][0] != '.') {
            TestCase t;
	    string s = "jetson/cv/cvtest/cv_test_images/" + fileList[i];
            t.img = imread(s, CV_LOAD_IMAGE_COLOR);
            int count = 0;

            //PARSING FILE

            t.hasTennisBall = true;
            while ((position = fileList[i].find("_")) != string::npos) {
                switch (count) {
                    case 0:
                        t.num = (unsigned)stoll(fileList[i].substr(0, position));
			 break;
                    case 1:
                        t.x = (unsigned)stoll(fileList[i].substr(0, position));
			break;
                    case 2:
                        t.y = (unsigned)stoll(fileList[i].substr(0, position));
			break;
                   case 3:
                         t.depth = stod(fileList[i].substr(0, position));
			 break;
                   default:
                         break;
                }
                fileList[i].erase(0, position + 1);
                ++count;
            }

	        //in the case where there is no ball:

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

    return testCases;
}

int main(int argc, char * argv[]) {

    cout << endl;
    DIR           *d;
    struct dirent *dir;
    vector<string> fileList;
    string directory_name = "jetson/cv/cvtest/cv_test_images";
    int i = 0;
    d = opendir(directory_name.c_str());

    //floats to be stored for calculating differences
    float minDiffX, minDiffY;
    float offBy;
if (d) {

	//CREATING FILE LIST
        while ((dir = readdir(d)) != NULL) {
            ++i;
            fileList.push_back(dir->d_name);
        }

        vector<TestCase> tc = parse_file(fileList);
	map<int, TestCase> testCases;
	for (unsigned p = 0; p < tc.size(); ++p)
	{
		testCases[tc[p].num] = tc[p];
	}
        vector<string> successRates;

	//cycling through test cases and creating output

        for (auto it : testCases)
	{
	    vector<Point2f> ball = findTennisBall((it.second).img);
	    ostringstream result;
	    result << "test number: " << (it.second).num;
	    if (ball.size() == 0)
	    {
	        if ((it.second).hasTennisBall == false)
		{
			result << " (PASSED)" << endl;
			result << "predicted:" << endl << "  no ball found" << endl << "real: " << endl;
			result << "  no ball found" << endl;
		} else
		{
			result << " (FAILED)" << endl;
			result << "predicted:" << endl << "  no ball found" << endl << "real: " << endl;
			result << "  x = " << (it.second).x << endl << "  y = " << (it.second).y;
		}
	    } else
		{
		if ((ball[0].x - (it.second).x) <= 5 && (ball[0].y - (it.second).y) <= 5)
		{
			result << " (PASSED)" << endl;
		} else
		{
			result << " (FAILED)" << endl;
		}
		minDiffX = abs(ball[0].x - (it.second).x);
		minDiffY = abs(ball[0].y - (it.second).y);
		offBy = minDiffX + minDiffY;

		int count = 1;
		result << "predicted:" << endl;
		while (ball.size() != 0)
		{
		result << "center " << count << ":" << endl << "  x = " << ball[0].x << endl << "  y = " << ball[0].y << endl;
		float xDiff = abs(ball[0].x - (it.second).x);
		float yDiff = abs(ball[0].y - (it.second).y);
		float offByNew = xDiff + yDiff;
		if (offByNew < offBy)
		{
			minDiffX = xDiff;
			minDiffY = yDiff;
		}

		ball.erase(ball.begin());
			++count;
		}
		count -= 1;
		result << "(" << count << " centers found)" << endl;
		result << "real:" << endl << "  x = " << (it.second).x << endl <<  "  y = " << (it.second).y << endl;
		result << "x-diff: " << minDiffX << endl << "y-diff: " << minDiffY;
	}
		string s(result.str());
  		successRates.push_back(s);
        }

	//closing directory

        closedir(d);

        for (unsigned i = 0; i < successRates.size(); ++i) {
            cout << successRates[i] << endl << endl;
        }
   }
    else {
        cerr << "\nTesting Error: Invalid Directory\n" << endl;
        exit(0);
    }



    return 0;
}
#endif
