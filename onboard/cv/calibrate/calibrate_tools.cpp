#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera.hpp"
#include <chrono>
#include <cmath>
#include <getopt.h>
#include <lcm/lcm-cpp.hpp>

using namespace cv;
using namespace std;

#define PI 3.14159265

int main(){
	Camera cam;
	int counter_fail = 0;
	while (!cam.grab()) {
        cerr << "grab failed.\n";
        counter_fail++;
        if (counter_fail > 1000000) break;
    }

    if (counter_fail  ) {
		cerr<< "camera fails\n"; 
        exit(-1);
    }else counter_fail = 0;

	char choice;
	cout << "Calibrate color bounds or obstacle parameters (type 'c' or 'o') ?		";
	cin >> choice;

	if(choice == 'c'){
		Mat src;
		Mat hsv;
		int hold = 0;

		Scalar lower_bound = Scalar(150, 255, 255);
		Scalar upper_bound = Scalar(0, 0, 0);

		cout << "Hold esc to grab valid values within the circle, hold space to run video without grabbing \n";
		while(waitKey(30)){//hold < 100){
			cam.grab();
			src = cam.image();

			Point2f center(src.cols/2, src.rows/2);

			cvtColor(src, hsv, CV_BGR2HSV);

			Vec3b color = hsv.at<Vec3b>(center);

			if( (char)cv::waitKey(0) == 27){
				cout << "pressed\n";
				if(color.val[0] < lower_bound[0]) lower_bound[0] = color.val[0];
				if(color.val[1] < lower_bound[1]) lower_bound[1] = color.val[1];
				if(color.val[2] < lower_bound[2]) lower_bound[2] = color.val[2];
				if(color.val[0] > upper_bound[0]) upper_bound[0] = color.val[0];
				if(color.val[1] > upper_bound[1]) upper_bound[1] = color.val[1];
				if(color.val[2] > upper_bound[2]) upper_bound[2] = color.val[2];

				cout << "lower_bound: (" << lower_bound[0] << ", " << lower_bound[1] << ", " << lower_bound[2] << ")\n";
				cout << "upper_bound: (" << upper_bound[0] << ", " << upper_bound[1] << ", " << upper_bound[2] << ")\n";
			}

			hold++;
			circle(src, center, 10, Scalar(0,0,255), 2, 8, 0);

			imshow("video", src);
		}
	}else if(choice == 'o'){
		int hold = 0;
	    Mat src;
	    Mat depth_img;

		const float roverWidth = 46 * 25.4; //inches to mm
		const float focalLength = 2.8; //zed focal length in mm
		const float fieldofView = 110 * PI/180;	//degrees to radians

		float distThreshold;    //meters, used to calculate rover pixels for scanning width

		cout << "Changing zed angle or height, or distance threshold will require recalibration.\n \n";
		cout << "Enter Distance Threshold (meters): ";
		cin >> distThreshold;

    	cout << "Place minumum obstacle at desired distance. Hold esc to calibrate. Hold space to run video (15 frames to setup, 25 frames to calibrate)\n";

	    float meanSum;
	    float minSum = 800000000;

    	while(waitKey(30)){// && hold < 40){
    		cam.grab();
	        src = cam.image();
	        depth_img = cam.depth();
	        depth_img = max(depth_img, 0.7);
			depth_img = min(depth_img, 20.0);
		    const int pixelWidth = depth_img.cols;
		    const int pixelHeight = depth_img.rows;
		    int type = depth_img.type();

	        imshow("image", src);

	        if( (char)cv::waitKey(0) == 27){
		        Mat mean_row_vec = Mat::zeros(1, pixelWidth, type);
		        reduce(depth_img, mean_row_vec, 0, CV_REDUCE_SUM);

		    	const float roverWidthSensor = roverWidth * focalLength/(distThreshold * 1000);
		    	const int roverPixWidth = roverWidthSensor*(pixelWidth/2)/(tan(fieldofView/2) * focalLength);

		        Point startScan(pixelWidth/2 - roverPixWidth/2, pixelHeight/2), endScan(pixelWidth/2 + roverPixWidth/2, pixelHeight/2);
				const Mat sub_col =  mean_row_vec.colRange(startScan.x, endScan.x);
				float window_sum = sum(sub_col)[0];
				rectangle(src, Point( startScan.x, 0), Point( endScan.x, 720), Scalar(0, 0, 255), 3);
			    cout<<"[middle], window sub_col sum is "<<window_sum<<endl;


			    if(window_sum < minSum) minSum = window_sum;
			    meanSum+=window_sum;
				hold++;
			}			
			if(hold > 30) break;
		}

		cout << "Minimun sum: " << minSum << endl;

	}

}
