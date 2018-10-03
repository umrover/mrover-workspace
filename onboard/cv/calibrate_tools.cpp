#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera.hpp"
#include <chrono>
#include <cmath>
#include <getopt.h>
#include <lcm/lcm-cpp.hpp>

using namespace cv;
using namespace std;

int main(){
	Camera cam;

	char choice;
	cout << "Calibrate color bounds or obstacle parameters (type 'c' or 'o') ?		";
	cin >> choice;

	if(choice == 'c'){
		Mat src;
		Mat hsv;
		int hold;

		Scalar lower_bound = Scalar(150, 255, 255);
		Scalar upper_bound = Scalar(0, 0, 0);

		cout << "Hold esc to grab valid values within the circle (40 frames to setup, 60 frames to calibrate) \n"
		while(hold < 100){
			while (!cam.grab()) {
	            cerr << "grab failed.\n";
	            counter_fail++;
	            if (counter_fail > 1000000) break;
        	}
	        if (counter_fail  ) {
	            cerr<< "camera fails\n"; 
	            break;
	        }else counter_fail = 0;
        
			src = camera.image();
			Point2f center(src.cols/2, src.rows/2);

			cvtColor(src, hsv CV_BGR2HSV);

			Vec3b color = hsv.at<Vec3b>(center);

			if( (char)cv::waitKey(0) == 27 && hold > 40){
				cout << "pressed\n";
				if(color.val[0] < lower_bound[0]) lower_bounds[0] = color.val[0];
				if(color.val[1] < lower_bound[1]) lower_bounds[1] = color.val[1];
				if(color.val[2] < lower_bound[2]) lower_bounds[2] = color.val[2];
				if(color.val[0] > upper_bound[0]) upper_bounds[0] = color.val[0];
				if(color.val[1] > upper_bound[1]) upper_bounds[1] = color.val[1];
				if(color.val[2] > upper_bound[2]) upper_bounds[2] = color.val[2];
			}
			hold++;
			circle(src, center, 10, (0,0,255), 2, 8, 0);

			imshow("video", src);

			cout << "lower_bound: (" << lower_bound[0] << ", " << lower_bounds[1] << ", " << lower_bounds[2] << ")\n";
			cout << "upper_bound: (" << upper_bound[0] << ", " << upper_bound[1] << ", " << upper_bound[2] << ")\n";

		}
	}else if(choice == 'o'){
		int hold;
	    Mat src;
	    Mat depth_img;

		const float roverWidth = 46 * 25.4; //inches to mm
		const float focalLength = 2.8; //zed focal length in mm
		const float fieldofView = 110 * PI/180;	//degrees to radians

		float distThreshold;    //meters, used to calculate rover pixels for scanning width

		cout << "Changing zed angle or height, or distance threshold will require recalibration.\n \n";
		cout << "Enter Distance Threshold (meters): ";
		cint >> distThreshold;

    	cout << "Place minumum obstacle at desired distance. Hold esc to calibrate. (15 frames to setup, 25 frames to calibrate)\n";

	    float meanSum;
	    float min = 800000000;

    	while(hold < 40){
    		while (!cam.grab()) {
	            cerr << "grab failed.\n";
	            counter_fail++;
	            if (counter_fail > 1000000) break;
        	}
	        if (counter_fail  ) {
	            cerr<< "camera fails\n"; 
	            break;
	        }else counter_fail = 0;

	        src = cam.image();
	        depth_img = cam.depth();
	        imshow("image", src);


	        if( (char)cv::waitKey(0) == 27 && hold > 15){
		        Mat mean_row_vec = Mat::zeros(1, pixelWidth, type);
		        int type = depth_img.type();

		        const int pixelWidth = depth_img.cols;
		    	const float roverWidthSensor = roverWidth * focalLength/(distThreshold * 1000);
		    	const int roverPixWidth = roverWidthSensor*(pixelWidth/2)/(tan(fieldofView/2) * focalLength);

		        Point startScan(pixelWidth/2 - roverPixWidth/2, pixelHeight/2), endScan(pixelWidth/2 + roverPixWidth/2, pixelHeight/2);
				const Mat sub_col =  mean_row_vec.colRange(startScan.x, endScan.x);
				float window_sum = sum(sub_col)[0];
				rectangle(src, Point( startScan.x, 0), Point( endScan.x, 720), Scalar(0, 0, 255), 3);
			    cout<<"[middle], window sub_col sum is "<<window_sum<<endl;

			    if(window_sum < min) min = window_sum;
			    meanSum+=window_sum;
			}

		}

		cout << "Minimun sum: " << min << endl;
		cout << "Mean sum: " << meanSum/25 << endl;

	}

}