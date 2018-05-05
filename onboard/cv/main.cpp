#include "perception.hpp"

using namespace cv;
using namespace std;

float minDepth = 1; //need to set
float pixelWidth = 1; //need to set
float pixelHeight = 1;
int roverPixWidth = 1; //initialize

int calcFocalWidth(){   //mm
    return tan(fieldofView/2) * focalLength;
}

int calcRoverPix(float dist){   //pix
    float roverWidthSensor = realWidth * focalLength/(dist * 1000);
    return roverWidthSensor*(pixelWidth/2)/calcFocalWidth();
}

float getGroundDist(float angleOffset){  // the expected distance if no obstacles
    return zedHeight/sin(angleOffset);
}

double getAngle(float xPixel, float wPixel){
    return atan((xPixel - wPixel/2)/(wPixel/2)* tan(fieldofView/2))* 180.0 /PI;
}

float getObstacleMin(float expected){
    return expected - obstacleThreshold/sin(angleOffset);
}



int main() {
    /*initialize camera*/
    Camera cam;
    int j = 0;
    double frame_time = 0;
    int counter_fail = 0;
    while (true) {
        while (!cam.grab()) {
            cerr << "grab failed.\n";
            counter_fail++;
            if (counter_fail > 1000000) break;
        }

        if (counter_fail  ) {

            cerr<< "camera fails\n"; 
            break;
        }
        else counter_fail = 0;

        auto start = chrono::high_resolution_clock::now();
        Mat src = cam.image();
        imshow("orginal", src);
        Mat depth_img = cam.depth();

        /*initialize lcm messages*/
        lcm::LCM lcm_;
        rover_msgs::TennisBall tennisMessage;
        rover_msgs::Obstacle obstacleMessage;
        tennisMessage.found = false;
        obstacleMessage.detected = false;

        /*initialize obstacle detection*/
        pixelWidth = src.cols;
        pixelHeight = src.rows;
        roverPixWidth = calcRoverPix(distThreshold);
        float expectedDist = getGroundDist(angleOffset);
        minDepth = getObstacleMin(expectedDist);

	/* obstacle detection */
        obstacle_return obstacle_detection =  avoid_obstacle_sliding_window(depth_img, src,  num_sliding_windows , roverPixWidth);
        if(obstacle_detection.bearing != 0) obstacleMessage.detected = true;    //if an obstacle is detected in front
        obstacleMessage.bearing = obstacle_detection.bearing;
        cout << "Turn " << obstacle_detection.bearing << endl;

	/* Tennis ball detection*/
        vector<Point2f> centers = findTennisBall(src);
        if(centers.size() != 0){
            tennisMessage.found = true;
            tennisMessage.distance = depth_img.at<float>(centers[0].x, centers[0].y);
            tennisMessage.bearing = getAngle((int)centers[0].x, src.cols);
            cout << centers.size() << " tennis ball(s) detected: " << tennisMessage.distance 
                                                        << "m, " << tennisMessage.bearing << "degrees\n";
        }else{
            //cout << "tennis ball not detected.\n";
        }
        lcm_.publish("/tennis_ball", &tennisMessage);
        lcm_.publish("/obstacle", &obstacleMessage);

	#ifdef PERCEPTION_DEBUG
    	imshow("depth", depth_img);
        imshow("camera", src);
	#endif
        auto end = chrono::high_resolution_clock::now();

        auto delta = chrono::duration_cast<chrono::duration<double>>(end - start);
        frame_time += delta.count();
        if(j % 100 == 0){
            cout << "framerate: " << 1.0f/(frame_time/j) << endl;
        }   
        j++;
        waitKey(FRAME_WAITKEY);
    }

    return 0;
}
