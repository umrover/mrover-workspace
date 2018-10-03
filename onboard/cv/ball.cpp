//#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <numeric>
#include <string> 
#include <cstring>
#include <getopt.h>
#include <chrono>

using namespace cv;
using namespace std;

float fieldofView = 110;

void printHelp(char *argv[]) {
    cout << "Usage: " << argv[0] << " [-v camera#]|-i imagefile" << '\n';
} // printHelp()

char* getMode(int argc, char * argv[], unsigned &cameranum){
    char* mode;

    // These are used with getopt_long()
    int choice;
    int option_index = 0;
    option long_options[] = {
        // TODO: Fill in two lines, for the "mode" ('m') and
        // the "help" ('h') options.
            // LONG        ARGUMENT USED?     (ignore) RETURN VALUE
        {"image",       required_argument, nullptr, 'i'},
        {"video",       required_argument, nullptr, 'v'},
        {"help",        no_argument, nullptr, 'h'},
    // this one tells getopt_long that there are no more options:
    {nullptr,      0,                 nullptr,  0}
    };
    char three = 'v';
    // TODO: Fill in the double quotes, to match the mode and help options.
    while ((choice = getopt_long(argc, argv, "hv:i:", long_options, &option_index)) != -1) {
        switch(choice) {
        case 'h':
            printHelp(argv);
            exit(0);
        case 'v':  
            mode = &three;
            cameranum = optarg[0] - '0';
            //mode = '3';
            break;
        case 'i':
            mode = optarg;
            break;
         
        default:
            cerr << "Error: invalid option " << endl;
            exit(1);
        } // switch
    } // while
    return mode;
} // getMode()

Mat greenFilter(const Mat& src){
    assert(src.type() == CV_8UC3);

    Mat greenOnly;
    Scalar lowerb = Scalar(26, 50, 150);
    Scalar upperb = Scalar(60, 210, 255);
    inRange(src, lowerb, upperb, greenOnly);

    return greenOnly;
}

float getAngle(int xPixel,int wPixel){
	return atan((xPixel - wPixel/2)/wPixel * tan(fieldofView));
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

int main(int argc, char** argv){
    unsigned cameranum;
    char* mode = getMode(argc, argv, cameranum);
    if(*mode != 'v'){
        const char* filename = mode;
        // Loads an image
        Mat src = imread(filename, IMREAD_COLOR );
        // Check if image is loaded fine
        if(src.empty()){
            printf(" Error opening image\n");
            return -1;
        }
        cout << "Processing image: " << mode << '\n';
        string output(mode);
        output = "out" + output;

        findTennisBall(src);

        imwrite(output, src);
        //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        //imshow( "Contours", drawing );
        //waitKey();
        return 0;
    }
    double frame_time = 0;

    VideoCapture cap(cameranum);
    if(!cap.isOpened())
        return -1;
    Mat src;
    int j = 0;
    while(waitKey(30)){
        auto start = chrono::high_resolution_clock::now();
        cap.read(src);

        if(findTennisBall(src)){
            //get depth from ZED
            //calculate angle from camera
            //call update THOR
        }
        
        imshow("detected circles", src);

        auto end = chrono::high_resolution_clock::now();

        auto delta = chrono::duration_cast<chrono::duration<double>>(end - start);
        frame_time += delta.count();
        if(j % 100 == 0){
            cout << 1.0f/(frame_time/j) << endl;
        }   
        j++;
    }    
    return 0;
}