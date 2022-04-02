#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <unistd.h>
#include <deque>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

class DoubleTrack{
private:
    double* ptr;
    int int_value;

    static constexpr double PRECISION = 100;

public:
    DoubleTrack(double* ptr) : ptr(ptr), int_value(*ptr * PRECISION) {}


    void setup(const std::string& field_name, const std::string& window_name, double max_value) {
        createTrackbar(field_name, window_name, &int_value, max_value * PRECISION, DoubleTrack::set_double, ptr);
    }

    static void set_double(int val, void* object) {
        *(static_cast<double*>(object)) = val / PRECISION;
    }

};
 
int main() {

 /* --- Reading in Config File --- */
  rapidjson::Document mRoverConfig;
  ifstream configFile;
  string configPath = getenv("MROVER_CONFIG");
  configPath += "/config_percep/config.json";
  configFile.open( configPath );
  string config = "";
  string setting;
  while( configFile >> setting ) {
    config += setting;
  }
  configFile.close();
  mRoverConfig.Parse( config.c_str() );

  /* --- Camera Initializations --- */
    Camera cam(mRoverConfig);
    int iterations = 0;
    cam.grab();

    #if PERCEPTION_DEBUG
        namedWindow("depth", 2);
    #endif
    
    #if AR_DETECTION
    Mat rgb;
    Mat src = cam.image();
    #endif

    #if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
        cam.disk_record_init();
    #endif

    /* -- LCM Messages Initializations -- */
    lcm::LCM lcm_;
    rover_msgs::TargetList arTagsMessage;
    rover_msgs::Target* arTags = arTagsMessage.targetList;
    arTags[0].distance = mRoverConfig["ar_tag"]["default_tag_val"].GetInt();
    arTags[1].distance = mRoverConfig["ar_tag"]["default_tag_val"].GetInt();
    
    rover_msgs::Obstacle obstacleMessage;
    obstacleMessage.bearing = 0;
    obstacleMessage.rightBearing = 0;
    obstacleMessage.distance = -1;
    
    /* --- AR Tag Initializations --- */
    TagDetector detector(mRoverConfig);
    pair<Tag, Tag> tagPair;
    
    /* --- Point Cloud Initializations --- */
    #if OBSTACLE_DETECTION

    PCL pointcloud(mRoverConfig);
    enum viewerType {
        newView, //set to 0 -or false- to be passed into updateViewer later
        originalView //set to 1 -or true- to be passed into updateViewer later
    };

    /* --- Outlier Detection --- */
    int numChecks = 3;
    deque <bool> outliers;
    outliers.resize(numChecks, true); //initializes outliers vector
    deque <bool> checkTrue(numChecks, true); //true deque to check our outliers deque against
    deque <bool> checkFalse(numChecks, false); //false deque to check our outliers deque against
    obstacle_return lastObstacle;

    #endif

    /* --- AR Recording Initializations and Implementation--- */ 
    
    time_t now = time(0);
    char* ltm = ctime(&now);
    string timeStamp(ltm);

    #if AR_RECORD
    //initializing ar tag videostream object
    cam.record_ar_init();
    #endif

    /* --- AR Tag Debugging --- */
    #if AR_DETECTION && PERCEPTION_DEBUG
        namedWindow("ARUCO Debug", WINDOW_AUTOSIZE);
        createTrackbar("adaptiveThreshWinSizeMin", "ARUCO Debug", &detector.getAlvarParams()->adaptiveThreshWinSizeMin, 50, nullptr);
        createTrackbar("adaptiveThreshWinSizeMax", "ARUCO Debug", &detector.getAlvarParams() -> adaptiveThreshWinSizeMax, 50, nullptr);
        createTrackbar("adaptiveThreshWinSizeStep", "ARUCO Debug", &detector.getAlvarParams()->adaptiveThreshWinSizeStep, 30, nullptr);
        DoubleTrack(&detector.getAlvarParams()->adaptiveThreshConstant).setup("adaptiveThreshConstant", "ARUCO Debug", 20);
        DoubleTrack(&detector.getAlvarParams()->minMarkerPerimeterRate).setup("minMarkerPerimeterRate", "ARUCO Debug", 1);
        DoubleTrack(&detector.getAlvarParams()->maxMarkerPerimeterRate).setup("maxMarkerPerimeterRate", "ARUCO Debug", 10);
        DoubleTrack(&detector.getAlvarParams()->polygonalApproxAccuracyRate).setup("polygonalApproxAccuracyRate", "ARUCO Debug", 1);
        DoubleTrack(&detector.getAlvarParams()->minCornerDistanceRate).setup("minCornerDistanceRate", "ARUCO Debug", 1);
        createTrackbar("minDistanceToBorder", "ARUCO Debug", &detector.getAlvarParams()->minDistanceToBorder, 10, nullptr);
        DoubleTrack(&detector.getAlvarParams()->minMarkerDistanceRate).setup("minMarkerDistanceRate", "ARUCO Debug", 1);
        createTrackbar("cornerRefinementMethod", "ARUCO Debug", &detector.getAlvarParams()->cornerRefinementMethod, 3, nullptr);
        createTrackbar("cornerRefinementWinSize", "ARUCO Debug", &detector.getAlvarParams()->cornerRefinementWinSize, 20, nullptr);
        createTrackbar("cornerRefinementMaxIterations", "ARUCO Debug", &detector.getAlvarParams()->cornerRefinementMaxIterations, 80, nullptr);
        DoubleTrack(&detector.getAlvarParams()->cornerRefinementMinAccuracy).setup("cornerRefinementMinAccuracy", "ARUCO Debug", 1);
        createTrackbar("markerBorderBits", "ARUCO Debug", &detector.getAlvarParams()->markerBorderBits, 10, nullptr);
        createTrackbar("perspectiveRemovePixelPerCell", "ARUCO Debug", &detector.getAlvarParams()->perspectiveRemovePixelPerCell, 10, nullptr);
        DoubleTrack(&detector.getAlvarParams()->perspectiveRemoveIgnoredMarginPerCell).setup("perspectiveRemoveIgnoredMarginPerCell", "ARUCO Debug", 5);
        DoubleTrack(&detector.getAlvarParams()->maxErroneousBitsInBorderRate).setup("maxErroneousBitsInBorderRate", "ARUCO Debug", 1);
        DoubleTrack(&detector.getAlvarParams()->minOtsuStdDev).setup("minOtsuStdDev", "ARUCO Debug", 20);
        DoubleTrack(&detector.getAlvarParams()->errorCorrectionRate).setup("errorCorrectionRate", "ARUCO Debug", 1);
    #endif

  /* --- Main Processing Stuff --- */
  while (true) {
        //Check to see if we were able to grab the frame
        if (!cam.grab()) break;

        #if AR_DETECTION
        //Grab initial images from cameras
        Mat rgb;
        Mat src = cam.image();
        Mat depth_img = cam.depth();
        #endif

        #if OBSTACLE_DETECTION
        //Update Point Cloud
        pointcloud.update();
        cam.getDataCloud(pointcloud.pt_cloud_ptr);
        #endif

        #if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
        int FRAME_WRITE_INTERVAL = mRoverConfig["camera"]["frame_write_interval"].GetInt();
            if (iterations % FRAME_WRITE_INTERVAL == 0) {
                Mat rgb_copy = src.clone(), depth_copy = depth_img.clone();
                #if PERCEPTION_DEBUG
                    cout << "Copied correctly" << endl;
                #endif
                cam.write_curr_frame_to_disk(rgb_copy, depth_copy, pointcloud.pt_cloud_ptr, iterations);
        }
        #endif

        /* --- AR Tag Processing --- */
        arTags[0].distance = mRoverConfig["ar_tag"]["default_tag_val"].GetInt();
        arTags[1].distance = mRoverConfig["ar_tag"]["default_tag_val"].GetInt();
        #if AR_DETECTION
            tagPair = detector.findARTags(src, depth_img, rgb);
            #if AR_RECORD
                cam.record_ar(rgb);
            #endif

            detector.updateDetectedTagInfo(arTags, tagPair, depth_img, src);

        #if PERCEPTION_DEBUG && AR_DETECTION
            imshow("depth", src);
            waitKey(1);  
        #endif

        #endif

        /* --- Point Cloud Processing --- */
        #if OBSTACLE_DETECTION && !WRITE_CURR_FRAME_TO_DISK
        
        #if PERCEPTION_DEBUG
            //Update Original 3D Viewer
            pointcloud.updateViewer(originalView);
            cout<<"Original W: " <<pointcloud.pt_cloud_ptr->width<<" Original H: "<<pointcloud.pt_cloud_ptr->height<<endl;
        #endif

        //Run Obstacle Detection
        pointcloud.pcl_obstacle_detection();  
        obstacle_return obstacleOutput (pointcloud.leftBearing, pointcloud.rightBearing, pointcloud.distance);

        //Outlier Detection Processing
        outliers.pop_back(); //Remove outdated outlier value

        if(pointcloud.leftBearing > 0.05 || pointcloud.leftBearing < -0.05)
            outliers.push_front(true);//if an obstacle is detected in front
        else 
            outliers.push_front(false); //obstacle is not detected

        if(outliers == checkTrue) //If past iterations see obstacles
            lastObstacle = obstacleOutput;
        else if (outliers == checkFalse) // If our iterations see no obstacles after seeing obstacles
            lastObstacle = obstacleOutput;

        //Update LCM 
        obstacleMessage.bearing = lastObstacle.leftBearing; // Update LCM bearing field
        obstacleMessage.rightBearing = lastObstacle.rightBearing;
        obstacleMessage.distance = lastObstacle.distance; // Update LCM distance field
        #if PERCEPTION_DEBUG
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Path Sent: " << obstacleMessage.bearing << "\n";
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Distance Sent: " << obstacleMessage.distance << "\n";
        #endif

        #if PERCEPTION_DEBUG
        //Update Processed 3D Viewer
        pointcloud.updateViewer(newView);
        #if PERCEPTION_DEBUG
            cout<<"Downsampled W: " <<pointcloud.pt_cloud_ptr->width<<" Downsampled H: "<<pointcloud.pt_cloud_ptr->height<<endl;
        #endif
        #endif
        
        #endif
        
        /* --- Publish LCMs --- */
        lcm_.publish("/target_list", &arTagsMessage);
        lcm_.publish("/obstacle", &obstacleMessage);

        #if !ZED_SDK_PRESENT
            std::this_thread::sleep_for(0.2s); // Iteration speed control not needed when using camera 
        #endif
        
        ++iterations;
  }


    /* --- Wrap Things Up --- */
    #if AR_RECORD
        cam.record_ar_finish();
    #endif
  
    return 0;
}

