#include <deque>

#include "rapidjson/istreamwrapper.h"

#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"

using namespace std::chrono_literals;

class DoubleTrack{
private:
    double* ptr;
    int int_value;

    static constexpr double PRECISION = 100;

public:
    DoubleTrack(double* ptr) : ptr(ptr), int_value(*ptr * PRECISION) {}


    void setup(const std::string& field_name, const std::string& window_name, double max_value) {
        cv::createTrackbar(field_name, window_name, &int_value, max_value * PRECISION, DoubleTrack::set_double, ptr);
    }

    static void set_double(int val, void* object) {
        *(static_cast<double*>(object)) = val / PRECISION;
    }

};
 
int main() {

    /* --- Reading in Config File --- */
    rapidjson::Document mRoverConfig;
    std::ifstream configFile;
    bool isJarvis = getenv("MROVER_CONFIG");
    std::string configPath = isJarvis
                             ? std::string{getenv("MROVER_CONFIG")} + "/config_percep/config.json"
                             : std::string{get_current_dir_name()} + "/config/percep/config.json";

    configFile.open(configPath);
    if (configFile) {
        rapidjson::IStreamWrapper isw(configFile);
        mRoverConfig.ParseStream(isw);
        configFile.close();
    } else {
        throw std::runtime_error("Failed to open config file at: " + configPath);
    }

    /* --- Camera Initializations --- */
    Camera cam(mRoverConfig);
    int iterations = 0;
    cam.grab();

#if AR_DETECTION
    cv::Mat rgb;
    cv::Mat src = cam.image();
#endif

#if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
    cam.disk_record_init();
#endif

    /* -- LCM Messages Initializations -- */
    lcm::LCM lcm_;
    rover_msgs::TargetList arTagsMessage{};
    rover_msgs::Target* arTags = arTagsMessage.targetList;
    arTags[0].distance = mRoverConfig["ar_tag"]["default_tag_val"].GetInt();
    arTags[1].distance = mRoverConfig["ar_tag"]["default_tag_val"].GetInt();

    rover_msgs::Obstacle obstacleMessage{};
    obstacleMessage.bearing = 0;
    obstacleMessage.rightBearing = 0;
    obstacleMessage.distance = -1;

    /* --- AR Tag Initializations --- */
    TagDetector detector(mRoverConfig);
    std::pair<Tag, Tag> tagPair;

    /* --- Point Cloud Initializations --- */
#if OBSTACLE_DETECTION

    PCL pointcloud(mRoverConfig);
    enum viewerType {
        newView, //set to 0 -or false- to be passed into updateViewer later
        originalView //set to 1 -or true- to be passed into updateViewer later
    };

    /* --- Outlier Detection --- */
    int numChecks = 3;
    std::deque <bool> outliers;
    outliers.resize(numChecks, true); //initializes outliers vector
    std::deque <bool> checkTrue(numChecks, true); //true std::deque to check our outliers std::deque against
    std::deque <bool> checkFalse(numChecks, false); //false std::deque to check our outliers std::deque against
    obstacle_return lastObstacle;

#endif

    /* --- AR Recording Initializations and Implementation--- */

    time_t now = time(nullptr);
    char* ltm = ctime(&now);
    std::string timeStamp(ltm);

#if AR_RECORD
    //initializing ar tag videostream object
    cam.record_ar_init();
#endif

    /* --- AR Tag Debugging --- */
    #if AR_DETECTION && PERCEPTION_DEBUG
        cv::namedWindow("ARUCO Debug", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("adaptiveThreshWinSizeMin", "ARUCO Debug", &detector.getAlvarParams()->adaptiveThreshWinSizeMin, 50, nullptr);
        cv::createTrackbar("adaptiveThreshWinSizeMax", "ARUCO Debug", &detector.getAlvarParams() -> adaptiveThreshWinSizeMax, 50, nullptr);
        cv::createTrackbar("adaptiveThreshWinSizeStep", "ARUCO Debug", &detector.getAlvarParams()->adaptiveThreshWinSizeStep, 30, nullptr);
        DoubleTrack(&detector.getAlvarParams()->adaptiveThreshConstant).setup("adaptiveThreshConstant", "ARUCO Debug", 20);
        DoubleTrack(&detector.getAlvarParams()->minMarkerPerimeterRate).setup("minMarkerPerimeterRate", "ARUCO Debug", 1);
        DoubleTrack(&detector.getAlvarParams()->maxMarkerPerimeterRate).setup("maxMarkerPerimeterRate", "ARUCO Debug", 10);
        DoubleTrack(&detector.getAlvarParams()->polygonalApproxAccuracyRate).setup("polygonalApproxAccuracyRate", "ARUCO Debug", 1);
        DoubleTrack(&detector.getAlvarParams()->minCornerDistanceRate).setup("minCornerDistanceRate", "ARUCO Debug", 1);
        cv::createTrackbar("minDistanceToBorder", "ARUCO Debug", &detector.getAlvarParams()->minDistanceToBorder, 10, nullptr);
        DoubleTrack(&detector.getAlvarParams()->minMarkerDistanceRate).setup("minMarkerDistanceRate", "ARUCO Debug", 1);
        cv::createTrackbar("cornerRefinementWinSize", "ARUCO Debug", &detector.getAlvarParams()->cornerRefinementWinSize, 20, nullptr);
        cv::createTrackbar("cornerRefinementMaxIterations", "ARUCO Debug", &detector.getAlvarParams()->cornerRefinementMaxIterations, 80, nullptr);
        DoubleTrack(&detector.getAlvarParams()->cornerRefinementMinAccuracy).setup("cornerRefinementMinAccuracy", "ARUCO Debug", 1);
        cv::createTrackbar("markerBorderBits", "ARUCO Debug", &detector.getAlvarParams()->markerBorderBits, 10, nullptr);
        cv::createTrackbar("perspectiveRemovePixelPerCell", "ARUCO Debug", &detector.getAlvarParams()->perspectiveRemovePixelPerCell, 10, nullptr);
        DoubleTrack(&detector.getAlvarParams()->perspectiveRemoveIgnoredMarginPerCell).setup("perspectiveRemoveIgnoredMarginPerCell", "ARUCO Debug", 5);
        DoubleTrack(&detector.getAlvarParams()->maxErroneousBitsInBorderRate).setup("maxErroneousBitsInBorderRate", "ARUCO Debug", 1);
        DoubleTrack(&detector.getAlvarParams()->minOtsuStdDev).setup("minOtsuStdDev", "ARUCO Debug", 20);
        DoubleTrack(&detector.getAlvarParams()->errorCorrectionRate).setup("errorCorrectionRate", "ARUCO Debug", 1);

        std::string const gstLaunch = "appsrc ! video/x-raw, format=BGR !"
                                    "queue ! videoconvert ! x264enc tune=zerolatency bitrate=1000000 speed-preset=superfast !"
                                    "rtph264pay ! udpsink host=127.0.0.1 port=5000";

        cv::Size streamSize(1280, 720);
        cv::VideoWriter writer(gstLaunch, 0, 30, streamSize);
        cv::Mat send(streamSize, CV_8UC3);
    #endif

    /* --- Main Processing Stuff --- */
    while (true) {
        //Check to see if we were able to grab the frame
        if (!cam.grab()) break;

#if AR_DETECTION
        //Grab initial images from cameras
        cv::Mat rgb;
        cv::Mat src = cam.image();
        cv::Mat depth_img = cam.depth();
        cv::Mat xyz_img = cam.xyz();

#endif

#if OBSTACLE_DETECTION
        //Update Point Cloud
        pointcloud.update();
        cam.getDataCloud(pointcloud.pt_cloud_ptr);
#endif

#if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
        int FRAME_WRITE_INTERVAL = mRoverConfig["camera"]["frame_write_interval"].GetInt();
            if (iterations % FRAME_WRITE_INTERVAL == 0) {
                cv::Mat rgb_copy = src.clone(), depth_copy = depth_img.clone();
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
#if PERCEPTION_DEBUG
        cv::Mat bgr;
        cv::cvtColor(src, bgr, cv::COLOR_RGBA2BGR);
        writer.write(bgr);
#endif

        detector.updateDetectedTagInfo(arTags, tagPair, depth_img, xyz_img);

#if PERCEPTION_DEBUG && AR_DETECTION
//        cv::imshow("depth", depth_img);
        cv::waitKey(1);
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

