#include "obs-detector.h"
#include "camera.hpp"
#include "common.hpp"
#include "artag_detector.hpp"

rapidjson::Document parse_config() {
    /* --- Reading in Config File --- */
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    std::string configPath = "config.json";
    configFile.open( configPath );
    std::string config = "";
    std::string setting;
    while( configFile >> setting ) {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse( config.c_str() );

    return mRoverConfig;
}

using namespace std;
using namespace cv;

/**
 * @brief 
 * Parses configuration file
 * Constructs Camera
 * Constructs Obstacle Detector
 * Constructs AR Tag Detector
 * while true
 *      grab frame from camera
 *      run obs detector
 *      run ar tag detector
 * @return int 
 */
int main() {
    rapidjson::Document mRoverConfig = parse_config();

    /* --- AR Tag Initializations --- */
    TagDetector detector(mRoverConfig);
    pair<Tag, Tag> tagPair;
    Source::Camera cam(mRoverConfig);
    
    while (true) {
        cam.grab_frame();
        rover_msgs::TargetList arTagsMessage;
        rover_msgs::Target* arTags = arTagsMessage.targetList;

        Mat rgb;
        Mat src = cam.get_image();
        Mat depth_img = cam.get_depth();

        /* --- AR Tag Processing --- */
        arTags[0].distance = mRoverConfig["ar_tag"]["default_tag_val"].GetInt();
        arTags[1].distance = mRoverConfig["ar_tag"]["default_tag_val"].GetInt();
        tagPair = detector.findARTags(src, depth_img, rgb);
        detector.updateDetectedTagInfo(arTags, tagPair, depth_img, src);
        imshow("depth", src);
        waitKey(1);  
    }
    


    // cv::VideoWriter vidWrite;
    // string s = "artag_number_.avi";
    // vidWrite =  VideoWriter(s, VideoWriter::fourcc('M','J','P','G'),10,img.size(),true);
    // vidWrite.write(img);
    // vidWrite.release();

    exit(0);

    try {
        ObsDetector obs(DataSource::FILESYSTEM, OperationMode::DEBUG, ViewerType::GL);
        while (obs.open()) {
            obs.update();
            obs.spinViewer();
        }
        return EXIT_SUCCESS;
    } catch (std::exception const& exception) {
        std::cerr << "Exception: " << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
