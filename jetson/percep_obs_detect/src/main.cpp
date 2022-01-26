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
    try {
        rapidjson::Document mRoverConfig = parse_config();

        Source::Camera cam(mRoverConfig);
        TagDetector detector(mRoverConfig, &cam);
        ObsDetector obs(mRoverConfig, &cam);
        
        while (cam.grab_frame()) {
            detector.update();
            
            obs.open();
            obs.update();
            obs.spinViewer();
        }
        return EXIT_SUCCESS;
    } catch (std::exception const& exception) {
        std::cerr << "Exception: " << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
    


    // cv::VideoWriter vidWrite;
    // string s = "artag_number_.avi";
    // vidWrite =  VideoWriter(s, VideoWriter::fourcc('M','J','P','G'),10,img.size(),true);
    // vidWrite.write(img);
    // vidWrite.release();


    // try {
    //     ObsDetector obs(DataSource::FILESYSTEM, OperationMode::DEBUG, ViewerType::GL);
    //     while (cam.grab_frame()) {
    //         obs.open();
    //         obs.update();
    //         obs.spinViewer();
    //     }
    //     return EXIT_SUCCESS;
    // } catch (std::exception const& exception) {
    //     std::cerr << "Exception: " << exception.what() << std::endl;
    //     return EXIT_FAILURE;
    // }
}
