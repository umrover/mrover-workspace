#include "obs-detector.h"
#include "camera.hpp"
#include "common.hpp"

rapidjson::Document parse_config() {
    /* --- Reading in Config File --- */
    rapidjson::Document mRoverConfig;
    ifstream configFile;
    std::string configPath = getenv("MROVER_CONFIG");
    configPath += "config.json";
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

    Camera_impl cam(mRoverConfig);

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
