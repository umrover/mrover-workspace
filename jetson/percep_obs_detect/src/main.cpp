#include "obs-detector.h"
#include "camera.hpp"
#include "common.hpp"
#include "artag_detector.hpp"

rapidjson::Document parse_config() {
    /* --- Reading in Config File --- */
    rapidjson::Document mRoverConfig;
    ifstream configFile;
#if WITH_JARVIS
    std::string configPath = getenv("MROVER_CONFIG");
    configPath += "/config_percep_obs/config.json";
#else
    std::string configPath = std::string(ROOT_DIR) + "/config.json";
#endif
    configFile.open(configPath);
    if (!configFile.is_open()) {
        throw std::runtime_error("Config file not found");
    }
    std::string config;
    std::string setting;
    while (configFile >> setting) {
        config += setting;
    }
    configFile.close();
    mRoverConfig.Parse(config.c_str());

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
 *      run ar tag detector
 *      run obs detector
 * @return int 
 */
int main() {
    try {
        // Parse Config
        rapidjson::Document mRoverConfig = parse_config();

        // Initialize Objects
        auto cam = std::make_shared<Source::Camera>(mRoverConfig);
        TagDetector detector(mRoverConfig, cam);
        ObsDetector obs(mRoverConfig, cam);

        while (cam->grab_frame() && obs.open()) {
            // AR Tag Detection
            if (mRoverConfig["startup"]["ar_tag_enabled"].GetInt()) {
                detector.update();
            }

            // Obstacle Detection
            if (mRoverConfig["startup"]["obs_enabled"].GetInt()) {
                obs.open();
                obs.update();
                obs.spinViewer();
            }
        }
        return EXIT_SUCCESS;
    } catch (std::exception const& exception) {
        std::cerr << "Exception: " << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
