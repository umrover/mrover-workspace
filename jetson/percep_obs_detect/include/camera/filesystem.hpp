#pragma once

#include "common.hpp"
#include "icamera.hpp"
#include "reader.h"

namespace Source {

class FileSystem : public ICamera {
private:
    int max_frame;
    int frame_counter;
    std::string read_location;
    GPU_Cloud pc;
    
public:
    PCDReader reader;
    FileSystem(){}
    FileSystem(const rapidjson::Document &config);
    ~FileSystem();
    bool grab_frame();
    void ignore_grab();
    cv::Mat& get_image();
    cv::Mat& get_depth(); 
    GPU_Cloud get_cloud();
    void write_data();
};

}
