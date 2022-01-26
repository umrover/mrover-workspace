#pragma once

#include "icamera.hpp"
#include <sl/Camera.hpp>

namespace Source {

class Zed : public ICamera {
private:
    sl::Camera zed;
    sl::InitParameters init_params;
    sl::Mat image_zed;
    sl::Mat depth_zed;

public:
    Zed(){}
    Zed(const rapidjson::Document &config);
    bool grab_frame();
    cv::Mat get_image();
    cv::Mat& get_depth(); 
    GPU_Cloud get_cloud();
    void write_data();
};

}
