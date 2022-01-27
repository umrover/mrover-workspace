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
    sl::Mat frame;
    std::string write_location;
    int frame_counter;
    int frame_gap;


public:
    Zed(){}
    Zed(const rapidjson::Document &config);
    virtual bool grab_frame() override;
    virtual void ignore_grab() override;
    virtual cv::Mat& get_image() override;
    virtual cv::Mat& get_depth() override; 
    virtual GPU_Cloud get_cloud() override;
    virtual void write_data() override;
};

}
