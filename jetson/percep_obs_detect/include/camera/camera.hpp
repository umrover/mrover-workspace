#pragma once

#include "icamera.hpp"
#include "zed.hpp"

namespace Source {

// typedef std::shared_ptr<ICamera> Camera_impl_ptr
// typedef Zed_ptr std::shared_ptr<Zed>

/**
 * @brief Camera shell, which stores a pointer to an actual 
 * implementation of a camera class
 */
class Camera : public ICamera{
private:
    std::shared_ptr<ICamera> cam_impl;

public:
    Camera(){}
    Camera(const rapidjson::Document &config) {
        std::string data_source = config["startup"]["data_source_type"].GetString();
        
        if (data_source == "filesystem") {

        }
        else if (data_source == "zed") {
            cam_impl = std::shared_ptr<Source::Zed>(new Source::Zed(config));
        }
        else {
            std::cerr << "Invalid data_source_type field\n";
            exit(-1);
        }
    }
    ~Camera() {}
    virtual bool grab_frame() override {return cam_impl->grab_frame();}
    virtual cv::Mat& get_image() override {return cam_impl->get_image();}
    virtual cv::Mat& get_depth() override {return cam_impl->get_depth();}
    virtual GPU_Cloud get_cloud() override {return cam_impl->get_cloud();}
    virtual void write_data() override {cam_impl->write_data();}
};

}

