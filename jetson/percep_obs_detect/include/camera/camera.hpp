#ifndef CAMERA
#define CAMERA

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "common.hpp"

class Camera_impl {
private:
    Camera_impl* cam;
    int FRAME_WRITE_INTERVAL;

public:
    Camera_impl(){}
    Camera_impl(const rapidjson::Document &config) {
        FRAME_WRITE_INTERVAL = config["camera"]["frame_write_interval"].GetInt();
        cout << "FRAME WRITE INTERVAL: " << FRAME_WRITE_INTERVAL << "\n";
        cam = new Camera_impl;
    }
    ~Camera_impl() {
        if (cam) {
            delete cam;
        }
    }
    bool grab_frame() {return cam->grab_frame();}
    cv::Mat get_image() {return cam->get_image();}
    cv::Mat get_depth() {return cam->get_depth();}
    GPU_Cloud get_cloud() {return cam->get_cloud();}
    void write_data() {cam->write_data();}
};

#endif
