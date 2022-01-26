#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "common.hpp"

/**
 * @brief Interface class designed as a template that all camera 
 * camera classses should comply by
 */
class ICamera {
public:
    sl::Resolution res;
    cv::Mat image;
	cv::Mat depth;

    virtual bool grab_frame() = 0;
    virtual cv::Mat get_image() = 0;
    virtual cv::Mat& get_depth() = 0;
    virtual GPU_Cloud get_cloud() = 0;
    virtual void write_data() = 0;
};
