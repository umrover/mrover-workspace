#pragma once

#include "perception.hpp"
#include "rapidjson/document.h"

#if OBSTACLE_DETECTION
#include <pcl/common/common_headers.h>
#endif

class Camera {
private:
    class Impl;

    Impl* impl_;
    std::string rgb_foldername;
    std::string depth_foldername;
    std::string pcl_foldername;
    cv::VideoWriter vidWrite;

    //reference to config file
    const rapidjson::Document& mRoverConfig;

public:

    int FRAME_WRITE_INTERVAL;

    explicit Camera(const rapidjson::Document& config);

    ~Camera();

    bool grab();

    cv::Mat const& image();

    cv::Mat const& depth();

    cv::Mat const& xyz();

#if OBSTACLE_DETECTION
    void getDataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
#endif

#if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
    void disk_record_init();
    void write_curr_frame_to_disk(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud, int counter);
#endif

    void record_ar_init();

    void record_ar(cv::Mat rgb);

    void record_ar_finish();
};
