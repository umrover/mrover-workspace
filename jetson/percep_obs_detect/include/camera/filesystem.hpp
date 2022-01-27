#pragma once

#include "common.hpp"
#include "icamera.hpp"
#include "reader.h"

namespace Source {

/**
 * @brief Imitates camera functionality while actually reading
 * from the file system
 */
class FileSystem : public ICamera {
private:
    int max_frame;
    int frame_counter;
    std::string read_location;
    GPU_Cloud pc;
    
public:
    PCDReader reader;
    FileSystem(const rapidjson::Document &config);
    ~FileSystem();
    virtual bool grab_frame() override;
    virtual void ignore_grab() override;
    virtual cv::Mat& get_image() override;
    virtual cv::Mat& get_depth() override; 
    virtual GPU_Cloud get_cloud() override;
    virtual void write_data() override;
};

}
