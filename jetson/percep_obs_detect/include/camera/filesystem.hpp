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

            explicit FileSystem(const rapidjson::Document& config);

            ~FileSystem();

            bool grab_frame() override;

            void ignore_grab() override;

            cv::Mat& get_image() override;

            cv::Mat& get_depth() override;

            GPU_Cloud get_cloud() override;

            void write_data() override;

            int get_frame() override;

            void set_frame(int frame) override;

            int get_max_frame() override;
    };
}
