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
            Zed() = default;

            explicit Zed(const rapidjson::Document& config);

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
