#pragma once

#include "icamera.hpp"
#include "zed.hpp"
#include "filesystem.hpp"

namespace Source {

    /**
     * @brief Camera shell, which stores a pointer to an actual
     * implementation of a camera class
     */
    class Camera : public ICamera {
        private:
            std::shared_ptr<Source::ICamera> cam_impl;

        public:
            Camera() = default;

            explicit Camera(const rapidjson::Document& config) {
                std::string data_source = config["startup"]["data_source_type"].GetString();

                if (data_source == "filesystem") {
                    cam_impl = std::make_shared<Source::FileSystem>(config);
                } else if (data_source == "zed") {
                    cam_impl = std::make_shared<Source::Zed>(config);
                } else {
                    std::cerr << "Invalid data_source_type field\n";
                    exit(-1);
                }
            }

            ~Camera() = default;

            bool grab_frame() override { return cam_impl->grab_frame(); }

            void ignore_grab() override { return cam_impl->ignore_grab(); }

            cv::Mat& get_image() override { return cam_impl->get_image(); }

            cv::Mat& get_depth() override { return cam_impl->get_depth(); }

            GPU_Cloud get_cloud() override { return cam_impl->get_cloud(); }

            void write_data() override { cam_impl->write_data(); }

            int get_frame() override { return cam_impl->get_frame(); }

            void set_frame(int frame) override { cam_impl->set_frame(frame); }

            int get_max_frame() override { return cam_impl->get_max_frame(); }
    };
}

typedef std::shared_ptr<Source::Camera> camera_ptr;
