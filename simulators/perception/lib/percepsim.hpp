#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <nanomsg/nn.h>

#include "wire_protocol.hpp"

namespace Perception {
    class SimulatedCamera {
        public:
            SimulatedCamera();
            ~SimulatedCamera();

            bool grab() noexcept;

            cv::Mat retrieve_image() noexcept;
            cv::Mat retrieve_depth() noexcept;
        private:
            int sock_;

            cv::Mat image_;
            cv::Mat depth_;
    };

    class Simulator {
        public:
            Simulator();
            ~Simulator();

            void publish(cv::Mat image, cv::Mat depth);
        private:
            int sock_;
    };
}
