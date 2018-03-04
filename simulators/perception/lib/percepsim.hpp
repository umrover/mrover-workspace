#pragma once

#include <string>
#include <opencv2/core.hpp>

#include "nn.hpp"

#include "wire_protocol.hpp"

namespace Perception {
    class SimulatedCamera {
        public:
            SimulatedCamera();

            bool grab() noexcept;

            cv::Mat retrieve_image() noexcept;
            cv::Mat retrieve_depth() noexcept;
        private:
            nn::socket sock_;

            cv::Mat image_;
            cv::Mat depth_;
    };

    class Simulator {
        public:
            Simulator();
            ~Simulator();

            void publish(cv::Mat image, cv::Mat depth);
        private:
            nn::socket sock_;
            int endpt_;
    };
}
