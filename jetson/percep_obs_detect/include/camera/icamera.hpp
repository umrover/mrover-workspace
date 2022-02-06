#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "common.hpp"

namespace Source {

    /**
     * @brief Interface class designed as a template that all camera
     * camera classes should comply by
     */
    class ICamera {
        public:
            const std::string PCD_FOLDER = "pcd";
            const std::string RGB_FOLDER = "rgb";
            const std::string DEPTH_FOLDER = "depth";

            sl::Resolution res;
            cv::Mat image;
            cv::Mat depth;
            bool no_grab = false;

            virtual bool grab_frame() = 0;

            /**
             * @brief When ignore_grab is called the next
             * time grab_frame is run, a new frame will not
             * be acquired. The following time grab_frame is
             * run a frame will be acquired. This was added
             * solely so the viewer would be able to step through
             * frames
             */
            virtual void ignore_grab() = 0;

            virtual cv::Mat& get_image() = 0;

            virtual cv::Mat& get_depth() = 0;

            virtual GPU_Cloud get_cloud() = 0;

            virtual void write_data() = 0;

            virtual int get_frame() = 0;

            virtual void set_frame(int frame) = 0;

            /**
             * @return Max frame in series or -1 if a streaming input
             */
            virtual int get_max_frame() = 0;
    };
}
