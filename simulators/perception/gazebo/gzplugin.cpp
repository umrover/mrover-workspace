#include <string>
#include <limits>
#include <iostream>
#include <cmath>

#include "percepsim/percepsim.hpp"

#include <sdf/Param.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>

namespace gazebo {
    class PercepSimDepthCamera : public DepthCameraPlugin {
        public:
            PercepSimDepthCamera();

            virtual void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf_);

        protected:
            virtual void OnNewDepthFrame(const float *image,
                    unsigned int width, unsigned int height,
                    unsigned int depth, const std::string &format);

            virtual void OnNewImageFrame(const unsigned char *image,
                    unsigned int width, unsigned int height,
                    unsigned int depth, const std::string &format);

        private:
            Perception::Simulator sim_;

            float min_point_cloud_;
            float max_point_cloud_;

            cv::Mat image_;
            cv::Mat depth_;

            // Bitflags here
            // __
            // 1  => has image frame
            //  1 => has depth frame
            // When all are 1, we can publish!
            char publish_ready_;

            const char PUBLISH_READY_IMAGE = 2;
            const char PUBLISH_READY_DEPTH = 1;
            const char PUBLISH_READY = 3;

            const float DEFAULT_MIN_POINT_CLOUD = 0.5;
            const float DEFAULT_MAX_POINT_CLOUD = 3.0;
            const float BAD_POINT = std::numeric_limits<float>::quiet_NaN();

            void publish_if_ready();
    };

    GZ_REGISTER_SENSOR_PLUGIN(PercepSimDepthCamera)
}


gazebo::PercepSimDepthCamera::PercepSimDepthCamera() {}

void gazebo::PercepSimDepthCamera::Load(sensors::SensorPtr parent, sdf::ElementPtr _sdf) {
    gazebo::DepthCameraPlugin::Load(parent, _sdf);

    if (!_sdf->HasElement("minPointCloud")) {
        this->min_point_cloud_ = DEFAULT_MIN_POINT_CLOUD;
    } else {
        this->min_point_cloud_ = _sdf->GetElement("minPointCloud")->Get<float>();
    }

    if (!_sdf->HasElement("maxPointCloud")) {
        this->max_point_cloud_ = DEFAULT_MAX_POINT_CLOUD;
    } else {
        this->max_point_cloud_ = _sdf->GetElement("maxPointCloud")->Get<float>();
    }

    this->publish_ready_ = 0;

    int type = CV_8UC3;
    if (this->format == "L8") {
        type = CV_8UC1;
    }
    this->image_.create(this->height, this->width, type);
    this->depth_.create(this->height, this->width, CV_32FC1);
}

void gazebo::PercepSimDepthCamera::publish_if_ready() {
    if (this->publish_ready_ == PUBLISH_READY) {
        this->sim_.publish(this->image_, this->depth_);
        this->publish_ready_ = 0;
    }
}

void gazebo::PercepSimDepthCamera::OnNewDepthFrame(const float *image,
        unsigned int width, unsigned int height, unsigned int depth, const std::string &format) {
    if (this->parentSensor->IsActive()) {
        if (this->depth_.cols != width || this->depth_.rows != height) {
            int type = this->depth_.type();
            this->depth_.create(height, width, type);
        }

        size_t index = 0;
        for (uint32_t j = 0; j < height; j++) {
            for (uint32_t i = 0; i < width; i++) {
                float depth = image[index++];

                if (depth > this->min_point_cloud_ && depth < this->max_point_cloud_) {
                    this->depth_.at<float>(j, i) = depth;
                } else {
                    this->depth_.at<float>(j, i) = BAD_POINT;
                }
            }
        }

        this->publish_ready_ |= PUBLISH_READY_DEPTH;
        this->publish_if_ready();
    } else {
        this->parentSensor->SetActive(true);
    }
}

void gazebo::PercepSimDepthCamera::OnNewImageFrame(const unsigned char *image,
        unsigned int width, unsigned int height, unsigned int depth, const std::string &format) {
    if (this->parentSensor->IsActive()) {
        size_t index = 0;

        if (this->image_.cols != width || this->image_.rows != height) {
            int type = this->image_.type();
            this->image_.create(height, width, type);
        }

        for (uint32_t j = 0; j < height; j++) {
            for (uint32_t i = 0; i < width; i++) {
                if (this->image_.type() == CV_8UC1) {
                    this->image_.at<uchar>(j, i) = image[index++];
                } else {
                    cv::Vec3b val;
                    val[2] = image[index++];
                    val[1] = image[index++];
                    val[0] = image[index++];
                    this->image_.at<cv::Vec3b>(j, i) = val;
                }
            }
        }

        this->publish_ready_ |= PUBLISH_READY_IMAGE;
        this->publish_if_ready();
    } else {
        this->parentSensor->SetActive(true);
    }
}
