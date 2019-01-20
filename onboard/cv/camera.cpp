#include "camera.hpp"

#include <sl/Camera.hpp>
#include <cassert>

class Camera::Impl {
public:
	Impl();
    ~Impl();
	bool grab();
	//void deleteZed();

	cv::Mat image();
	cv::Mat depth();
private:
	sl::RuntimeParameters runtime_params_;
	sl::Resolution image_size_;
	sl::Camera zed_;

	sl::Mat image_zed_;
	sl::Mat depth_zed_;

	cv::Mat image_;
	cv::Mat depth_;
};

Camera::Impl::Impl() {
	sl::InitParameters init_params;
	init_params.camera_resolution = sl::RESOLUTION_HD720; // default: 720p
	init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
	init_params.coordinate_units = sl::UNIT_METER;
	init_params.camera_fps = 15;
	// TODO change this below?
	assert(this->zed_.open(init_params) == sl::SUCCESS);
	std::cout<<"ZED init success\n";
	this->runtime_params_.sensing_mode = sl::SENSING_MODE_STANDARD;

	this->image_size_ = this->zed_.getResolution();
	this->image_zed_.alloc(this->image_size_.width, this->image_size_.height,
						   sl::MAT_TYPE_8U_C4);
	this->image_ = cv::Mat(
		this->image_size_.height, this->image_size_.width, CV_8UC4,
		this->image_zed_.getPtr<sl::uchar1>(sl::MEM_CPU));
	this->depth_zed_.alloc(this->image_size_.width, this->image_size_.height,
		                   sl::MAT_TYPE_32F_C1);
	this->depth_ = cv::Mat(
		this->image_size_.height, this->image_size_.width, CV_32FC1,
		this->depth_zed_.getPtr<sl::uchar1>(sl::MEM_CPU));
}

bool Camera::Impl::grab() {
	return this->zed_.grab(this->runtime_params_) == sl::SUCCESS;
}

cv::Mat Camera::Impl::image() {
	this->zed_.retrieveImage(this->image_zed_, sl::VIEW_LEFT, sl::MEM_CPU,
							 this->image_size_.width, this->image_size_.height);
	return this->image_;
}

cv::Mat Camera::Impl::depth() {

    this->zed_.retrieveMeasure(this->depth_zed_, sl::MEASURE_DEPTH,  sl::MEM_CPU,  this->image_size_.width, 
    	 this->image_size_.height);

	return this->depth_;
}

Camera::Impl::~Impl() {
	this->zed_.close();

}

/*void Camera::Impl::deleteZed(){
	delete this;
}*/

Camera::Camera() : impl_(new Camera::Impl) {
}

Camera::~Camera() {
	delete this->impl_;
}

bool Camera::grab() {
	return this->impl_->grab();
}

cv::Mat Camera::image() {
	return this->impl_->image();
}

cv::Mat Camera::depth() {
	return this->impl_->depth();
}
