#pragma once

#include <opencv2/opencv.hpp>

class Camera {
private:
	class Impl;
public:
	Camera();
	~Camera();

	bool grab();

	cv::Mat image();
	cv::Mat depth();

private:
	Impl *impl_;
};