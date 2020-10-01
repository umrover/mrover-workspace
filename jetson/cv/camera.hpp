#pragma once

#include <opencv2/opencv.hpp>

class Camera {
private:
	class Impl;
public:
	Camera();
	~Camera();

	bool grab();
	//void deleteZed();

	cv::Mat image();
	cv::Mat depth();

	void record_ar_init();
	void record_ar(Mat rgb);

private:
	Impl *impl_;

  	//Video Stuff
  	pair<Tag, Tag> tp;
  	TagDetector d1;

};