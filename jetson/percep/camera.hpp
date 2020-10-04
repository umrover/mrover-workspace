#pragma once
#include <opencv2/opencv.hpp>
#include "artag_detector.hpp"

#if OBSTACLE_DETECTION
	#include <pcl/common/common_headers.h>
#endif


class Camera {
private:
	class Impl;
public:
	Camera();
	~Camera();

	bool grab();

	cv::Mat image();
	cv::Mat depth();

	void record_ar_init();
	void record_ar(cv::Mat rgb);

	
	
	#if OBSTACLE_DETECTION
	void getDataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
	#endif
	
private:
	Impl *impl_;

  	//Video Stuff
  	std::pair<Tag, Tag> tp;
  	TagDetector d1;

};