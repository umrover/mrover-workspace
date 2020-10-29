#pragma once
#include <opencv2/opencv.hpp>
#include "perception.hpp"

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
	
	
//	#if OBSTACLE_DETECTION
	void getDataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
//	#endif
	
private:
	Impl *impl_;
};