#pragma once
#include <pcl/common/common_headers.h>
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
	void getDataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);

private:
	Impl *impl_;
};