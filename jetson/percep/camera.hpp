#pragma once
#include <opencv2/opencv.hpp>
#include "perception.hpp"


#if OBSTACLE_DETECTION
	#include <pcl/common/common_headers.h>
#endif

#include "artag_detector.hpp"

class Camera {
private:
	class Impl;
	Impl *impl_;
	std::string rgb_foldername;
	std::string depth_foldername;
	//Video Stuff
  	std::pair<Tag, Tag> tp;
  	TagDetector d1;
	cv::VideoWriter vidWrite;
	
public:
	Camera();
	~Camera();

	bool grab();

	cv::Mat image();
	cv::Mat depth();
	
	#if OBSTACLE_DETECTION
	void getDataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
	#endif

	void record_ar_init();
	void record_ar(cv::Mat rgb);
	void record_ar_finish();

	void disk_record_init();
	void write_curr_frame_to_disk(cv::Mat rgb, cv::Mat depth, int counter);

};

