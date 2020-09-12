#include "camera.hpp"
#include "perception.hpp"

#if ZED_SDK_PRESENT

#pragma GCC diagnostic ignored "-Wreorder" //Turns off warning checking for sl lib files

#include <sl/Camera.hpp>
#include <cassert>
#include <pcl/common/common_headers.h>

#pragma GCC diagnostic pop
//Class created to implement all Camera class' functions
//Abstracts away details of using Stereolab's camera interface
//so we can just use a simple custom one
//Also allows us to not be restricted to using the ZED for testing,
//but can use sample images for testing
class Camera::Impl {
public:
	Impl();
    ~Impl();
	bool grab();
	//void deleteZed();

	cv::Mat image();
	cv::Mat depth();
  void dataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
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
	init_params.camera_resolution = sl::RESOLUTION::HD720; // default: 720p
	init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
	init_params.coordinate_units = sl::UNIT::METER;
	init_params.camera_fps = 15;
	// TODO change this below?

	assert(this->zed_.open() == sl::ERROR_CODE::SUCCESS);
  
  //Parameters for Positional Tracking
  init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
  
  this->zed_.setCameraSettings(sl::VIDEO_SETTINGS::BRIGHTNESS, 1);

	this->runtime_params_.confidence_threshold = THRESHOLD_CONFIDENCE;
	std::cout<<"ZED init success\n";
	this->runtime_params_.sensing_mode = sl::SENSING_MODE::STANDARD;

	this->image_size_ = this->zed_.getCameraInformation().camera_resolution;
	this->image_zed_.alloc(this->image_size_.width, this->image_size_.height,
						   sl::MAT_TYPE::U8_C4);
	this->image_ = cv::Mat(
		this->image_size_.height, this->image_size_.width, CV_8UC4,
		this->image_zed_.getPtr<sl::uchar1>(sl::MEM::CPU));
	this->depth_zed_.alloc(this->image_size_.width, this->image_size_.height,
		                   sl::MAT_TYPE::F32_C1);
	this->depth_ = cv::Mat(
		this->image_size_.height, this->image_size_.width, CV_32FC1,
		this->depth_zed_.getPtr<sl::uchar1>(sl::MEM::CPU));
}

bool Camera::Impl::grab() {
  return this->zed_.grab() == sl::ERROR_CODE::SUCCESS;
}

cv::Mat Camera::Impl::image() {
	this->zed_.retrieveImage(this->image_zed_, sl::VIEW::LEFT, sl::MEM::CPU,
							 this->image_size_);
	return this->image_;
}

cv::Mat Camera::Impl::depth() {

    this->zed_.retrieveMeasure(this->depth_zed_, sl::MEASURE::DEPTH,  sl::MEM::CPU,  this->image_size_);
	return this->depth_;
}

//This function convert a RGBA color packed into a packed RGBA PCL compatible format
inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}

void Camera::Impl::dataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & p_pcl_point_cloud) {
  //Might need an overloaded assignment operator
  
  //Grab ZED Depth Image
  sl::Resolution cloud_res(p_pcl_point_cloud->width, p_pcl_point_cloud->height);
  sl::Mat data_cloud;
  this->zed_.retrieveMeasure(data_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, cloud_res);
  
  //Populate Point Cloud
  float *p_data_cloud = data_cloud.getPtr<float>();
  int index = 0;
  for (auto &it : p_pcl_point_cloud->points) {
    float X = p_data_cloud[index];
    if (!isValidMeasure(X)) // Checking if it's a valid point
        it.x = it.y = it.z = it.rgb = 0;
    else {
        it.x = X;
        it.y = p_data_cloud[index + 1];
        it.z = p_data_cloud[index + 2];
        it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
    }
    index += 4;
  }

} 

Camera::Impl::~Impl() {
  this->depth_zed_.free(sl::MEM::CPU);
  this->image_zed_.free(sl::MEM::CPU);
	this->zed_.close();

}


#else //if OFFLINE_TEST
#include <sys/types.h>
#include <dirent.h>
#include <string>
#include <errno.h>
#include <vector>
#include <unordered_set>
class Camera::Impl {
public:
  Impl();
  ~Impl();
  bool grab();
  cv::Mat image();
  cv::Mat depth();
private:
  std::vector<std::string> img_names;
  int idx_curr_img;

  std::string path;
  std::string rgb_path;
  DIR * rgb_dir;
  std::string depth_path;
  DIR * depth_dir;
};

Camera::Impl::~Impl() {
  closedir(rgb_dir);
  closedir(depth_dir);
}

Camera::Impl::Impl() {
  std::cout<<"Please input the folder path (there should be a rgb and depth existing in this folder): ";
  std::cin>>path;
  rgb_path = path + "/rgb";
  depth_path = path + "/depth";
  rgb_dir = opendir(rgb_path.c_str() );
  depth_dir = opendir(depth_path.c_str() );
  if ( NULL==rgb_dir || NULL==depth_dir ) {
    std::cerr<<"Input folder not exist\n";    
    return;
  }

  // get the vector of image names, jpg/png for rgb files, .exr for depth files
  // we only read the rgb folder, and assume that the depth folder's images have the same name
  struct dirent *dp = NULL;
  std::unordered_set<std::string> img_tails({".exr", ".jpg"}); // for rgb
  int img_tail_str_len = 4;
  std::cout<<"Read image names\n";
  do {
    errno = 0;
    if ((dp = readdir(rgb_dir)) != NULL) {
      std::string file_name(dp->d_name);
      std::cout<<"file_name is "<<file_name<<std::endl;
      if (file_name.size() < 5) continue; // the lengh of the tail str is at least 4
      std::string tail = file_name.substr(file_name.size()-4, 4);
      std::string head = file_name.substr(0, file_name.size()-4);
      if (img_tails.find(tail)!= img_tails.end()) {
        img_names.push_back(file_name);
      }
    }
  } while  (dp != NULL);
  std::sort(img_names.begin(), img_names.end());
  std::cout<<"Read image names complete\n";
  idx_curr_img = 0;
}

bool Camera::Impl::grab() {
  idx_curr_img++;
  if (idx_curr_img >= img_names.size()-2) {
    std::cout<<"Ran out of images\n";
    exit(1);
    return false;
  } else
    return true;
}


cv::Mat Camera::Impl::image() {
  std::string full_path = rgb_path + std::string("/") + (img_names[idx_curr_img]);

  cv::Mat img = cv::imread(full_path.c_str(), CV_LOAD_IMAGE_COLOR);
  if (!img.data){
    std::cerr<<"Load image "<<full_path<< " error\n";
  }
  return img;
}

cv::Mat Camera::Impl::depth() {
  std::string rgb_name = img_names[idx_curr_img];
  std::string full_path = depth_path + std::string("/") +
                          rgb_name.substr(0, rgb_name.size()-4) + std::string(".exr");
  std::cout<<full_path<<std::endl;
  cv::Mat img = cv::imread(full_path.c_str(), cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
  if (!img.data){
    std::cerr<<"Load image "<<full_path<< " error\n";
  }
  return img;
}

#endif

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

void Camera::getDataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud) {
  this->impl_->dataCloud(p_pcl_point_cloud);
}
