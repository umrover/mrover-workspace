#include "camera.hpp"
#include "perception.hpp"

#if OBSTACLE_DETECTION
  #include <pcl/common/common_headers.h>
#endif

#if ZED_SDK_PRESENT

#pragma GCC diagnostic ignored "-Wreorder" //Turns off warning checking for sl lib files

#include <sl/Camera.hpp>
#include <cassert>

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

	cv::Mat image();
	cv::Mat depth();

  #if OBSTACLE_DETECTION
  void dataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
  #endif
  
  
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

Camera::Impl::~Impl() {
  this->depth_zed_.free(sl::MEM::CPU);
  this->image_zed_.free(sl::MEM::CPU);
	this->zed_.close();
}

#if OBSTACLE_DETECTION
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
#endif

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

  #if AR_DETECTION
  cv::Mat image();
  cv::Mat depth();
  #endif

  #if OBSTACLE_DETECTION
  void dataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);
  void pcl_write(const cv::String &filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);

  #endif

  void disk_record_init();
  void write_curr_frame_to_disk(cv::Mat rgb, cv::Mat depth, int counter);

private:
  std::vector<std::string> img_names;
  std::vector<std::string> pcd_names;

  size_t idx_curr_img;
  size_t idx_curr_pcd_img;

  std::string path;
  std::string rgb_path;
  DIR * rgb_dir;
  std::string depth_path;
  DIR * depth_dir;
  std::string pcd_path;
  DIR * pcd_dir;
};

Camera::Impl::~Impl() {
  closedir(rgb_dir);
  closedir(depth_dir);
  closedir(pcd_dir);
}

Camera::Impl::Impl() {
  
  std::cout<<"Please input the folder path (there should be a rgb and depth existing in this folder): ";
  std::cin>>path;
  #if AR_DETECTION
  rgb_path = path + "/rgb";
  depth_path = path + "/depth";
  rgb_dir = opendir(rgb_path.c_str() );
  depth_dir = opendir(depth_path.c_str() );
  if ( NULL==rgb_dir || NULL==depth_dir) {
    return;
  }

  #endif  

  #if OBSTACLE_DETECTION
  pcd_path = path + "/pcl";
  pcd_dir = opendir(pcd_path.c_str() );
  if (NULL==pcd_dir) {
    std::cerr<<"Input folder not exist\n";    
    return;
  }
  #endif
  

  // get the vector of image names, jpg/png for rgb files, .exr for depth files
  // we only read the rgb folder, and assume that the depth folder's images have the same name
  struct dirent *dp = NULL;
  #if AR_DETECTION
  
  std::unordered_set<std::string> img_tails({".exr", ".jpg"}); // for rgb
  std::cout<<"Read image names\n";
  do {
    errno = 0;
    if ((dp = readdir(rgb_dir)) != NULL) {
      std::string file_name(dp->d_name);
      std::cout<<"file_name is "<<file_name<<std::endl;
      if (file_name.size() < 5) continue; // the lengh of the tail str is at least 4
      std::string tail = file_name.substr(file_name.size()-4, 4);
      std::string head = file_name.substr(0, file_name.size()-4);
      if (img_tails.find(tail) != img_tails.end()) {
        img_names.push_back(file_name);
      }
    }
  } while  (dp != NULL);
  std::sort(img_names.begin(), img_names.end());
  std::cout<<"Read image names complete\n";
  idx_curr_img = 0;

#endif

#if OBSTACLE_DETECTION
 dp = NULL;

 std::cout<<"Read PCL image names\n";
  
  do{
    
    if ((dp = readdir(pcd_dir)) != NULL) {
      
      std::string file_name(dp->d_name);
      std::cout<<"file_name is "<<file_name<<std::endl;
      
      // the lengh of the tail str is at least 4
      if (file_name.size() < 5) continue;

      pcd_names.push_back(file_name);
      
    }

 } while (dp != NULL);

  std::sort(pcd_names.begin(), pcd_names.end());
  std::cout<<"Read .pcd image names complete\n";
  idx_curr_pcd_img = 0;

#endif
}

bool Camera::Impl::grab() {

  bool end = true;

  #if AR_DETECTION
  idx_curr_img++;
  if (idx_curr_img >= img_names.size()) {
    std::cout<<"Ran out of images\n";
    end = false;
  }
  #endif

  #if OBSTACLE_DETECTION
  idx_curr_pcd_img++;
  if (idx_curr_pcd_img >= pcd_names.size()-2) {
    std::cout<<"Ran out of images\n";
    end = false;  
  }
  #endif
  if(!end){
    exit(1);
  }
  return end;
}

#if AR_DETECTION
cv::Mat Camera::Impl::image() {
  std::string full_path = rgb_path + std::string("/") + (img_names[idx_curr_img]);
cerr << img_names[idx_curr_img] << "\n";
cerr << full_path << "\n";
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

void Camera::record_ar_init() {
  //initializing ar tag videostream object
  std::pair<Tag, Tag> tp;
  TagDetector d1;

  Mat depth_img = depth();
  Mat rgb;
  Mat src = image();

  tp = d1.findARTags(src, depth_img, rgb);
  Size fsize = rgb.size();

  time_t now = time(0);
  char* ltm = ctime(&now);
  string timeStamp(ltm);

  string s = "artag_number_" + timeStamp + ".avi";

  vidWrite =  VideoWriter(s, VideoWriter::fourcc('M','J','P','G'),10,fsize,true);

  if(vidWrite.isOpened() == false)
  {
	  cerr << "ar record didn't open\n";
	  exit(1);
  }
}

void Camera::record_ar(Mat rgb) {
  vidWrite.write(rgb);
}

void Camera::record_ar_finish() {
  vidWrite.release();
}
#endif


//Reads the point data cloud p_pcl_point_cloud
#if OBSTACLE_DETECTION
void Camera::Impl::dataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud){
 
 //Read in image names
 std::string pcd_name = pcd_names[idx_curr_pcd_img];
 std::string full_path = pcd_path + std::string("/") + pcd_name;
  //Load in the file  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (full_path, *p_pcl_point_cloud) == -1){ //* load the file 
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n"); 
  }
}
#endif

#endif

Camera::Camera() : impl_{new Camera::Impl}, rgb_foldername{""},
                   depth_foldername{""}, pcl_foldername{""} {}

Camera::~Camera() {
	delete this->impl_;
}

bool Camera::grab() {
	return this->impl_->grab();
}

#if AR_DETECTION
cv::Mat Camera::image() {
	return this->impl_->image();
}

cv::Mat Camera::depth() {
	return this->impl_->depth();
}
#endif

#if OBSTACLE_DETECTION
void Camera::getDataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud) {
  this->impl_->dataCloud(p_pcl_point_cloud);
}
#endif

#if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION

// creates and opens folder to write to
void Camera::disk_record_init() {
    //defining directories to write to
    rgb_foldername = DEFAULT_ONLINE_DATA_FOLDER "rgb/";
    depth_foldername = DEFAULT_ONLINE_DATA_FOLDER "depth/";
    string mkdir_rgb =  std::string("mkdir -p ") + rgb_foldername;
    string mkdir_depth =  std::string("mkdir -p ") + depth_foldername;
    pcl_foldername = DEFAULT_ONLINE_DATA_FOLDER "pcl/";
    string mkdir_pcl = std::string("mkdir -p ") + pcl_foldername;

    //creates new folder in the system
    if (-1 == system(mkdir_pcl.c_str()) || -1 == system( mkdir_rgb.c_str()) || -1 == system(mkdir_depth.c_str())) 
    {
      exit(1);
    }
}

//writes the Mat to a file

//Writes point cloud data to data folder specified in build tag 
void pcl_write(const cv::String &filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud){
  std::cout << "name of path is: " << filename << endl;
  try{ pcl::io::savePCDFileASCII (filename, *p_pcl_point_cloud); }
  catch (pcl::IOException &e){cerr << e.what();}
}

void Camera::write_curr_frame_to_disk(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud, int counter){
    string fileName = to_string(counter / FRAME_WRITE_INTERVAL);
    while(fileName.length() < 4){
      fileName = '0'+fileName;
    }

    pcl_write(pcl_foldername + fileName + std::string(".pcd"), p_pcl_point_cloud);
    cv::imwrite(rgb_foldername +  fileName + std::string(".jpg"), rgb );
    cv::imwrite(depth_foldername +  fileName + std::string(".exr"), depth );
}

#endif
