#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <sl/Camera.hpp>

#include "pcl.hpp"


using namespace std;


void loadPCD(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pc, std::string full_path) {
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (full_path, *pc) == -1){ //* load the file 
    	std::cerr << "Couldn't read file\n";
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n"); 
  	}
	cerr << "> Loaded point cloud of " << pc->width << "*" << pc->height << endl;

	//Reduce resolution
  /*
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGB>(pc->width/2, pc->height/2));
	for(size_t y = 0; y < pc->height/2; y++) {
			for(size_t x = 0; x < pc->width/2; x++) {
				pcl::PointXYZRGB p = pc->at(x*2, y*2);
				downsampled->at(x, y) = p;
			}
	}
	pc = downsampled; */
}

inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t *) & colorIn;
    unsigned char *color_uchar = (unsigned char *) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float *> (&color_uint);
}

void ZedToPcl(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & p_pcl_point_cloud, sl::Mat zed_cloud) {
  sl::Mat zed_cloud_cpu;
  zed_cloud.copyTo(zed_cloud_cpu,  sl::COPY_TYPE::GPU_CPU);
 
  p_pcl_point_cloud->points.resize(zed_cloud.getResolution().area());

	
  float* p_data_cloud = zed_cloud_cpu.getPtr<float>();
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

void PclToZed(sl::Mat &zed, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pcl) {
	sl::Resolution cloudRes(pcl->width, pcl->height);

	for(size_t y = 0; y < pcl->height; y++) {
		for(size_t x = 0; x < pcl->width; x++) {
			pcl::PointXYZRGB p = pcl->at(x, y);

			// unpack rgb into r/g/b
			std::uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
			std::uint8_t r = (rgb >> 16) & 0x0000ff;
			std::uint8_t g = (rgb >> 8)  & 0x0000ff;
			std::uint8_t b = (rgb)       & 0x0000ff;
			
			std::uint32_t bgr = ((std::uint32_t)b << 16 | (std::uint32_t)g << 8 | (std::uint32_t)r);

			//float color = *(float *) &rgb;
			float color = *(float *) &bgr;
			//we will need an RGB Color conversion here
			zed.setValue(x, y, sl::float4(p.x, p.y, p.z, color));
		}
	}

	//Upload to device
	zed.updateGPUfromCPU();
}

shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pt_cloud_ptr) {
    // Open 3D pclViewer and add point clousl::ERROR_CODE_ 
    shared_ptr<pcl::visualization::PCLVisualizer> pclViewer(
      new pcl::visualization::PCLVisualizer("PCL 3D pclViewer")); //This is a smart pointer so no need to worry ab deleteing it
    pclViewer->setBackgroundColor(0.87, 0.9, 0.91);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pt_cloud_ptr);
    pclViewer->addPointCloud<pcl::PointXYZRGB>(pt_cloud_ptr, rgb);
    pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5.5);
    pclViewer->addCoordinateSystem(1.0);
    pclViewer->initCameraParameters();
    pclViewer->setCameraPosition(0,0,-800,0,-1,0);
    return (pclViewer);
}
