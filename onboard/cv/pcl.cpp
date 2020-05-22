#include <iostream>
#include "perception.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

using namespace pcl;
using namespace std;


/* --- Pass Through Filter --- */
//Filters out all points with z values that aren't within a threshold
//Z values are depth values in mm
void PassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr) {
    PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pt_cloud_ptr);
    pass.setFilterFieldName("z");
    //The z values for depth are in mm
    pass.setFilterLimits(0.0,4000.0);
    pass.filter(*pt_cloud_ptr);
}

/* --- Voxel Filter --- */
//Creates clusters given by the size of a leaf
//All points in a cluster are then reduced to a single point
//This point is the centroid of the cluster
void DownsampleVoxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr) {
    VoxelGrid<PointXYZRGB> sor;
    sor.setInputCloud (pt_cloud_ptr);
    sor.setLeafSize(30.0f, 30.0f, 30.0f);
    sor.filter (*pt_cloud_ptr);
}

/* --- Main --- */
//This is the main point cloud processing function
//It returns the bearing the rover should traverse
//This function is called in main.cpp
advanced_obstacle_return pcl_obstacle_detection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                                                                                int rover_width) {
    advanced_obstacle_return result {0};
    PassThroughFilter(pt_cloud_ptr);
    DownsampleVoxelFilter(pt_cloud_ptr);
    return result;
}