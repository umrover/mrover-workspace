#include <iostream>
#include "perception.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
using namespace pcl;
using namespace std;


/* --- Pass Through Filter --- */
//Filters out all points with z values that aren't within a threshold
//Z values are depth values in mm
void PassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr) {
    #if PERCEPTION_DEBUG
        ScopeTime t ("PassThroughFilter");
    #endif

    PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pt_cloud_ptr);
    pass.setFilterFieldName("z");
    //The z values for depth are in mm
    pass.setFilterLimits(0.0,7000.0);
    pass.filter(*pt_cloud_ptr);
}

/* --- Voxel Filter --- */
//Creates clusters given by the size of a leaf
//All points in a cluster are then reduced to a single point
//This point is the centroid of the cluster
void DownsampleVoxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr) {
    #if PERCEPTION_DEBUG
        ScopeTime t ("VoxelFilter");
    #endif

    VoxelGrid<PointXYZRGB> sor;
    sor.setInputCloud (pt_cloud_ptr);
    sor.setLeafSize(30.0f, 30.0f, 30.0f);
    sor.filter (*pt_cloud_ptr);
}

/* --- RANSAC Plane Segmentation Blue --- */
//Picks three random points in point cloud
//Counts how many points lie on or near the plane made by these three
//If the number of points in the plane (score) is greater than 
//some threshold then a valid plane has been found
//Colors all points in this plane blue or
//removes points completely from point cloud
void RANSACSegmentationColorBlue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, string type) {
    #if PERCEPTION_DEBUG
        ScopeTime t ("RANSACSegmentation");
    #endif

    //Creates instance of RANSAC Algorithm
    SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(400);
    seg.setDistanceThreshold(60); //Distance in mm away from actual plane a point can be
    // to be considered an inlier
    seg.setAxis(Eigen::Vector3f(0, 0, 1)); //Looks for a plane along the Z axis
    double segmentation_epsilon = 45; //Max degree the normal of plane can be from Z axis
    seg.setEpsAngle(pcl::deg2rad(segmentation_epsilon));

    //Objects where segmented plane is stored
    ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    PointIndices::Ptr inliers(new pcl::PointIndices());
    
    seg.setInputCloud(pt_cloud_ptr);
    seg.segment(*inliers, *coefficients);

    if(type == "blue"){
        for(int i = 0; i < (int)inliers->indices.size(); i++){
        pt_cloud_ptr->points[inliers->indices[i]].r = 0;
        pt_cloud_ptr->points[inliers->indices[i]].g = 0;
        pt_cloud_ptr->points[inliers->indices[i]].b = 255;
    }
    else {
        //Creates object that filters out identified points
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(pt_cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*pt_cloud_ptr);
    }
    }
    
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
    RANSACSegmentation(pt_cloud_ptr, "blue");
    return result;
}