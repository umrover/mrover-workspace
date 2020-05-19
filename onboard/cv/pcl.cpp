#include <iostream>
#include "perception.hpp"
//Steps:
//Yoink Point Cloud
//Filter
//Post Process
//Return LCM's based on post processing

void filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud_ptr) {

}

advanced_obstacle_return pcl_obstacle_detection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud_ptr, int rover_width) {
    advanced_obstacle_return result {0};
    filter(pt_cloud_ptr);

    return result;
}