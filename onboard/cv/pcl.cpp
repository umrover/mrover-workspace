#include "perception.hpp"

/* --- Pass Through Filter --- */
//Filters out all points with z values that aren't within a threshold
//Z values are depth values in mm
//Source: https://rb.gy/kkyi80
void PassThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("PassThroughFilter");
    #endif

    pcl::PassThrough<pcl::PointXYZRGB> pass;
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
//Source: https://rb.gy/2ybg8n
void DownsampleVoxelFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("VoxelFilter");
    #endif

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (pt_cloud_ptr);
    sor.setLeafSize(20.0f, 20.0f, 20.0f);
    sor.filter (*pt_cloud_ptr);
}

/* --- RANSAC Plane Segmentation Blue --- */
//Picks three random points in point cloud
//Counts how many points lie on or near the plane made by these three
//If the number of points in the plane (score) is greater than 
//some threshold then a valid plane has been found
//Colors all points in this plane blue or
//removes points completely from point cloud
//Source: https://rb.gy/zx6ojh
void RANSACSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, string type) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("RANSACSegmentation");
    #endif

    //Creates instance of RANSAC Algorithm
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
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
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    
    seg.setInputCloud(pt_cloud_ptr);
    seg.segment(*inliers, *coefficients);

    if(type == "blue"){
        for(int i = 0; i < (int)inliers->indices.size(); i++){
        pt_cloud_ptr->points[inliers->indices[i]].r = 0;
        pt_cloud_ptr->points[inliers->indices[i]].g = 0;
        pt_cloud_ptr->points[inliers->indices[i]].b = 255;
        }
    }
    else {
        //Creates object that filters out identified points
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(pt_cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(true); //Controls whether chosen indices are retained or removed
        extract.filter(*pt_cloud_ptr);
    }
    
}

/* --- Euclidian Cluster Extraction --- */
//Creates a KdTree structure from point cloud
//Use this tree to traverse point cloud and create vector of clusters
//Return vector of clusters
//Source: https://rb.gy/qvjati
void EuclidianClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("Cluster Extraction");
    #endif

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (pt_cloud_ptr);
    
    //Extracts clusters using nearet neighbors search
    std::vector<pcl::PointIndices> cluster_indices; //PointIndices holds all indices in one cluster
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (40); // 40 mm radius per point
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (pt_cloud_ptr);
    ec.extract (cluster_indices);

    //Colors all clusters
    #if PERCEPTION DEBUG
        std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            {
                if(j%3) {
                    pt_cloud_ptr->points[*pit].r = 100+j*15;
                    pt_cloud_ptr->points[*pit].g = 0;
                    pt_cloud_ptr->points[*pit].b = 0;
                }
                else if(j%2) {
                    pt_cloud_ptr->points[*pit].r = 0;
                    pt_cloud_ptr->points[*pit].g = 100+j*15;
                    pt_cloud_ptr->points[*pit].b = 0;
                }
                else {
                    pt_cloud_ptr->points[*pit].r = 0;
                    pt_cloud_ptr->points[*pit].g = 0;
                    pt_cloud_ptr->points[*pit].b = 100+j*15;
                }
            }
            j++;
        }
    #endif

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
    RANSACSegmentation(pt_cloud_ptr, "remove");
    EuclidianClusterExtraction(pt_cloud_ptr);
    return result;
}