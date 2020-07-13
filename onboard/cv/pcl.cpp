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
    pass.setFilterLimits(0.0,4500.0);
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
void EuclidianClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr,  
                                    std::vector<pcl::PointIndices> &cluster_indices) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("Cluster Extraction");
    #endif

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (pt_cloud_ptr);
    
    //Extracts clusters using nearet neighbors search
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (40); // 40 mm radius per point
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (pt_cloud_ptr);
    ec.extract (cluster_indices);

    //Colors all clusters
    #if PERCEPTION_DEBUG
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

/* --- Find Interest Points --- */
//Finds the edges of each cluster by comparing x and y
//values of all points in the cluster to find desired ones
void FindInterestPoints(std::vector<pcl::PointIndices> &cluster_indices, 
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                        std::vector<std::vector<int>> &interest_points) {

    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("Find Interest Points");
    #endif

    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        std::vector<int>* curr_cluster = &interest_points[i];

        //Initialize interest points
        std::fill(curr_cluster->begin(), curr_cluster->end(), cluster_indices[i].indices[0]);
        
        //Order of interest points: 0=Up Left 1=Up Right 2=Low Right 3=Low Left
        for (auto index : cluster_indices[i].indices)
        {
            auto curr_point = pt_cloud_ptr->points[index];
            
            //Upper Left
            if(curr_point.x < pt_cloud_ptr->points[curr_cluster->at(0)].x && curr_point.y > pt_cloud_ptr->points[curr_cluster->at(0)].y)
                curr_cluster->at(0) = index;
            //Upper Right
            else if(curr_point.x > pt_cloud_ptr->points[curr_cluster->at(1)].x && curr_point.y > pt_cloud_ptr->points[curr_cluster->at(1)].y)
                curr_cluster->at(1) = index;
            //Low Left
            else if(curr_point.x < pt_cloud_ptr->points[curr_cluster->at(2)].x && curr_point.y < pt_cloud_ptr->points[curr_cluster->at(2)].y)
                curr_cluster->at(2) = index;
            //Low Right
            else if(curr_point.x > pt_cloud_ptr->points[curr_cluster->at(3)].x && curr_point.y < pt_cloud_ptr->points[curr_cluster->at(3)].y)
                curr_cluster->at(3) = index;
        }
    #if PERCEPTION_DEBUG
        for(auto interest_point : *curr_cluster)
        {
            pt_cloud_ptr->points[interest_point].r = 255;
            pt_cloud_ptr->points[interest_point].g = 255;
            pt_cloud_ptr->points[interest_point].b = 255;
        }
    #endif
    }

}

/* --- Check Center Path --- */
//Returns T or F based on whether or not the center path is obstructed
//If it is obstructed returns false
bool CheckCenterPath(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                              std::vector<std::vector<int>> interest_points,
                      shared_ptr<pcl::visualization::PCLVisualizer> viewer){
     #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("Check Center Path");
    #endif

    bool end = true;
    int centerx = PT_CLOUD_WIDTH/2;
    int halfRover = ROVER_W_MM/2;
    //Iterate through interest points
    for(auto cluster : interest_points) {
        for (auto index : cluster) {
            if(centerx-halfRover < pt_cloud_ptr->points[index].x && centerx+halfRover > pt_cloud_ptr->points[index].x){
                end = false;
                
                #if PERCEPTION_DEBUG
                //Make interest points orange if they are within rover path
                pt_cloud_ptr->points[index].r = 255;
                pt_cloud_ptr->points[index].g = 69;
                pt_cloud_ptr->points[index].b = 0;
                #endif
            }
        }
    }

    #if PERCEPTION_DEBUG
    //Project path in viewer
    pcl::PointXYZRGB pt1;
    pt1.x = centerx+halfRover;
    pt1.y = 0;
    pt1.z = 1000;
    pcl::PointXYZRGB pt2;
    pt2.x = centerx-halfRover;
    pt2.y = 0;
    pt2.z = 1000;
    pcl::PointXYZRGB pt3(pt1);
    pt3.z=7000;
    pcl::PointXYZRGB pt4(pt2);
    pt4.z=7000;
    
    if(end) {
        viewer->removeShape("l1");
        viewer->removeShape("l2");
        viewer->addLine(pt1,pt3,0,255,0,"l1");
        viewer->addLine(pt2,pt4,0,255,0,"l2");
    }
    else {
        viewer->removeShape("l1");
        viewer->removeShape("l2");
        viewer->addLine(pt1,pt3,255,0,0,"l1");
        viewer->addLine(pt2,pt4,255,0,0,"l2");
    }
    #endif
    
    return end;
}

/* --- Main --- */
//This is the main point cloud processing function
//It returns the bearing the rover should traverse
//This function is called in main.cpp
advanced_obstacle_return pcl_obstacle_detection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                                                shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
    advanced_obstacle_return result;
    PassThroughFilter(pt_cloud_ptr);
    DownsampleVoxelFilter(pt_cloud_ptr);
    RANSACSegmentation(pt_cloud_ptr, "remove");
    std::vector<pcl::PointIndices> cluster_indices;
    EuclidianClusterExtraction(pt_cloud_ptr, cluster_indices);
    std::vector<std::vector<int>> interest_points(cluster_indices.size(), vector<int> (4));
    FindInterestPoints(cluster_indices, pt_cloud_ptr, interest_points);
    if( CheckCenterPath(pt_cloud_ptr, interest_points, viewer))
        return result;
    else {
        return result;    
    }
}















/* --- Calculate Centroid --- */
//Calculates the center point of 4 points in space
std::pair<double, double> calcCentroid (int p1, int p2, int p3, int p4, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr) {
    std::pair<double, double> center;
    double centroid1x = (pt_cloud_ptr->points[p1].x + pt_cloud_ptr->points[p2].x +
                        pt_cloud_ptr->points[p4].x)/3;
    double centroid2x = (pt_cloud_ptr->points[p2].x + pt_cloud_ptr->points[p3].x +
                        pt_cloud_ptr->points[p4].x)/3;
    center.first = (centroid1x+centroid2x)/2;
    
    double centroid1y = (pt_cloud_ptr->points[p1].y + pt_cloud_ptr->points[p2].y +
                        pt_cloud_ptr->points[p4].y)/3;
    double centroid2y = (pt_cloud_ptr->points[p2].y + pt_cloud_ptr->points[p3].y +
                        pt_cloud_ptr->points[p4].y)/3;
    center.second = (centroid1x+centroid2x)/2;

    return center;
}

//Experimentation shows that all points are plotted in mm even in visualizer, so no need to calculate rover pix at a point
//46 inch = 1168.4mm