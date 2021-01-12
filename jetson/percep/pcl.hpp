#if OBSTACLE_DETECTION
#pragma once

#include "perception.hpp"
#include <pcl/common/common_headers.h>


/* --- Compare Line Class --- */
//Functor that indicates where a point is in
//relation to a line in 2D space
class compareLine {
public:
    double m;
    int b;
    
    compareLine(double slope_in, int b_in) : m(slope_in), b(b_in){}

    //Returns 1 if point is above line, 0 if on, -1 if below
    int operator()(int x, int y) {
        double yc = x*m+b;
        if(y > yc)
            return 1;
        else if (y == yc)
            return 0;
        else
            return -1; 
    }
};

class PCL {
    public:

    //Constructor
    PCL(const rapidjson::Document &config) : 
        bearing{0}, distance{0}, detected{false},
        pt_cloud_ptr{new pcl::PointCloud<pcl::PointXYZRGB>},  mRoverConfig{config} {
        double PT_CLOUD_WIDTH= mRoverConfig["pt_cloud"]["pt_cloud_width"].GetInt();
        double PT_CLOUD_HEIGHT =  mRoverConfig["pt_cloud"]["pt_cloud_height"].GetInt();


        #if ZED_SDK_PRESENT
        sl::Resolution cloud_res = sl::Resolution(PT_CLOUD_WIDTH, PT_CLOUD_HEIGHT);
        cloudArea = cloud_res.area();
        #else
        cloudArea = PT_CLOUD_WIDTH*PT_CLOUD_HEIGHT;
        #endif

    };

    const rapidjson::Document mRoverConfig
    double bearing;
    double distance;
    bool detected;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud_ptr;
    int cloudArea;

    private:
        //reference to config file
        const rapidjson::Document& mRoverConfig;

        //Filters points with values beyond certain threshold
        void PassThroughFilter();
        
        //Clusters nearby points to reduce total number of points
        void DownsampleVoxelFilter();
        
        //Finds the ground plane
        void RANSACSegmentation(string type);
        
        //Clusters nearby points into large obstacles
        void CPUEuclidianClusterExtraction(std::vector<pcl::PointIndices> &cluster_indices);
        
        //Finds the four corners of the clustered obstacles
        void FindInterestPoints(std::vector<pcl::PointIndices> &cluster_indices, std::vector<std::vector<int>> &interest_points);
        
        //Finds a clear path given the obstacle corners
        double FindClearPath(std::vector<std::vector<int>> interest_points,
                           shared_ptr<pcl::visualization::PCLVisualizer> viewer);

        //Determines whether the input path is obstructed
        bool CheckPath(std::vector<std::vector<int>> interest_points,
               shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               std::vector<int> &obstacles, compareLine leftLine, compareLine rightLine);

    public:
        //Main function that runs the above 
        obstacle_return pcl_obstacle_detection(shared_ptr<pcl::visualization::PCLVisualizer> viewer);

        //Creates a point cloud visualizer
        shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer();

        //Cleares and resizes cloud for new data
        void update();
};

#endif