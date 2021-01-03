#if OBSTACLE_DETECTION
#pragma once

#include "perception.hpp"
#include <pcl/common/common_headers.h>

/* --- Compare Line Class --- */
/**
\brief Functor that indicates where a point is in
relation to a line in 2D space
recieves angle off y axis
*/
class compareLine {
public:
    int xIntercept;
    double slope;
    
    compareLine(double angle_in, int xInt_in) : xIntercept{xInt_in}, 
                        slope{tan(angle_in*PI/180)} {
                            if(slope != 0) {
                                slope = 1/slope;
                            }
                        }

    //Returns 1 if point is right of line, 0 if on, -1 if left of line
    int operator()(int x, int y) {
        
        //Make sure don't divide by 0
        double xc = xIntercept; //x calculated
        if(slope != 0) {
            xc = y/slope+xIntercept; //Find x value on line with same y value as input point
        }
            
        //Point is right of the line
        if(x > xc) {
            return 1;
        }
        //Point is on the line
        else if (x == xc) {
            return 0;
        }
        //Point is left of the line
        else {
            return -1;
        } 
    }
};

class PCL {
    public:

    //Constructor
    PCL() : 
        bearing{0}, distance{0}, detected{false},
        pt_cloud_ptr{new pcl::PointCloud<pcl::PointXYZRGB>} {

        #if ZED_SDK_PRESENT
        sl::Resolution cloud_res = sl::Resolution(PT_CLOUD_WIDTH, PT_CLOUD_HEIGHT);
        cloudArea = cloud_res.area();
        #else
        cloudArea = PT_CLOUD_WIDTH*PT_CLOUD_HEIGHT;
        #endif

    };

    double bearing;
    double distance;
    bool detected;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud_ptr;
    int cloudArea;

    private:

        //Filters points with values beyond certain threshold
        void PassThroughFilter(const std::string axis, const double upperLimit);
        
        //Clusters nearby points to reduce total number of points
        void DownsampleVoxelFilter();
        
        //Finds the ground plane
        void RANSACSegmentation(string type);
        
        //Clusters nearby points into large obstacles
        void CPUEuclidianClusterExtraction(std::vector<pcl::PointIndices> &cluster_indices);
        
        //Finds the four corners of the clustered obstacles
        void FindInterestPoints(std::vector<pcl::PointIndices> &cluster_indices, std::vector<std::vector<int>> &interest_points);
        
        //Finds a clear path given the obstacle corners
        double FindClearPath(const std::vector<std::vector<int>> &interest_points,
                           shared_ptr<pcl::visualization::PCLVisualizer> viewer);

        //Determines whether the input path is obstructed
        bool CheckPath(const std::vector<std::vector<int>> &interest_points,
               shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               std::vector<int> &obstacles, compareLine leftLine, compareLine rightLine);
        
        /**
        \brief Determines angle off center a clear path can be found
        \param direction: given 0 finds left clear path given 1 find right clear path
        */
        double getAngleOffCenter(int buffer, int direction, const std::vector<std::vector<int>> &interest_points,
                    shared_ptr<pcl::visualization::PCLVisualizer> viewer, std::vector<int> &obstacles);

    public:
        //Main function that runs the above 
        void pcl_obstacle_detection(shared_ptr<pcl::visualization::PCLVisualizer> viewer);

        //Creates a point cloud visualizer
        shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer();

        //Cleares and resizes cloud for new data
        void update();
};

#endif
