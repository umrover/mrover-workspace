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
    pass.setFilterLimits(0.0,2000.0);
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
void CPUEuclidianClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr,  
                                    std::vector<pcl::PointIndices> &cluster_indices) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("CPU Cluster Extraction");
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


/* --- Check Path --- */
//Returns T or F based on whether or not the input path is obstructed
//If it is obstructed returns false
//The path is constructed using the left x value and right x value of
//the furthest points on the path
bool CheckPath(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                        std::vector<std::vector<int>> interest_points,
                      shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    std::vector<int> &obstacles, compareLine leftLine, compareLine rightLine){
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("Check Path");
    #endif

    bool end = true;
    
    //Iterate through interest points
    for(auto cluster : interest_points) {
        for (auto index : cluster) {
            if(leftLine(pt_cloud_ptr->points[index].z,pt_cloud_ptr->points[index].x) >= 0  && 
               rightLine(pt_cloud_ptr->points[index].z,pt_cloud_ptr->points[index].x) <=0){
                end = false;
            
                //Check if obstacles is initialized
                if(obstacles.size() == 0){
                    obstacles.push_back(index);
                    obstacles.push_back(index);
                }
                //Check if leftmost interest point in rover path
                else if(pt_cloud_ptr->points[index].x < pt_cloud_ptr->points[obstacles.at(0)].x)
                    obstacles.at(0) = index;
                //Check if rightmost interest point in rover path
                else if(pt_cloud_ptr->points[index].x > pt_cloud_ptr->points[obstacles.at(1)].x)
                    obstacles.at(1) = index;

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
    pt1.x = leftLine.b;
    pt1.y = 0;
    pt1.z = 0;
    pcl::PointXYZRGB pt2;
    pt2.x = rightLine.b;
    pt2.y = 0;
    pt2.z = 0;
    pcl::PointXYZRGB pt3(pt1);
    pt3.z=7000;
    pt3.x=leftLine.m*pt3.z+leftLine.b;
    pcl::PointXYZRGB pt4(pt2);
    pt4.z=7000;
    pt4.x=rightLine.m*pt4.z+rightLine.b;
    
    
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

/* --- Find Clear Path --- */
//Returns the angle to a clear path
double FindClearPath(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                            std::vector<std::vector<int>> interest_points,
                      shared_ptr<pcl::visualization::PCLVisualizer> viewer, 
                      obstacle_return & result) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("Find Clear Path");
    #endif

    int buffer = 50;

    std::vector<int> obstacles; //X and Y values of the left and rightmost obstacles in path

    //Check Center Path
    if(CheckPath(pt_cloud_ptr, interest_points, viewer, obstacles, 
        compareLine(0,CENTERX-HALF_ROVER), compareLine(0,CENTERX+HALF_ROVER))){
        return 0;
    }
    
    //Initialize base cases outside of scope
    vector<int> centerObstacles = {obstacles.at(0), obstacles.at(1)};
    double newAngle = 0;
    double newSlope = 0;

    // create left and right angle vlaue that default at 360 degrees
    double leftAngle = 360;
    double rightAngle = 360;

    //If Center Path is blocked check left until have to turn too far
    while(newAngle > -70 && newSlope <= 0){
        
        //Declare Stuff
        double x = pt_cloud_ptr->points[obstacles.at(0)].x;
        double z = pt_cloud_ptr->points[obstacles.at(0)].z;
        result.distance = z;
        double zOffset = 0;
        double xOffset = 0;
        
        if(x <= 0){
            //Calculate angle from X axis, slope of line, and length of line to left obstacle
            double slope = x/z;
            double length = sqrt(x*x+z*z);
            double angleOffset = acos(z/length)*180/PI;

            //Calculate angle from X axis and length of line that is half rover distance from obstacle
            double interior = asin((HALF_ROVER+buffer)/length)*180/PI;
            double adjacent = 90-fabs(interior)-fabs(angleOffset);
            double key = 90-fabs(adjacent);
            zOffset = sin(key*PI/180)*(HALF_ROVER+buffer);
            xOffset = cos(key*PI/180)*(HALF_ROVER+buffer);
        }

        else{
            //Calculate angle from X axis, slope of line, and length of line to left obstacle
            double slope = x/z;
            double length = sqrt(x*x+z*z);
            double angleOffset = acos(z/length)*180/PI;

            //Calculate angle new point will be from X axis
            double interior = asin((HALF_ROVER+buffer)/length)*180/PI;
            double key = 90-(fabs(interior)-fabs(angleOffset));
            xOffset = sin(key*PI/180)*(HALF_ROVER+buffer);
            zOffset = cos(key*PI/180)*(HALF_ROVER+buffer);
        }

        //Find points on line of new path
        double newPathz = z-fabs(zOffset);
        double newPathx = x-fabs(xOffset);
        newSlope = newPathx/newPathz;
        newAngle = atan(newPathx/newPathz)*180/PI;

        //Create left and right lines based on new path
        double shiftX = HALF_ROVER/sin((90-newAngle)*PI/180);

        compareLine leftLine (newSlope,-shiftX);
        compareLine rightLine (newSlope, shiftX);

        obstacles.clear();
        if(CheckPath(pt_cloud_ptr, interest_points, viewer, obstacles, leftLine, rightLine))
        {
            #if PERCEPTION_DEBUG
            std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FOUND NEW PATH AT: "<<newAngle<<std::endl;
            #endif
            leftAngle = newAngle;
            break;
        }
            
    }

    //Reset global variables
    obstacles = {0, 0};
    obstacles.at(0) = centerObstacles.at(0);
    obstacles.at(1) = centerObstacles.at(1);
    newAngle = 0;
    newSlope = 0;
    
    int i = 0; //Sets max iterations
    //If have to turn too far left check right
    while(newAngle < 70 && newSlope >= 0 && i < 10){
        i++;
       
       //Declare Stuff
        double x = pt_cloud_ptr->points[obstacles.at(1)].x;
        double z = pt_cloud_ptr->points[obstacles.at(1)].z;
        double zOffset = 0;
        double xOffset = 0;
        
        if(x >= 0){
            //Calculate angle from X axis, slope of line, and length of line to left obstacle
            double slope = x/z;
            double length = sqrt(x*x+z*z);
            double angleOffset = acos(z/length)*180/PI;
            
            //Calculate angle from X axis and length of line that is half rover distance from obstacle
            double interior = asin((HALF_ROVER+buffer)/length)*180/PI;
            double adjacent = 90-fabs(interior)-fabs(angleOffset);
            double compliment = 90-fabs(interior);
            double key = 180-adjacent-compliment-fabs(interior);
            zOffset = sin(key*PI/180)*(HALF_ROVER+buffer);
            xOffset = cos(key*PI/180)*(HALF_ROVER+buffer);
            }

        else{
            //Calculate angle from X axis, slope of line, and length of line to left obstacle
            double slope = x/z;
            double length = sqrt(x*x+z*z);
            double angleOffset = acos(z/length)*180/PI;

            //Calculate angle new point will be from X axis
            double interior = asin((HALF_ROVER+buffer)/length)*180/PI;
            double key = 90-(fabs(interior)-fabs(angleOffset));
            xOffset = sin(key*PI/180)*(HALF_ROVER+buffer);
            zOffset = cos(key*PI/180)*(HALF_ROVER+buffer);
        }
        
        //Find points on line of new path
        double newPathz = z-fabs(zOffset);
        double newPathx = x+fabs(xOffset);
        newSlope = newPathx/newPathz;
        newAngle = atan(newPathx/newPathz)*180/PI;

        //Create left and right lines based on new path
        double shiftX = HALF_ROVER/sin((90-newAngle)*PI/180);

        compareLine leftLine (newSlope,-shiftX);
        compareLine rightLine (newSlope, shiftX);

        obstacles.clear();
        if(CheckPath(pt_cloud_ptr, interest_points, viewer, obstacles, leftLine, rightLine)){
            #if PERCEPTION_DEBUG
            std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FOUND NEW PATH AT: "<<newAngle<<std::endl;
            #endif
            rightAngle =  newAngle;
            break;
        }

    }

    //If there is no clear path both ways return an impossible number
    if(rightAngle == 360 && leftAngle == 360) {
        return 360;
    }

    //Take the absolute value of the left angle to compare
    double leftAngleAbs = abs(leftAngle);

    //Return the smallest angle (left if equal)
    if(rightAngle < leftAngleAbs) {
        return rightAngle;
    }
        
    return leftAngle;
}

/* --- Main --- */
//This is the main point cloud processing function
//It returns the bearing the rover should traverse
//This function is called in main.cpp
obstacle_return pcl_obstacle_detection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pt_cloud_ptr, 
                                                shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
    obstacle_return result;
    PassThroughFilter(pt_cloud_ptr);
    DownsampleVoxelFilter(pt_cloud_ptr);
    RANSACSegmentation(pt_cloud_ptr, "remove");
    std::vector<pcl::PointIndices> cluster_indices;
    CPUEuclidianClusterExtraction(pt_cloud_ptr, cluster_indices);
    std::vector<std::vector<int>> interest_points(cluster_indices.size(), vector<int> (4));
    FindInterestPoints(cluster_indices, pt_cloud_ptr, interest_points);
    result.bearing = FindClearPath(pt_cloud_ptr, interest_points, viewer, result);  
    return result;
}