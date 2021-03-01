#include "pcl.hpp"
#include "perception.hpp"

#if OBSTACLE_DETECTION

    //Constructor
    PCL::PCL(const rapidjson::Document &mRoverConfig) : 

        //Populate Constants from Config File
        MAX_FIELD_OF_VIEW_ANGLE{mRoverConfig["pt_cloud"]["max_field_of_view_angle"].GetInt()},
        PT_CLOUD_WIDTH{mRoverConfig["pt_cloud"]["pt_cloud_width"].GetInt()},
        PT_CLOUD_HEIGHT{mRoverConfig["pt_cloud"]["pt_cloud_height"].GetInt()},
        HALF_ROVER{mRoverConfig["pt_cloud"]["half_rover"].GetInt()},
        UP_BD_Z{mRoverConfig["pt_cloud"]["pass_through"]["upperBdZ"].GetDouble()},
        UP_BD_Y{mRoverConfig["pt_cloud"]["pass_through"]["upperBdY"].GetDouble()},
        LOW_BD{mRoverConfig["pt_cloud"]["pass_through"]["lowerBd"].GetDouble()},
        ROVER_W_MM{mRoverConfig["pt_cloud"]["rover_w_mm"].GetDouble()},
        LEAF_SIZE{mRoverConfig["pt_cloud"]["downsample_voxel_filter"].GetFloat()},
        MAX_ITERATIONS{mRoverConfig["pt_cloud"]["ransac"]["max_iterations"].GetInt()},
        SEGMENTATION_EPSLION{mRoverConfig["pt_cloud"]["ransac"]["segmentation_epsilon"].GetDouble()},
        DISTANCE_THRESHOLD{mRoverConfig["pt_cloud"]["ransac"]["distance_threshold"].GetDouble()},
        CLUSTER_TOLERANCE{mRoverConfig["pt_cloud"]["euclidean_cluster"]["cluster_tolerance"].GetInt()},
        MIN_CLUSTER_SIZE{mRoverConfig["pt_cloud"]["euclidean_cluster"]["min_cluster_size"].GetInt()},
        MAX_CLUSTER_SIZE{mRoverConfig["pt_cloud"]["euclidean_cluster"]["max_cluster_size"].GetInt()},
        
        //Other Values
        bearing{0}, distance{0}, detected{false},
        pt_cloud_ptr{new pcl::PointCloud<pcl::PointXYZRGB>} {

        #if PERCEPTION_DEBUG
        viewer = createRGBVisualizer(); //This is a smart pointer so no need to worry ab deleteing it
        viewer_original = createRGBVisualizer();
        #endif

        #if ZED_SDK_PRESENT
           sl::Resolution cloud_res = sl::Resolution(PT_CLOUD_WIDTH, PT_CLOUD_HEIGHT);
           cloudArea = cloud_res.area();
        #else
            cloudArea = PT_CLOUD_WIDTH*PT_CLOUD_HEIGHT;
        #endif

    };

/* --- Pass Through Filter --- */
//Filters out all points on a given axis passed as a string ("x", "y", or "z") that aren't within the threshold
//The threshold covers points from 0.0 to upperLimit
//Values are depth values in mm
//Source: https://rb.gy/kkyi80
void PCL::PassThroughFilter(const std::string axis, const double upperLimit) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t("PassThroughFilter");
    #endif

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pt_cloud_ptr);

    pass.setFilterFieldName(axis);
    pass.setFilterLimits( LOW_BD, upperLimit);
    pass.filter(*pt_cloud_ptr);
}

/* --- Voxel Filter --- */
//Creates clusters given by the size of a leaf
//All points in a cluster are then reduced to a single point
//This point is the centroid of the cluster
//Source: https://rb.gy/2ybg8n
void PCL::DownsampleVoxelFilter() {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t("VoxelFilter");
    #endif

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (pt_cloud_ptr);
    sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
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
void PCL::RANSACSegmentation(string type) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t("RANSACSegmentation");
    #endif

    //Creates instance of RANSAC Algorithm
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(MAX_ITERATIONS);
    seg.setDistanceThreshold(DISTANCE_THRESHOLD); //Distance in mm away from actual plane a point can be
    // to be considered an inlier
    seg.setAxis(Eigen::Vector3f(0, 1, 0)); //Looks for a plane along the Z axis
    //Max degree the normal of plane can be from Z axis
    seg.setEpsAngle(pcl::deg2rad(SEGMENTATION_EPSLION));

    //Objects where segmented plane is stored
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    seg.setInputCloud(pt_cloud_ptr);
    seg.segment(*inliers, *coefficients);

    if(type == "blue") {
        for (int i = 0; i < (int)inliers->indices.size(); i++) {
            pt_cloud_ptr->points[inliers->indices[i]].r = 255;
            pt_cloud_ptr->points[inliers->indices[i]].g = 255;
            pt_cloud_ptr->points[inliers->indices[i]].b = 0;
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
void PCL::CPUEuclidianClusterExtraction(std::vector<pcl::PointIndices> &cluster_indices) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t("CPU Cluster Extraction");
    #endif

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(pt_cloud_ptr);

    //Extracts clusters using nearet neighbors search
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE); // 60 mm radius per point
    ec.setMinClusterSize (MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize (MAX_CLUSTER_SIZE);
    ec.setSearchMethod (tree);
    ec.setInputCloud (pt_cloud_ptr);
    ec.extract (cluster_indices);

    //Colors all clusters
    #if PERCEPTION_DEBUG
        std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;
        int j = 0;

        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                if(j % 3) {
                    pt_cloud_ptr->points[*pit].r = 100 + j * 15;
                    pt_cloud_ptr->points[*pit].g = 0;
                    pt_cloud_ptr->points[*pit].b = 0;
                }
                else if(j % 2) {
                    pt_cloud_ptr->points[*pit].r = 0;
                    pt_cloud_ptr->points[*pit].g = 100 + j * 15;
                    pt_cloud_ptr->points[*pit].b = 0;
                }
                else {
                    pt_cloud_ptr->points[*pit].r = 0;
                    pt_cloud_ptr->points[*pit].g = 0;
                    pt_cloud_ptr->points[*pit].b = 100 + j * 15;
                }
            }
            j++;
        }
    #endif
}

/* --- Find Interest Points --- */
//Finds the edges of each cluster by comparing x and y
//values of all points in the cluster to find desired ones
//Interest points are a collection of points that allow us
//to define the edges of an obsacle
void PCL::FindInterestPoints(std::vector<pcl::PointIndices> &cluster_indices,
                             std::vector<std::vector<int>> &interest_points) {

    #if PERCEPTION_DEBUG
        pcl::ScopeTime t("Find Interest Points");
    #endif

    for (int i = 0; i < (int)cluster_indices.size(); ++i)
    {
        std::vector<int>* curr_cluster = &interest_points[i];
        
        //Initialize interest points
        std::fill(curr_cluster->begin(), curr_cluster->end(), cluster_indices[i].indices[0]);

        //Interest Points: 0=Leftmost Point 1=Rightmost Point 2=Lowest Point 3=Highest Point 4=Closest Point 5=Furthest Point.
        for (auto index : cluster_indices[i].indices)
        {
            auto curr_point = pt_cloud_ptr->points[index];
            
            if(curr_point.x < pt_cloud_ptr->points[curr_cluster->at(0)].x){
                curr_cluster->at(0) = index;
            }
            if(curr_point.x > pt_cloud_ptr->points[curr_cluster->at(1)].x){
                curr_cluster->at(1) = index;
            }
            if(curr_point.y < pt_cloud_ptr->points[curr_cluster->at(2)].y){
                curr_cluster->at(2) = index;
            }
            if(curr_point.y > pt_cloud_ptr->points[curr_cluster->at(3)].y){
                curr_cluster->at(3) = index;
            }
            if(curr_point.z < pt_cloud_ptr->points[curr_cluster->at(4)].z){
                curr_cluster->at(4) = index;
            }
            if(curr_point.z > pt_cloud_ptr->points[curr_cluster->at(5)].z){
                curr_cluster->at(5) = index;
            }
        }

        //Calulates the width of the obstacle based on the difference between the leftmost and rightmost interest point.
        double width = std::abs(pt_cloud_ptr->points[curr_cluster->at(1)].x - pt_cloud_ptr->points[curr_cluster->at(0)].x);
        //Calculates the number of rover widths that fit within the obstacle. The x10 multiplier adds more width increments.
        int roverWidths = ((int) width/ROVER_W_MM) * 10;

        //Only want to add interest points if the obstacle's width > rover's Width.
        if(roverWidths > 0) {
            //Create a map data structure where the Key is an int from 0  to roverWidths-1.
            //Each key index represents the a percentile increment.
            //Example: if roverWidths = 40, then index 0 would represent leftmost + 0.025 * obstacle width,
            //index 1 would represent leftmost + 0.05 * obstacle width and so on.
            std::map<int, double> increments;
            for(size_t i = 0; i < (size_t) roverWidths; ++i) {
                //The defualt value stored in each key is the value of the leftmost interest point.
                increments[i] = pt_cloud_ptr->points[curr_cluster->at(0)].x;
                //Creates a new interest point and sets it equal to the index of the leftmost point.
                curr_cluster->push_back(curr_cluster->at(0));
            }
            
            //Using the x value of the current point, calculate the percentile that the current point would fall under, 
            //and then compare that x value to the one of the point that is currently representing that percentile.
            for (auto index : cluster_indices[i].indices) {
                auto curr_point = pt_cloud_ptr->points[index];
                if(curr_point.x > pt_cloud_ptr->points[curr_cluster->at(0)].x && curr_point.x < pt_cloud_ptr->points[curr_cluster->at(1)].x) {
                    //If roverWidths = 40 and if your x value falls between leftmost + 0.025 * obstacle width and leftmost + 0.05 * obstacle width,
                    //then the value of i would be 1 which represents the index of increment map the we want to check.
                    int j = ((double)(std::abs(curr_point.x - pt_cloud_ptr->points[curr_cluster->at(0)].x)/width)/((double) 1/roverWidths));
                    //If the x value of the current point is greater than the value representing that percentile, 
                    //we set the value represnting the percentile equal to the x value of the current point.
                    if(increments[j] < curr_point.x) {
                        increments[j] = curr_point.x;
                        curr_cluster->at(6 + j) = index;
                    }
                }
            }
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

/* --- Get Angle Off Center--- */
//This function finds the angle off center the
//line that passes through both these points is
//Direction of 0 is left and 1 is right
double PCL::getAngleOffCenter(int buffer, int direction, const std::vector<std::vector<int>> &interest_points,
                              std::vector<int> &obstacles) {
    double newAngle = 0;
    //If Center Path is blocked check the left or right path depending on direction parameter
    while (newAngle > -MAX_FIELD_OF_VIEW_ANGLE && newAngle < MAX_FIELD_OF_VIEW_ANGLE) {

        //Finding angle off center
        double oppSideRTri = pt_cloud_ptr->points[obstacles.at(direction)].x;
        double adjSideRTri = pt_cloud_ptr->points[obstacles.at(0)].z;            //Length of adjacent side of right triangle
        oppSideRTri += direction ? buffer + HALF_ROVER : -(buffer + HALF_ROVER); //Calculate length of opposite side of right triangle
        newAngle = atan(oppSideRTri / adjSideRTri) * 180 / PI;                   //arctan(opposite/adjacent)

        //Create compareLine Functors
        compareLine leftLine(newAngle, -HALF_ROVER);
        compareLine rightLine(newAngle, HALF_ROVER);

        obstacles.clear();

        if(CheckPath(interest_points, obstacles, leftLine, rightLine)) {
            #if PERCEPTION_DEBUG
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FOUND NEW PATH AT: " << newAngle << std::endl;
            #endif
            return newAngle;
        }
    }
    return direction ? MAX_FIELD_OF_VIEW_ANGLE : -MAX_FIELD_OF_VIEW_ANGLE; //If couldn't find clear path
}

/* --- Find Clear Path --- */
//Returns the angle to a clear path
double PCL::FindClearPath(const std::vector<std::vector<int>> &interest_points) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t("Find Clear Path");
    #endif

    std::vector<int> obstacles; //index of the leftmost and rightmost obstacles in path

    //Check Center Path
    if(CheckPath(interest_points, obstacles, compareLine(0,-HALF_ROVER), compareLine(0,HALF_ROVER))) {
        #if PERCEPTION_DEBUG
            std::cout << "CENTER PATH IS CLEAR!!!" << std::endl;
        #endif
        return 0;
    }
    else { //otherwise, check to find left and right paths

        //Values that store the distances of the last obstacle from a given CheckPath. Center value gets its distance from previous loop of CheckPath
        double centerDistance = distance, leftDistance, rightDistance;

        //Initialize base cases outside of scope
        vector<int> centerObstacles = {obstacles.at(0), obstacles.at(1)};

        //Find Clear left path
        double leftAngle = getAngleOffCenter(10, 0, interest_points, obstacles);
        leftDistance = distance; 

        //Reset global variables
        obstacles = {0, 0};
        obstacles.at(0) = centerObstacles.at(0);
        obstacles.at(1) = centerObstacles.at(1);

        //Find clear right path
        double rightAngle = getAngleOffCenter(10, 1, interest_points, obstacles);
        rightDistance = distance;

        //return smallest distance of an obstacle from all the paths
        if(rightDistance < leftDistance && rightDistance < centerDistance) distance = rightDistance/1000.0;
        else if(leftDistance < rightDistance && leftDistance < centerDistance) distance = leftDistance/1000.0;
        else distance = centerDistance/1000.0;

        //Return the smallest angle (left if equal)
        return fabs(rightAngle) < fabs(leftAngle) ? rightAngle : leftAngle;
    }
}

/* --- Check Path --- */
//Returns T or F based on whether or not the input path is obstructed
//If it is obstructed returns false
//The path is constructed using the left x value and right x value of
//the furthest points on the path
bool PCL::CheckPath(const std::vector<std::vector<int>> &interest_points,
                    std::vector<int> &obstacles, compareLine leftLine, compareLine rightLine) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t("Check Path");
    #endif

    bool end = true; 
    double previousDistance = DBL_MAX;

    //if there are no interest points, the distance from the last obstacle should be DBL_MAX
    distance = previousDistance;
    
    //Iterate through interest points
    for(auto cluster : interest_points) {
        double sizeOfCluster = 0;
        double currentDistance = 0;
        for (auto index : cluster) {
            //Check if the obstacle interest point is to the right of the left projected path of the rover 
            //and to the left of the right projected path of the rover
            if(leftLine(pt_cloud_ptr->points[index].x, pt_cloud_ptr->points[index].z) >= 0 &&
                rightLine(pt_cloud_ptr->points[index].x, pt_cloud_ptr->points[index].z) <= 0) {
                end = false;

                //Check if obstacles is initialized
                if(obstacles.size() == 0) {
                    obstacles.push_back(index);
                    obstacles.push_back(index);
                }
                //Check if leftmost interest point in rover path
                else if(pt_cloud_ptr->points[index].x < pt_cloud_ptr->points[obstacles.at(0)].x) {
                    obstacles.at(0) = index;
                }
                //Check if rightmost interest point in rover path
                else if(pt_cloud_ptr->points[index].x > pt_cloud_ptr->points[obstacles.at(1)].x) {
                    obstacles.at(1) = index;
                }

                #if PERCEPTION_DEBUG
                    //Make interest points orange if they are within rover path
                    pt_cloud_ptr->points[index].r = 255;
                    pt_cloud_ptr->points[index].g = 69;
                    pt_cloud_ptr->points[index].b = 0;
                #endif

                //adds distance from a point in a cluster to currentDistance, and keeps track of the cluster size
                currentDistance += pt_cloud_ptr->points[index].z;
                sizeOfCluster++; 
            }
        }
        //to find the distance from an obstacle detected, add up all the z values from a given cluster of points
        //then divide by the number of points in the cluster
        currentDistance = 1.0 * currentDistance / sizeOfCluster;

        //if the last obstacle distance from a CheckPath loop was smaller, keep the smaller value, otherwise keep the current distance value
        if(currentDistance < previousDistance) {
            previousDistance = currentDistance;
            distance = previousDistance;
        }
    }

    #if PERCEPTION_DEBUG
        //Project path in viewer
        pcl::PointXYZRGB pt1;
        pt1.x = leftLine.xIntercept;
        pt1.y = 0;
        pt1.z = 0;
        pcl::PointXYZRGB pt2;
        pt2.x = rightLine.xIntercept;
        pt2.y = 0;
        pt2.z = 0;
        pcl::PointXYZRGB pt3(pt1);
        pt3.z = 7000;
        pt3.x = leftLine.xIntercept;

        if(leftLine.slope != 0) { //Don't want to divide by 0
            pt3.x = pt3.z / leftLine.slope + leftLine.xIntercept;
        }

        pcl::PointXYZRGB pt4(pt2);
        pt4.z = 7000;
        pt4.x = rightLine.xIntercept;

        if(rightLine.slope != 0) { //Don't want to divide by 0
            pt4.x = pt4.z / rightLine.slope + rightLine.xIntercept;
        }

        if(end) {
            viewer->removeShape("l1");
            viewer->removeShape("l2");
            viewer->addLine(pt1, pt3, 0, 255, 0, "l1");
            viewer->addLine(pt2, pt4, 0, 255, 0, "l2");
        }
        else {
            viewer->removeShape("l1");
            viewer->removeShape("l2");
            viewer->addLine(pt1, pt3, 255, 0, 0, "l1");
            viewer->addLine(pt2, pt4, 255, 0, 0, "l2");
        }
    #endif

    return end;
}


void PCL::updateViewer(bool is_original) {
    if(is_original) {
        viewer_original->updatePointCloud(pt_cloud_ptr);
        viewer_original->spinOnce(10);
    }

    else {
        viewer->updatePointCloud(pt_cloud_ptr);
        viewer->spinOnce(20);
    }
}
/* --- Create Visualizer --- */
//Creates a point cloud visualizer
shared_ptr<pcl::visualization::PCLVisualizer> PCL::createRGBVisualizer() {
    // Open 3D viewer and add point cloud

    //Creates visualizer with window if PERCEPTION_DEBUG is true, and mutes output otherwise
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer", PERCEPTION_DEBUG));

    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pt_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB>(pt_cloud_ptr, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, -800, 0, -1, 0);

    return (viewer);
}

/* --- Main --- */
//This is the main point cloud processing function
//It returns the bearing the rover should traverse
//For the PassThroughFilter function we can trust the ZED depth for up to 7000 mm (7 m) for "z" axis.
//3000 mm (3m) for "x" is a placeholder, we will chnage this value based on further testing.
//This function is called in main.cpp
void PCL::pcl_obstacle_detection() {
    obstacle_return result;
    PassThroughFilter("z", UP_BD_Z);
    PassThroughFilter("y", UP_BD_Y);
    DownsampleVoxelFilter();
    RANSACSegmentation("remove");
    std::vector<pcl::PointIndices> cluster_indices;
    CPUEuclidianClusterExtraction(cluster_indices);
    std::vector<std::vector<int>> interest_points(cluster_indices.size(), vector<int> (6));
    FindInterestPoints(cluster_indices, interest_points);
    bearing = FindClearPath(interest_points);
}



/* --- Update --- */
//Cleares and resizes cloud for new data
void PCL::update() {
    pt_cloud_ptr->clear();
    pt_cloud_ptr->points.resize(cloudArea);
    pt_cloud_ptr->width = PT_CLOUD_WIDTH;
    pt_cloud_ptr->height = PT_CLOUD_HEIGHT;

    #if PERCEPTION_DEBUG
        std::cout << "Width: " << pt_cloud_ptr->width << std::endl;
        std::cout << "Height: " << pt_cloud_ptr->height << "\n";
    #endif
}

#endif
