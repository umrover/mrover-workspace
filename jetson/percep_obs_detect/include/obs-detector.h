#include <string>
#include <sl/Camera.hpp>
#include "plane-ransac.hpp"
#include "pass-through.hpp"
#include "viewer.hpp"
#include "voxel-grid.hpp"
#include "euclidean-cluster.hpp"
#include <thread>
#include "timer.hpp"
#include "common.hpp"
#include "voxel-grid.hpp"
#include "find-clear-path.hpp"
#include <cstring>
#include <iostream>
#include <float.h>
#include <stdlib.h>
#include <unistd.h>

#ifndef NO_JARVIS
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Obstacle.hpp"
#endif

// TODO: move as many of these includes to cpp as possible
//using namespace boost::interprocess;

/*
 *** Determines where to input clouds for obstacle detection ***
 *      ZED: inputs clouds directly from a connected ZED
 *      GPUMEM: receives a pointer to cloud GPU memory from external source
 *      FILESYSTEM: reads .pc files from specified location
 */
enum class DataSource {ZED, GPUMEM, FILESYSTEM}; 

/*
 *** Set up debugging level ***
 */
enum class OperationMode {DEBUG, SILENT};

/*
 *** Choose which viewer to use ***
 */
enum ViewerType {NONE, GL};

/** 
 * \class ObsDetector
 * \brief class that contains framework, algorithm instances, and state variables to perform obstacle detection
 */
class ObsDetector {
    public:
        /**
         * \brief ObsDetector constructor
         * \param source: Source of point clouds, either ZED to read from camera, GPUMEM to pass a pointer to a frame, or FILESYSTEM for static test files
         * \param mode: Debugging level, either DEBUG for testing or SILENT for competition
         * \param viewer: Viewer type, use NONE for competition, PCLV if you are on Great Lakes, and GL otherwise
         */
        ObsDetector(DataSource source, OperationMode mode, ViewerType viewer);

        //Destructor 
        ~ObsDetector();

        /**
         * \brief Grabs the next frame from either file or zed and performs an obstacle detection
         */
        void update();

        /**
         * \brief This is the underlying method called by update(), if DataSource::GPUMEM is selected, call this version 
         * of the function directly with a pointer to your frame in GPU memory
         * \param frame: sl::Mat frame to do detection on with memory allocated on the GPU
         */
        void update(GPU_Cloud pc);

        /**
         * \brief Do viewer update tick, it may be desirable to call this in its own thread 
         */
        void spinViewer();

        /**
         * \brief [TODO] Start recording frames from ZED
         * \param frame: string directory in which to write the pcd files, directory should already exist
         */
        void startRecording(std::string directory);

        /**
         * \brief [TODO] Stop recording frames (this does not need to be called if you want to record until program exit)
         */
        void stopRecording();

        /**
         * \brief Populates the LCM message
         * \param leftBearing left bearing of path
         * \param rightBearing right bearing of path
         * \param distance distance to nearest obstacle
         */
        void populateMessage(float leftBearing, float rightBearing, float distance);


    private:

        //Sets up detection paramaters from a JSON file
        void setupParamaters(std::string parameterFile);


    private: 
        //Lcm
        #ifndef NO_JARVIS
        lcm::LCM lcm_;
        rover_msgs::Obstacle obstacleMessage;
        #endif

        //Data sources
        sl::Camera zed;
        PCDReader fileReader;

        //Viwers
        Viewer viewer;

        //Operation paramaters
        DataSource source;
        OperationMode mode;
        ViewerType viewerType;
        bool record = false;

        //Detection algorithms 
        PassThrough *passZ;
        RansacPlane *ransacPlane;
        VoxelGrid *voxelGrid;
        EuclideanClusterExtractor *ece;
        FindClearPath *findClear;

        //Paramaters
        sl::Resolution cloud_res;
        sl::InitParameters init_params;
        sl::CameraParameters defParams;
        std::string readDir;
        
        //Output data
        Plane planePoints;
        EuclideanClusterExtractor::ObsReturn obstacles;
        float3 bearingCombined;
        float leftBearing;
        float rightBearing;
        float distance;

        //Other
        int frameNum = 0;
        bool framePlay = true;
        
};
