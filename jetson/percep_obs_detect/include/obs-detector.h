#include <string>
#include <sl/Camera.hpp>
#include "recorder.hpp"
#include "plane-ransac.hpp"
#include "pass-through.hpp"
#include "GLViewer.hpp"
#include "euclidean-cluster.hpp"
#include <thread>
#include "Timer.hpp"
#include "common.hpp"
#include "pcl.hpp"
#include "voxel-grid.hpp"
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/Obstacle.hpp"
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <cstring>
#include <iostream>

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
enum ViewerType {NONE, PCLV, GL};

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
        void update(sl::Mat &frame);

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
        lcm::LCM lcm_;
        rover_msgs::Obstacle obstacleMessage;

        //Data sources
        sl::Camera zed;
        Reader fileReader;

        //Viwers
        GLViewer glViewer;
        shared_ptr<pcl::visualization::PCLVisualizer> pclViewer; 
        void pclKeyCallback(const pcl::visualization::KeyboardEvent &event, void* junk);

        //Operation paramaters
        DataSource source;
        OperationMode mode;
        ViewerType viewer;
        bool record = false;

        //Detection algorithms 
        PassThrough *passZ;
        RansacPlane *ransacPlane;
        VoxelGrid *voxelGrid;
        EuclideanClusterExtractor *ece;

        //Paramaters
        sl::Resolution cloud_res;
        sl::InitParameters init_params;
        sl::CameraParameters defParams;
        std::string readDir;
        
        //Output data
        RansacPlane::Plane planePoints;
        EuclideanClusterExtractor::ObsReturn obstacles;
        float leftBearing;
        float rightBearing;
        float distance;

        //Other
        Recorder recorder;
        int frameNum = 0;
        bool framePlay = true;
        
};