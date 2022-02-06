#include <string>
#include "camera.hpp"
#include "plane-ransac.hpp"
#include "pass-through.hpp"
#include "viewer.hpp"
#include "voxel-grid.hpp"
#include "euclidean-cluster.hpp"
#include <thread>
#include "common.hpp"
#include "voxel-grid.hpp"
#include "find-clear-path.hpp"
#include <cstring>
#include <sstream>
#include <iostream>
#include <cfloat>
#include <cstdlib>
#include <unistd.h>

// TODO: move as many of these includes to cpp as possible
//using namespace boost::interprocess;

/*
 *** Determines where to input clouds for obstacle detection ***
 *      ZED: inputs clouds directly from a connected ZED
 *      GPUMEM: receives a pointer to cloud GPU memory from external source
 *      FILESYSTEM: reads .pc files from specified location
 */
enum class DataSource {
    ZED, GPUMEM, FILESYSTEM
};

/*
 *** Choose which viewer to use ***
 */
enum ViewerType {
    NONE, GL
};

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
        ObsDetector(const rapidjson::Document& mRoverConfig, camera_ptr cam);

        //Destructor
        ~ObsDetector();

        /**
         * \brief Grabs the next frame from either file or zed and performs an obstacle detection
         */
        void update();

        void handleParameters();

        /**
         * \brief This is the underlying method called by update(), if DataSource::GPUMEM is selected, call this version
         * of the function directly with a pointer to your frame in GPU memory
         * \param frame: sl::Mat frame to do detection on with memory allocated on the GPU
         */
        void update(GPU_Cloud pc);

        /**
         * \brief Create bounding box and add viewer object for each obstacle
         */
        void createBoundingBoxes();

        /**
         * \brief Find and make viewer object for path bearings
         */
        void createBearing();

        /**
         * \brief Do viewer update tick, it may be desirable to call this in its own thread
         */
        void spinViewer();

        /**
         * \brief Populates the LCM message
         * \param leftBearing left bearing of path
         * \param rightBearing right bearing of path
         * \param distance distance to nearest obstacle
         */
        void populateMessage(float leftBearing, float rightBearing, float distance);


        bool open();

    private:
        //Sets up detection paramaters from a JSON file
        void setupParamaters(const rapidjson::Document& mRoverConfig);

        // Lcm
#ifdef WITH_JARVIS
        lcm::LCM lcm_;
        rover_msgs::Obstacle obstacleMessage;
#endif

        // Data sources
        camera_ptr cam;
        PCDReader fileReader;

        // Recorder
        // int frameCounter;
        // int frameGap;

        // Viewers
        Viewer viewer;

        // Operation parameters
        OperationMode mode;
        ViewerType viewerType;

        // Detection algorithms
        PassThrough* passZ;
        RansacPlane* ransacPlane;
        VoxelGrid* voxelGrid;
        EuclideanClusterExtractor* ece;
        FindClearPath* findClear;

        // Parameters
        sl::Resolution cloud_res;
        sl::InitParameters init_params;
        sl::CameraParameters defParams;
        std::string readDir;

        // Output data
        Plane planePoints;
        EuclideanClusterExtractor::ObsReturn obstacles;
        float3 bearingCombined;
        float leftBearing;
        float rightBearing;
        float distance;

        void drawGround(Plane const& plane);
};
