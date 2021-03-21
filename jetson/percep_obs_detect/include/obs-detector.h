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

class ObsDetector {
    public:
        //Construct the obstacle detector with appropriate options
        ObsDetector(DataSource source, OperationMode mode, ViewerType viewer);

        //Destructor 
        ~ObsDetector();

        // Grabs the next frame from either file or zed and performs an obstacle detection
        void update();

        // This is the underlying method called by update(), if DataSource::GPUMEM is selected, call this one with the frame
        void update(sl::Mat &frame);

        //Do viewer update tick, it may be desirable to call this in its own thread 
        void spinViewer();

        //Start recording frames from ZED
        void startRecording(std::string directory);

        //Stop recording frames (this does not need to be called if you want to record until program exit)
        void stopRecording();

    private:

        //Sets up detection paramaters from a JSON file
        void setupParamaters(std::string parameterFile);


    private: 
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

        //Other
        Recorder recorder;
        int frameNum = 0;
        bool framePlay = true;
        
};