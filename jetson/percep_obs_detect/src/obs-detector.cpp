#include "obs-detector.h"
#include <chrono>

using namespace std;
using namespace std::chrono;
//using namespace boost::interprocess;
#include <glm/vec4.hpp> // glm::vec4
#include <glm/glm.hpp>

ObsDetector::ObsDetector(DataSource source, OperationMode mode, ViewerType viewer) : source(source), mode(mode), viewer(viewer), record(false)
{
    setupParamaters("");
    
    //Init data stream from source
    if(source == DataSource::ZED) {
        zed.open(init_params); 
        auto camera_config = zed.getCameraInformation(cloud_res).camera_configuration;
        defParams = camera_config.calibration_parameters.left_cam;
    } else if(source == DataSource::FILESYSTEM) {
        cout << "File data dir: " << endl;
        cout << "[e.g: /home/ashmg/Documents/mrover-workspace/jetson/percep_obs_detect/data]" << endl;
        getline(cin, readDir);
        fileReader.open(readDir);
    }

    //Init Viewers
    if(mode != OperationMode::SILENT && viewer == ViewerType::GL) {
        int argc = 1;
        char *argv[1] = {(char*)"Window"};
        viewer.init(argc, argv);
        viewer.addPointCloud();
    }

   // shm = shared_memory_object(open_only, "gpuhandle", read_write);
   // region = mapped_region(shm, read_write);
};

//TODO: Make it read params from a file
void ObsDetector::setupParamaters(std::string parameterFile) {
    //Operating resolution
    cloud_res = sl::Resolution(320, 180);
    readDir = "/home/mrover/mrover-workspace/jetson/percep_obs_detect/data";

    //Zed params
    init_params.coordinate_units = sl::UNIT::MILLIMETER;
    init_params.camera_resolution = sl::RESOLUTION::VGA; 
    init_params.camera_fps = 100;
    
    //Set the viewer paramas
    defParams.fx = 79.8502;
    defParams.fy = 80.275;
    defParams.cx = 78.8623;
    defParams.cy = 43.6901;
    defParams.image_size.width = cloud_res.width;
    defParams.image_size.height = cloud_res.height;

    //Obs Detecting Algorithm Params
    passZ = new PassThrough('z', 100, 7000); //7000
    ransacPlane = new RansacPlane(make_float3(0, 1, 0), 8, 600, 80, cloud_res.area(), 80);
    voxelGrid = new VoxelGrid(10);
    ece = new EuclideanClusterExtractor(300, 30, 0, cloud_res.area(), 9); 
}
        

void ObsDetector::update() {
    if(source == DataSource::ZED) {
        sl::Mat frame(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);
        zed.grab();
        zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res); 
        update(frame);
    } else if(source == DataSource::FILESYSTEM) {
        //DEBUG 
        //frameNum = 250;
        sl::Mat frame(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
        fileReader.readCloud(frameNum, frame, true);
        update(frame);
    }
} 

// Call this directly with ZED GPU Memory
void ObsDetector::update(sl::Mat &frame) {

    // Get a copy if debug is enabled
    sl::Mat orig; 
    if(mode != OperationMode::SILENT) {
        frame.copyTo(orig, sl::COPY_TYPE::GPU_GPU);
    }

    // Convert ZED format into CUDA compatible type
    GPU_Cloud pc; 
    getRawCloud(pc, frame);

    // Processing
    passZ->run(pc);
    ransacPlane->computeModel(pc);
    Bins bins;
    #if VOXEL
        bins = voxelGrid->run(pc);
    #endif
    obstacles = ece->extractClusters(pc, bins); 

    // Rendering
    if(mode != OperationMode::SILENT) {
        if(viewer == ViewerType::GL) {
            viewer.updatePointCloud(frame);
        } 
    }

    // Recording
    if(record) {
        recorder.writeFrame(frame);
    }

    if(framePlay) frameNum++;
    
}

void ObsDetector::populateMessage(float leftBearing, float rightBearing, float distance) {
    this->leftBearing = leftBearing;
    this->rightBearing = rightBearing;
    this->distance = distance;
    //obstacleMessage.leftBearing = leftBearing;
    //lcm_.publish("/obstacle", &obstacleMessage);
}

void ObsDetector::spinViewer() {
    if(viewer == ViewerType::GL) {
        //updateObjectBoxes(obstacles.size, obstacles.minX, obstacles.maxX, obstacles.minY, obstacles.maxY, obstacles.minZ, obstacles.maxZ );
        //updateProjectedLines(ece->bearingRight, ece->bearingLeft);
        viewer.update();
        viewer.clearEphemerals();
    } 
}

 ObsDetector::~ObsDetector() {
     delete passZ;
     delete ransacPlane;
     delete voxelGrid;
     delete ece;
 }



int main() {
    ObsDetector obs(DataSource::FILESYSTEM, OperationMode::DEBUG, ViewerType::GL);

    std::thread updateTick( [&]{while(true) { obs.update();} });

    while(true) {
        obs.spinViewer();
    }
    

    return 0;
}
