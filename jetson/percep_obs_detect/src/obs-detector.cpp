#include "obs-detector.h"
#include "common.hpp"
#include <chrono>
using namespace std;
using namespace std::chrono;

ObsDetector::ObsDetector(DataSource source, OperationMode mode, ViewerType viewer) : source(source), mode(mode), viewer(viewer), record(false)
{
    setupParamaters("");
    
    //Init data stream from source
    if(source == DataSource::ZED) {
        zed.open(init_params); 
        auto camera_config = zed.getCameraInformation(cloud_res).camera_configuration;
        defParams = camera_config.calibration_parameters.left_cam;
    } else if(source == DataSource::FILESYSTEM) {
        fileReader.open(readDir);
    }

    //Init Viewers
    if(mode != OperationMode::SILENT && viewer == ViewerType::PCLV) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
        pclViewer = createRGBVisualizer(pc_pcl);
        pclViewer->registerKeyboardCallback(&ObsDetector::pclKeyCallback, *this);
    } else if(mode != OperationMode::SILENT && viewer == ViewerType::GL) {
        int argc = 1;
        char *argv[1] = {(char*)"Window"};
        glViewer.init(argc, argv, defParams);
    }
};

void ObsDetector::pclKeyCallback(const pcl::visualization::KeyboardEvent &event, void* junk) {
    if (event.getKeySym() == "d" && event.keyDown()){
        frameNum++;
    }
    if (event.getKeySym() == "a" && event.keyDown()){
        frameNum--;
    }
    if (event.getKeySym() == "p" && event.keyUp()){
        framePlay = !framePlay;
    }
}

//TODO: Make it read params from a file
void ObsDetector::setupParamaters(std::string parameterFile) {
    //Operating resolution
    cloud_res = sl::Resolution(320/2, 180/2);
    readDir = "/home/ashwin/Documents/mrover-workspace/jetson/percep_obs_detect/data";

    //Zed params
    init_params.coordinate_units = sl::UNIT::MILLIMETER;
    init_params.camera_resolution = sl::RESOLUTION::VGA; 
    init_params.camera_fps = 100;
    
    //Set the viewer paramas
    defParams.fx = 79.8502;
    defParams.fy = 80.275;
    defParams.cx = 78.8623;
    defParams.cy = 43.6901;
    defParams.image_size.width = 160;
    defParams.image_size.height = 90;

    //Obs Detecting Algorithm Params
    passZ = new PassThrough('z', 200.0, 8000.0); //7000
    ransacPlane = new RansacPlane(sl::float3(0, 1, 0), 8, 600, 80, cloud_res.area());
    voxelGrid = new VoxelGrid(10);
    ece = new EuclideanClusterExtractor(150, 30, 0, cloud_res.area(), 9); 
}



void ObsDetector::update() {
    if(source == DataSource::ZED) {
        sl::Mat frame; // (cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);
        zed.grab();
        zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res); 
        update(frame);
    } else if(source == DataSource::FILESYSTEM) {
        //DEBUG 
        //frameNum = 250;
        sl::Mat frame(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
        fileReader.load(frameNum, frame, true);
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

    // Convert ZED format into CUDA compatible 
    GPU_Cloud_F4 pc; 
    pc = getRawCloud(frame);

    // Processing 
    passZ->run(pc);
    //std::cout << "pre ransac:" << pc.size << endl;
    ransacPlane->computeModel(pc, true);
    
    
    






    // Voxel Grid Testing Code
    /*
    int size = 4;
    sl::float4 testCPU[size] = {
        {1,-3,-2,4},
        {2,2,2,4},
        {0,0,0,4},
        {0,0,0,1}
    };
    
    GPU_Cloud_F4 testCPUpc {
        testCPU, size, size
    };
    
    sl::float4* testGPU;
    cudaMalloc(&testGPU, sizeof(sl::float4)*size);
    cudaMemcpy(testGPU, testCPU, sizeof(sl::float4)*size, cudaMemcpyHostToDevice);
    GPU_Cloud_F4 testPC = { testGPU, size, size};
    */
    Bins bins;

    #if VOXEL
        bins = voxelGrid->run(pc);
    #endif









    //std::cout << "post ransac:" << pc.size << endl;
    auto grabStart = high_resolution_clock::now();
    obstacles = ece->extractClusters(pc, bins); 
    auto grabEnd = high_resolution_clock::now();
    auto grabDuration = duration_cast<microseconds>(grabEnd - grabStart); 
    //cout << "ECE time: " << (grabDuration.count()/1.0e3) << " ms" << endl; 

    // Rendering
    if(mode != OperationMode::SILENT) {
        clearStale(pc, cloud_res.area());
        if(viewer == ViewerType::GL) {
            glViewer.updatePointCloud(orig);
        } else {
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
           ZedToPcl(pc_pcl, frame);
           pclViewer->updatePointCloud(pc_pcl); //update the viewer 
        }
    }

    // Recording
    if(record) {
        recorder.writeFrame(frame);
    }

    if(framePlay) frameNum++;
}

void ObsDetector::spinViewer() {
    if(viewer == ViewerType::GL) {
        glViewer.isAvailable();
        updateObjectBoxes(obstacles.size, obstacles.minX, obstacles.maxX, obstacles.minY, obstacles.maxY, obstacles.minZ, obstacles.maxZ );
        updateProjectedLines(ece->bearingRight, ece->bearingLeft);
    } else if(viewer == ViewerType::PCLV) {
        pclViewer->removeAllShapes();
        for(int i = 0; i < obstacles.size; i++) {
            float xMin = obstacles.minX[i];
            float xMax = obstacles.maxX[i];
            float yMin = obstacles.minY[i];
            float yMax = obstacles.maxY[i];
            float zMin = obstacles.minZ[i];
            float zMax = obstacles.maxZ[i];
            if(zMax < 0.01) {
                xMin = 0; xMax = 0; yMin = 0; yMax = 0; zMin = 0; zMax = 0;
            };
            pclViewer->addCube(xMin, xMax, yMin, yMax, zMin, zMax, 0.0, 1.0, 0.0, to_string(i));
            pclViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, to_string(i));
            pclViewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, to_string(i));

        }
        pclViewer->spinOnce(10);
    }
}

void ObsDetector::startRecording(std::string directory) {
    recorder.open(directory);
    record = true;
}

 ObsDetector::~ObsDetector() {
     delete passZ;
     delete ransacPlane;
     delete ece;
 }



int main() {
    ObsDetector obs(DataSource::FILESYSTEM, OperationMode::DEBUG, ViewerType::GL);
    //obs.startRecording("test-record3");
    //obs.update();
    std::thread viewerTick( [&]{while(true) { obs.update();} });
    
    while(true) {
        //obs.update();
        obs.spinViewer();
    }

    return 0;
}