#include "obs-detector.h"
#include <chrono>

using namespace std;
using namespace std::chrono;
#include <glm/vec4.hpp> // glm::vec4
#include <glm/glm.hpp>
#include <vector>

ObsDetector::ObsDetector(DataSource source, OperationMode mode, ViewerType viewerType) : source(source), mode(mode), viewerType(viewerType), record(false)
{
    setupParamaters("");

    //Init data stream from source
    if(source == DataSource::ZED) {
        zed.open(init_params); 
        auto camera_config = zed.getCameraInformation(cloud_res).camera_configuration;
        defParams = camera_config.calibration_parameters.left_cam;
    } else if(source == DataSource::FILESYSTEM) {
        std::string s = ROOT_DIR;
        s+= "/data/";
        
        cout << "File data dir: " << endl;
        cout << "[defaulting to: " << s << endl;
        getline(cin, readDir);
        if(readDir == "")  readDir = s;
        fileReader.open(readDir);
    }

    //Init Viewer
    if(mode != OperationMode::SILENT && viewerType == ViewerType::GL) {
        int argc = 1;
        char *argv[1] = {(char*)"Window"};
        viewer.init(argc, argv);
        viewer.addPointCloud();
    }

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
    findClear = new FindClearPath();
}
        

void ObsDetector::update() {
    GPU_Cloud pc; 

    if(source == DataSource::ZED) {

        sl::Mat frame(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);
        zed.grab();
        zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res); 
        getRawCloud(pc, frame);
        
    } else if(source == DataSource::FILESYSTEM) {

        pc = fileReader.readCloudGPU(59);
    }
    update(pc);

    if(source == DataSource::FILESYSTEM) deleteCloud(pc);
} 
///home/ashwin/Documents/mrover-workspace/jetson/percep_obs_detect/data
// Call this directly with ZED GPU Memory
void ObsDetector::update(GPU_Cloud pc) {

    // Get a copy if debug is enabled
    viewer.updatePointCloud(pc);

    // Processing

    passZ->run(pc);
    
    ransacPlane->computeModel(pc);    
    
    Bins bins;
    #if VOXEL
        bins = voxelGrid->run(pc);
    #endif
    obstacles = ece->extractClusters(pc, bins); 

    //TESTING: Make our own obstacles
    obstacles.obs.clear();
    EuclideanClusterExtractor::Obstacle test; // Change parameters of obstacle below
    test.minX = -250;
    test.maxX = 250; //"Right" is positive X direction
    test.minY = 250; 
    test.maxY = 750; //"Down" is positive Y direction
    test.minZ = 3500;
    test.maxZ = 4000; //"Forward" is positive Z direction

    obstacles.obs.push_back(test);

    findClear->find_clear_path_initiate(obstacles);
    
    
    ///*/
    // Rendering
    if(mode != OperationMode::SILENT) {
        //viewer.addPointCloud();
        //viewer.remove
        //viewer.updatePointCloud(pc);
    }
    populateMessage(9, 10, 11);


    // Recording
    if(record) record = true;

    if(framePlay) frameNum++;
    
}

void ObsDetector::populateMessage(float leftBearing, float rightBearing, float distance) {
    this->leftBearing = leftBearing;
    this->rightBearing = rightBearing;
    this->distance = distance;
//    obstacleMessage.bearing = leftBearing;
//    lcm_.publish("/obstacle", &obstacleMessage);
}

void ObsDetector::spinViewer() {
    // This creates bounding boxes for visualization
    // There might be a clever automatic indexing scheme to optimize this
    for(int i = 0; i < obstacles.obs.size(); i++) {
        std::vector<vec3> points = {vec3(obstacles.obs[i].minX, obstacles.obs[i].minY, obstacles.obs[i].minZ), 
                                    vec3(obstacles.obs[i].maxX, obstacles.obs[i].minY, obstacles.obs[i].minZ), 
                                    vec3(obstacles.obs[i].maxX, obstacles.obs[i].maxY, obstacles.obs[i].minZ), 
                                    vec3(obstacles.obs[i].minX, obstacles.obs[i].maxY, obstacles.obs[i].minZ),
                                    vec3(obstacles.obs[i].minX, obstacles.obs[i].minY, obstacles.obs[i].maxZ), 
                                    vec3(obstacles.obs[i].maxX, obstacles.obs[i].minY, obstacles.obs[i].maxZ), 
                                    vec3(obstacles.obs[i].maxX, obstacles.obs[i].maxY, obstacles.obs[i].maxZ), 
                                    vec3(obstacles.obs[i].minX, obstacles.obs[i].maxY, obstacles.obs[i].maxZ),};
        std::vector<vec3> colors;
        for(int q = 0; q < 8; q++) colors.push_back(vec3(0.0f, 1.0f, 0.0f));
        std::vector<int> indicies = {0, 1, 2, 2, 3, 0, 1, 2, 5, 5, 6, 2, 0, 3, 4, 3, 7, 4, 4, 5, 6, 7, 6, 5};
        Object3D obj(points, colors, indicies);
        viewer.addObject(obj, true);
    }

    // Test bearing one
    /*
    float d = 7000;
    float theta = 20 * 3.14159/180.0;
    std::vector<vec3> pts = {
        vec3(0, 0, 0), 
        vec3(d*sin(theta), 0, d*cos(theta))
    };
    std::vector<vec3> colors = {
        vec3(1.0f, 0.0f, 0.0f),
        vec3(1.0f, 0.0f, 0.0f)
    };
    std::vector<int> indicies = {0, 1, 0}; 
    Object3D bearing(pts, colors, indicies);
    viewer.addObject(bearing, true); */

    //TESTING: draw double bearing
    //Note: "straight ahead" is 0 degree bearing, -80 degree on left, +80 degree to right
    float degAngle = 0; //Change angle of bearing to draw here
    float d = 7000;
    float theta = degAngle * 3.14159/180.0;
    float roverWidthDiv2 = 1500/2;

    vec3 bearing = vec3(d*sin(theta), 0, d*cos(theta));

    vec3 leftBearingStart = vec3(-roverWidthDiv2 * cos(theta), 500, roverWidthDiv2 * sin(theta));
    vec3 rightBearingStart = vec3(roverWidthDiv2 * cos(theta), 500, -roverWidthDiv2 * sin(theta));

    std::vector<vec3> ptsLft = {
        leftBearingStart,
        leftBearingStart + bearing
    };
    std::vector<vec3> ptsRght = {
        rightBearingStart,
        rightBearingStart + bearing
    };
    std::vector<vec3> colors = {
        vec3(1.0f, 0.0f, 0.0f),
        vec3(1.0f, 0.0f, 0.0f)
    };
    std::vector<int> indicies = {0, 1, 0}; 

    Object3D leftBearing(ptsLft, colors, indicies);
    Object3D rightBearing(ptsRght, colors, indicies);

    viewer.addObject(leftBearing, true);
    viewer.addObject(rightBearing, true);
    
    viewer.update();
    viewer.clearEphemerals();
    
}

 ObsDetector::~ObsDetector() {
     delete passZ;
     delete ransacPlane;
     delete voxelGrid;
     delete ece;
 }



int main() {
    ObsDetector obs(DataSource::FILESYSTEM, OperationMode::DEBUG, ViewerType::GL);

    //std::thread updateTick( [&]{while(true) { obs.update();} });

    while(true) {
       obs.update();
       obs.spinViewer();
    }
    

    return 0;
}
