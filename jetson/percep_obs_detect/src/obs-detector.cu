#include "obs-detector.h"
#include <chrono>

using namespace std;
using namespace std::chrono;
#include <glm/vec4.hpp> // glm::vec4
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <vector>

ObsDetector::ObsDetector(DataSource source, OperationMode mode, ViewerType viewerType) : source(source), mode(mode), viewerType(viewerType), record(false)
{
    setupParamaters("");

    //Init data stream from source
    if(source == DataSource::ZED) {
        auto error = zed.open(init_params);
        if (error != sl::ERROR_CODE::SUCCESS) {
            throw std::runtime_error("Error opening ZED camera");
        }
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
    sl::Mat frame(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);

    if(source == DataSource::ZED) {
        zed.grab();
        zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res); 
        getRawCloud(pc, frame);
        
    } else if(source == DataSource::FILESYSTEM) {
        pc = fileReader.readCloudGPU(viewer.frame);
        if (viewer.frame == 1) viewer.setCenter();
    }
    update(pc);

    if(source == DataSource::FILESYSTEM) deleteCloud(pc);
}

vec3 closestPointOnPlane(Plane plane, vec3 point) {
    vec3 normal{plane.normal.x, plane.normal.y, plane.normal.z};
    vec3 p0{plane.p0.x, plane.p0.y, plane.p0.z};
    float dist = (glm::dot(normal, point) - glm::dot(normal, p0)) / glm::length(normal);
    return point - glm::normalize(normal) * dist;
}

void ObsDetector::drawGround(Plane const& plane) {
    auto normal = glm::normalize(vec3{plane.normal.x, plane.normal.y, plane.normal.z});
    // make the center of the plane quad lined up with point cloud center
    vec3 centerProjOnPlane = closestPointOnPlane(plane, viewer.getCenter());
    // have edge flush with camera, treating camera as (0, 0, 0)
    vec3 cameraProjOnPlane = closestPointOnPlane(plane, {});
    // "forward" as in facing center
    vec3 cameraForward = centerProjOnPlane - cameraProjOnPlane;
    vec3 cameraRight = glm::cross(cameraForward, normal);
    vector<vec3> vertices {
            cameraProjOnPlane + cameraRight,
            cameraProjOnPlane - cameraRight,
            cameraProjOnPlane + cameraForward * 2.0f + cameraRight,
            cameraProjOnPlane + cameraForward * 2.0f - cameraRight,
    };
    vector<vec3> colors(vertices.size(), {1.0f, 1.0f, 0.0f});
    vector<int> indices{0, 1, 2, 1, 2, 3};
    viewer.addObject({vertices, colors, indices}, true);
}

///home/ashwin/Documents/mrover-workspace/jetson/percep_obs_detect/data
// Call this directly with ZED GPU Memory
void ObsDetector::update(GPU_Cloud pc) {

    // Get a copy if debug is enabled
    viewer.updatePointCloud(pc);

    // Processing
<<<<<<< HEAD
    if(viewer.procStage != ProcStage::RAW) {
        passZ->run(pc);
        if(viewer.procStage != ProcStage::POSTPASS) {
            ransacPlane->computeModel(pc);
        }
    }    
    
=======

    passZ->run(pc);

    Plane plane = ransacPlane->computeModel(pc);
    drawGround(plane);

>>>>>>> b68db059 (Add draw plane function, minimal error detection for opening ZED camera)
    Bins bins;
    #if VOXEL
            bins = voxelGrid->run(pc);
    #endif
    if(viewer.procStage == ProcStage::POSTECE || viewer.procStage == ProcStage::POSTBOUNDING || viewer.procStage == ProcStage::POSTBEARING) {
        obstacles = ece->extractClusters(pc, bins);
    }

    bearingCombined = findClear->find_clear_path_initiate(obstacles);
    leftBearing = bearingCombined.x;
    rightBearing = bearingCombined.y;
    distance = bearingCombined.z; 
    
    ///*/
    // Rendering
    if(mode != OperationMode::SILENT) {
        viewer.updatePointCloud(pc);
    }
    populateMessage(leftBearing, rightBearing, distance);

    // Recording
    if(record) record = true;

    if(viewer.framePlay) viewer.frame++;
    
}

void ObsDetector::populateMessage(float leftBearing, float rightBearing, float distance) {
    #ifndef NO_JARVIS
    this->leftBearing = leftBearing;
    this->rightBearing = rightBearing;
    this->distance = distance;
    obstacleMessage.bearing = leftBearing;
    obstacleMessage.distance = distance;
    lcm_.publish("/obstacle", &obstacleMessage);
    #endif
}

void ObsDetector::createBoundingBoxes() {
    // This creates bounding boxes for visualization
    // There might be a clever automatic indexing scheme to optimize this
    for(int i = 0; i < obstacles.obs.size(); i++) {
         if(obstacles.obs[i].minX < obstacles.obs[i].maxX && obstacles.obs[i].minZ < obstacles.obs[i].maxZ){ 
            std::vector<vec3> points = {vec3(obstacles.obs[i].minX, obstacles.obs[i].minY, obstacles.obs[i].minZ), 
                                        vec3(obstacles.obs[i].maxX, obstacles.obs[i].minY, obstacles.obs[i].minZ), 
                                        vec3(obstacles.obs[i].maxX, obstacles.obs[i].maxY, obstacles.obs[i].minZ), 
                                        vec3(obstacles.obs[i].minX, obstacles.obs[i].maxY, obstacles.obs[i].minZ),
                                        vec3(obstacles.obs[i].minX, obstacles.obs[i].minY, obstacles.obs[i].maxZ), 
                                        vec3(obstacles.obs[i].maxX, obstacles.obs[i].minY, obstacles.obs[i].maxZ), 
                                        vec3(obstacles.obs[i].maxX, obstacles.obs[i].maxY, obstacles.obs[i].maxZ), 
                                        vec3(obstacles.obs[i].minX, obstacles.obs[i].maxY, obstacles.obs[i].maxZ),};
            std::vector<vec3> colors;
            for(int q = 0; q < 8; q++) colors.emplace_back(0.0f, 1.0f, 0.0f);
            std::vector<int> indices = {0, 1, 2, 2, 3, 0, 1, 2, 5, 5, 6, 2, 0, 3, 4, 3, 7, 4, 4, 5, 6, 7, 6, 5};
            viewer.addObject({points, colors, indices}, true);
         }
    }
}

void ObsDetector::createBearing() {
    //----start TESTING: draw double bearing------------------------------------------------
    //Note: "straight ahead" is 0 degree bearing, -80 degree on left, +80 degree to right
    float degAngle = leftBearing; //Change angle of bearing to draw here
    float degAngle2 = rightBearing;
    float d = 7000;
    float theta = degAngle * 3.14159/180.0;
    float theta2 = degAngle2 * 3.14159/180.0;
    float roverWidthDiv2 = 1500/2;

    vec3 bearing = vec3(d*sin(theta), 0, d*cos(theta));
    vec3 bearing2 = vec3(d*sin(theta2), 0, d*cos(theta2));

    vec3 leftBearingStart = vec3(-roverWidthDiv2 * cos(theta), 500, roverWidthDiv2 * sin(theta));
    vec3 rightBearingStart = vec3(roverWidthDiv2 * cos(theta), 500, -roverWidthDiv2 * sin(theta));

    vec3 leftBearingStart2 = vec3(-roverWidthDiv2 * cos(theta2), 500, roverWidthDiv2 * sin(theta2));
    vec3 rightBearingStart2 = vec3(roverWidthDiv2 * cos(theta2), 500, -roverWidthDiv2 * sin(theta2));

    std::vector<vec3> ptsLft1 = {
        leftBearingStart,
        leftBearingStart + bearing
    };
    std::vector<vec3> ptsRght1 = {
        rightBearingStart,
        rightBearingStart + bearing
    };
    std::vector<vec3> colors = {
        vec3(1.0f, 0.0f, 0.0f),
        vec3(1.0f, 0.0f, 0.0f)
    };

    std::vector<vec3> ptsLft2 = {
      leftBearingStart2,
      leftBearingStart2 + bearing2
    };
    std::vector<vec3> ptsRght2 = {
        rightBearingStart2,
        rightBearingStart2 + bearing2
    };

    std::vector<int> indices {0, 1, 0};

    Object3D leftBearing(ptsLft1, colors, indices);
    Object3D rightBearing(ptsRght1, colors, indices);

    Object3D leftBearing2(ptsLft2, colors, indices);
    Object3D rightBearing2(ptsRght2, colors, indices);

    viewer.addObject(std::move(leftBearing), true);
    viewer.addObject(std::move(rightBearing), true);

    viewer.addObject(std::move(leftBearing2), true);
    viewer.addObject(std::move(rightBearing2), true);
}

void ObsDetector::spinViewer() {
    if(viewer.procStage == ProcStage::POSTBEARING || viewer.procStage == ProcStage::POSTBOUNDING) {
        createBoundingBoxes();
        if(viewer.procStage == ProcStage::POSTBEARING) createBearing();
    }

    //---end TESTING add double bearing ----------------------------------------------------
    viewer.update();
    viewer.clearEphemerals();
    
}

 ObsDetector::~ObsDetector() {
     delete passZ;
     delete ransacPlane;
     delete voxelGrid;
     delete ece;
 }

bool ObsDetector::open() {
    return viewer.open();
}


int main() {
    ObsDetector obs(DataSource::ZED, OperationMode::DEBUG, ViewerType::GL);

    //std::thread updateTick( [&]{while(true) { obs.update();} });

    try {
        ObsDetector obs(DataSource::FILESYSTEM, OperationMode::DEBUG, ViewerType::GL);
        while(obs.open()) {
            obs.update();
            obs.spinViewer();
        }
        return EXIT_SUCCESS;
    } catch (std::exception const& exception) {
        std::cerr << "Exception: " << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
