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

    //set AR Tag params
    cv::FileStorage fsr("percep_obs_detect/src/alvar_dict.yml", cv::FileStorage::READ);
        if (!fsr.isOpened()) {  //throw error if dictionary file does not exist
            std::cerr << "ERR: \"alvar_dict.yml\" does not exist! Create it before running main\n";
            throw Exception();
        }

        // read dictionary from file
        int mSize, mCBits;
        cv::Mat bits;
        fsr["MarkerSize"] >> mSize;
        fsr["MaxCorrectionBits"] >> mCBits;
        fsr["ByteList"] >> bits;
        fsr.release();
        alvarDict = new cv::aruco::Dictionary(bits, mSize, mCBits);

    alvarParams = new cv::aruco::DetectorParameters();
    alvarParams.markerBorderBits = 2; 
    alvarParams.doCornerRefinement = 0;
    alvarParams.polygonalApproxAccuracyRate = 0.08;
}

cv::Point2f ObsDetector::getAverageTagCoordinateFromCorners(const vector<cv::Point2f> &corners) {  //gets coordinate of center of tag
    // RETURN:
    // Point2f object containing the average location of the 4 corners
    // of the passed-in tag
    cv::Point2f avgCoord;
    for (auto &corner : corners) {
        avgCoord.x += corner.x;
        avgCoord.y += corner.y;
    }
    avgCoord.x /= corners.size();
    avgCoord.y /= corners.size();
    return avgCoord;
}

pair<Tag, Tag> ObsDetector::findARTags(cv::Mat &src, cv::Mat &depth_src, cv::Mat &rgb) {  //detects AR tags in source Mat and outputs Tag objects for use in LCM
    // RETURN:
    // pair of target objects- each object has an x and y for the center,
    // and the tag ID number return them such that the "leftmost" (x
    // coordinate) tag is at index 0
    cv::cvtColor(src, rgb, cv::COLOR_RGBA2RGB);
    // clear ids and corners vectors for each detection
    ids.clear();
    corners.clear();

    // Find tags
    cv::aruco::detectMarkers(rgb, alvarDict, corners, ids, alvarParams);

    // create Tag objects for the detected tags and return them
    pair<Tag, Tag> discoveredTags;
    if (ids.size() == 0) {
        // no tags found, return invalid objects with tag set to -1
        discoveredTags.first.id = -1;
        discoveredTags.first.loc = cv::Point2f();
        discoveredTags.second.id = -1;
        discoveredTags.second.loc = cv::Point2f();

    } else if (ids.size() == 1) {  // exactly one tag found
        discoveredTags.first.id = ids[0];
        discoveredTags.first.loc = getAverageTagCoordinateFromCorners(corners[0]);
        // set second tag to invalid object with tag as -1
        discoveredTags.second.id = -1;
        discoveredTags.second.loc = cv::Point2f();
    } else if (ids.size() == 2) {  // exactly two tags found
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t1.id = ids[1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[1]);
        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    } else {  // detected >=3 tags
        // return leftmost and rightsmost detected tags to account for potentially seeing 2 of each tag on a post
        Tag t0, t1;
        t0.id = ids[0];
        t0.loc = getAverageTagCoordinateFromCorners(corners[0]);
        t1.id = ids[ids.size() - 1];
        t1.loc = getAverageTagCoordinateFromCorners(corners[ids.size() - 1]);
        if (t0.loc.x < t1.loc.x) {  //if tag 0 is left of tag 1, put t0 first
            discoveredTags.first = t0;
            discoveredTags.second = t1;
        } else {  //tag 1 is left of tag 0, put t1 first
            discoveredTags.first = t1;
            discoveredTags.second = t0;
        }
    }
    return discoveredTags;
}   

void ObsDetector::update() {
    GPU_Cloud pc; 
    sl::Mat frame(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);

    if(source == DataSource::ZED) {
        zed.grab();
        zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res); 

        cv::Mat zedDepth(sl::RESOLUTION::VGA, sl::MAT_TYPE::F32_C1, sl::MEM::GPU);
        zed.retrieveMeasure(zedDepth, sl::MEASURE::DEPTH, sl::MEM::GPU, zed.getResolution());

        cv::Mat zedImage(sl::RESOLUTION::VGA, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
        zed.retrieveImage(zedImage, sl::VIEW::LEFT, sl::MEM::GPU, zed.getResolution());

        getRawCloud(pc, frame);

        sl::Mat rgb;

        pair<Tag, Tag> tags = findARTags(zedImage, zedDepth, rgb);
        
    } else if(source == DataSource::FILESYSTEM) {
        pc = fileReader.readCloudGPU(viewer.frame);
        if (viewer.frame == 1) viewer.setTarget();
    }
    update(pc);
    findARTags(zedImage, , );

    if(source == DataSource::FILESYSTEM) deleteCloud(pc);
} 

///home/ashwin/Documents/mrover-workspace/jetson/percep_obs_detect/data
// Call this directly with ZED GPU Memory
void ObsDetector::update(GPU_Cloud pc) {

    // Get a copy if debug is enabled
    viewer.updatePointCloud(pc);

    // Processing
    if(viewer.procStage != ProcStage::RAW) {
        passZ->run(pc);
        if(viewer.procStage != ProcStage::POSTPASS) {
            ransacPlane->computeModel(pc);
        }
    }    
    
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
            for(int q = 0; q < 8; q++) colors.push_back(vec3(0.0f, 1.0f, 0.0f));
            std::vector<int> indicies = {0, 1, 2, 2, 3, 0, 1, 2, 5, 5, 6, 2, 0, 3, 4, 3, 7, 4, 4, 5, 6, 7, 6, 5};
            Object3D obj(points, colors, indicies);
            viewer.addObject(obj, true);
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

    std::vector<int> indicies = {0, 1, 0}; 

    Object3D leftBearing(ptsLft1, colors, indicies);
    Object3D rightBearing(ptsRght1, colors, indicies);

    Object3D leftBearing2(ptsLft2, colors, indicies);
    Object3D rightBearing2(ptsRght2, colors, indicies);

    viewer.addObject(leftBearing, true);
    viewer.addObject(rightBearing, true);

    viewer.addObject(leftBearing2, true);
    viewer.addObject(rightBearing2, true);
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



int main() {
    ObsDetector obs(DataSource::ZED, OperationMode::DEBUG, ViewerType::GL);

    //std::thread updateTick( [&]{while(true) { obs.update();} });

    while(true) {
        obs.update();
        obs.spinViewer();
    }
    

    return 0;
}
