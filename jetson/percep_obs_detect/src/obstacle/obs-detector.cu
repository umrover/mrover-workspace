#include "obs-detector.h"

#include <chrono>

using namespace std;
using namespace std::chrono;

#include <glm/glm.hpp>
#include <vector>

ViewerType parse_viewer_type(const rapidjson::Document& mRoverConfig) {
    // Find viewer type
    ViewerType viewerType;
    std::string v_type = mRoverConfig["startup"]["viewer_type"].GetString();
    if (v_type == "gl") {
        viewerType = ViewerType::GL;
    } else if (v_type == "none") {
        viewerType = ViewerType::NONE;
    } else {
        throw std::runtime_error("Invalid viewer type");
    }
    return viewerType;
}

ObsDetector::ObsDetector(const rapidjson::Document& mRoverConfig, camera_ptr cam)
        : cam{cam}, timer("Update") {
    setupParamaters(mRoverConfig);

    mode = parse_operation_mode(mRoverConfig);
    viewerType = parse_viewer_type(mRoverConfig);

    previousTime = std::chrono::steady_clock::now();

    //Init Viewer
    if (viewerType == ViewerType::GL) {
        viewer.initGraphics();
        viewer.addPointCloud();
        if (cam->get_max_frame() == -1) {
            viewer.framePlay = true;
        }
    }
};

//TODO: Make it read params from a file
void ObsDetector::setupParamaters(const rapidjson::Document& config) {
    //Operating resolution
    cloud_res = sl::Resolution(
            config["camera"]["resolution_width"].GetInt(),
            config["camera"]["resolution_height"].GetInt());

    //Obs Detecting Algorithm Params
    // TODO: add these to the config file
    passZ = new PassThrough('z', 100, 7000); //7000
    ransacPlane = new RansacPlane(make_float3(0, 1, 0), 8, 600, 80, cloud_res.area(), 80);
    voxelGrid = new VoxelGrid(10);
    ece = new EuclideanClusterExtractor(128, 30, 0, cloud_res.area(), 9);
    findClear = new FindClearPath();
    refineGround = new RefineGround();
}


void ObsDetector::update() {
    auto now = chrono::steady_clock::now();
    auto delta = now - previousTime;
    frameCount++;
    isEverySecondMarker = delta > chrono::seconds(1);
    if (isEverySecondMarker) {
        currentFPS = viewer.currentFPS = frameCount;
        previousTime = now;
        frameCount = 0;
    }
    GPU_Cloud pc = cam->get_cloud();

    if (viewer.record) {
        cam->write_data();
    }

    process(pc);
}

vec3 closestPointOnPlane(Plane plane, vec3 point) {
    vec3 normal{plane.normal.x, plane.normal.y, plane.normal.z};
    vec3 p0{plane.p0.x, plane.p0.y, plane.p0.z};
    // from equation of distance from point to plane
    float distanceToPlane = (glm::dot(normal, point) - glm::dot(normal, p0)) / glm::length(normal);
    // slide point along the normal vector until it lies on the plane (which is its distance to the plane)
    return point - glm::normalize(normal) * distanceToPlane;
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
    vector<vec3> vertices{
            cameraProjOnPlane + cameraRight,
            cameraProjOnPlane - cameraRight,
            cameraProjOnPlane + cameraForward * 2.0f + cameraRight,
            cameraProjOnPlane + cameraForward * 2.0f - cameraRight,
    };
    vector<vec3> colors(vertices.size(), {1.0f, 1.0f, 0.0f});
    vector<int> indices{0, 1, 2, 1, 2, 3};
    viewer.addObject({vertices, colors, indices}, true);
}

void ObsDetector::handleParameters() {
    if (viewer.doParameterInit) {
        viewer.epsilon = ransacPlane->epsilon;
        viewer.iterations = ransacPlane->getIterations();
        viewer.threshold = ransacPlane->threshold;
        viewer.removalRadius = ransacPlane->removalRadius;
        viewer.removeGround = true;
        viewer.minSize = ece->minSize;
        viewer.tolerance = ece->tolerance;
        viewer.doParameterInit = false;
        viewer.refineDistance = refineGround->refineDistance;
        viewer.refineHeight = refineGround->refineHeight;
    } else {
        ransacPlane->epsilon = viewer.epsilon;
        ransacPlane->setIterations(viewer.iterations);
        ransacPlane->threshold = viewer.threshold;
        ransacPlane->removalRadius = viewer.removalRadius;
        ransacPlane->filterOp = viewer.removeGround ? FilterOp::REMOVE : FilterOp::COLOR;
        ece->minSize = viewer.minSize;
        ece->tolerance = viewer.tolerance;
        refineGround->refineDistance = viewer.refineDistance;
        refineGround->refineHeight = viewer.refineHeight;
    }
}

// Call this directly with ZED GPU Memory
void ObsDetector::process(GPU_Cloud pc) {
    handleParameters();

    Plane plane;

    timer.reset();

    GPU_Cloud pcRaw;
    if (viewer.procStage == ProcStage::RAW_BOUNDING_BEARING) {
        pcRaw = createCloud(pc.size);
        copyCloud(pcRaw, pc);
    }

    // Processing
    if (viewer.procStage > ProcStage::RAW) {
        passZ->run(pc);
        averages["Pass Z"].add(timer.reset());
    }

    if (viewer.procStage > ProcStage::POSTPASS) {
        plane = ransacPlane->computeModel(pc);
        if (viewerType == ViewerType::GL && viewer.procStage != ProcStage::RAW_BOUNDING_BEARING) drawGround(plane);
        averages["RANSAC"].add(timer.reset());
    }

    viewer.maxFrame = cam->get_max_frame();
    if (viewer.framePlay || viewer.maxFrame == -1) {
        viewer.frame = cam->get_frame();
    } else {
        cam->set_frame(viewer.frame);
    }
    if (viewerType == ViewerType::GL && viewer.procStage != ProcStage::RAW_BOUNDING_BEARING) viewer.updatePointCloud(pc);
    averages["Update PC"].add(timer.reset());

    if (viewer.procStage > ProcStage::POSTRANSAC) {
        Bins bins = voxelGrid->run(pc);
        obstacles = ece->extractClusters(pc, bins);
        averages["Cluster"].add(timer.reset());
        refineGround->pruneObstacles(plane, obstacles);
        averages["Refine Ground"].add(timer.reset());
        bearingCombined = findClear->find_clear_path_initiate(obstacles);
        averages["Clear Path"].add(timer.reset());
        leftBearing = bearingCombined.x;
        rightBearing = bearingCombined.y;
        distance = bearingCombined.z;
        populateMessage(leftBearing, rightBearing, distance);
    }

    std::cout << std::flush;

    if (viewerType == ViewerType::GL) {
        if (viewer.procStage > ProcStage::POSTECE) {
            drawBoundingBoxes();
        }

        if (viewer.procStage > ProcStage::POSTBOUNDING) {
            drawBearing();
        }

        if (viewer.procStage == ProcStage::RAW_BOUNDING_BEARING) {
            viewer.updatePointCloud(pcRaw);
        }
    }

    if (!viewer.framePlay) {
        cam->ignore_grab();
    }
    averages["Draw"].add(timer.reset());
    if (isEverySecondMarker) {
        viewer.stageTimings.clear();
        for (auto& average: averages) {
            viewer.stageTimings[average.first] = average.second.getAverage();
            average.second.reset();
        }
    }
}

void ObsDetector::populateMessage(float leftBearing, float rightBearing, float distance) {
#ifdef WITH_JARVIS
    this->leftBearing = leftBearing;
    this->rightBearing = rightBearing;
    this->distance = distance;
    obstacleMessage.bearing = leftBearing;
    obstacleMessage.distance = distance;
    lcm_.publish("/obstacle", &obstacleMessage);
#endif
}


void ObsDetector::drawBoundingBoxes() {
    // This creates bounding boxes for visualization
    // There might be a clever automatic indexing scheme to optimize this
    for (int i = 0; i < obstacles.obs.size(); i++) {
        if (obstacles.obs[i].minX < obstacles.obs[i].maxX && obstacles.obs[i].minZ < obstacles.obs[i].maxZ) {
            std::vector<vec3> points = {vec3(obstacles.obs[i].minX, obstacles.obs[i].minY, obstacles.obs[i].minZ),
                                        vec3(obstacles.obs[i].maxX, obstacles.obs[i].minY, obstacles.obs[i].minZ),
                                        vec3(obstacles.obs[i].maxX, obstacles.obs[i].maxY, obstacles.obs[i].minZ),
                                        vec3(obstacles.obs[i].minX, obstacles.obs[i].maxY, obstacles.obs[i].minZ),
                                        vec3(obstacles.obs[i].minX, obstacles.obs[i].minY, obstacles.obs[i].maxZ),
                                        vec3(obstacles.obs[i].maxX, obstacles.obs[i].minY, obstacles.obs[i].maxZ),
                                        vec3(obstacles.obs[i].maxX, obstacles.obs[i].maxY, obstacles.obs[i].maxZ),
                                        vec3(obstacles.obs[i].minX, obstacles.obs[i].maxY, obstacles.obs[i].maxZ),};
            std::vector<vec3> colors;
            for (int q = 0; q < 8; q++) colors.emplace_back(0.0f, 1.0f, 0.0f);
            std::vector<int> indices = {0, 1, 2, 2, 3, 0, 1, 2, 5, 5, 6, 2, 0, 3, 4, 3, 7, 4, 4, 5, 6, 7, 6, 5};
            viewer.addObject({points, colors, indices}, true);
        }
    }
}

void ObsDetector::drawBearing() {
    //----start TESTING: draw double bearing------------------------------------------------
    //Note: "straight ahead" is 0 degree bearing, -80 degree on left, +80 degree to right
    float degAngle = leftBearing; //Change angle of bearing to draw here
    float degAngle2 = rightBearing;
    float d = 7000;
    float theta = degAngle * 3.14159 / 180.0;
    float theta2 = degAngle2 * 3.14159 / 180.0;
    float roverWidthDiv2 = 1500 / 2;

    vec3 bearing = vec3(d * sin(theta), 0, d * cos(theta));
    vec3 bearing2 = vec3(d * sin(theta2), 0, d * cos(theta2));

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

    std::vector<int> indices{0, 1, 0};

    Object3D leftBearing1(ptsLft1, colors, indices);
    Object3D rightBearing1(ptsRght1, colors, indices);

    Object3D leftBearing2(ptsLft2, colors, indices);
    Object3D rightBearing2(ptsRght2, colors, indices);

    viewer.addObject(std::move(leftBearing1), true);
    viewer.addObject(std::move(rightBearing1), true);

    viewer.addObject(std::move(leftBearing2), true);
    viewer.addObject(std::move(rightBearing2), true);
}

void ObsDetector::spinViewer() {
    if (viewerType == ViewerType::GL) {
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

bool ObsDetector::open() {
    return viewerType == ViewerType::NONE || viewer.open();
}

