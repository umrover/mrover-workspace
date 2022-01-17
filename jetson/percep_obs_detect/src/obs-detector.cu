#include "obs-detector.h"
#include <chrono>

using namespace std;
using namespace std::chrono;

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>

ObsDetector::ObsDetector(DataSource source, OperationMode mode, ViewerType viewerType)
        : source(source), mode(mode), viewerType(viewerType) {
    setupParamaters("");

    //Init data stream from source
    if (source == DataSource::ZED) {
        auto error = zed.open(init_params);
        if (error != sl::ERROR_CODE::SUCCESS) {
            throw std::runtime_error("Error opening ZED camera");
        }
        //auto camera_config = zed.getCameraInformation(cloud_res).camera_configuration;
        //defParams = camera_config.calibration_parameters.left_cam;
    } else if (source == DataSource::FILESYSTEM) {
        std::string s = ROOT_DIR;
        s += "/data2/";

        cout << "File data dir: " << endl;
        cout << "[defaulting to: " << s << endl;
        getline(cin, readDir);
        if (readDir == "") readDir = s;
        fileReader.open(readDir);
    }

    previousTime = std::chrono::steady_clock::now();

    //Init Viewer
    if (viewerType == ViewerType::GL) {
        viewer.initGraphics();
        viewer.addPointCloud();
    }

    frameCounter = 0;

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

    // Set writer params
    frameGap = 10;

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
    ece = new EuclideanClusterExtractor(128, 30, 0, cloud_res.area(), 9);
    findClear = new FindClearPath();
}


void ObsDetector::update() {
    auto now = chrono::steady_clock::now();
    auto delta = now - previousTime;
    frameCount++;
    if (delta > chrono::seconds(1)) {
        currentFPS = viewer.currentFPS = frameCount;
        previousTime = now;
        frameCount = 0;
    }
    GPU_Cloud pc;
    // sl::Mat frame is outside of if statement scope since it owns the memory and we don't want that to de-allocate sooner than expected
    sl::Mat frame(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);
    if (source == DataSource::ZED) {
        zed.grab();
        zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res);
        getRawCloud(pc, frame);
        if (viewer.record) {
            if ((frameCounter % frameGap) == 0) {
                std::stringstream ss;
                ss << ROOT_DIR << "/data3/pcl" << std::to_string(frameCounter / frameGap) << ".pcd";
                fileWriter.writeCloud(ss.str(), pc, cloud_res.width, cloud_res.height);
            }
            ++frameCounter;
        }
    } else if (source == DataSource::FILESYSTEM) {
        pc = fileReader.getCloudGPU(viewer.frame);
        viewer.maxFrame = fileReader.size();
        if (viewer.frame == 1) viewer.setCenter();
    }
    update(pc);

    if (source == DataSource::FILESYSTEM) deleteCloud(pc);
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
    // make the center of the plane quad lined up with point cloud centerf
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
    } else {
        ransacPlane->epsilon = viewer.epsilon;
        ransacPlane->setIterations(viewer.iterations);
        ransacPlane->threshold = viewer.threshold;
        ransacPlane->removalRadius = viewer.removalRadius;
        ransacPlane->filterOp = viewer.removeGround ? FilterOp::REMOVE : FilterOp::COLOR;
        ece->minSize = viewer.minSize;
        ece->tolerance = viewer.tolerance;
    }
}

///home/ashwin/Documents/mrover-workspace/jetson/percep_obs_detect/data
// Call this directly with ZED GPU Memory
void ObsDetector::update(GPU_Cloud pc) {
    handleParameters();

    // Processing
    if (viewer.procStage > ProcStage::RAW) {
        passZ->run(pc);
    }

    if (viewer.procStage > ProcStage::POSTPASS) {
        Plane plane = ransacPlane->computeModel(pc);
        if (viewerType == ViewerType::GL) drawGround(plane);
    }

    if (viewerType == ViewerType::GL) viewer.updatePointCloud(pc);

    if (viewer.procStage > ProcStage::POSTRANSAC) {
        Bins bins = voxelGrid->run(pc);
        obstacles = ece->extractClusters(pc, bins);
        bearingCombined = findClear->find_clear_path_initiate(obstacles);
        leftBearing = bearingCombined.x;
        rightBearing = bearingCombined.y;
        distance = bearingCombined.z;
        populateMessage(leftBearing, rightBearing, distance);
    }

    if (viewerType == ViewerType::GL) {
        if (viewer.procStage > ProcStage::POSTECE) {
            drawBoundingBoxes();
        }

        if (viewer.procStage > ProcStage::POSTBOUNDING) {
            drawBearing();
        }

        if (viewer.framePlay) {
            viewer.frame++;
            if (viewer.frame >= viewer.maxFrame) {
                viewer.frame = 0;
            }
        }
    }

//    writer.write()
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

int main() {
    try {
        ObsDetector obs(DataSource::FILESYSTEM, OperationMode::DEBUG, ViewerType::GL);

        std::string const gstLaunch = "appsrc ! video/x-raw, format=BGR !"
                                      "queue ! videoconvert ! x264enc tune=zerolatency bitrate=1000000 speed-preset=superfast !"
                                      "rtph264pay ! udpsink host=127.0.0.1 port=5000";
        // Corresponding viewer command:
        // gst-launch-1.0 udpsrc port=5000 ! "application/x-rtp, encoding-name=(string)H264" ! queue ! rtph264depay ! decodebin ! videoconvert ! autovideosink
        cv::Size streamSize(640, 480);
        cv::VideoWriter writer(gstLaunch, 0, 60, streamSize);
        cv::Mat send(streamSize, CV_8UC3);

        while (obs.open()) {
            obs.update();
            obs.spinViewer();

            if (writer.isOpened()) {
                int width, height;
                obs.viewer.getWindowSize(&width, &height);
                cv::Mat framebuffer(height, width, CV_8UC3);
                glPixelStorei(GL_PACK_ALIGNMENT, 1); // ensure no padding in between pixel rows, 1=byte alignment, see glPixelStorei documentation
                glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, framebuffer.data);
                cv::resize(framebuffer, send, send.size());
                // TODO: use cv::copyMakeBorder for black bars
                cv::flip(send, send, 0);
                writer.write(send);
            }
        }
        return EXIT_SUCCESS;
    } catch (std::exception const& exception) {
        std::cerr << "Exception: " << exception.what() << std::endl;
        return EXIT_FAILURE;
    }
}
