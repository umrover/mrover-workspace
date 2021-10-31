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
	//fileReader.open("data/");
        cout << "File data dir: " << endl;
        cout << "[e.g: /home/ashwin/Documents/mrover-workspace/jetson/percep_obs_detect/data]" << endl;
        getline(cin, readDir);
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
}


void ObsDetector::update() {
    GPU_Cloud pc;

    if(source == DataSource::ZED) {

        sl::Mat frame(cloud_res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);
        zed.grab();
        zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, cloud_res);
        getRawCloud(pc, frame);

    } else if(source == DataSource::FILESYSTEM) {

        pc = fileReader.readCloudGPU(frameNum);
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


    ///*/
    // Rendering
    if(mode != OperationMode::SILENT) {
        //viewer.addPointCloud();
        //viewer.remove
        //viewer.updatePointCloud(pc);
    }


    // Recording
    if(record) record = true;

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

    viewer.update();
    viewer.clearEphemerals();

}

void ObsDetector::test(const vector<GPU_Cloud>& raw_data, const vector<ObsReturn>& truth_list)
{
  vector<ObsReturn> measured;
  measured.reserve(raw_data.size()); //vector of experimental ObsReturns

  //run the raw_data through the code and store the ObsReturn structs in measured
  for(size_t i = 0; i < raw_data.size(); i++)
  {
    Bins b;
    passZ->run(raw_data[i]);
    ransacPlane->computeModel(raw_data[i]);
    b = voxelGrid->run(raw_data[i]);
    measured.push_back(ece->extractClusters(raw_data[i],b));
  }

  //calculate the total volume of the truth obstacles for each ObsReturn in truth_list
  vector<float> true_volumes; //total volume of all truth obstacles
  true_volumes.reserve(truth_list.size());

  for (size_t i = 0; i < truth_list.size(); ++i) { //all ObsReturns
      float volume = 0;
      for (size_t j = 0; j < truth_list[i].obs.size(); ++j) { // all Obstacles
          volume += (truth_list[i].obs[j].maxX - truth_list[i].obs[j].minX)
              * (truth_list[i].obs[j].maxY - truth_list[i].obs[j].minY)
              * (truth_list[i].obs[j].maxZ - truth_list[i].obs[j].minZ); //volume calculation for this obstacle
      }
      true_volumes.push_back(volume);
  }
  //now true_volumes has total volume for each cloud in the same indices as truth_list and raw_data

  //need to find volume of truth detected / volume truth for each ObsReturn
  vector<float> voldet_voltru;
  voldet_voltru.reserve(truth_list.size());
  for (size_t i = 0; i < measured.size(); ++i) {
      //i can be the index for both measured and truth since they are the same size
      float running_intersection = 0;
      for (size_t tru_ob = 0; tru_ob < truth_list[i].obs.size(); ++tru_ob) { //loop through truth objects for this cloud
          for (size_t exp_ob = 0; exp_ob < measured[i].obs.size(); ++exp_ob) { //loop through experimental objects for this cloud
              //calculate intersection and add to running total
              running_intersection += calculateIntersection(truth_list[i].obs[tru_ob], measured[i].obs[exp_ob]);
          }
      }
      //divide the total intersection found by the total truth volume - this is one of the indicators of performance
      float evaluation = running_intersection / true_volumes[i];
      voldet_voltru.push_back(evaluation);
      //Ideally, they would be 1, the further from 1 any of the values in voldet_voltru are, the lower the accuracy
 }

  //TODO: find the vol false positive over volume of truth for each point cloud

}

float ObsDetector::calculateIntersection(const EuclideanClusterExtractor::Obstacle& truth_obst, const EuclideanClusterExtractor::Obstacle& eval_obst) {
    float xa = (std::max(truth_obst.minX, eval_obst.minX));
    float xb = (std::min(truth_obst.maxX, eval_obst.maxX));
    float ya = (std::max(truth_obst.minY, eval_obst.minY));
    float yb = (std::min(truth_obst.maxY, eval_obst.maxY));
    float za = (std::max(truth_obst.minZ, eval_obst.minZ));
    float zb = (std::min(truth_obst.maxZ, eval_obst.maxZ));
    return (xa - xb) * (ya - yb) * (za - zb);
}

double ObsDetector::calculateIOU(const EuclideanClusterExtractor::Obstacle& truth_obst, const EuclideanClusterExtractor::Obstacle& eval_obst)
{
  float xa = (std::max(truth_obst->minX,eval_obst->minX));
  float xb = (std::min(truth_obst->maxX,eval_obst->maxX));
  float ya = (std::max(truth_obst->minY,eval_obst->minY));
  float yb = (std::min(truth_obst->maxY,eval_obst->maxY));
  float za = (std::max(truth_obst->minZ,eval_obst->minZ));
  float zb = (std::min(truth_obst->maxZ,eval_obst->maxZ));

  float intersection = (xa - xb) * (ya - yb) * (za - zb);
  float u = (
    (truth_obst->maxX - truth_obst->minX) * (truth_obst->maxY - truth_obst->minY) * (truth_obst->maxZ - truth_obst->minZ) +
    (eval_obst->maxX - eval_obst->minX) * (eval_obst->maxY - eval_obst->minY) * (eval_obst->maxZ - eval_obst->minZ) -
    intersection);

  return intersection / u;
}

void test_test_test()
{
  vector<ObsReturn> measured;
  measured.reserve(truth_list.size());
  vector<float> volumes;
  volumes.reserve(truth_list.size());
  vector<float> g_t;
  intersection_totals.reserve(truth_list.size());

  for(size_t i = 0; i < truth_list.size(); i++)
  {
    /* Evaluate Raw Data */
    Bins b;
    passZ->run(raw_data[i]);
    ransacPlane->computeModel(raw_data[i]);
    b = voxelGrid->run(raw_data[i]);
    measured.push_back(ece->extractClusters(raw_data[i],b));
    /* --------------- */

    float current_volume_sum = 0;
    float current_intersection_sum = 0;
    for(size_t j = 0; j < truth_list[i].obs.size(); j++)
      for(size_t k = 0; k < measured[i].obs.size(); k++)
      {
        current_intersection_sum +=
        calculateIntersection(truth_list[i].obs[j], measured[i].obs[k]) > 0 ? calculateIntersection(truth_list[i].obs[j], measured[i].obs[k]) : 0;
        current_volume_sum += (truth_list[i].obs[j].maxX - truth_list[i].obs[j].minX)
            * (truth_list[i].obs[j].maxY - truth_list[i].obs[j].minY)
            * (truth_list[i].obs[j].maxZ - truth_list[i].obs[j].minZ);
      }
    volumes.push_back(current_volume_sum);
    g_t.push_back(current_intersection_sum / current_volume_sum);
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

    //std::thread updateTick( [&]{while(true) { obs.update();} });

    while(true) {
       obs.update();
       obs.spinViewer();
    }


    return 0;
}
