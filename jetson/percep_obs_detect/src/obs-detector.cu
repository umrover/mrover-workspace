#include "obs-detector.h"
#include <chrono>

using namespace std;
using namespace std::chrono;
#include <glm/vec4.hpp> // glm::vec4
#include <glm/glm.hpp>
#include <vector>
#include <string>
#include <numeric>
#include <iomanip>
#include <algorithm>
#include <fstream>

ObsDetector::ObsDetector(DataSource source, OperationMode mode, ViewerType viewerType) : source(source), mode(mode), viewerType(viewerType), record(false)
{
    setupParamaters("");
    if (mode == OperationMode::TEST) {
        std::cout << "In Testing Mode\n";
    }else{
        //Init data stream from source
        if (source == DataSource::ZED) {
            zed.open(init_params);
            auto camera_config = zed.getCameraInformation(cloud_res).camera_configuration;
            defParams = camera_config.calibration_parameters.left_cam;
        }
        else if (source == DataSource::FILESYSTEM) {
            std::string s = ROOT_DIR;
            s += "/data/";

            cout << "File data dir: " << endl;
            cout << "[defaulting to: " << s << endl;
            getline(cin, readDir);

            if (mode != OperationMode::TEST) {
                //if we're testing, we only want one cloud
                if (readDir == "")  readDir = s;
                fileReader.open(readDir);
            }
        }
      }//end if not test
        //Init Viewer
        if (mode != OperationMode::SILENT && viewerType == ViewerType::GL) {
            int argc = 1;
            char* argv[1] = { (char*)"Window" };
            viewer.init(argc, argv);
            viewer.addPointCloud();
        }
        if (mode == OperationMode::TEST) {
            test_input_file();
        }
};

//TODO: Make it read params from a file
void ObsDetector::setupParamaters(std::string parameterFile) {
    //Operating resolution
    cloud_res = sl::Resolution(320, 180);
    readDir = "/home/mrover/mrover-workspace/jetson/percep_obs_detect/data/";

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
        if (viewer.frame == 1) viewer.setTarget();
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
    bearingCombined = findClear->find_clear_path_initiate(obstacles);
    leftBearing = bearingCombined.x;
    rightBearing = bearingCombined.y;
    distance = bearingCombined.z;

    ///*/
    // Rendering
    if(mode != OperationMode::SILENT) {
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

void ObsDetector::drawCubes(EuclideanClusterExtractor::ObsReturn obsList, bool color_flag)
{
    for (int i = 0; i < obsList.obs.size(); i++) {
        std::vector<vec3> points = {vec3(obsList.obs[i].minX, obsList.obs[i].minY, obsList.obs[i].minZ),
                                    vec3(obsList.obs[i].maxX, obsList.obs[i].minY, obsList.obs[i].minZ),
                                    vec3(obsList.obs[i].maxX, obsList.obs[i].maxY, obsList.obs[i].minZ),
                                    vec3(obsList.obs[i].minX, obsList.obs[i].maxY, obsList.obs[i].minZ),
                                    vec3(obsList.obs[i].minX, obsList.obs[i].minY, obsList.obs[i].maxZ),
                                    vec3(obsList.obs[i].maxX, obsList.obs[i].minY, obsList.obs[i].maxZ),
                                    vec3(obsList.obs[i].maxX, obsList.obs[i].maxY, obsList.obs[i].maxZ),
                                    vec3(obsList.obs[i].minX, obsList.obs[i].maxY, obsList.obs[i].maxZ), };
        std::vector<vec3> colors;
        if (color_flag)
            for (int q = 0; q < 8; q++) colors.push_back(vec3(0.0f, 1.0f, 0.0f)); //green
        else
            for (int q = 0; q < 8; q++) colors.push_back(vec3(1.0f, 0.0f, 0.0f)); //red

        std::vector<int> indicies = { 0, 1, 2, 2, 3, 0, 1, 2, 5, 5, 6, 2, 0, 3, 4, 3, 7, 4, 4, 5, 6, 7, 6, 5 };
        Object3D obj(points, colors, indicies);
        viewer.addObject(obj, true);
    }
}

void ObsDetector::spinViewer() {
    // This creates bounding boxes for visualization
    // There might be a clever automatic indexing scheme to optimize this
    drawCubes(obstacles, 1); // 1 is green, 0 is red

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

    std::vector<int> indicies = { 0, 1, 0 };

    Object3D leftBearing(ptsLft1, colors, indicies);
    Object3D rightBearing(ptsRght1, colors, indicies);

    Object3D leftBearing2(ptsLft2, colors, indicies);
    Object3D rightBearing2(ptsRght2, colors, indicies);

    viewer.addObject(leftBearing, true);
    viewer.addObject(rightBearing, true);

    viewer.addObject(leftBearing2, true);
    viewer.addObject(rightBearing2, true);
    //---end TESTING add double bearing ----------------------------------------------------

    viewer.update();
    viewer.clearEphemerals();
}


//TESTING: edit SimpleSceneGenerator main function to make the cloud and obsreturn
//          This lets you make the obstacles you want, edit noise settings etc. (jetson/percep_obs_detect/tools/SimpleSceneGenerator.cpp)
//Instructions: When running the code, stop the run after it prints the test output so it isn't lost
void ObsDetector::test_input_file()
{
  //If you are testing a cloud, change the filepath to work for your computer and change the PCD file name to the one selected in SimpleSceneGenerator
    std::cout << "Enter filepath to data that you want to test:\n";
    std::string fpath;
    std::cin >> fpath;
  GPU_Cloud gpuc = fileReader.readCloudGPU(fpath); //read the cloud
  std::cout << "AAAAAAAA\n\n";

  std::vector<GPU_Cloud> raw_data;
  raw_data.push_back(gpuc);

  std::vector<EuclideanClusterExtractor::ObsReturn> truths;
  EuclideanClusterExtractor::ObsReturn objects;
  //Init all obstacles to be added to the scene and push them to ObsReturn.obs
  // In th future (with scene generator): Use commented code below rather than running a cloud through the pipeline for the truth
  //Obstacle <name> = {minX, maxX, minY, maxY, minZ, maxZ};
  //EuclideanClusterExtractor::Obstacle one = { 0, 20, 0, 20, 0, 20 };*/
  //objects.obs.push_back(one);
  //truths.push_back(objects);

  Bins b;

  passZ->run(raw_data[0]); //The filter and RANSAC get rid of small scenes entirely
  ransacPlane->computeModel(raw_data[0]);

  b = voxelGrid->run(raw_data[0]);
  objects = ece->extractClusters(raw_data[0], b);
  truths.push_back(objects);
  cout << "\n\nBBBBBBB\n\n";
  test(raw_data, truths);

}


std::chrono::time_point<std::chrono::steady_clock> ObsDetector::test_awake(int secs)
{
    //delays the viewer being cleared for testing
    return std::chrono::steady_clock::now() + std::chrono::seconds(secs);
}


void ObsDetector::test(vector<GPU_Cloud> raw_data, const vector<EuclideanClusterExtractor::ObsReturn>& truth_list)
{
  std::vector<EuclideanClusterExtractor::ObsReturn> measured; // Raw data put through obs detector
  //measured.reserve(truth_list.size());

  std::vector<float> truth_volumes; // Total vol of all ground truths
  //truth_volumes.reserve(truth_list.size());

  std::vector<float> g_t; //Output of I / total true vol
 //g_t.reserve(truth_list.size());

  std::vector<float> false_positive_vol; // False positive vol / true vol
  //false_positive_vol.reserve(truth_list.size());

  std::vector<float> clock_times; // GPU obs det runtimes
  //clock_times.reserve(truth_list.size());

  std::vector<int> true_count; // num true obs
  //true_count.reserve(truth_list.size());

  std::vector<int> obs_count; // num test obs
  //obs_count.reserve(truth_list.size());

  std::vector<vector<float>> discrete_truth_pct;
  discrete_truth_pct.resize(truth_list.size());


  /* Calculate total truth volume */
  /* Evaluate raw test data */
  for(size_t i = 0; i < truth_list.size(); i++)
  {
    /* Evaluate Raw Data */
    Timer clock_count("testing timer"); // GPU Obs detect runtime counter
    Bins b;
    passZ->run(raw_data[i]); //The filter and RANSAC get rid of small scenes entirely
    ransacPlane->computeModel(raw_data[i]);
    b = voxelGrid->run(raw_data[i]);

    measured.push_back(ece->extractClusters(raw_data[i],b));

    clock_times.push_back(clock_count.getTime());
    clock_count.reset();

    /* Add time delay in viewer */
    drawCubes(measured[i],   1);
    drawCubes(truth_list[i], 0);
    viewer.update();
    std::this_thread::sleep_until(test_awake(5));
    viewer.clearEphemerals();

    /* ––––––––––––––––––––––––––––––––– */

    float current_volume_sum = 0; //global volume sum
    float current_intersection_sum = 0; //global intersection sum
    true_count.push_back(static_cast<int>(truth_list[i].obs.size()));
    obs_count.push_back(static_cast<int>(measured[i].obs.size()));

    /* loop through all truth obs in vector */
    for(size_t j = 0; j < truth_list[i].obs.size(); j++)
    {
      float current_intersection = 0; // Intsct for each truth obs
      float current_volume = 0; // vol of each truh obs

      /* Loop through all test obs */
      for(size_t k = 0; k < measured[i].obs.size(); k++)
      {
        /* find intersection between test and truth */
        /* if intersection > 0, add vol of intersection */
        float cinter = calculateIntersection(truth_list[i].obs[j], measured[i].obs[k]);
        if (!std::isinf(cinter) && cinter > 0) {
            current_intersection += cinter;
        }//only add if valid

      }
      // Volume of current ground truth obs
      current_volume = (truth_list[i].obs[j].maxX - truth_list[i].obs[j].minX)
          * (truth_list[i].obs[j].maxY - truth_list[i].obs[j].minY)
          * (truth_list[i].obs[j].maxZ - truth_list[i].obs[j].minZ);

      if(std::isinf(current_volume)) // sometimes it's NaN (infinty or 0/0)
        current_volume = 0;

      current_intersection_sum += current_intersection; //add to running total intersection
      current_volume_sum += current_volume; //global vol sum
      /* push %-detected of each truth obs to current ObsReturn index */


      if(current_volume != 0)
        discrete_truth_pct[i].push_back(current_intersection / current_volume);
    }
    if (current_volume_sum != 0) {
        truth_volumes.push_back(current_volume_sum); //global volume
        g_t.push_back(current_intersection_sum / current_volume_sum); //global quasi-IOU
    }
    //else {
    //    truth_volumes.push_back(0); //invalid object - there is no volume
    //    g_t.push_back(0); //invalid -
    //}

  }

  /* Now calculate false positives over ground truth */
  /* More or less the same calculation, but reversed */
  for(size_t x = 0; x < truth_list.size(); x++) //loop of ObsReturns
  {
    float volsum = 0;
    float temp_intersection = 0;
    for(size_t y = 0; y < measured[x].obs.size(); y++)
    {
      for(size_t z = 0; z < truth_list[x].obs.size(); z++)
      {
        temp_intersection +=
        calculateIntersection(truth_list[x].obs[z], measured[x].obs[y]) > 0 ? calculateIntersection(truth_list[x].obs[z], measured[x].obs[y]) : 0;
      }
      /* volume of current test obs */
      if (!(measured[x].obs[y].minX > measured[x].obs[y].maxX || measured[x].obs[y].minY > measured[x].obs[y].maxY || measured[x].obs[y].minZ > measured[x].obs[y].maxZ)) {
          //valid obstacle
          volsum += (measured[x].obs[y].maxX - measured[x].obs[y].minX)
              * (measured[x].obs[y].maxY - measured[x].obs[y].minY)
              * (measured[x].obs[y].maxZ - measured[x].obs[y].minZ);
      }
    }
      /* return total false positive volume for current obsreturn */
      float f_p_v = (volsum - temp_intersection)/truth_volumes[x];
      if(f_p_v > 0)
        false_positive_vol.push_back(f_p_v);
      else
        false_positive_vol.push_back(0);
  }

  test_print(g_t, false_positive_vol, clock_times, true_count, obs_count, discrete_truth_pct);
}


void ObsDetector::test_print(const std::vector<float>& iot,
                             const std::vector<float>& fot,
                             const std::vector<float>& times,
                             const std::vector<int>& num_true_obs,
                             const std::vector<int>& num_det_obs,
                             const std::vector<std::vector<float>>& discrete_truths) {
  /* Output Log Format */
  //std::cout.setprecision(4);
  std::cout << "Evaluating GPU Cloud #[NUMBER]\n";
  std::cout << "GPU Obstacle Detection Runtime: [TIME]\n";
  std::cout << "Number of Detected Obstacles: [NUMDET]\n";
  std::cout << "Number of True Obstacles: [NUMTRUE]\n";
  std::cout << "\t(What Should be detected)\n";
  std::cout << "Percent Truth Detected: [TRUE]\n";
  std::cout << "\t(Total intersection divided by total true area)\n\tShould be close to 1\n";
  std::cout << "False Positive over True Volume: [FALSE]\n";
  std::cout << "\t(Total detected volume not contained in set of true obstacles)\n\t(Divided by total true volume)\n";
  std::cout << "Mean % Truth: [TRUEPCT]\n";
  std::cout << "Average volume of a truth detected\n";
  std::cout << "Obstacle #[NUMBER] % detected: [PERCENT]\n";
  std::cout << "Processed Clouds: " << iot.size() << "\n";

  for (size_t i = 0; i < iot.size(); i++) // Loop through each cloud
  {
      std::cout << "\n–––––––––––––––––––––––\n–––––––––––––––––––––––\n\n";
      std::cout << "Evaluating GPU Cloud #" << i << "\n";
      std::cout << "GPU Obstacle Detection Runtime: " << times[i] << "\n";
      std::cout << "Number of Detected Obstacles: " << num_det_obs[i] << "\n";
      std::cout << "Number of True Obstacles: " << num_true_obs[i] << "\n";
      std::cout << "Percent Truth Detected: " << iot[i] * 100 << "\n";
      std::cout << "False Positive over True Volume: " << fot[i] << "\n";
      std::cout << "Mean % Truth: " <<
          std::accumulate(discrete_truths[i].begin(),
              discrete_truths[i].end(),
              static_cast<float>(0),
              [&](const float& a, const float& b) {
                  return a + b / discrete_truths[i].size();
              }) * 100 << "\n";
      for (size_t j = 0; j < discrete_truths[i].size(); ++j) {
          std::cout << "Obstacle #" << j << "\t% detected: " << discrete_truths[i][j] * static_cast<float>(100) << "\n";
      }
  }
}//End print()

float ObsDetector::calculateIntersection(const EuclideanClusterExtractor::Obstacle& truth_obst, const EuclideanClusterExtractor::Obstacle& eval_obst) {
    /* Find coordinates of rect prism of intersection */
    if (truth_obst.minX > truth_obst.maxX || truth_obst.minY > truth_obst.maxY || truth_obst.minZ > truth_obst.maxZ
        || eval_obst.minX > eval_obst.maxX || eval_obst.minY > eval_obst.maxY || eval_obst.minZ > eval_obst.maxZ) return 0;
    float xa = (std::max(truth_obst.minX, eval_obst.minX));
    float xb = (std::min(truth_obst.maxX, eval_obst.maxX));
    float ya = (std::max(truth_obst.minY, eval_obst.minY));
    float yb = (std::min(truth_obst.maxY, eval_obst.maxY));
    float za = (std::max(truth_obst.minZ, eval_obst.minZ));
    float zb = (std::min(truth_obst.maxZ, eval_obst.maxZ));
    return (xb - xa) * (yb - ya) * (zb - za);
}

 ObsDetector::~ObsDetector() {
     delete passZ;
     delete ransacPlane;
     delete voxelGrid;
     delete ece;
 }



int main() {
    ObsDetector obs(DataSource::FILESYSTEM, OperationMode::TEST, ViewerType::GL);


    //std::thread updateTick( [&]{while(true) { obs.update();} });

    //if testing comment out the loop
    /*
    while (true) {
       //obs.update();
       obs.spinViewer();
    }
    */
    return 0;
}
