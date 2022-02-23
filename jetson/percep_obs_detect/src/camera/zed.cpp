#include "zed.hpp"
#include <iomanip>
#include <vector>
#include <glm/vec4.hpp>
#include <iostream>
#include <fstream>

typedef glm::vec4 vec4;

using namespace std;

namespace Source {

    Zed::Zed(const rapidjson::Document& config) {
        // Read Config file variables
        res = sl::Resolution(
                config["camera"]["resolution_width"].GetInt(),
                config["camera"]["resolution_height"].GetInt());
        init_params.coordinate_units = sl::UNIT::MILLIMETER; // TODO: Read from config
        sl::RESOLUTION resolution;
        if (config["camera"]["resolution_type"] == "VGA") {
            resolution = sl::RESOLUTION::VGA;
        } else if (config["camera"]["resolution_type"] == "HD720") {
            resolution = sl::RESOLUTION::HD720;
        } else {
            throw std::runtime_error("Unsupported resolution type");
        }
        init_params.camera_resolution = resolution;
        init_params.camera_fps = config["camera"]["zed_params"]["camera_fps"].GetInt();
        write_location = config["startup"]["write_location"].GetString();
        frame_gap = config["camera"]["frame_write_interval"].GetInt();
        frame_counter = 0;

        // Allocate sl::Mats
        image_zed.alloc(res.width, res.height, sl::MAT_TYPE::U8_C4);
        depth_zed.alloc(res.width, res.height, sl::MAT_TYPE::F32_C1);
        frame.alloc(res, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);

        // Map sl::Mats to cv::Mat, so update to sl::Mat updates the cv::Mat
        image = cv::Mat(res.height, res.width, CV_8UC4, image_zed.getPtr<sl::uchar1>(sl::MEM::CPU));
        depth = cv::Mat(res.height, res.width, CV_32FC1, depth_zed.getPtr<sl::uchar1>(sl::MEM::CPU));

        auto error = zed.open(init_params);
        if (error != sl::ERROR_CODE::SUCCESS) {
            throw std::runtime_error("Error opening ZED camera");
        }
    }

    bool Zed::grab_frame() {
        if (no_grab) {
            no_grab = false;
            return true;
        } else {
            return zed.grab() == sl::ERROR_CODE::SUCCESS;
        }
    }

    void Zed::ignore_grab() {
        no_grab = true;
    }

    GPU_Cloud Zed::get_cloud() {
        GPU_Cloud pc;
        zed.retrieveMeasure(frame, sl::MEASURE::XYZRGBA, sl::MEM::GPU, res);
        getRawCloud(pc, frame);
        return pc;
    }

    cv::Mat& Zed::get_image() {
        zed.retrieveImage(image_zed, sl::VIEW::LEFT, sl::MEM::CPU, res); // Update sl::Mat
        return image;
    }

    cv::Mat& Zed::get_depth() {
        zed.retrieveMeasure(depth_zed, sl::MEASURE::DEPTH, sl::MEM::CPU, res);
        return depth;
    }

    void writeCloud(string filename, GPU_Cloud gpu_pc, int width, int height) {
        ofstream fout(filename);
        if (!fout.is_open()) cerr << "Could not open file!" << endl;

        glm::vec4* pc = new glm::vec4[gpu_pc.size];
        cudaMemcpy(pc, gpu_pc.data, sizeof(glm::vec4) * gpu_pc.size, cudaMemcpyDeviceToHost);

        fout << "# .PCD v0.7 - Point Cloud Data file format\n"
             << "VERSION 0.7\n"
             << "FIELDS x y z rgb\n"
             << "SIZE 4 4 4 4\n"
             << "TYPE F F F U\n"
             << "COUNT 1 1 1 1\n"
             << "WIDTH " << width << '\n'
             << "HEIGHT " << height << '\n'
             << "VIEWPOINT 0 0 0 1 0 0 0\n"
             << "POINTS " << gpu_pc.size << '\n'
             << "DATA ascii\n";

        unsigned int rgba;
        for (int i = 0; i < gpu_pc.size; ++i) {
            const vec4& point = pc[i];
            rgba = *reinterpret_cast<const unsigned int*>(&point.w);
            rgba = (rgba & 0xFF) << 16 | (rgba & 0xFF00) | (rgba & 0xFF0000) >> 16;

            fout << std::setprecision(8) << point.x << ' ' << point.y << ' ' << point.z << ' ' << rgba << '\n';
        }

        std::cout << "Wrote a point cloud of size " << width << " x " << height << '\n';

        delete[] pc;
    }

    // creates and opens folder to write to
    void disk_record_init(string data_folder) {
        //defining directories to write to
        string rgb_foldername = data_folder + "/rgb/";
        string depth_foldername = data_folder + "/depth/";
        string mkdir_rgb = std::string("mkdir -p ") + rgb_foldername;
        string mkdir_depth = std::string("mkdir -p ") + depth_foldername;
        string pcl_foldername = data_folder + "/pcd/";
        string mkdir_pcl = std::string("mkdir -p ") + pcl_foldername;

        //creates new folder in the system
        if (-1 == system(mkdir_pcl.c_str()) || -1 == system(mkdir_rgb.c_str()) || -1 == system(mkdir_depth.c_str())) {
            throw std::runtime_error("Failed to make folders");
        }
    }

    void Zed::write_data() {
        if ((frame_counter % frame_gap) == 0) {
            if (frame_counter == 0) { // First time around
                cout << "Creating folders\n";
                disk_record_init(write_location);
            }
            string file_num = std::to_string(frame_counter / frame_gap);
            string pcd_filename = write_location + "/" + PCD_FOLDER + "/" + PCD_FOLDER + file_num + ".pcd";
            string rgb_filename = write_location + "/" + RGB_FOLDER + "/" + RGB_FOLDER + file_num + ".jpg";
            string depth_filename = write_location + "/" + DEPTH_FOLDER + "/" + DEPTH_FOLDER + file_num + ".exr";

            writeCloud(pcd_filename, get_cloud(), res.width, res.height);
            cv::imwrite(rgb_filename, get_image());
            cv::imwrite(depth_filename, get_depth());

        }
        ++frame_counter;
    }

    int Zed::get_frame() {
        return frame_counter;
    }

    void Zed::set_frame(int frame) {
    }

    int Zed::get_max_frame() {
        return -1;
    }
}
