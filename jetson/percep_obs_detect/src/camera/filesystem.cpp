#include "filesystem.hpp"
#include "reader.h"

namespace Source {

    FileSystem::FileSystem(const rapidjson::Document& config) {
        frame_counter = -1;
        read_location = config["startup"]["read_location"].GetString();
        max_frame = -1;

        if (config["startup"]["obs_enabled"].GetInt()) {
            reader.open(read_location + "/pcd/");
            max_frame = reader.size();
        }
    }

    FileSystem::~FileSystem() {
        // deleteCloud(pc);
    }

    /**
     * @brief Will loop over all images infinitely
     * @return true
     * @return false
     */
    bool FileSystem::grab_frame() {
        if (pc.data) {
            deleteCloud(pc);
        }
        if (no_grab) {
            no_grab = false;
        } else {
            frame_counter++;

            if (max_frame != -1) {
                frame_counter %= max_frame;
            }
        }

        return true;
    }

    void FileSystem::ignore_grab() {
        no_grab = true;
    }

    cv::Mat& FileSystem::get_image() {
        string filename = read_location + "/" + RGB_FOLDER + "/" +
                          RGB_FOLDER + std::to_string(frame_counter) + ".jpg";
        image = cv::imread(filename.c_str());
        return image;
    };

    cv::Mat& FileSystem::get_depth() {
        string filename = read_location + "/" + DEPTH_FOLDER + "/" +
                          DEPTH_FOLDER + std::to_string(frame_counter) + ".exr";
        depth = cv::imread(filename.c_str());
        return depth;
    }

    GPU_Cloud FileSystem::get_cloud() {
        pc = reader.getCloudGPU(frame_counter);
        return pc;
    }


    void FileSystem::write_data() {
        std::cerr << "You Fool!\n";
        exit(-1);
    }

    int FileSystem::get_frame() {
        return frame_counter;
    }

    int FileSystem::get_max_frame() {
        return max_frame;
    }

    void FileSystem::set_frame(int frame) {
        frame_counter = frame;
    }
}
