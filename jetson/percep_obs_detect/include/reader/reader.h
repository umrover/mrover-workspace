#pragma once

#include <fstream>
#include <glm/vec4.hpp> // glm::vec4
#include <iostream>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <map>
#include <algorithm>

#include <string>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

#ifndef VIEWER_ONLY

#include "common.hpp"

#endif

typedef glm::vec4 vec4;
using namespace std;

class PCDReader {
    public:
        PCDReader() = default;

        void open(const string& dir) {
            this->dir = dir;
            map<int, string> fData;

            // Long-winded directory opening (no glob, sad)
            DIR* pcd_dir;
            pcd_dir = opendir(dir.c_str());
            if (!pcd_dir) {
                throw std::runtime_error("Input folder does not exist");
            }

            struct dirent* dp;
            do {
                if ((dp = readdir(pcd_dir)) != nullptr) {
                    std::string file_name(dp->d_name);
                    // the length of the tail str is at least 4
                    if (file_name.size() < 5) continue; // make it 5 to get the single digit
                    std::cout << "file_name is " << file_name << std::endl;

                    size_t s = file_name.find_first_of("0123456789");
                    size_t l = file_name.find('.');
                    string keyS = file_name.substr(s, l - s);
                    int key = stoi(keyS);
                    fData[key] = file_name;
                }
            } while (dp != nullptr);

            pcd_names.reserve(fData.size());
            for (auto i: fData) {
                pcd_names.push_back(i.second);
                cout << "reader says: " << i.second << endl;
            }
            pcds.resize(fData.size());
        }

        std::vector<vec4>& readCloudCPU(int i) {
            if (pcds.at(i).empty()) {
                pcds[i] = readCloudCPU(dir + pcd_names[i]);
            }
            return pcds[i];
        }

        // Reads a PCD file from a given filename into a vector of vec4 on the CPU
        static std::vector<vec4> readCloudCPU(const string& file) {
            ifstream fin(file);
            if (!fin.is_open()) cerr << "Could not open file!" << endl;
            string line;

            // PC data
            std::vector<vec4> pc;
            int width, height;

            while (getline(fin, line)) {
                // Get the tokens of this line
                stringstream ss(line);
                string token;
                vector<string> tokens;
                while (ss >> token) tokens.push_back(token);

                // Process line
                if (tokens[0] == "WIDTH") width = stoi(tokens[1]);
                if (tokens[0] == "HEIGHT") height = stoi(tokens[1]);
                if (tokens[0] == "DATA") break;
            }

            pc.reserve(width * height);


            float x, y, z;
            unsigned int rgba;

            std::string x_s, y_s, z_s, rgba_s;

            while (fin >> x_s >> y_s >> z_s >> rgba_s) {
                //if(x_s[0] == 'n' || x_s[0] == 'i' || x_s[1] = 'i') {
                //    pc.emplace_back(0, 0, 0, 0);
                //}
                x = std::stof(x_s);
                y = std::stof(y_s);
                z = std::stof(z_s);
                rgba = std::stof(rgba_s);

                rgba = (rgba & 0xFF) << 16 | (rgba & 0xFF00) | (rgba & 0xFF0000) >> 16;
                float rgba_float = *((float*) &rgba);
                pc.emplace_back(x, y, z, rgba_float);
            }

            std::cout << "Read point cloud " << file << " of size " << width << " x " << height << endl;
            return pc;
        }

#ifndef VIEWER_ONLY

        // Reads a cloud from a specified file name into a GPU_Cloud on the GPU
        GPU_Cloud readCloudGPU(const string& file) {
            std::vector<glm::vec4> pc_raw = readCloudCPU(file);
            GPU_Cloud pc = createCloud(pc_raw.size());
            cudaMemcpy(pc.data, &pc_raw[0], sizeof(glm::vec4) * pc_raw.size(), cudaMemcpyHostToDevice);
            return pc;
        }

        // Read a cloud by sequentially loaded indices after calling open in a directory
        GPU_Cloud getCloudGPU(int i) {
            if (pcds.at(i).empty()) {
                pcds[i] = readCloudCPU(dir + pcd_names[i]);
            }
            std::vector<glm::vec4> const& points = pcds[i];
            GPU_Cloud pc = createCloud(points.size());
            cudaMemcpy(pc.data, points.data(), sizeof(glm::vec4) * points.size(), cudaMemcpyHostToDevice);
            return pc;
        }

#endif

        size_t size() {
            return pcd_names.size();
        }

    private:
        string dir;
        std::vector<std::string> pcd_names;
        std::vector<std::vector<glm::vec4>> pcds;
};
