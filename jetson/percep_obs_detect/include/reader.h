#pragma once

#include <fstream>
#include <glm/vec4.hpp> // glm::vec4
#include <iostream>
#include <vector>
#include <sstream>
#include <stdlib.h>
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
        PCDReader() {
        }

        void open(string dir) {
            this->dir = dir;
            map<int, string> fData;

            //Long-winded directory opening (no glob, sad)
            DIR * pcd_dir;
            pcd_dir = opendir(dir.c_str());
            if (NULL == pcd_dir) std::cerr<<"Input folder not exist\n";

            struct dirent *dp = NULL;
            do {
                if ((dp = readdir(pcd_dir)) != NULL) {
                    std::string file_name(dp->d_name);
                    // the lengh of the tail str is at least 4
                    if (file_name.size() < 5) continue; //make it 5 to get the single digit
                    std::cout<<"file_name is "<<file_name<<std::endl;

                    pcd_names.push_back(file_name);

                    int s = file_name.find_first_of("0123456789");
                    int l = file_name.find(".");
                    string keyS = file_name.substr(s,l-s );
                    int key = stoi(keyS);
                    //cout << key << endl;
                    fData[key] = file_name;

                }
            } while (dp != NULL);
            std::sort(pcd_names.begin(), pcd_names.end());

            pcd_names.clear();
            for(auto i : fData) {
                pcd_names.push_back(i.second);
                cout << "reader says: " <<  i.second << endl;
            }
	    }

        // Reads a PCD file from a given filename into a vector of vec4 on the CPU
        std::vector<vec4> readCloudCPU(string file) {
            ifstream fin(file);
            if (!fin.is_open()) cerr << "Could not open file!" << endl;
            string line;

            // PC data
            std::vector<vec4> pc;
            int width, height;

            while(getline(fin, line)) {
                // Get the tokens of this line
                stringstream ss(line);
                string token;
                vector<string> tokens;
                while(ss >> token) tokens.push_back(token);

                // Process line
                if(tokens[0] == "WIDTH") width = stoi(tokens[1]);
                if(tokens[0] == "HEIGHT") height = stoi(tokens[1]);
                if(tokens[0] == "DATA") break;
            }

            pc.reserve(width * height);
            float x, y, z;
            unsigned int rgba;
            while(fin >> x >> y >> z >> rgba) {
                rgba = (rgba & 0xFF) << 16 | (rgba & 0xFF00) | (rgba & 0xFF0000) >> 16;
                float rgba_float = *((float*) &rgba);
                pc.push_back(vec4(x, y, z, rgba_float));
            }

            std::cout << "Read a point cloud of size " << width << " x " << height << endl;
            return pc;
        }
        #ifndef VIEWER_ONLY
        // Reads a cloud from a specified file name into a GPU_Cloud on the GPU
        GPU_Cloud readCloudGPU(string file) {
            std::vector<glm::vec4> pc_raw = readCloudCPU(file);
            GPU_Cloud pc = createCloud(pc_raw.size());
            cudaMemcpy(pc.data, &pc_raw[0], sizeof(glm::vec4) * pc_raw.size(), cudaMemcpyHostToDevice);
            return pc;
        }

        // Read a cloud by sequentially loaded indicies after calling open in a directory
        GPU_Cloud readCloudGPU(int i) {
            i = i % pcd_names.size();

            std::vector<glm::vec4> pc_raw = readCloudCPU(dir + pcd_names[i]);
            GPU_Cloud pc = createCloud(pc_raw.size());
            cudaMemcpy(pc.data, &pc_raw[0], sizeof(glm::vec4) * pc_raw.size(), cudaMemcpyHostToDevice);
            return pc;
        }
        #endif

    private:
        string dir;
        std::vector<std::string> pcd_names;
};
