#include <fstream>
#include <glm/vec4.hpp> // glm::vec4
#include <iostream>
#include <vector>
#include <sstream>

#include "common.hpp"

typedef glm::vec4 vec4;
using namespace std;

class PCDReader {
    public:
        PCDReader() {
        }

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

        GPU_Cloud readCloudGPU(string file) {
            std::vector<glm::vec4> pc_raw = readCloudCPU(file);
            GPU_Cloud pc = createCloud(pc_raw.size());
            cudaMemcpy(pc.data, &pc_raw[0], sizeof(glm::vec4) * pc_raw.size(), cudaMemcpyHostToDevice);
            return pc;
        }
};