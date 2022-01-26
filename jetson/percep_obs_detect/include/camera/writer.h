#pragma once

#include <vector>
#include <glm/vec4.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "common.hpp"

typedef glm::vec4 vec4;
using namespace std;

class PCDWriter {
public:
    PCDWriter() {
    }

    void writeCloud(string filename, GPU_Cloud gpu_pc, int width, int height) {
        ofstream fout(filename);
        if (!fout.is_open()) cerr << "Could not open file!" << endl;

        glm::vec4 *pc = new glm::vec4[gpu_pc.size];
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
};
