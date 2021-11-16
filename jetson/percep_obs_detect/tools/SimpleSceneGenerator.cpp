#include <iostream>
#include <cmath>
#include <vector>
#include <glm/vec4.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <random>
#include <functional>

typedef glm::vec4 vec4;
using namespace std;

struct float4 {
    float x;
    float y;
    float z;
    float w;
};

ostream& operator<<(ostream& os, float4 in) {
    os << "X: " << in.x << " Y: " << in.y << " Z: " << in.z << "\n";
    return os;
}


struct Box {
    float minX;
    float maxX;
};

float4 make_float4(float x, float y, float z, float w) {
    float4 f = {x, y, z, w};
    return f;
}

struct Obstacle {
    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
};

struct ObsReturn {
    int size = 0;
    vector<Obstacle> obs;
};

struct GPU_Cloud {
    vector<float4> data;
    int size;
};

class PCDWriter {
public:
    PCDWriter() {
    }

    void writeCloud(string filename, GPU_Cloud gpu_pc, int width, int height) {
        ofstream fout(filename);
        if (!fout.is_open()) cerr << "Could not open file!" << endl;

        glm::vec4 *pc = new glm::vec4[gpu_pc.size];
        //cudaMemcpy(pc, gpu_pc.data, sizeof(glm::vec4) * gpu_pc.size, cudaMemcpyDeviceToHost);

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
		const float4& point = gpu_pc.data[i];
            rgba = *reinterpret_cast<const unsigned int*>(&point.w);
            rgba = (rgba & 0xFF) << 16 | (rgba & 0xFF00) | (rgba & 0xFF0000) >> 16;

            fout << std::setprecision(8) << point.x << ' ' << point.z << ' ' << point.y << ' ' << rgba << '\n';
        }

        std::cout << "Wrote a point cloud of size " << width << " x " << height << '\n';

        delete[] pc;
    }
};

class PcGenerator {
private:
    float plane_spacing;
    float obj_spacing;
    vector<Obstacle> obstacles;
    vector<float4> pc;
    bool hollow;
    bool noise;

    float plane_size;
    float4 normal_vector;
    float4 plane_center;

    void addPoint(float x, float y, float z, float w) {
        if(noise) {
            float mean = 0.0f;
            float stddev = 0.05f;

            auto dist = std::bind(std::normal_distribution<float>{mean, stddev},
                      std::mt19937(std::random_device{}()));

            x = x + dist();
            y = y + dist();
            z = z + dist();
        }

        pc.push_back(make_float4(x, y, z, w));

    }
    
    void planeGen() {
        
        //Create matrix with numLines x numLines points spanning from (pointxy-size/2) to (pointxy+size/2) in x and y and 0 for z
        if(normal_vector.z == 0) {
            normal_vector.z = 0.0001f;
        }
        for(float i = plane_center.x - plane_size / 2.0f; i <= plane_center.x + plane_size / 2.0f; i+= plane_spacing) {
            for(float j = plane_center.y - plane_size / 2.0f; j <= plane_center.y + plane_size / 2.0f; j+= plane_spacing) {
                //initialize points
                float k = ((-1)*(i-plane_center.x)*normal_vector.x + (-1)*(j-plane_center.y)*normal_vector.y + normal_vector.z*plane_center.z)/normal_vector.z;
                addPoint(i, j, k, 1.0f);
            }
        }

    }

    void boxGen(int i) {
        for (float x = obstacles[i].minX; x <= obstacles[i].maxX; x += obj_spacing) {
            for (float y = obstacles[i].minY; y <= obstacles[i].maxY; y += obj_spacing) {
                for (float z = obstacles[i].minZ; z <= obstacles[i].maxZ; z += obj_spacing) {
                    if(hollow) {
                        if((x == obstacles[i].minX || (x <= obstacles[i].maxX && (x+obj_spacing > obstacles[i].maxX))) ||
                            (y == obstacles[i].minY || (y <= obstacles[i].maxY && (y+obj_spacing > obstacles[i].maxY))) ||
                            (z == obstacles[i].minZ || (z <= obstacles[i].maxZ && (z+obj_spacing > obstacles[i].maxZ)))) {
                            addPoint(x, y, z, 0.0f);
                        }
                    } else {
                        addPoint(x, y, z, 0.0f);
                    }
                    
                }
            }
        }
    }
    
    void pcGen() {
        planeGen();

        for (size_t i = 0; i < obstacles.size(); i++) {
            boxGen((int)i);
        }
    }
    
public:
    
    //Custom constructors to add customization
    PcGenerator(ObsReturn in, float spacing_in, float size_in, float4 normal_vector_in, float4 plane_center_in, bool hollow_in, bool noise_in) {
        obj_spacing = spacing_in;
        plane_spacing = spacing_in;
        obstacles = in.obs;

        plane_size = size_in;
        normal_vector = normal_vector_in;
        plane_center = plane_center_in;
        hollow = hollow_in;
        noise = noise_in;
    }

    //Just Object input; creates plane size and center automatically as a flat plane on y=0
    PcGenerator(ObsReturn in, float spacing_in, bool hollow_in, bool noise_in) {
        obj_spacing = spacing_in;
        plane_spacing = spacing_in;
        obstacles = in.obs;

        float minX_total = INT16_MAX;
        float maxX_total = INT16_MIN;
        float minY_total = INT16_MAX;
        float maxY_total = INT16_MIN;

        for(size_t i = 0; i < obstacles.size(); i++) {
            if(obstacles[i].minX < minX_total) {
                minX_total = obstacles[i].minX;
            }
            if(obstacles[i].maxX > maxX_total) {
                maxX_total = obstacles[i].maxX;
            }
            if(obstacles[i].minY < minY_total) {
                minY_total = obstacles[i].minY;
            }
            if(obstacles[i].maxY > maxY_total) {
                maxY_total = obstacles[i].maxY;
            }
        }

        plane_center = make_float4((minX_total+maxX_total)/2, (minY_total+maxY_total)/2, 0, 0);
        normal_vector = make_float4(0, 0, 1, 0);
        plane_size = max(maxX_total-minX_total,maxY_total-minY_total) * 1.5f;

        hollow = hollow_in;
        noise = noise_in;
    }
    
    GPU_Cloud getGPU() {
        pcGen();
        GPU_Cloud temp;
        temp.data = pc;
        temp.size = (int)pc.size();
        return temp;
    }
};



int main() {
    //Storage of scene obstacles
    ObsReturn objects; 

    //Init all obstacles to be added to the scene and push them to ObsReturn.obs
    //Obstacle <name> = {minX, maxX, minY, maxY, minZ, maxZ};
    Obstacle one = {-4, -3, 3, 4, 0, 1};
    objects.obs.push_back(one);
    
    //Init an object of PcGenerator Class using one of the following constructors 
    //(Constructor purpose is shown in comments above the constructor)
    //FORMAT PcGenerator(ObsReturn, spacing, size, normal vector of plane, center point of plane, hollow, noise)
    //OR     PcGenetator(ObsReturn, spacing, hollow, noise)

    float4 normalVector = make_float4(0, 0, 1, 0);
    float4 center = make_float4(0,0,0,0);

    PcGenerator scene(objects, 0.33f, 10.0, normalVector, center, false, true);
    GPU_Cloud cloud = scene.getGPU();


    PCDWriter writer;
    writer.writeCloud("pc.pcd", cloud, 10, 10);

}