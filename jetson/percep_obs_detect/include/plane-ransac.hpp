#pragma once

#include "common.hpp"

class RansacPlane {
    public:
        struct Plane {
            sl::float3 p1;
            sl::float3 p2;
            sl::float3 p3;
        };

        /*
        REQUIRES: 
        - Zed point cloud allocated on GPU
        - Axis perpendicular to the desired plane 
        - How far off angle the found plane can be from that axis
        - Maximum number of iterations to find the plane
        - Maximum distance from the plane to be considered an inlier
        EFFECTS:
        - Computes a plane perpendicular to the given axis within the tolerance that fits 
        the most data using the RANSAC algorithm
        */
        RansacPlane(sl::float3 axis, float epsilon, int iterations, float threshold,  int pcSize);
        RansacPlane();

        ~RansacPlane();

        /*
        EFFECTS:
        - Computes the RANSAC model on the GPU and returns the coefficients 
        */
        Plane computeModel(GPU_Cloud_F4 pc);
        //non debug kernel
        Plane computeModel(GPU_Cloud_F4 &pc, bool flag);


        /*
        EFFECTS:
        - Gets GPU pointer for indicies of the inliers of the model
        */
        GPU_Indicies getInliers();

        /*
        - Plane equation in standard form
        */



    private:
        //user given model parms
        GPU_Cloud_F4 pc;
        GPU_Indicies inliers;
        sl::float3 axis;
        float epsilon;
        int iterations;
        float threshold;

        //internal info [GPU]
        float* inlierCounts; 
        int* modelPoints; 
        float* selection;
        int* inlierListGPU;
        int* optimalModelIndex;

        //internal info [CPU]
        float* selectedModel;
        //int* inlierList;
        

};