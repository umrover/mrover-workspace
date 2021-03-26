#pragma once

#include "common.hpp"

/** 
 * \class RansacPlane
 * \brief Uses RANSAC segmentation method to extract a plane in the point cloud. Computes a plane perpendicular to the given axis within the tolerance that fits 
 * the most data using the RANSAC algorithm
 */
class RansacPlane {
    public:
        /** 
         * \class Plane
         * \brief Defines the found plane by 3 points
         */
        struct Plane {
            sl::float3 p1;
            sl::float3 p2;
            sl::float3 p3;
        };

        /**
         * \brief RansacPlane constructor 
         * \param axis Unit vector giving the axis normal to the plane we're trying to find
         * \param epsilon How far off-angle a plane can be from the input axis and still be considered. Unit of degree
         * \param iterations The number of randomly selected models to attempt to fit to the data, must be less than 1024
         * \param threshold The maximum allowed distance from the model for a point to be considered an inlier
         * \param pcSize The maximum expected size of a point cloud passed to this algorithm (use the frame resolution area)
         */
        RansacPlane(sl::float3 axis, float epsilon, int iterations, float threshold, int pcSize);

        ~RansacPlane();

        /**
         * 
         * \brief [DEBUG-Only] Computes the RANSAC model on the GPU and returns the optimal model. Colors inliers
         * \param pc Point cloud to search 
         * \return Plane found by RANSAC segmentation
         */
        Plane computeModel(GPU_Cloud_F4 pc);

        /**
         * \brief Computes the RANSAC model on the GPU and returns the optimal model. Removes inliers from the cloud and changes its size
         * \param pc Point cloud to search 
         * \param flag Dummy parameter to distinguish from the debug version of this function
         * \return Plane found by RANSAC segmentation
         */
        Plane computeModel(GPU_Cloud_F4 &pc, bool flag);

    private:
        //user given model parms
        GPU_Cloud_F4 pc;
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