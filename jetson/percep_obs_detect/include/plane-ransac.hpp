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
            float3d p1;
            float3d p2;
            float3d p3;
        };

        /**
         * \brief RansacPlane constructor 
         * \param axis Unit vector giving the axis normal to the plane we're trying to find
         * \param epsilon How far off-angle a plane can be from the input axis and still be considered. Unit of degree
         * \param iterations The number of randomly selected models to attempt to fit to the data, must be less than 1024
         * \param threshold The maximum allowed distance from the model for a point to be considered an inlier
         * \param pcSize The maximum expected size of a point cloud passed to this algorithm (use the frame resolution area)
         * \param removalRadius The max distance of points from the detected plane to be removed 
         */
        RansacPlane(float3d axis, float epsilon, int iterations, float threshold, int pcSize, float removalRadius);

        ~RansacPlane();

        /**
         * 
         * \brief [DEBUG-Only] Computes the RANSAC model on the GPU and returns the optimal model. Colors inliers
         * \param pc Point cloud to search 
         * \return Plane found by RANSAC segmentation
         */
        Plane computeModel(GPU_Cloud pc);

        /**
         * \brief Computes the RANSAC model on the GPU and returns the optimal model. Removes inliers from the cloud and changes its size
         * \param pc Point cloud to search 
         * \param flag Dummy parameter to distinguish from the debug version of this function
         * \return Plane found by RANSAC segmentation
         */
        Plane computeModel(GPU_Cloud &pc, bool flag);

    private:
        //user given model parms
        GPU_Cloud pc;
        float3d axis;
        float epsilon;
        int iterations;
        float threshold;
        float removalRadius;

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