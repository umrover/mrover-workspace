#pragma once

#include "common.hpp"
#include "filter.hpp"

/** 
 * \class Plane
 * \brief Defines the found plane by 3 points
 */
class Plane {
    public:
        float3 p0;
        float3 p1;
        float3 p2;
        float3 normal;

        Plane() {
            float3 zero = make_float3(0, 0, 0);
            p0 = zero;
            p1 = zero;
            p2 = zero;
            normal = zero;
        };

        __host__ __device__ Plane(float3 p0, float3 p1, float3 p2) : p0{p0}, p1{p1}, p2{p2} {
            ComputeNormal();
        };

        __host__ __device__ void ComputeNormal() {
            // Get the two vectors on the plane
            float3 v1(p1 - p0);
            float3 v2(p2 - p0);

            //Get vector normal to plane
            normal = cross(v1, v2);
        }

        __host__ __device__ float3& operator[](int pt) {
            if (pt == 0)
                return p0;
            else if (pt == 1)
                return p1;
            else
                return p2;
        };
};


/** 
 * \class RansacPlane
 * \brief Uses RANSAC segmentation method to extract a plane in the point cloud. Computes a plane perpendicular to the given axis within the tolerance that fits 
 * the most data using the RANSAC algorithm
 */
class RansacPlane {
    public:
        // TODO: better way than making public?
        float epsilon;
        float threshold;
        float removalRadius;
        FilterOp filterOp = FilterOp::REMOVE;

        /**
         * \brief RansacPlane constructor
         * \param axis Unit vector giving the axis normal to the plane we're trying to find
         * \param epsilon How far off-angle a plane can be from the input axis and still be considered. Unit of degrees
         * \param iterations The number of randomly selected models to attempt to fit to the data, must be less than 1024
         * \param threshold The maximum allowed distance from the model for a point to be considered an inlier
         * \param pcSize The maximum expected size of a point cloud passed to this algorithm (use the frame resolution area)
         * \param removalRadius The max distance of points from the detected plane to be removed
         */
        RansacPlane(float3 axis, float epsilon, int iterations, float threshold, int pcSize, float removalRadius);

        ~RansacPlane();

        /**
         * \brief Computes the RANSAC model on the GPU and returns the optimal model. Removes inliers from the cloud and changes its size
         * \param pc Point cloud to search
         * \return Plane found by RANSAC segmentation
         */
        Plane computeModel(GPU_Cloud& pc);

        int getIterations() const;

        void setIterations(int iterations);

    private:
        /**
         * \brief Picks the model with the highest inlier count and updates the Plane "selection"
         */
        void selectOptimalModel();

        // user given model parms
        GPU_Cloud pc;
        float3 axis;
        int iterations = -1;
        int pcSize;

        // internal info [GPU]
        int* inlierCounts = nullptr;
        int3* candidatePlanePoints = nullptr;
        Plane* selection = nullptr;

        //internal info [CPU]
        Plane* selectionCPU = nullptr;
};
