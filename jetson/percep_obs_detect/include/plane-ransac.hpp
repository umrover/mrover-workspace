#pragma once
#include "common.hpp"

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
        float3 zero = make_float3(0,0,0);
        p0 = zero;
        p1 = zero;
        p2 = zero;
        normal = zero;
    };

    Plane(float3 p0, float3 p1, float3 p2) : p0{ p0 }, p1{ p1 }, p2{ p2 } {
        ComputeNormal();
    };

    __host__ __device__ void ComputeNormal() {
        // Get the two vectors on the plane
        float3 v1 (p1 - p0);
        float3 v2 (p2 - p0);

        //Get vector normal to plane
        normal = cross(v1, v2);
    }

    __host__ __device__ float3 &operator[](int pt) {
        if (pt == 0)
            return p0;
        else if (pt == 1)
            return p1;
        else
            return p2;
    };
};

// Functor predicate to check if a point is 
class InPlane : public IPredicateFunctor {
    public:
    InPlane(float3 planeNormal, int threshold) : planeNormal{ planeNormal }, threshold{ threshold } {}

    virtual __host__ __device__ bool operator()(const float4 val) override {
        
        // Compute distsance point is from plane
        float3 curPt = make_float3(val.x, val.y, val.z);
        float3 d_to_model_pt = curPt - planeNormal;
        float d = abs(dot(planeNormal, d_to_model_pt));

        // Check distance against threshold
        return (d < threshold) ? 1 : 0;
    }

private:
    float3 planeNormal;
    int threshold;
};

/** 
 * \class RansacPlane
 * \brief Uses RANSAC segmentation method to extract a plane in the point cloud. Computes a plane perpendicular to the given axis within the tolerance that fits 
 * the most data using the RANSAC algorithm
 */
class RansacPlane {
    public:
        /**
         * \brief RansacPlane constructor 
         * \param axis Unit vector giving the axis normal to the plane we're trying to find
         * \param epsilon How far off-angle a plane can be from the input axis and still be considered. Unit of degree
         * \param iterations The number of randomly selected models to attempt to fit to the data, must be less than 1024
         * \param threshold The maximum allowed distance from the model for a point to be considered an inlier
         * \param pcSize The maximum expected size of a point cloud passed to this algorithm (use the frame resolution area)
         * \param removalRadius The max distance of points from the detected plane to be removed 
         */
        RansacPlane(float3 axis, float epsilon, int iterations, float threshold, int pcSize, float removalRadius);

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
        float3 axis;
        float epsilon;
        int iterations;
        float threshold;
        float removalRadius;

        //internal info [GPU]
        float* inlierCounts; 
        int* modelPoints; 
        Plane* selection;
        int* inlierListGPU;
        int* optimalModelIndex;

        //internal info [CPU]
        Plane* selectedModel;
        //int* inlierList;
        

};